 /**
 ******************************************************************************
 * @file    tracker.c
 * @author  GPM Application Team
 *
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include "tracker.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

typedef struct {
  double cx;
  double cy;
  double w;
  double h;
} trk_box_t;

static double trk_overlap(double x1, double w1, double x2, double w2)
{
  double l1 = x1 - w1 / 2;
  double l2 = x2 - w2 / 2;
  double r1 = x1 + w1 / 2;
  double r2 = x2 + w2 / 2;
  double left = l1 > l2 ? l1 : l2;
  double right = r1 < r2 ? r1 : r2;

  return right - left;
}

static double trk_box_intersection(trk_box_t *a, trk_box_t *b)
{
  double w = trk_overlap(a->cx, a->w, b->cx, b->w);
  double h = trk_overlap(a->cy, a->h, b->cy, b->h);

  return w < 0 || h < 0 ? 0 : w * h;
}

static double trk_box_union(trk_box_t *a, trk_box_t *b)
{
  double I = trk_box_intersection(a, b);

  return a->w * a->h + b->w * b->h - I;
}

static double trk_compute_iou(trk_tbox_t *tbox, trk_dbox_t *dbox)
{
  trk_box_t boxa = {dbox->cx, dbox->cy, dbox->w, dbox->h};
  trk_box_t boxb = {tbox->cx, tbox->cy, tbox->w, tbox->h};
  double I, U;

  I = trk_box_intersection(&boxa, &boxb);
  U = trk_box_union(&boxa, &boxb);

  return I == 0 || U == 0 ? 0 : I / U;
}

static void trk_kalman_init(trk_tbox_t *tbox, trk_dbox_t *dbox)
{
  struct kf_box m;

  m.cx = dbox->cx;
  m.cy = dbox->cy;
  m.a = dbox->w / dbox->h;
  m.h = dbox->h;
  kf_init(&tbox->kf_state, &m);
}

static void trk_kalman_pred(trk_tbox_t *tbox)
{
  struct kf_box predicted;

  if (tbox->tlost_cnt)
    tbox->kf_state.mean[7] = 0;
  kf_predict(&tbox->kf_state, &predicted);
  tbox->cx = predicted.cx;
  tbox->cy = predicted.cy;
  tbox->w = predicted.a * predicted.h;
  tbox->h = predicted.h;
}

static void trk_kalman_update(trk_tbox_t *tbox, trk_dbox_t *dbox)
{
  struct kf_box m;

  m.cx = dbox->cx;
  m.cy = dbox->cy;
  m.a = dbox->w / dbox->h;
  m.h = dbox->h;
  kf_update(&tbox->kf_state, &m);
  tbox->cx = tbox->kf_state.mean[0];
  tbox->cy = tbox->kf_state.mean[1];
  tbox->w = tbox->kf_state.mean[2] * tbox->kf_state.mean[3];
  tbox->h = tbox->kf_state.mean[3];
}

static void trk_tbox_set_free(trk_ctx_t *ctx, trk_tbox_t *tbox)
{
  tbox->is_tracking = 0;
  tbox->dbox_userdata = NULL;
  ulist_add(&tbox->list, &ctx->tfree);
}

static void trk_tbox_set_lost(trk_ctx_t *ctx, trk_tbox_t *tbox)
{
  tbox->dbox_userdata = NULL;
  ulist_add_tail(&tbox->list, &ctx->tlost);
}

static void trk_tbox_set_tracking(trk_ctx_t *ctx, trk_dbox_t *dbox)
{
  trk_tbox_t *tbox;

  if (ulist_is_empty(&ctx->tfree))
    return ;

  tbox = ulist_entry(ctx->tfree.next, trk_tbox_t, list);
  trk_kalman_init(tbox, dbox);
  tbox->is_tracking = 1;
  tbox->id = ctx->next_id++;
  tbox->tlost_cnt = 0;
  tbox->cx = dbox->cx;
  tbox->cy = dbox->cy;
  tbox->w = dbox->w;
  tbox->h = dbox->h;
  tbox->dbox_userdata = dbox->userdata;
  ulist_move_tail(&tbox->list, &ctx->ttracking);
}

static void trk_tbox_continue_tracking(trk_ctx_t *ctx, trk_tbox_t *tbox, trk_dbox_t *dbox)
{
  tbox->tlost_cnt = 0;
  tbox->dbox_userdata = dbox->userdata;
  trk_kalman_update(tbox, dbox);
  ulist_move_tail(&tbox->list, &ctx->ttracking);
}

static void trk_kalman_pred_tboxes(trk_ctx_t *ctx)
{
  trk_tbox_t *tbox;

  ulist_for_each_entry(tbox, &ctx->ttracking, list)
    trk_kalman_pred(tbox);
  ulist_for_each_entry(tbox, &ctx->tlost, list)
    trk_kalman_pred(tbox);
}

static void trk_dbox_split(trk_ctx_t *ctx, int trk_dbox_nb, trk_dbox_t *dboxes)
{
  int i;

  /* clear list */
  ulist_init_head(&ctx->dhigh);
  ulist_init_head(&ctx->dlow);

  for (i = 0; i < trk_dbox_nb; i++) {
    trk_dbox_t *dbox = &dboxes[i];

    if (dbox->conf > ctx->cfg.track_thresh)
      ulist_add_tail(&dbox->list, &ctx->dhigh);
    else
      ulist_add_tail(&dbox->list, &ctx->dlow);
  }
}

static void trk_matching_step1(trk_ctx_t *ctx)
{
  trk_tbox_t *tbox, *ttmp, *tboxhigh;
  trk_dbox_t *dbox, *dtmp;
  double max_score;
  double score;

  /* move tracked box from ttracking/tlost to tremain */
  ulist_for_each_entry_safe(tbox, ttmp, &ctx->ttracking, list)
    ulist_move_tail(&tbox->list, &ctx->tremain);
  ulist_for_each_entry_safe(tbox, ttmp, &ctx->tlost, list)
    ulist_move_tail(&tbox->list, &ctx->tremain);

  /* match tbox into tremain with dbox in dhigh */
  ulist_for_each_entry_safe(dbox, dtmp, &ctx->dhigh, list) {
    max_score = -1;
    tboxhigh = NULL;
    ulist_for_each_entry_safe(tbox, ttmp, &ctx->tremain, list) {
      score = trk_compute_iou(tbox, dbox) * dbox->conf;
      if (score <= max_score)
        continue;
      max_score = score;
      tboxhigh = tbox;
    }
    if (max_score < 1 - ctx->cfg.sim1_thresh)
      continue;
    trk_tbox_continue_tracking(ctx, tboxhigh, dbox);
    ulist_del(&dbox->list);
  }
}

static void trk_matching_step2(trk_ctx_t *ctx)
{
  trk_tbox_t *tbox, *ttmp, *tboxhigh;
  trk_dbox_t *dbox, *dtmp;
  double max_score;
  double score;

  /* match tbox into tremain with dbox in dlow */
  ulist_for_each_entry_safe(dbox, dtmp, &ctx->dlow, list) {
    max_score = -1;
    tboxhigh = NULL;
    ulist_for_each_entry_safe(tbox, ttmp, &ctx->tremain, list) {
      score = trk_compute_iou(tbox, dbox);
      if (score <= max_score)
        continue;
      max_score = score;
      tboxhigh = tbox;
    }
    if (max_score < ctx->cfg.sim2_thresh)
      continue;
    trk_tbox_continue_tracking(ctx, tboxhigh, dbox);
    ulist_del(&dbox->list);
  }
}

static void trk_update_tlost(trk_ctx_t *ctx)
{
  trk_tbox_t *tbox, *ttmp;

  ulist_for_each_entry_safe(tbox, ttmp, &ctx->tremain, list) {
    tbox->tlost_cnt++;
    ulist_del(&tbox->list);
    if (tbox->tlost_cnt == ctx->cfg.tlost_cnt)
      trk_tbox_set_free(ctx, tbox);
    else
      trk_tbox_set_lost(ctx, tbox);
  }

  assert(ulist_is_empty(&ctx->tremain) == 1);
}

static void trk_add_new_tracks(trk_ctx_t *ctx)
{
  trk_dbox_t *dbox;

  ulist_for_each_entry(dbox, &ctx->dhigh, list) {
    if (dbox->conf < ctx->cfg.det_thresh)
      continue;
    trk_tbox_set_tracking(ctx, dbox);
  }
}

int trk_init(trk_ctx_t *ctx, trk_conf_t *cfg, int trk_tbox_nb, trk_tbox_t *tboxes)
{
  int i;

  ctx->cfg = *cfg;
  ctx->next_id = 1;
  ulist_init_head(&ctx->tfree);
  ulist_init_head(&ctx->ttracking);
  ulist_init_head(&ctx->tlost);
  ulist_init_head(&ctx->tremain);
  ulist_init_head(&ctx->dhigh);
  ulist_init_head(&ctx->dlow);
  for (i = 0; i < trk_tbox_nb; i++)
    trk_tbox_set_free(ctx, &tboxes[i]);

  return 0;
}

int trk_update(trk_ctx_t *ctx, int trk_dbox_nb, trk_dbox_t *dboxes)
{
  trk_kalman_pred_tboxes(ctx);
  trk_dbox_split(ctx, trk_dbox_nb, dboxes);
  trk_matching_step1(ctx);
  trk_matching_step2(ctx);
  trk_update_tlost(ctx);
  trk_add_new_tracks(ctx);

  return 0;
}
