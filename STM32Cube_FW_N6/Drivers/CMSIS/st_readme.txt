
@verbatim
******************************************************************************
* @file    st_readme.txt
* @author  MCD Application Team
* @brief   This file lists the main modification done by STMicroelectronics on
*          CMSIS Base Software, CMSIS-DSP and CMSIS-NN Software Libraries for 
*          integration in STM32Cube (folders location updates only).
******************************************************************************
*
*   Copyright 2024-2025 STMicroelectronics.
*
*   Licensed under the Apache License, Version 2.0 (the "License");
*   you may not use this file except in compliance with the License.
*   You may obtain a copy of the License at
*
*       http://www.apache.org/licenses/LICENSE-2.0
*
*   Unless required by applicable law or agreed to in writing, software
*   distributed under the License is distributed on an "AS IS" BASIS,
*   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*   See the License for the specific language governing permissions and
*   limitations under the License.
* ******************************************************************************
@endverbatim

=======

### 28-July-2025 ###
====================

  + CMSIS Base Software component V6.2.0
  
  Suppress CMSIS/ folder level (done also in ARM.CMSIS.pdsc to match the STM32Cube legacy file structure)
  No import ARM.CMSIS.sha1 file, Core/Test and Driver folders
  To preserve CMSIS v5-based STM32Cube legacy file structure compatibility:
  - Add Include/ folder as a copy of Core/Include but without Core/Include/core_ca.h and Core/Include/a-profile folder
  - Add Core_A/ folder with a copy of Core/Include/cmsis_gcc.h, Core/Include/core_ca.h and Core/Include/a-profile folder 

  + CMSIS-DSP Software Library pack V1.16.2

  Component pack copied under CMSIS\DSP (without ARM.CMSIS-DSP.sha1 file)
  CMSIS-DSP Software Library full document available from DSP/Documentation/html/index.html

  + Remove CMSIS-NN Software Library pack

  Due to the multiplicity of CMSIS-NN Software Library pack versions any application resorting to CMSIS-NN shall
  download the selected CMSIS-NN Software Library pack version from https://github.com/ARM-software/CMSIS-NN

### 22-July-2024 ###
====================

  + CMSIS Base Software component V6.0.0
  
  Suppress CMSIS/ folder level (done also in ARM.CMSIS.pdsc to match the STM32Cube legacy file structure)
  No import ARM.CMSIS.sha1 file, Core/Test and Driver folders
  To preserve CMSIS v5-based STM32Cube legacy file structure compatibility:
  - Add Include/ folder as a copy of Core/Include but without Core/Include/core_ca.h and Core/Include/a-profile folder
  - Add Core_A/ folder as a copy of Core/Include/core_ca.h and Core/Include/a-profile folder 

  + CMSIS-DSP Software Library pack V1.15.0

  Component pack copied under CMSIS\DSP (without ARM.CMSIS-DSP.sha1 file)
  CMSIS-DSP Software Library full document available from DSP/Documentation/html/index.html

  + CMSIS-NN Software Library pack V5.0.0

  Component pack copied under CMSIS\NN (without ARM.CMSIS-NN.sha1 file)
  CMSIS-NN Software Library full documentation available from NN/Documentation/html/index.html
