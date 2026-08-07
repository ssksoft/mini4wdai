/*
 * Trial License - for use to evaluate programs for possible purchase as
 * an end-user only.
 *
 * File: xiao_codegen_sample.c
 *
 * Code generated for Simulink model 'xiao_codegen_sample'.
 *
 * Model version                  : 1.6
 * Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
 * C/C++ source code generated on : Thu Aug  6 17:21:09 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "xiao_codegen_sample.h"
#include "rtwtypes.h"

/* Block states (default storage) */
DW_xiao_codegen_sample_T xiao_codegen_sample_DW;

/* External outputs (root outports fed by signals with default storage) */
ExtY_xiao_codegen_sample_T xiao_codegen_sample_Y;

/* Real-time model */
static RT_MODEL_xiao_codegen_sample_T xiao_codegen_sample_M_;
RT_MODEL_xiao_codegen_sample_T *const xiao_codegen_sample_M =
  &xiao_codegen_sample_M_;

/* Model step function */
void xiao_codegen_sample_step(void)
{
  /* Outport: '<Root>/Out1' incorporates:
   *  DiscretePulseGenerator: '<Root>/Pulse Generator'
   */
  xiao_codegen_sample_Y.Out1 = ((xiao_codegen_sample_DW.clockTickCounter < 10) &&
    (xiao_codegen_sample_DW.clockTickCounter >= 0));

  /* DiscretePulseGenerator: '<Root>/Pulse Generator' */
  if (xiao_codegen_sample_DW.clockTickCounter >= 19) {
    xiao_codegen_sample_DW.clockTickCounter = 0;
  } else {
    xiao_codegen_sample_DW.clockTickCounter++;
  }
}

/* Model initialize function */
void xiao_codegen_sample_initialize(void)
{
  /* (no initialization code required) */
}

/* Model terminate function */
void xiao_codegen_sample_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
