/*
 * Trial License - for use to evaluate programs for possible purchase as
 * an end-user only.
 *
 * File: xiao_codegen_sample.c
 *
 * Code generated for Simulink model 'xiao_codegen_sample'.
 *
 * Model version                  : 1.3
 * Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
 * C/C++ source code generated on : Wed Jul 22 13:52:40 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "xiao_codegen_sample.h"

/* External outputs (root outports fed by signals with default storage) */
ExtY_xiao_codegen_sample_T xiao_codegen_sample_Y;

/* Real-time model */
static RT_MODEL_xiao_codegen_sample_T xiao_codegen_sample_M_;
RT_MODEL_xiao_codegen_sample_T *const xiao_codegen_sample_M =
  &xiao_codegen_sample_M_;

/* Model step function */
void xiao_codegen_sample_step(void)
{
  /* (no output/update code required) */
}

/* Model initialize function */
void xiao_codegen_sample_initialize(void)
{
  /* ConstCode for Outport: '<Root>/Out1' incorporates:
   *  Constant: '<Root>/Constant'
   */
  xiao_codegen_sample_Y.Out1 = 0.0;
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
