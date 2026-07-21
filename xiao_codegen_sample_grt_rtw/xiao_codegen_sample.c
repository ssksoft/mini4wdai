/*
 * xiao_codegen_sample.c
 *
 * Trial License - for use to evaluate programs for possible purchase as
 * an end-user only.
 *
 * Code generation for model "xiao_codegen_sample".
 *
 * Model version              : 1.2
 * Simulink Coder version : 25.2 (R2025b) 28-Jul-2025
 * C source code generated on : Wed Jul 22 07:38:48 2026
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "xiao_codegen_sample.h"
#include "xiao_codegen_sample_private.h"

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
   *  Constant: '<Root>/Constant'
   */
  xiao_codegen_sample_Y.Out1 = xiao_codegen_sample_P.Constant_Value;

  /* Matfile logging */
  rt_UpdateTXYLogVars(xiao_codegen_sample_M->rtwLogInfo,
                      (&xiao_codegen_sample_M->Timing.taskTime0));

  /* signal main to stop simulation */
  {                                    /* Sample time: [0.2s, 0.0s] */
    if ((rtmGetTFinal(xiao_codegen_sample_M)!=-1) &&
        !((rtmGetTFinal(xiao_codegen_sample_M)-
           xiao_codegen_sample_M->Timing.taskTime0) >
          xiao_codegen_sample_M->Timing.taskTime0 * (DBL_EPSILON))) {
      rtmSetErrorStatus(xiao_codegen_sample_M, "Simulation finished");
    }
  }

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++xiao_codegen_sample_M->Timing.clockTick0)) {
    ++xiao_codegen_sample_M->Timing.clockTickH0;
  }

  xiao_codegen_sample_M->Timing.taskTime0 =
    xiao_codegen_sample_M->Timing.clockTick0 *
    xiao_codegen_sample_M->Timing.stepSize0 +
    xiao_codegen_sample_M->Timing.clockTickH0 *
    xiao_codegen_sample_M->Timing.stepSize0 * 4294967296.0;
}

/* Model initialize function */
void xiao_codegen_sample_initialize(void)
{
  /* Registration code */

  /* initialize real-time model */
  (void) memset((void *)xiao_codegen_sample_M, 0,
                sizeof(RT_MODEL_xiao_codegen_sample_T));
  rtmSetTFinal(xiao_codegen_sample_M, -1);
  xiao_codegen_sample_M->Timing.stepSize0 = 0.2;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = (NULL);
    xiao_codegen_sample_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(xiao_codegen_sample_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(xiao_codegen_sample_M->rtwLogInfo, (NULL));
    rtliSetLogT(xiao_codegen_sample_M->rtwLogInfo, "tout");
    rtliSetLogX(xiao_codegen_sample_M->rtwLogInfo, "");
    rtliSetLogXFinal(xiao_codegen_sample_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(xiao_codegen_sample_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(xiao_codegen_sample_M->rtwLogInfo, 4);
    rtliSetLogMaxRows(xiao_codegen_sample_M->rtwLogInfo, 0);
    rtliSetLogDecimation(xiao_codegen_sample_M->rtwLogInfo, 1);
    rtliSetLogY(xiao_codegen_sample_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(xiao_codegen_sample_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(xiao_codegen_sample_M->rtwLogInfo, (NULL));
  }

  /* external outputs */
  xiao_codegen_sample_Y.Out1 = 0.0;

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime(xiao_codegen_sample_M->rtwLogInfo, 0.0,
    rtmGetTFinal(xiao_codegen_sample_M), xiao_codegen_sample_M->Timing.stepSize0,
    (&rtmGetErrorStatus(xiao_codegen_sample_M)));
}

/* Model terminate function */
void xiao_codegen_sample_terminate(void)
{
  /* (no terminate code required) */
}
