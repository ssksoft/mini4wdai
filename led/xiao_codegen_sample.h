/*
 * Trial License - for use to evaluate programs for possible purchase as
 * an end-user only.
 *
 * File: xiao_codegen_sample.h
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

#ifndef xiao_codegen_sample_h_
#define xiao_codegen_sample_h_
#ifndef xiao_codegen_sample_COMMON_INCLUDES_
#define xiao_codegen_sample_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "math.h"
#endif                                /* xiao_codegen_sample_COMMON_INCLUDES_ */

#include "xiao_codegen_sample_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T Out1;                         /* '<Root>/Out1' */
} ExtY_xiao_codegen_sample_T;

/* Real-time Model Data Structure */
struct tag_RTM_xiao_codegen_sample_T {
  const char_T * volatile errorStatus;
};

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_xiao_codegen_sample_T xiao_codegen_sample_Y;

/* Model entry point functions */
extern void xiao_codegen_sample_initialize(void);
extern void xiao_codegen_sample_step(void);
extern void xiao_codegen_sample_terminate(void);

/* Real-time Model object */
extern RT_MODEL_xiao_codegen_sample_T *const xiao_codegen_sample_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'xiao_codegen_sample'
 */
#endif                                 /* xiao_codegen_sample_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
