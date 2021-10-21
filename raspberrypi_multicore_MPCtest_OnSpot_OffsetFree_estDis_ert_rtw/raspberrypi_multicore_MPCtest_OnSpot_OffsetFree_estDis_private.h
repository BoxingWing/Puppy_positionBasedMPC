/*
 * raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis_private.h
 *
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * Code generation for model "raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis".
 *
 * Model version              : 2.392
 * Simulink Coder version : 9.5 (R2021a) 14-Nov-2020
 * C source code generated on : Thu Oct 21 19:49:16 2021
 *
 * Target selection: ert.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis_private_h_
#define RTW_HEADER_raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis_private_h_
#include "rtwtypes.h"
#include "multiword_types.h"
#include "raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis.h"

/* Private macros used by the generated code to access rtModel */
#ifndef rtmSetTFinal
#define rtmSetTFinal(rtm, val)         ((rtm)->Timing.tFinal = (val))
#endif

extern real_T rt_powd_snf(real_T u0, real_T u1);
extern real_T rt_roundd_snf(real_T u);
extern real_T rt_atan2d_snf(real_T u0, real_T u1);
extern real_T rt_hypotd_snf(real_T u0, real_T u1);
int_T rt_WriteMat4FileHeader(FILE *fp,
  int32_T m,
  int32_T n,
  const char_T *name);
extern void raspber_MATLABSystem4_Start(DW_MATLABSystem4_raspberrypi__T *localDW);
extern void raspberrypi_m_MATLABSystem4(real_T rtu_0,
  B_MATLABSystem4_raspberrypi_m_T *localB, DW_MATLABSystem4_raspberrypi__T
  *localDW);

#endif
/* RTW_HEADER_raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis_private_h_ */
