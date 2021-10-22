/*
 * raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis_data.c
 *
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * Code generation for model "raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis".
 *
 * Model version              : 2.399
 * Simulink Coder version : 9.5 (R2021a) 14-Nov-2020
 * C source code generated on : Fri Oct 22 11:55:15 2021
 *
 * Target selection: ert.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis.h"
#include "raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis_private.h"

/* Block parameters (default storage) */
P_raspberrypi_multicore_MPCte_T raspberrypi_multicore_MPCtest_P = {
  /* Variable: T_gait
   * Referenced by:
   *   '<Root>/Constant23'
   *   '<Root>/MATLAB System17'
   */
  0.6,

  /* Variable: Ts
   * Referenced by:
   *   '<Root>/MATLAB System21'
   *   '<Root>/MATLAB System7'
   */
  0.025,

  /* Variable: Ts_DynSim
   * Referenced by:
   *   '<Root>/MATLAB System'
   *   '<Root>/MATLAB System16'
   *   '<Root>/MATLAB System17'
   *   '<Root>/MATLAB System19'
   *   '<S15>/Constant11'
   *   '<S15>/MATLAB System3'
   */
  0.005,

  /* Expression: diag( [ 0.0078, 0.0275, 0.0328 ] )
   * Referenced by: '<Root>/MATLAB System10'
   */
  { 0.0078, 0.0, 0.0, 0.0, 0.0275, 0.0, 0.0, 0.0, 0.0328 },

  /* Expression: 3
   * Referenced by: '<Root>/MATLAB System10'
   */
  3.0,

  /* Expression: 0.19
   * Referenced by: '<Root>/MATLAB System10'
   */
  0.19,

  /* Expression: 0.097
   * Referenced by: '<Root>/MATLAB System10'
   */
  0.097,

  /* Expression: 0.2108
   * Referenced by: '<Root>/MATLAB System10'
   */
  0.2108,

  /* Expression: 0.037
   * Referenced by: '<Root>/MATLAB System10'
   */
  0.037,

  /* Expression: [ 0; 0; 0 ]
   * Referenced by: '<Root>/MATLAB System14'
   */
  { 0.0, 0.0, 0.0 },

  /* Expression: [ 0; 0; 0 ]
   * Referenced by: '<Root>/MATLAB System14'
   */
  { 0.0, 0.0, 0.0 },

  /* Expression: [ 0; 0; 0 ]
   * Referenced by: '<Root>/MATLAB System14'
   */
  { 0.0, 0.0, 0.0 },

  /* Expression: [ 0; 0; 0 ]
   * Referenced by: '<Root>/MATLAB System14'
   */
  { 0.0, 0.0, 0.0 },

  /* Expression: 0.097
   * Referenced by: '<Root>/MATLAB System15'
   */
  0.097,

  /* Expression: 0.2108
   * Referenced by: '<Root>/MATLAB System15'
   */
  0.2108,

  /* Expression: 0.037
   * Referenced by: '<Root>/MATLAB System15'
   */
  0.037,

  /* Expression: 0.19
   * Referenced by: '<Root>/MATLAB System15'
   */
  0.19,

  /* Expression: 15
   * Referenced by: '<Root>/MATLAB System16'
   */
  15.0,

  /* Expression: [ 0; 0; 0.19 ]
   * Referenced by: '<Root>/MATLAB System17'
   */
  { 0.0, 0.0, 0.19 },

  /* Expression: [ 0; 0; 0 ]
   * Referenced by: '<Root>/MATLAB System17'
   */
  { 0.0, 0.0, 0.0 },

  /* Expression: [ 0; 0; 0 ]
   * Referenced by: '<Root>/MATLAB System17'
   */
  { 0.0, 0.0, 0.0 },

  /* Expression: [ 0; 0; 0 ]
   * Referenced by: '<Root>/MATLAB System17'
   */
  { 0.0, 0.0, 0.0 },

  /* Expression: 0.097
   * Referenced by: '<Root>/MATLAB System17'
   */
  0.097,

  /* Expression: 0.2108
   * Referenced by: '<Root>/MATLAB System17'
   */
  0.2108,

  /* Expression: 0.037
   * Referenced by: '<Root>/MATLAB System17'
   */
  0.037,

  /* Expression: 3
   * Referenced by: '<Root>/MATLAB System17'
   */
  3.0,

  /* Expression: 0.9
   * Referenced by: '<Root>/MATLAB System17'
   */
  0.9,

  /* Expression: 0.9
   * Referenced by: '<Root>/MATLAB System17'
   */
  0.9,

  /* Expression: 0
   * Referenced by: '<Root>/MATLAB System17'
   */
  0.0,

  /* Expression: 0.04
   * Referenced by: '<Root>/MATLAB System17'
   */
  0.04,

  /* Expression: 0
   * Referenced by: '<Root>/MATLAB System17'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/MATLAB System17'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/MATLAB System17'
   */
  0.0,

  /* Expression: 0.097
   * Referenced by: '<Root>/MATLAB System18'
   */
  0.097,

  /* Expression: 0.2108
   * Referenced by: '<Root>/MATLAB System18'
   */
  0.2108,

  /* Expression: 0.037
   * Referenced by: '<Root>/MATLAB System18'
   */
  0.037,

  /* Expression: 3
   * Referenced by: '<Root>/MATLAB System19'
   */
  3.0,

  /* Expression: diag( [0.0046125 0.0230942 0.0253262] )
   * Referenced by: '<Root>/MATLAB System19'
   */
  { 0.0046125, 0.0, 0.0, 0.0, 0.0230942, 0.0, 0.0, 0.0, 0.0253262 },

  /* Expression: 0.097
   * Referenced by: '<Root>/MATLAB System2'
   */
  0.097,

  /* Expression: 0.2108
   * Referenced by: '<Root>/MATLAB System2'
   */
  0.2108,

  /* Expression: 0.037
   * Referenced by: '<Root>/MATLAB System2'
   */
  0.037,

  /* Expression: 0.19
   * Referenced by: '<Root>/MATLAB System2'
   */
  0.19,

  /* Expression: 3
   * Referenced by: '<Root>/MATLAB System2'
   */
  3.0,

  /* Expression: 20
   * Referenced by: '<Root>/MATLAB System2'
   */
  20.0,

  /* Expression: 20
   * Referenced by: '<Root>/MATLAB System2'
   */
  20.0,

  /* Expression: 20
   * Referenced by: '<Root>/MATLAB System2'
   */
  20.0,

  /* Expression: diag( [0.0046125 0.0230942 0.0253262] )
   * Referenced by: '<Root>/MATLAB System21'
   */
  { 0.0046125, 0.0, 0.0, 0.0, 0.0230942, 0.0, 0.0, 0.0, 0.0253262 },

  /* Expression: 3
   * Referenced by: '<Root>/MATLAB System21'
   */
  3.0,

  /* Expression: 0.19
   * Referenced by: '<Root>/MATLAB System21'
   */
  0.19,

  /* Expression: 0.2
   * Referenced by: '<Root>/MATLAB System6'
   */
  0.2,

  /* Expression: 0.2
   * Referenced by: '<Root>/MATLAB System6'
   */
  0.2,

  /* Expression: 0.8
   * Referenced by: '<Root>/MATLAB System6'
   */
  0.8,

  /* Expression: [ 0; 0; 0.19 ]
   * Referenced by: '<Root>/MATLAB System7'
   */
  { 0.0, 0.0, 0.19 },

  /* Expression: [ 0; 0; 0 ]
   * Referenced by: '<Root>/MATLAB System7'
   */
  { 0.0, 0.0, 0.0 },

  /* Expression: [ 0; 0; 0 ]
   * Referenced by: '<Root>/MATLAB System7'
   */
  { 0.0, 0.0, 0.0 },

  /* Expression: [ 0; 0; 0 ]
   * Referenced by: '<Root>/MATLAB System7'
   */
  { 0.0, 0.0, 0.0 },

  /* Expression: 0.19
   * Referenced by: '<Root>/MATLAB System7'
   */
  0.19,

  /* Expression: [0;0;0]
   * Referenced by: '<Root>/MATLAB System7'
   */
  { 0.0, 0.0, 0.0 },

  /* Expression: 0
   * Referenced by: '<S14>/Constant12'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/Constant6'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<Root>/Constant7'
   */
  1.0,

  /* Expression: sampleTime
   * Referenced by: '<S52>/Digital Read'
   */
  0.005,

  /* Expression: sampleTime
   * Referenced by: '<S53>/Digital Read'
   */
  0.005,

  /* Expression: sampleTime
   * Referenced by: '<S54>/Digital Read'
   */
  0.005,

  /* Expression: sampleTime
   * Referenced by: '<S55>/Digital Read'
   */
  0.005,

  /* Expression: diag([1e-7,5e-8,1e-8,1e-6,1e-6,1e-6,1e-5,1e-5,1e-5])
   * Referenced by: '<S15>/Constant1'
   */
  { 1.0E-7, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0E-8, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0E-8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0E-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0E-6, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0E-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0E-5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0E-5, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0E-5 },

  /* Expression: [11,12,13]
   * Referenced by: '<S14>/Constant'
   */
  { 11.0, 12.0, 13.0 },

  /* Expression: [21,22,23]
   * Referenced by: '<S14>/Constant13'
   */
  { 21.0, 22.0, 23.0 },

  /* Expression: [31,32,33]
   * Referenced by: '<S14>/Constant14'
   */
  { 31.0, 32.0, 33.0 },

  /* Expression: [41,42,43]
   * Referenced by: '<S14>/Constant15'
   */
  { 41.0, 42.0, 43.0 },

  /* Expression: 5
   * Referenced by: '<Root>/Switch'
   */
  5.0,

  /* Expression: 2*pi
   * Referenced by: '<Root>/Constant22'
   */
  6.2831853071795862,

  /* Expression: zeros(67,1)
   * Referenced by: '<Root>/Rate Transition'
   */
  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

  /* Expression: [1,1,1,1]
   * Referenced by: '<Root>/Unit Delay3'
   */
  { 1.0, 1.0, 1.0, 1.0 },

  /* Expression: [0;0;0.19;zeros(9,1)]
   * Referenced by: '<Root>/Unit Delay6'
   */
  { 0.0, 0.0, 0.19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

  /* Expression: [0.1075,0.1075,-0.1075,-0.1075;0.0750,-0.0750,0.0750,-0.0750;0,0,0,0];
   * Referenced by: '<Root>/Unit Delay5'
   */
  { 0.1075, 0.075, 0.0, 0.1075, -0.075, 0.0, -0.1075, 0.075, 0.0, -0.1075,
    -0.075, 0.0 },

  /* Expression: 1
   * Referenced by: '<Root>/Constant8'
   */
  1.0,

  /* Expression: [0;0;0;0]
   * Referenced by: '<Root>/Unit Delay4'
   */
  { 0.0, 0.0, 0.0, 0.0 },

  /* Expression: zeros(12,1)
   * Referenced by: '<Root>/Unit Delay'
   */
  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

  /* Expression: 3
   * Referenced by: '<S14>/Constant10'
   */
  3.0,

  /* Expression: 5
   * Referenced by: '<S14>/Constant11'
   */
  5.0,

  /* Expression: 1
   * Referenced by: '<Root>/Constant4'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<S14>/Memory1'
   */
  1.0,

  /* Expression: 0.5
   * Referenced by: '<S14>/Switch1'
   */
  0.5,

  /* Expression: zeros(12,1)
   * Referenced by: '<S15>/Unit Delay'
   */
  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

  /* Expression: [0,0,0]
   * Referenced by: '<Root>/Constant5'
   */
  { 0.0, 0.0, 0.0 },

  /* Expression: 9.8
   * Referenced by: '<Root>/Constant10'
   */
  9.8,

  /* Expression: 1
   * Referenced by: '<Root>/Constant9'
   */
  1.0,

  /* Expression: 66
   * Referenced by: '<Root>/Constant25'
   */
  66.0,

  /* Expression: zeros(61,1)
   * Referenced by: '<Root>/Rate Transition1'
   */
  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0 },

  /* Expression: [0;0;0.19;zeros(9,1)]
   * Referenced by: '<Root>/Unit Delay7'
   */
  { 0.0, 0.0, 0.19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

  /* Expression: 0.5
   * Referenced by: '<Root>/Switch1'
   */
  0.5,

  /* Expression: Yscale(:,ones(1,max(nCC,1)))'
   * Referenced by: '<S18>/ymin_scale1'
   */
  { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 },

  /* Expression: MVscale(:,ones(1,max(nCC,1)))'
   * Referenced by: '<S18>/umin_scale4'
   */
  { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 },

  /* Expression: MDscale(:,ones(1,max(nCC,1)))'
   * Referenced by: '<S18>/ymin_scale2'
   */
  { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 },

  /* Expression: 9.8
   * Referenced by: '<Root>/Constant2'
   */
  9.8,

  /* Expression: lastu+uoff
   * Referenced by: '<S18>/last_mv'
   */
  { 0.0, 0.0, 7.3500000000000005, 0.0, 0.0, 7.3500000000000005, 0.0, 0.0,
    7.3500000000000005, 0.0, 0.0, 7.3500000000000005 },

  /* Expression: zeros(nym,1)
   * Referenced by: '<S18>/ym_zero'
   */
  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

  /* Expression: zeros(12,1)
   * Referenced by: '<S1>/umin_zero'
   */
  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

  /* Expression: zeros(12,1)
   * Referenced by: '<S1>/umax_zero'
   */
  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

  /* Expression: zeros(13,1)
   * Referenced by: '<S1>/ymin_zero'
   */
  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

  /* Expression: zeros(13,1)
   * Referenced by: '<S1>/ymax_zero'
   */
  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

  /* Expression: [0;0;7.5;0;0;7.5;0;0;7.5;0;0;7.5];
   * Referenced by: '<Root>/Unit Delay2'
   */
  { 0.0, 0.0, 7.5, 0.0, 0.0, 7.5, 0.0, 0.0, 7.5, 0.0, 0.0, 7.5 },

  /* Expression: RMVscale
   * Referenced by: '<S18>/ext.mv_scale'
   */
  { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 },

  /* Expression: zeros(12,1)
   * Referenced by: '<S1>/mv.target_zero'
   */
  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

  /* Expression: RMVscale
   * Referenced by: '<S18>/uref_scale'
   */
  { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 },

  /* Expression: zeros(13,1)
   * Referenced by: '<S1>/y.wt_zero'
   */
  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

  /* Expression: zeros(12,1)
   * Referenced by: '<S1>/u.wt_zero'
   */
  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

  /* Expression: zeros(12,1)
   * Referenced by: '<S1>/du.wt_zero'
   */
  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

  /* Expression: zeros(1,1)
   * Referenced by: '<S1>/ecr.wt_zero'
   */
  0.0,

  /* Expression: lastPcov
   * Referenced by: '<S18>/LastPcov'
   */
  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0 },

  /* Expression: MVscale
   * Referenced by: '<S18>/u_scale'
   */
  { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 },

  /* Expression: 1
   * Referenced by: '<Root>/Constant1'
   */
  1.0,

  /* Expression: 0
   * Referenced by: '<Root>/Unit Delay1'
   */
  0.0,

  /* Expression: 66
   * Referenced by: '<Root>/Constant3'
   */
  66.0,

  /* Expression: Ndis
   * Referenced by: '<S46>/FixedHorizonOptimizer'
   */
  0,

  /* Expression: iA
   * Referenced by: '<S18>/Memory'
   */
  { false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false }
};
