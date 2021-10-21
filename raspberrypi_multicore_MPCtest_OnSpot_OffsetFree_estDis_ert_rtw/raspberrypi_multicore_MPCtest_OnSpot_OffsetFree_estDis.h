/*
 * raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis.h
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

#ifndef RTW_HEADER_raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis_h_
#define RTW_HEADER_raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis_h_
#include <stddef.h>
#include <math.h>
#include <string.h>
#include <float.h>
#ifndef raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis_COMMON_INCLUDES_
#define raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis_COMMON_INCLUDES_
#include <stdio.h>
#include <string.h>
#include "rtwtypes.h"
#include "rtw_extmode.h"
#include "sysran_types.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "ext_work.h"
#include "joystick_raspi.h"
#include "MW_Raspi_SPI_Helper.h"
#include "MW_SPI.h"
#include "MW_gpio.h"
#endif
   /* raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis_COMMON_INCLUDES_ */

#include "raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis_types.h"

/* Shared type includes */
#include "multiword_types.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"
#include "rt_defines.h"
#include "rtw_linux.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetFinalTime
#define rtmGetFinalTime(rtm)           ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetRTWExtModeInfo
#define rtmGetRTWExtModeInfo(rtm)      ((rtm)->extModeInfo)
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmStepTask
#define rtmStepTask(rtm, idx)          ((rtm)->Timing.TaskCounters.TID[(idx)] == 0)
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   ((rtm)->Timing.taskTime0)
#endif

#ifndef rtmGetTFinal
#define rtmGetTFinal(rtm)              ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                (&(rtm)->Timing.taskTime0)
#endif

#ifndef rtmTaskCounter
#define rtmTaskCounter(rtm, idx)       ((rtm)->Timing.TaskCounters.TID[(idx)])
#endif

#define raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis_M (raspberrypi_multicore_MPCtes_M)

/* Block signals for system '<Root>/MATLAB System4' */
typedef struct {
  real_T MATLABSystem4;                /* '<Root>/MATLAB System4' */
} B_MATLABSystem4_raspberrypi_m_T;

/* Block states (default storage) for system '<Root>/MATLAB System4' */
typedef struct {
  edgeDetector_raspberrypi_mult_T obj; /* '<Root>/MATLAB System4' */
  boolean_T objisempty;                /* '<Root>/MATLAB System4' */
} DW_MATLABSystem4_raspberrypi__T;

/* Block signals (default storage) */
typedef struct {
  real_T ab[22932];
  real_T MvCC[20384];
  real_T Fp1[20384];
  real_T dv[20384];
  real_T Fp1_m[20384];
  real_T Ep1[18816];
  real_T Ep1_c[18816];
  real_T Dv[8281];
  real_T c_Hv[7098];
  real_T db[6300];
  real_T Su[5616];
  real_T MuCC[5376];
  real_T Ep1_k[5376];
  real_T Fp1_c[5376];
  real_T dv1[5184];
  real_T x[3276];
  real_T y[3024];
  real_T MxCC[2912];
  real_T Mu1CC[2688];
  real_T Ep1_b[2688];
  real_T Fp1_p[2688];
  real_T c_Kv[2184];
  real_T dv2[2184];
  real_T b_Jm[2016];
  real_T I2[2016];
  real_T c_SuJm[1872];
  real_T c_Kr[1872];
  real_T c_Kut[1728];
  real_T dv3[1728];
  real_T dv4[1728];
  real_T dv5[1197];
  real_T Bv[1183];
  real_T Dv_c[1183];
  real_T CA[1183];
  real_T b_C[1183];
  real_T Bu[1092];
  real_T dv6[1092];
  real_T c_Sx[1014];
  real_T b_B[962];
  real_T b_B_f[962];
  real_T c_Su1[936];
  real_T Sum[936];
  int8_T I2_g[7056];
  real_T CovMat[676];
  real_T b_varargout_2[625];
  real_T bb[625];
  real_T c_Linv[625];
  real_T d_Linv[625];
  real_T c_A[625];
  real_T RLinv[625];
  real_T b_D[625];
  real_T b_H[625];
  real_T U[625];
  real_T TL[625];
  real_T QQ[625];
  real_T RR[625];
  real_T Anew[625];
  real_T Ppre[625];
  real_T Anew_g[625];
  real_T Anew_m[625];
  real_T obj[625];
  real_T b[576];
  real_T b_SuJm[576];
  real_T b_Jm_n[576];
  real_T h[494];
  real_T l[494];
  real_T dv7[416];
  real_T S[416];
  real_T Eb2[384];
  real_T dv8[384];
  real_T K[325];
  real_T dv9[325];
  real_T Ppre_p[325];
  real_T b_varargout_2_l[312];
  real_T dv10[312];
  real_T c_Kx[312];
  real_T B[300];
  real_T c_Ku1[288];
  real_T b_Su1[288];
  real_T b_I1[288];
  real_T b_Mlim[252];
  real_T b_Mlim_j[252];
  real_T b_Mu1[252];
  real_T b_Mlim_d[252];
  real_T a__1[252];
  real_T cTol[252];
  real_T dv11[252];
  real_T E[224];
  real_T b_A[169];
  real_T b_C_g[169];
  real_T d[169];
  real_T f[169];
  real_T Qk[169];
  real_T Rk[169];
  real_T Nk[169];
  real_T b_varargout_1[169];
  real_T dv12[169];
  real_T A[169];
  real_T c_A_l[169];
  real_T CA_d[169];
  real_T b_C_d[169];
  real_T B_l[156];
  real_T Sum_o[156];
  real_T B_b[156];
  real_T b_I[144];
  int8_T b_I1_n[1008];
  real_T Eb2_b[96];
  real_T Eb3[96];
  real_T Eb1[96];
  real_T vseq[91];
  real_T yseq[91];                     /* '<S46>/FixedHorizonOptimizer' */
  real_T xseq[91];                     /* '<S46>/FixedHorizonOptimizer' */
  real_T dv13[91];
  real_T useq[84];                     /* '<S46>/FixedHorizonOptimizer' */
  real_T b_varargout_2_ln[81];
  real_T obj_h[81];
  real_T obj_b[81];
  real_T obj_d[81];
  real_T obj_e[81];
  int8_T c_I[625];
  int8_T b_b[625];
  real_T b_varargout_3[78];
  real_T b_varargout_1_j[78];
  real_T b_utarget[72];
  real_T dv14[72];
  real_T b_I1_f[72];
  real_T dv15[72];
  real_T uopt_dim[72];
  real_T b_a[72];
  real_T RateTransition[67];           /* '<Root>/Rate Transition' */
  int16_T iAnew[252];
  int16_T iC[252];
  real_T RateTransition1[61];          /* '<Root>/Rate Transition1' */
  real_T K_j[54];
  real_T obj_j[54];
  real_T P[54];
  real_T Opt[50];
  real_T Rhs[50];
  real_T c_I_o[49];
  real_T Eb4[48];
  real_T Eb5[48];
  real_T obj_n[36];
  real_T c_A_i[36];
  real_T dv16[32];
  real_T E_o[32];
  real_T F[32];
  real_T E_n[32];
  boolean_T bv[252];
  real_T TmpSignalConversionAtToFi_p[26];
  real_T zopt[25];
  real_T f_m[25];
  real_T r[25];
  real_T z[25];
  real_T b_Ac[25];
  real_T tau[25];
  real_T work[25];
  real_T b_varargout_1_c[25];
  real_T Xpre[25];
  real_T Anew_md[25];
  real_T B_m[25];
  real_T varargin_1[25];
  real_T work_j[25];
  real_T b_varargout_6[24];
  real_T dv17[24];
  real_T U_h[24];
  int8_T b_I_c[144];
  real_T TmpSignalConversionAtToFile[16];
  int16_T b_varargout_3_c[64];
  int16_T b_varargout_2_p[64];
  real_T b_yoff[13];
  real_T b_myoff[13];
  real_T b_varargout_8[13];
  real_T b_varargout_7[13];
  real_T X_FB[13];
  real_T xk1[13];                      /* '<S46>/FixedHorizonOptimizer' */
  real_T xest[13];                     /* '<S46>/FixedHorizonOptimizer' */
  real_T TmpSignalConversionAtToFi_n[13];
  real_T dv18[13];
  real_T dv19[13];
  real_T b_varargout_4[13];
  real_T tmp[13];
  real_T Anew_p[13];
  real_T Bnew[13];
  real_T xQP[13];
  real_T vk[13];
  real_T b_C_a[13];
  real_T Dv_e[13];
  real_T ref[13];
  real_T dv20[13];
  real_T b_uoff[12];
  real_T b_voff[12];
  real_T b_varargout_17[12];
  real_T b_varargout_6_a[12];
  real_T dv21[12];
  real_T dv22[12];
  real_T old_u[12];
  real_T u[12];
  real_T W[12];
  real_T temp[12];
  real_T desAllL[12];
  real_T pL_sw[12];
  real_T PendAllLocal_st_tmp[12];
  real_T vArray[12];
  real_T vBM[12];
  real_T MATLABSystem10_o1[12];        /* '<Root>/MATLAB System10' */
  real_T pArray_L_Adm[12];             /* '<Root>/MATLAB System2' */
  real_T pArray_B[12];                 /* '<Root>/Leg Cor 2 Body Cor' */
  real_T pArray_float_tmp[12];
  real_T umax[12];
  real_T umin[12];
  real_T dumin[12];
  real_T dumax[12];
  real_T PendAlltmp[12];
  int8_T b_I_a[81];
  real_T b_varargout_9[9];
  real_T V[9];
  real_T U_i[9];
  real_T MRz[9];
  real_T R[9];
  real_T R_tmp[9];
  real_T Rz[9];
  real_T Inow[9];
  real_T Rz_l[9];
  real_T Xpre_o[9];
  real_T obj_o[9];
  real_T Rz_i[9];
  real_T Iinv[9];
  real_T Rsur[9];
  real_T dv23[9];
  real_T headX_tmp[9];
  real_T headX_tmp_f[9];
  real_T b_A_i[9];
  real_T Vf[9];
  real_T dv24[9];
  real_T pP_tmp[9];
  real_T pP_tmp_f[9];
  real_T Gb1[8];
  real_T TmpSignalConversionAtToFi_h[8];
  uint8_T rdDataRaw[60];
  uint8_T cmd[60];                     /* '<S14>/MATLAB Function' */
  int32_T ipiv[13];
  int32_T b_index_data[12];
  int32_T ii_data[12];
  real_T b_varargout_1_g[6];
  real_T y_c[6];
  int8_T UnknownIn[37];
  real_T b_varargout_5[4];
  real_T pxNew[4];
  real_T pyNew[4];
  real_T b_varargout_10[3];
  real_T b_varargout_9_o[3];
  real_T b_varargout_8_l[3];
  real_T b_varargout_7_m[3];
  real_T a[3];
  real_T v1[3];
  real_T deltaP2[3];
  real_T deltaP3[3];
  real_T deltaP4[3];
  real_T Angle2new[3];
  real_T Angle3new[3];
  real_T Angle4new[3];
  real_T surP[3];                      /* '<Root>/MATLAB Function2' */
  real_T rtb_pArray_L_Adm_m[3];
  real_T pL_sw_c[3];
  real_T pW[3];
  real_T pW_f[3];
  real_T pW_p[3];
  real_T pW_e[3];
  real_T PendAll[3];
  real_T PendAll_o[3];
  real_T PendAll_h[3];
  real_T PendAll_l[3];
  real_T surVN[3];
  real_T headX[3];
  real_T desr[3];
  real_T b_s[3];
  real_T e[3];
  real_T work_h[3];
  real_T pP_tmp_m[3];
  real_T pP_tmp_mc[3];
  boolean_T x_h[12];
  int8_T Unow[12];
  int8_T b_I_cs[9];
  int8_T b_I_k[9];
  int8_T b_I_p[9];
  uint32_T PinNameLoc;
  MW_SPI_Mode_type ClockModeValue;
  MW_SPI_FirstBitTransfer_Type MsbFirstTransferLoc;
  real_T err;                          /* '<S14>/MATLAB Function1' */
  real_T MATLABSystem8;                /* '<Root>/MATLAB System8' */
  real_T MATLABSystem11_o2;            /* '<Root>/MATLAB System11' */
  real_T oscEN;                        /* '<Root>/Chart' */
  real_T mpcSTOP;                      /* '<Root>/Chart' */
  real_T b_y;
  real_T scale;
  real_T absxk;
  real_T t;
  real_T b_varargout_16;
  real_T b_varargout_15;
  real_T b_varargout_14;
  real_T b_varargout_13;
  real_T b_varargout_12;
  real_T surVhead;
  real_T surVhead_p;
  real_T b_varargout_1_idx_0;
  real_T b_varargout_3_idx_0;
  real_T b_varargout_2_idx_0;
  real_T b_varargout_4_idx_0;
  real_T b_varargout_1_idx_1;
  real_T b_varargout_3_idx_1;
  real_T b_varargout_2_idx_1;
  real_T b_varargout_4_idx_1;
  real_T b_varargout_1_idx_2;
  real_T b_varargout_3_idx_2;
  real_T b_varargout_2_idx_2;
  real_T b_varargout_4_idx_2;
  real_T surVhead_idx_0;
  real_T surVhead_idx_1;
  real_T surVhead_idx_2;
  real_T BadH;
  real_T b_Kx;
  real_T b_Kr;
  real_T b_Ku1;
  real_T b_Kv;
  real_T b_Kut;
  real_T rMin;
  real_T Xnorm0;
  real_T cMin;
  real_T cVal;
  real_T zTa;
  real_T b_Linv;
  real_T b_atmp;
  real_T beta1;
  real_T temp_p;
  real_T absx;
  real_T s;
  real_T b_varargout_5_a;
  real_T b_varargout_4_j;
  real_T q;
  real_T t4;
  real_T t6;
  real_T t7;
  real_T t8;
  real_T t9;
  real_T t10;
  real_T t11;
  real_T t4_e;
  real_T t6_o;
  real_T t7_b;
  real_T t8_a;
  real_T t9_g;
  real_T t11_e;
  real_T t12;
  real_T t6_f;
  real_T t7_h;
  real_T t8_e;
  real_T t9_c;
  real_T t10_a;
  real_T t11_d;
  real_T t6_a;
  real_T t7_p;
  real_T t8_m;
  real_T t9_o;
  real_T t11_n;
  real_T t12_l;
  real_T t14;
  real_T t16;
  real_T t5;
  real_T t10_p;
  real_T t14_p;
  real_T t16_f;
  real_T t5_i;
  real_T t10_o;
  real_T t14_k;
  real_T s_i;
  real_T Switch1;                      /* '<S14>/Switch1' */
  real_T rtb_headG_tmp;
  real_T rtb_headG_tmp_o;
  real_T rtb_headG_tmp_m;
  real_T rtb_headG_tmp_c;
  real_T b_varargout_10_idx_1;
  real_T b_varargout_9_idx_1;
  real_T b_varargout_9_idx_0;
  real_T b_varargout_10_idx_0;
  real_T v2_idx_2;
  real_T v2_idx_1;
  real_T Angle3new_f;
  real_T rtb_pArray_L_Adm_tmp;
  real_T rtb_pArray_L_Adm_tmp_tmp;
  real_T rtb_headG_tmp_tmp;
  real_T rtb_headG_tmp_tmp_h;
  real_T rtb_pArray_L_Adm_tmp_tmp_m;
  real_T MATLABSystem3_o1;             /* '<Root>/MATLAB System3' */
  real_T MATLABSystem3_o2_f;           /* '<Root>/MATLAB System3' */
  real_T absx11;
  real_T absx21;
  real_T absx31;
  real_T temp_a;
  real_T smax;
  real_T s_k;
  real_T d_p;
  real_T smax_b;
  real_T s_c;
  real_T absx11_n;
  real_T absx21_i;
  real_T absx31_m;
  real_T t2;
  real_T t3;
  real_T t28;
  real_T t31;
  real_T t92;
  real_T t117;
  real_T t118;
  real_T t119;
  real_T t120;
  real_T t121;
  real_T t122;
  real_T t123;
  real_T t124;
  real_T t125;
  real_T t126;
  real_T t127;
  real_T t128;
  real_T t132;
  real_T t133;
  real_T t134;
  real_T t135;
  real_T t136;
  real_T t137;
  real_T t138;
  real_T t139;
  real_T t140;
  real_T t141;
  real_T t142;
  real_T t143;
  real_T t144;
  real_T t145;
  real_T t146;
  real_T t147;
  real_T t148;
  real_T t149;
  real_T t150;
  real_T t151;
  real_T t152;
  real_T t153;
  real_T t154;
  real_T t155;
  real_T t131;
  real_T t156;
  real_T t134_tmp;
  real_T t134_tmp_j;
  real_T t140_tmp;
  real_T t140_tmp_e;
  real_T t133_tmp;
  real_T t133_tmp_m;
  real_T t138_tmp;
  real_T t138_tmp_m;
  real_T t132_tmp;
  real_T t132_tmp_j;
  real_T t136_tmp;
  real_T t136_tmp_f;
  real_T t139_tmp;
  real_T t139_tmp_a;
  real_T t146_tmp;
  real_T t146_tmp_g;
  real_T t137_tmp;
  real_T t137_tmp_n;
  real_T t144_tmp;
  real_T t144_tmp_d;
  real_T t135_tmp;
  real_T t135_tmp_n;
  real_T t142_tmp;
  real_T t142_tmp_c;
  real_T t145_tmp;
  real_T t145_tmp_f;
  real_T t152_tmp;
  real_T t152_tmp_p;
  real_T t143_tmp;
  real_T t143_tmp_p;
  real_T t150_tmp;
  real_T t150_tmp_n;
  real_T t141_tmp;
  real_T t141_tmp_k;
  real_T t148_tmp;
  real_T t148_tmp_n;
  real_T t151_tmp;
  real_T t151_tmp_o;
  real_T t155_tmp;
  real_T t155_tmp_g;
  real_T t149_tmp;
  real_T t149_tmp_c;
  real_T t154_tmp;
  real_T normH;
  real_T s_cj;
  real_T t2_m;
  real_T t3_j;
  real_T t28_k;
  real_T t31_m;
  real_T t92_p;
  real_T t117_d;
  real_T t118_g;
  real_T t119_c;
  real_T t120_c;
  real_T t121_i;
  real_T t122_d;
  real_T t123_g;
  real_T t124_l;
  real_T t125_f;
  real_T t126_d;
  real_T t127_j;
  real_T t128_i;
  real_T t132_h;
  real_T t133_n;
  real_T t134_o;
  real_T t135_c;
  real_T t136_b;
  real_T t137_e;
  real_T t138_d;
  real_T t139_i;
  real_T t140_g;
  real_T t141_n;
  real_T t142_l;
  real_T t143_c;
  real_T t144_n;
  real_T t145_p;
  real_T t146_d;
  real_T t147_o;
  real_T t148_j;
  real_T t149_c;
  real_T t150_h;
  real_T t151_d;
  real_T t152_c;
  real_T t153_p;
  real_T t154_p;
  real_T t155_a;
  real_T t131_o;
  real_T t156_j;
  real_T t134_tmp_p;
  real_T t134_tmp_o;
  real_T t140_tmp_l;
  real_T t140_tmp_k;
  real_T t133_tmp_j;
  real_T t133_tmp_f;
  real_T t138_tmp_c;
  real_T t138_tmp_f;
  real_T t132_tmp_n;
  real_T t132_tmp_i;
  real_T t136_tmp_l;
  real_T t136_tmp_i;
  real_T t139_tmp_k;
  real_T t139_tmp_f;
  real_T t146_tmp_a;
  real_T t146_tmp_d;
  real_T t137_tmp_e;
  real_T t137_tmp_eh;
  real_T t144_tmp_b;
  real_T t144_tmp_a;
  real_T t135_tmp_i;
  real_T t135_tmp_f;
  real_T t142_tmp_j;
  real_T t142_tmp_o;
  real_T t145_tmp_fr;
  real_T t145_tmp_o;
  real_T t152_tmp_l;
  real_T t152_tmp_lu;
  real_T t143_tmp_g;
  real_T t143_tmp_d;
  real_T t150_tmp_d;
  real_T t150_tmp_j;
  real_T t141_tmp_f;
  real_T t141_tmp_j;
  real_T t148_tmp_h;
  real_T t148_tmp_c;
  real_T t151_tmp_n;
  real_T t151_tmp_k;
  real_T t155_tmp_a;
  real_T t155_tmp_f;
  real_T desH;
  real_T sy;
  real_T headX_tmp_j;
  real_T headX_tmp_k;
  real_T nrm;
  real_T rt;
  real_T ztest0;
  real_T smm1;
  real_T emm1;
  real_T sqds;
  real_T shift;
  real_T roe;
  real_T absa;
  real_T absb;
  real_T xW;
  real_T yW;
  real_T d1;
  real_T d2;
  real_T c;
  real_T alpha;
  real_T beta;
  real_T RP;
  real_T pRy;
  real_T pRz;
  real_T errFlag;
  real_T xout;
  real_T yout;
  real_T pC_tmp;
  real_T pC;
  real_T pB_idx_1;
  real_T pP_idx_0;
  real_T pP_idx_1;
  real_T D;
  real_T E_b;
  real_T F_h;
  real_T temp_e;
  real_T yo1;
  real_T a_tmp;
  real_T RP_h;
  real_T b_pRy;
  real_T b_pRz;
  real_T b_errFlag;
  real_T b_xout;
  real_T b_yout;
  real_T pC_tmp_k;
  real_T pC_j;
  real_T pB_idx_1_o;
  real_T pP_idx_0_c;
  real_T pP_idx_1_h;
  real_T D_i;
  real_T E_p;
  real_T F_f;
  real_T temp_ew;
  real_T yo1_n;
  real_T scale_h;
  real_T scale_hp;
  int8_T b_ipiv[6];
  int32_T b_k;
  int32_T i;
  int32_T Eb2_tmp;
  int32_T Eb2_tmp_f;
  int32_T k;
  int32_T e_i;
  int32_T i_i;
  int32_T i_f;
  int32_T i_c;
  int32_T i1;
  int32_T b_k_n;
  int32_T c_k;
  int32_T ct;
  int32_T i_h;
  int32_T i2;
  int32_T i_k;
  int32_T b_i;
  int32_T c_i;
  int32_T e_i_h;
  int32_T iC_b;
  int32_T j;
  int32_T i_o;
  int32_T ii;
  int32_T knt;
  int32_T b_k_nq;
  int32_T c_k_m;
  int32_T i_ku;
  int32_T mmi_tmp;
  int32_T vcol;
  int32_T ar;
  int32_T ia;
  int32_T b_ic;
  int32_T c_j;
  int32_T estEN;                       /* '<Root>/Chart' */
  int32_T i_hc;
  int32_T p1;
  int32_T p2;
  int32_T p3;
  int32_T itmp;
  int32_T b_info;
  int32_T jBcol;
  int32_T jAcol;
  int32_T kBcol;
  int32_T b_j;
  int32_T b_f;
  int32_T i_d;
  int32_T i3;
  int32_T itau;
  int32_T iaii;
  int32_T b_l;
  int32_T b_k_k;
  int32_T i_ic;
  B_MATLABSystem4_raspberrypi_m_T MATLABSystem9;/* '<Root>/MATLAB System4' */
  B_MATLABSystem4_raspberrypi_m_T MATLABSystem4;/* '<Root>/MATLAB System4' */
} B_raspberrypi_multicore_MPCte_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  KalmanFilter_DIY_MPCDis_raspb_T obj; /* '<Root>/MATLAB System20' */
  IMU_regulation_raspberrypi_mu_T obj_c;/* '<Root>/MATLAB System1' */
  KalmanFilter_DIY_Offrm_raspbe_T obj_ch;/* '<S15>/MATLAB System3' */
  LegSequence_RT_v3_raspberrypi_T obj_l;/* '<Root>/MATLAB System17' */
  refTrajectory_v4_raspberrypi__T obj_cm;/* '<Root>/MATLAB System7' */
  estX_byJoint_raspberrypi_mult_T obj_b;/* '<S15>/MATLAB System' */
  UDP_decoder_raspi_raspberrypi_T obj_cy;/* '<Root>/MATLAB System10' */
  message_decoder_PC_raspberryp_T obj_e;/* '<Root>/MATLAB System15' */
  IK_raspberrypi_multicore_MPCt_T obj_k;/* '<Root>/MATLAB System14' */
  groundAdaptionP1_raspberrypi__T obj_kb;/* '<Root>/MATLAB System18' */
  AdmittanceCtr_raspberrypi_mul_T obj_f;/* '<Root>/MATLAB System2' */
  LegStateIndicator_raspberrypi_T obj_lj;/* '<S15>/MATLAB System6' */
  PosAssign_vel_raspberrypi_mul_T obj_p;/* '<Root>/MATLAB System16' */
  ssModelgen_estDis_raspberrypi_T obj_g;/* '<Root>/MATLAB System21' */
  buttonDecoder_2_raspberrypi_m_T obj_a;/* '<Root>/MATLAB System3' */
  footEndPos_protect_raspberryp_T obj_b0;/* '<Root>/MATLAB System12' */
  FK_raspberrypi_multicore_MPCt_T obj_m;/* '<Root>/MATLAB System13' */
  ref_ctr_raspberrypi_multicore_T obj_as;/* '<Root>/MATLAB System6' */
  JoystickRead_raspberrypi_mult_T obj_e1;/* '<Root>/MATLAB System' */
  codertarget_linux_blocks_Digi_T obj_fv;/* '<S55>/Digital Read' */
  codertarget_linux_blocks_Digi_T obj_i;/* '<S54>/Digital Read' */
  codertarget_linux_blocks_Digi_T obj_cu;/* '<S53>/Digital Read' */
  codertarget_linux_blocks_Digi_T obj_ls;/* '<S52>/Digital Read' */
  codertarget_raspi_internal_SP_T obj_h;/* '<S14>/SPI Master Transfer' */
  real_T UnitDelay6_DSTATE[12];        /* '<Root>/Unit Delay6' */
  real_T UnitDelay5_DSTATE[12];        /* '<Root>/Unit Delay5' */
  real_T UnitDelay_DSTATE[12];         /* '<Root>/Unit Delay' */
  real_T UnitDelay_DSTATE_l[12];       /* '<S15>/Unit Delay' */
  real_T UnitDelay7_DSTATE[12];        /* '<Root>/Unit Delay7' */
  real_T UnitDelay2_DSTATE[12];        /* '<Root>/Unit Delay2' */
  real_T UnitDelay1_DSTATE;            /* '<Root>/Unit Delay1' */
  real_T RateTransition_Buf[201];      /* '<Root>/Rate Transition' */
  real_T Memory1_PreviousInput;        /* '<S14>/Memory1' */
  real_T RateTransition1_Buf0[61];     /* '<Root>/Rate Transition1' */
  real_T RateTransition1_Buf1[61];     /* '<Root>/Rate Transition1' */
  real_T RateTransition1_Buf2[61];     /* '<Root>/Rate Transition1' */
  PhaseOscillator_SG_raspberryp_T obj_iw;/* '<Root>/MATLAB System11' */
  ESgen_raspberrypi_multicore_M_T obj_d;/* '<Root>/MATLAB System8' */
  void* RateTransition_d0_SEMAPHORE;   /* '<Root>/Rate Transition' */
  struct {
    void *FilePtr;
  } ToFile6_PWORK;                     /* '<Root>/To File6' */

  struct {
    void *FilePtr;
  } ToFile19_PWORK;                    /* '<Root>/To File19' */

  void* RateTransition1_d0_SEMAPHORE;  /* '<Root>/Rate Transition1' */
  struct {
    void *FilePtr;
  } ToFile3_PWORK;                     /* '<Root>/To File3' */

  struct {
    void *FilePtr;
  } ToFile13_PWORK;                    /* '<Root>/To File13' */

  struct {
    void *FilePtr;
  } ToFile4_PWORK;                     /* '<Root>/To File4' */

  struct {
    void *FilePtr;
  } ToFile16_PWORK;                    /* '<Root>/To File16' */

  struct {
    void *FilePtr;
  } ToFile9_PWORK;                     /* '<Root>/To File9' */

  struct {
    void *FilePtr;
  } ToFile15_PWORK;                    /* '<Root>/To File15' */

  struct {
    void *FilePtr;
  } ToFile2_PWORK;                     /* '<Root>/To File2' */

  struct {
    void *FilePtr;
  } ToFile12_PWORK;                    /* '<Root>/To File12' */

  struct {
    void *FilePtr;
  } ToFile5_PWORK;                     /* '<Root>/To File5' */

  struct {
    void *FilePtr;
  } ToFile14_PWORK;                    /* '<Root>/To File14' */

  struct {
    void *AQHandles;
  } TAQSigLogging_InsertedFor_Servo;   /* synthesized block */

  struct {
    void *AQHandles;
  } TAQSigLogging_InsertedFor_MATLA;   /* synthesized block */

  struct {
    void *AQHandles;
  } TAQSigLogging_InsertedFor_MAT_c;   /* synthesized block */

  struct {
    void *AQHandles;
  } TAQSigLogging_InsertedFor_Chart;   /* synthesized block */

  struct {
    void *AQHandles;
  } TAQSigLogging_InsertedFor_Cha_j;   /* synthesized block */

  struct {
    void *FilePtr;
  } ToFile8_PWORK;                     /* '<Root>/To File8' */

  struct {
    void *FilePtr;
  } ToFile7_PWORK;                     /* '<Root>/To File7' */

  struct {
    void *FilePtr;
  } ToFile1_PWORK;                     /* '<Root>/To File1' */

  struct {
    void *FilePtr;
  } ToFile_PWORK;                      /* '<Root>/To File' */

  int32_T sfEvent;                     /* '<Root>/Chart' */
  uint32_T is_c12_raspberrypi_multicore_MP;/* '<Root>/Chart' */
  struct {
    int_T Count;
    int_T Decimation;
  } ToFile6_IWORK;                     /* '<Root>/To File6' */

  struct {
    int_T Count;
    int_T Decimation;
  } ToFile19_IWORK;                    /* '<Root>/To File19' */

  struct {
    int_T Count;
    int_T Decimation;
  } ToFile3_IWORK;                     /* '<Root>/To File3' */

  struct {
    int_T Count;
    int_T Decimation;
  } ToFile13_IWORK;                    /* '<Root>/To File13' */

  struct {
    int_T Count;
    int_T Decimation;
  } ToFile4_IWORK;                     /* '<Root>/To File4' */

  struct {
    int_T Count;
    int_T Decimation;
  } ToFile16_IWORK;                    /* '<Root>/To File16' */

  struct {
    int_T Count;
    int_T Decimation;
  } ToFile9_IWORK;                     /* '<Root>/To File9' */

  struct {
    int_T Count;
    int_T Decimation;
  } ToFile15_IWORK;                    /* '<Root>/To File15' */

  struct {
    int_T Count;
    int_T Decimation;
  } ToFile2_IWORK;                     /* '<Root>/To File2' */

  struct {
    int_T Count;
    int_T Decimation;
  } ToFile12_IWORK;                    /* '<Root>/To File12' */

  struct {
    int_T Count;
    int_T Decimation;
  } ToFile5_IWORK;                     /* '<Root>/To File5' */

  struct {
    int_T Count;
    int_T Decimation;
  } ToFile14_IWORK;                    /* '<Root>/To File14' */

  struct {
    int_T Count;
    int_T Decimation;
  } ToFile8_IWORK;                     /* '<Root>/To File8' */

  struct {
    int_T Count;
    int_T Decimation;
  } ToFile7_IWORK;                     /* '<Root>/To File7' */

  struct {
    int_T Count;
    int_T Decimation;
  } ToFile1_IWORK;                     /* '<Root>/To File1' */

  struct {
    int_T Count;
    int_T Decimation;
  } ToFile_IWORK;                      /* '<Root>/To File' */

  int8_T RateTransition_LstBufWR;      /* '<Root>/Rate Transition' */
  int8_T RateTransition_RDBuf;         /* '<Root>/Rate Transition' */
  int8_T RateTransition1_LstBufWR;     /* '<Root>/Rate Transition1' */
  int8_T RateTransition1_RDBuf;        /* '<Root>/Rate Transition1' */
  uint8_T is_active_c12_raspberrypi_multi;/* '<Root>/Chart' */
  uint8_T temporalCounter_i1;          /* '<Root>/Chart' */
  boolean_T Memory_PreviousInput[252]; /* '<S18>/Memory' */
  boolean_T objisempty;                /* '<S55>/Digital Read' */
  boolean_T objisempty_d;              /* '<S54>/Digital Read' */
  boolean_T objisempty_a;              /* '<S53>/Digital Read' */
  boolean_T objisempty_i;              /* '<S52>/Digital Read' */
  boolean_T objisempty_p;              /* '<S15>/MATLAB System6' */
  boolean_T objisempty_i5;             /* '<S15>/MATLAB System3' */
  boolean_T objisempty_k;              /* '<S15>/MATLAB System' */
  boolean_T objisempty_g;              /* '<S14>/SPI Master Transfer' */
  boolean_T objisempty_m;              /* '<Root>/MATLAB System8' */
  boolean_T objisempty_n;              /* '<Root>/MATLAB System7' */
  boolean_T objisempty_gx;             /* '<Root>/MATLAB System6' */
  boolean_T objisempty_c;              /* '<Root>/MATLAB System3' */
  boolean_T objisempty_f;              /* '<Root>/MATLAB System21' */
  boolean_T objisempty_do;             /* '<Root>/MATLAB System20' */
  boolean_T objisempty_g3;             /* '<Root>/MATLAB System2' */
  boolean_T objisempty_ab;             /* '<Root>/MATLAB System18' */
  boolean_T objisempty_ci;             /* '<Root>/MATLAB System17' */
  boolean_T objisempty_m3;             /* '<Root>/MATLAB System16' */
  boolean_T objisempty_mb;             /* '<Root>/MATLAB System15' */
  boolean_T objisempty_e;              /* '<Root>/MATLAB System14' */
  boolean_T objisempty_h;              /* '<Root>/MATLAB System13' */
  boolean_T objisempty_k1;             /* '<Root>/MATLAB System12' */
  boolean_T objisempty_o;              /* '<Root>/MATLAB System11' */
  boolean_T objisempty_my;             /* '<Root>/MATLAB System10' */
  boolean_T objisempty_iw;             /* '<Root>/MATLAB System1' */
  boolean_T objisempty_k5;             /* '<Root>/MATLAB System' */
  DW_MATLABSystem4_raspberrypi__T MATLABSystem9;/* '<Root>/MATLAB System4' */
  DW_MATLABSystem4_raspberrypi__T MATLABSystem4;/* '<Root>/MATLAB System4' */
} DW_raspberrypi_multicore_MPCt_T;

/* Parameters (default storage) */
struct P_raspberrypi_multicore_MPCte_T_ {
  real_T T_gait;                       /* Variable: T_gait
                                        * Referenced by:
                                        *   '<Root>/Constant23'
                                        *   '<Root>/MATLAB System17'
                                        */
  real_T Ts;                           /* Variable: Ts
                                        * Referenced by:
                                        *   '<Root>/MATLAB System21'
                                        *   '<Root>/MATLAB System7'
                                        */
  real_T Ts_DynSim;                    /* Variable: Ts_DynSim
                                        * Referenced by:
                                        *   '<Root>/MATLAB System'
                                        *   '<Root>/MATLAB System16'
                                        *   '<Root>/MATLAB System17'
                                        *   '<Root>/MATLAB System20'
                                        *   '<S15>/Constant11'
                                        *   '<S15>/MATLAB System3'
                                        */
  real_T MATLABSystem10_Inorm[9];
                               /* Expression: diag( [ 0.0078, 0.0275, 0.0328 ] )
                                * Referenced by: '<Root>/MATLAB System10'
                                */
  real_T MATLABSystem10_m;             /* Expression: 3
                                        * Referenced by: '<Root>/MATLAB System10'
                                        */
  real_T MATLABSystem10_hIni;          /* Expression: 0.19
                                        * Referenced by: '<Root>/MATLAB System10'
                                        */
  real_T MATLABSystem10_lateral_width; /* Expression: 0.097
                                        * Referenced by: '<Root>/MATLAB System10'
                                        */
  real_T MATLABSystem10_sagetial_width;/* Expression: 0.2108
                                        * Referenced by: '<Root>/MATLAB System10'
                                        */
  real_T MATLABSystem10_roll_Off;      /* Expression: 0.037
                                        * Referenced by: '<Root>/MATLAB System10'
                                        */
  real_T MATLABSystem14_LF_Off[3];     /* Expression: [ 0; 0; 0 ]
                                        * Referenced by: '<Root>/MATLAB System14'
                                        */
  real_T MATLABSystem14_RF_Off[3];     /* Expression: [ 0; 0; 0 ]
                                        * Referenced by: '<Root>/MATLAB System14'
                                        */
  real_T MATLABSystem14_LH_Off[3];     /* Expression: [ 0; 0; 0 ]
                                        * Referenced by: '<Root>/MATLAB System14'
                                        */
  real_T MATLABSystem14_RH_Off[3];     /* Expression: [ 0; 0; 0 ]
                                        * Referenced by: '<Root>/MATLAB System14'
                                        */
  real_T MATLABSystem15_lateral_width; /* Expression: 0.097
                                        * Referenced by: '<Root>/MATLAB System15'
                                        */
  real_T MATLABSystem15_sagetial_width;/* Expression: 0.2108
                                        * Referenced by: '<Root>/MATLAB System15'
                                        */
  real_T MATLABSystem15_roll_Off;      /* Expression: 0.037
                                        * Referenced by: '<Root>/MATLAB System15'
                                        */
  real_T MATLABSystem15_hIni;          /* Expression: 0.19
                                        * Referenced by: '<Root>/MATLAB System15'
                                        */
  real_T MATLABSystem16_cons_Vel;      /* Expression: 15
                                        * Referenced by: '<Root>/MATLAB System16'
                                        */
  real_T MATLABSystem17_r0[3];         /* Expression: [ 0; 0; 0.19 ]
                                        * Referenced by: '<Root>/MATLAB System17'
                                        */
  real_T MATLABSystem17_theta0[3];     /* Expression: [ 0; 0; 0 ]
                                        * Referenced by: '<Root>/MATLAB System17'
                                        */
  real_T MATLABSystem17_dr0[3];        /* Expression: [ 0; 0; 0 ]
                                        * Referenced by: '<Root>/MATLAB System17'
                                        */
  real_T MATLABSystem17_omega0[3];     /* Expression: [ 0; 0; 0 ]
                                        * Referenced by: '<Root>/MATLAB System17'
                                        */
  real_T MATLABSystem17_lateral_width; /* Expression: 0.097
                                        * Referenced by: '<Root>/MATLAB System17'
                                        */
  real_T MATLABSystem17_sagetial_width;/* Expression: 0.2108
                                        * Referenced by: '<Root>/MATLAB System17'
                                        */
  real_T MATLABSystem17_roll_Off;      /* Expression: 0.037
                                        * Referenced by: '<Root>/MATLAB System17'
                                        */
  real_T MATLABSystem17_m;             /* Expression: 3
                                        * Referenced by: '<Root>/MATLAB System17'
                                        */
  real_T MATLABSystem17_kx;            /* Expression: 0.9
                                        * Referenced by: '<Root>/MATLAB System17'
                                        */
  real_T MATLABSystem17_ky;            /* Expression: 0.9
                                        * Referenced by: '<Root>/MATLAB System17'
                                        */
  real_T MATLABSystem17_kRz;           /* Expression: 0
                                        * Referenced by: '<Root>/MATLAB System17'
                                        */
  real_T MATLABSystem17_StepH;         /* Expression: 0.04
                                        * Referenced by: '<Root>/MATLAB System17'
                                        */
  real_T MATLABSystem17_OffsetTime;    /* Expression: 0
                                        * Referenced by: '<Root>/MATLAB System17'
                                        */
  real_T MATLABSystem17_TickTime;      /* Expression: 0
                                        * Referenced by: '<Root>/MATLAB System17'
                                        */
  real_T MATLABSystem17_startPhase;    /* Expression: 0
                                        * Referenced by: '<Root>/MATLAB System17'
                                        */
  real_T MATLABSystem18_lateral_width; /* Expression: 0.097
                                        * Referenced by: '<Root>/MATLAB System18'
                                        */
  real_T MATLABSystem18_sagetial_width;/* Expression: 0.2108
                                        * Referenced by: '<Root>/MATLAB System18'
                                        */
  real_T MATLABSystem18_roll_Off;      /* Expression: 0.037
                                        * Referenced by: '<Root>/MATLAB System18'
                                        */
  real_T MATLABSystem2_lateral_width;  /* Expression: 0.097
                                        * Referenced by: '<Root>/MATLAB System2'
                                        */
  real_T MATLABSystem2_sagetial_width; /* Expression: 0.2108
                                        * Referenced by: '<Root>/MATLAB System2'
                                        */
  real_T MATLABSystem2_roll_Off;       /* Expression: 0.037
                                        * Referenced by: '<Root>/MATLAB System2'
                                        */
  real_T MATLABSystem2_hIni;           /* Expression: 0.19
                                        * Referenced by: '<Root>/MATLAB System2'
                                        */
  real_T MATLABSystem2_m;              /* Expression: 3
                                        * Referenced by: '<Root>/MATLAB System2'
                                        */
  real_T MATLABSystem2_ks1;            /* Expression: 25
                                        * Referenced by: '<Root>/MATLAB System2'
                                        */
  real_T MATLABSystem2_ks2;            /* Expression: 25
                                        * Referenced by: '<Root>/MATLAB System2'
                                        */
  real_T MATLABSystem2_ks3;            /* Expression: 25
                                        * Referenced by: '<Root>/MATLAB System2'
                                        */
  real_T MATLABSystem20_m;             /* Expression: 3
                                        * Referenced by: '<Root>/MATLAB System20'
                                        */
  real_T MATLABSystem20_Inorm[9];
                          /* Expression: diag( [0.0046125 0.0230942 0.0253262] )
                           * Referenced by: '<Root>/MATLAB System20'
                           */
  real_T MATLABSystem21_Inorm[9];
                          /* Expression: diag( [0.0046125 0.0230942 0.0253262] )
                           * Referenced by: '<Root>/MATLAB System21'
                           */
  real_T MATLABSystem21_m;             /* Expression: 3
                                        * Referenced by: '<Root>/MATLAB System21'
                                        */
  real_T MATLABSystem21_hIni;          /* Expression: 0.19
                                        * Referenced by: '<Root>/MATLAB System21'
                                        */
  real_T MATLABSystem6_vxMax;          /* Expression: 0.2
                                        * Referenced by: '<Root>/MATLAB System6'
                                        */
  real_T MATLABSystem6_vyMax;          /* Expression: 0.2
                                        * Referenced by: '<Root>/MATLAB System6'
                                        */
  real_T MATLABSystem6_wzMax;          /* Expression: 0.8
                                        * Referenced by: '<Root>/MATLAB System6'
                                        */
  real_T MATLABSystem7_r0[3];          /* Expression: [ 0; 0; 0.19 ]
                                        * Referenced by: '<Root>/MATLAB System7'
                                        */
  real_T MATLABSystem7_theta0[3];      /* Expression: [ 0; 0; 0 ]
                                        * Referenced by: '<Root>/MATLAB System7'
                                        */
  real_T MATLABSystem7_dr0[3];         /* Expression: [ 0; 0; 0 ]
                                        * Referenced by: '<Root>/MATLAB System7'
                                        */
  real_T MATLABSystem7_omega0[3];      /* Expression: [ 0; 0; 0 ]
                                        * Referenced by: '<Root>/MATLAB System7'
                                        */
  real_T MATLABSystem7_desHeight;      /* Expression: 0.19
                                        * Referenced by: '<Root>/MATLAB System7'
                                        */
  real_T MATLABSystem7_sitaErr_K[3];   /* Expression: [0;0;0]
                                        * Referenced by: '<Root>/MATLAB System7'
                                        */
  real_T Constant12_Value;             /* Expression: 0
                                        * Referenced by: '<S14>/Constant12'
                                        */
  real_T Constant6_Value;              /* Expression: 0
                                        * Referenced by: '<Root>/Constant6'
                                        */
  real_T Constant7_Value;              /* Expression: 1
                                        * Referenced by: '<Root>/Constant7'
                                        */
  real_T DigitalRead_SampleTime;       /* Expression: sampleTime
                                        * Referenced by: '<S52>/Digital Read'
                                        */
  real_T DigitalRead_SampleTime_f;     /* Expression: sampleTime
                                        * Referenced by: '<S53>/Digital Read'
                                        */
  real_T DigitalRead_SampleTime_n;     /* Expression: sampleTime
                                        * Referenced by: '<S54>/Digital Read'
                                        */
  real_T DigitalRead_SampleTime_p;     /* Expression: sampleTime
                                        * Referenced by: '<S55>/Digital Read'
                                        */
  real_T Q_Value[625];
  /* Expression: diag([5e-3,5e-3,5e-3,5e-3,5e-3,5e-3,1e-3,1e-3,1e-3,1e-3,1e-3,1e-3,1e-3,1e-7,1e-7,1e-7,5e-7,5e-7,5e-7,1e-7,1e-7,1e-7,5e-7,5e-7,5e-7])
   * Referenced by: '<Root>/Q'
   */
  real_T RateTransition_InitialCondition[67];/* Expression: zeros(67,1)
                                              * Referenced by: '<Root>/Rate Transition'
                                              */
  real_T Constant_Value[3];            /* Expression: [11,12,13]
                                        * Referenced by: '<S14>/Constant'
                                        */
  real_T Constant13_Value[3];          /* Expression: [21,22,23]
                                        * Referenced by: '<S14>/Constant13'
                                        */
  real_T Constant14_Value[3];          /* Expression: [31,32,33]
                                        * Referenced by: '<S14>/Constant14'
                                        */
  real_T Constant15_Value[3];          /* Expression: [41,42,43]
                                        * Referenced by: '<S14>/Constant15'
                                        */
  real_T Switch_Threshold;             /* Expression: 5
                                        * Referenced by: '<Root>/Switch'
                                        */
  real_T Constant22_Value;             /* Expression: 2*pi
                                        * Referenced by: '<Root>/Constant22'
                                        */
  real_T UnitDelay3_InitialCondition[4];/* Expression: [1,1,1,1]
                                         * Referenced by: '<Root>/Unit Delay3'
                                         */
  real_T UnitDelay6_InitialCondition[12];/* Expression: [0;0;0.19;zeros(9,1)]
                                          * Referenced by: '<Root>/Unit Delay6'
                                          */
  real_T UnitDelay5_InitialCondition[12];
  /* Expression: [0.1075,0.1075,-0.1075,-0.1075;0.0750,-0.0750,0.0750,-0.0750;0,0,0,0];
   * Referenced by: '<Root>/Unit Delay5'
   */
  real_T Constant8_Value;              /* Expression: 1
                                        * Referenced by: '<Root>/Constant8'
                                        */
  real_T UnitDelay4_InitialCondition[4];/* Expression: [0;0;0;0]
                                         * Referenced by: '<Root>/Unit Delay4'
                                         */
  real_T UnitDelay_InitialCondition[12];/* Expression: zeros(12,1)
                                         * Referenced by: '<Root>/Unit Delay'
                                         */
  real_T Constant10_Value;             /* Expression: 3
                                        * Referenced by: '<S14>/Constant10'
                                        */
  real_T Constant11_Value;             /* Expression: 5
                                        * Referenced by: '<S14>/Constant11'
                                        */
  real_T Constant4_Value;              /* Expression: 1
                                        * Referenced by: '<Root>/Constant4'
                                        */
  real_T Memory1_InitialCondition;     /* Expression: 1
                                        * Referenced by: '<S14>/Memory1'
                                        */
  real_T Switch1_Threshold;            /* Expression: 0.5
                                        * Referenced by: '<S14>/Switch1'
                                        */
  real_T UnitDelay_InitialCondition_j[12];/* Expression: zeros(12,1)
                                           * Referenced by: '<S15>/Unit Delay'
                                           */
  real_T Constant5_Value[3];           /* Expression: [0,0,0]
                                        * Referenced by: '<Root>/Constant5'
                                        */
  real_T Constant1_Value[81];
             /* Expression: diag([1e-7,5e-8,1e-8,1e-6,1e-6,1e-6,1e-5,1e-5,1e-5])
              * Referenced by: '<S15>/Constant1'
              */
  real_T Constant_Value_n;             /* Expression: 9.8
                                        * Referenced by: '<Root>/Constant'
                                        */
  real_T R_Value[169];
  /* Expression: diag([1e-7,5e-8,1e-8,1e-6,1e-6,1e-6,1e-5,1e-3,1e-3,1e-3,1e-3,1e-3,1e-3])
   * Referenced by: '<Root>/R'
   */
  real_T Constant9_Value;              /* Expression: 1
                                        * Referenced by: '<Root>/Constant9'
                                        */
  real_T Constant25_Value;             /* Expression: 66
                                        * Referenced by: '<Root>/Constant25'
                                        */
  real_T RateTransition1_InitialConditio[61];/* Expression: zeros(61,1)
                                              * Referenced by: '<Root>/Rate Transition1'
                                              */
  real_T UnitDelay7_InitialCondition[12];/* Expression: [0;0;0.19;zeros(9,1)]
                                          * Referenced by: '<Root>/Unit Delay7'
                                          */
  real_T Switch1_Threshold_k;          /* Expression: 0.5
                                        * Referenced by: '<Root>/Switch1'
                                        */
  real_T ymin_scale1_Gain[416];     /* Expression: Yscale(:,ones(1,max(nCC,1)))'
                                     * Referenced by: '<S18>/ymin_scale1'
                                     */
  real_T umin_scale4_Gain[384];    /* Expression: MVscale(:,ones(1,max(nCC,1)))'
                                    * Referenced by: '<S18>/umin_scale4'
                                    */
  real_T ymin_scale2_Gain[384];    /* Expression: MDscale(:,ones(1,max(nCC,1)))'
                                    * Referenced by: '<S18>/ymin_scale2'
                                    */
  real_T Constant2_Value;              /* Expression: 9.8
                                        * Referenced by: '<Root>/Constant2'
                                        */
  real_T last_mv_InitialCondition[12]; /* Expression: lastu+uoff
                                        * Referenced by: '<S18>/last_mv'
                                        */
  real_T ym_zero_Value[13];            /* Expression: zeros(nym,1)
                                        * Referenced by: '<S18>/ym_zero'
                                        */
  real_T umin_zero_Value[12];          /* Expression: zeros(12,1)
                                        * Referenced by: '<S1>/umin_zero'
                                        */
  real_T umax_zero_Value[12];          /* Expression: zeros(12,1)
                                        * Referenced by: '<S1>/umax_zero'
                                        */
  real_T ymin_zero_Value[13];          /* Expression: zeros(13,1)
                                        * Referenced by: '<S1>/ymin_zero'
                                        */
  real_T ymax_zero_Value[13];          /* Expression: zeros(13,1)
                                        * Referenced by: '<S1>/ymax_zero'
                                        */
  real_T UnitDelay2_InitialCondition[12];
                               /* Expression: [0;0;7.5;0;0;7.5;0;0;7.5;0;0;7.5];
                                * Referenced by: '<Root>/Unit Delay2'
                                */
  real_T extmv_scale_Gain[12];         /* Expression: RMVscale
                                        * Referenced by: '<S18>/ext.mv_scale'
                                        */
  real_T mvtarget_zero_Value[12];      /* Expression: zeros(12,1)
                                        * Referenced by: '<S1>/mv.target_zero'
                                        */
  real_T uref_scale_Gain[12];          /* Expression: RMVscale
                                        * Referenced by: '<S18>/uref_scale'
                                        */
  real_T ywt_zero_Value[13];           /* Expression: zeros(13,1)
                                        * Referenced by: '<S1>/y.wt_zero'
                                        */
  real_T uwt_zero_Value[12];           /* Expression: zeros(12,1)
                                        * Referenced by: '<S1>/u.wt_zero'
                                        */
  real_T duwt_zero_Value[12];          /* Expression: zeros(12,1)
                                        * Referenced by: '<S1>/du.wt_zero'
                                        */
  real_T ecrwt_zero_Value;             /* Expression: zeros(1,1)
                                        * Referenced by: '<S1>/ecr.wt_zero'
                                        */
  real_T LastPcov_InitialCondition[169];/* Expression: lastPcov
                                         * Referenced by: '<S18>/LastPcov'
                                         */
  real_T u_scale_Gain[12];             /* Expression: MVscale
                                        * Referenced by: '<S18>/u_scale'
                                        */
  real_T Constant1_Value_o;            /* Expression: 1
                                        * Referenced by: '<Root>/Constant1'
                                        */
  real_T UnitDelay1_InitialCondition;  /* Expression: 0
                                        * Referenced by: '<Root>/Unit Delay1'
                                        */
  real_T Constant3_Value;              /* Expression: 66
                                        * Referenced by: '<Root>/Constant3'
                                        */
  int16_T FixedHorizonOptimizer_Ndis;  /* Expression: Ndis
                                        * Referenced by: '<S46>/FixedHorizonOptimizer'
                                        */
  boolean_T Memory_InitialCondition[252];/* Expression: iA
                                          * Referenced by: '<S18>/Memory'
                                          */
};

/* Real-time Model Data Structure */
struct tag_RTM_raspberrypi_multicore_T {
  const char_T *errorStatus;
  RTWExtModeInfo *extModeInfo;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    uint32_T checksums[4];
  } Sizes;

  /*
   * SpecialInfo:
   * The following substructure contains special information
   * related to other components that are dependent on RTW.
   */
  struct {
    const void *mappingInfo;
  } SpecialInfo;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    time_T taskTime0;
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    uint32_T clockTick1;
    uint32_T clockTickH1;
    struct {
      uint8_T TID[2];
    } TaskCounters;

    time_T tFinal;
    boolean_T stopRequestedFlag;
  } Timing;
};

/* Block parameters (default storage) */
extern P_raspberrypi_multicore_MPCte_T raspberrypi_multicore_MPCtest_P;

/* Block signals (default storage) */
extern B_raspberrypi_multicore_MPCte_T raspberrypi_multicore_MPCtest_B;

/* Block states (default storage) */
extern DW_raspberrypi_multicore_MPCt_T raspberrypi_multicore_MPCtes_DW;

/* External function called from main */
extern void
  raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis_SetEventsForThisBaseStep
  (boolean_T *eventFlags);
extern void rate_scheduler(void);

/* Model entry point functions */
extern void
  raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis_SetEventsForThisBaseStep
  (boolean_T *eventFlags);
extern void raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis_initialize
  (void);
extern void raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis_step0(void);
extern void raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis_step1(void);
extern void raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis_step(int_T
  tid);
extern void raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis_terminate
  (void);

/* Real-time Model object */
extern RT_MODEL_raspberrypi_multicor_T *const raspberrypi_multicore_MPCtes_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S18>/Data Type Conversion22' : Unused code path elimination
 * Block '<S18>/Data Type Conversion23' : Unused code path elimination
 * Block '<S18>/Floor' : Unused code path elimination
 * Block '<S18>/Floor1' : Unused code path elimination
 * Block '<S19>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S20>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S21>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S22>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S23>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S24>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S25>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S26>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S27>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S28>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S29>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S30>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S31>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S32>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S33>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S34>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S35>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S36>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S37>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S38>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S39>/Vector Dimension Check' : Unused code path elimination
 * Block '<S40>/Vector Dimension Check' : Unused code path elimination
 * Block '<S41>/Vector Dimension Check' : Unused code path elimination
 * Block '<S42>/Vector Dimension Check' : Unused code path elimination
 * Block '<S43>/Vector Dimension Check' : Unused code path elimination
 * Block '<S44>/Vector Dimension Check' : Unused code path elimination
 * Block '<S45>/Vector Dimension Check' : Unused code path elimination
 * Block '<S18>/last_x' : Unused code path elimination
 * Block '<S18>/useq_scale' : Unused code path elimination
 * Block '<S18>/useq_scale1' : Unused code path elimination
 * Block '<S1>/m_zero' : Unused code path elimination
 * Block '<S1>/p_zero' : Unused code path elimination
 * Block '<S18>/Data Type Conversion1' : Eliminate redundant data type conversion
 * Block '<S18>/Data Type Conversion10' : Eliminate redundant data type conversion
 * Block '<S18>/Data Type Conversion11' : Eliminate redundant data type conversion
 * Block '<S18>/Data Type Conversion12' : Eliminate redundant data type conversion
 * Block '<S18>/Data Type Conversion13' : Eliminate redundant data type conversion
 * Block '<S18>/Data Type Conversion14' : Eliminate redundant data type conversion
 * Block '<S18>/Data Type Conversion15' : Eliminate redundant data type conversion
 * Block '<S18>/Data Type Conversion16' : Eliminate redundant data type conversion
 * Block '<S18>/Data Type Conversion17' : Eliminate redundant data type conversion
 * Block '<S18>/Data Type Conversion18' : Eliminate redundant data type conversion
 * Block '<S18>/Data Type Conversion19' : Eliminate redundant data type conversion
 * Block '<S18>/Data Type Conversion2' : Eliminate redundant data type conversion
 * Block '<S18>/Data Type Conversion20' : Eliminate redundant data type conversion
 * Block '<S18>/Data Type Conversion21' : Eliminate redundant data type conversion
 * Block '<S18>/Data Type Conversion3' : Eliminate redundant data type conversion
 * Block '<S18>/Data Type Conversion4' : Eliminate redundant data type conversion
 * Block '<S18>/Data Type Conversion5' : Eliminate redundant data type conversion
 * Block '<S18>/Data Type Conversion6' : Eliminate redundant data type conversion
 * Block '<S18>/Data Type Conversion7' : Eliminate redundant data type conversion
 * Block '<S18>/Data Type Conversion8' : Eliminate redundant data type conversion
 * Block '<S18>/Data Type Conversion9' : Eliminate redundant data type conversion
 * Block '<S18>/E Conversion' : Eliminate redundant data type conversion
 * Block '<S18>/F Conversion' : Eliminate redundant data type conversion
 * Block '<S18>/G Conversion' : Eliminate redundant data type conversion
 * Block '<S18>/Reshape' : Reshape block reduction
 * Block '<S18>/Reshape1' : Reshape block reduction
 * Block '<S18>/Reshape2' : Reshape block reduction
 * Block '<S18>/Reshape3' : Reshape block reduction
 * Block '<S18>/Reshape4' : Reshape block reduction
 * Block '<S18>/Reshape5' : Reshape block reduction
 * Block '<S18>/S Conversion' : Eliminate redundant data type conversion
 * Block '<S18>/mo or x Conversion' : Eliminate redundant data type conversion
 * Block '<Root>/Reshape' : Reshape block reduction
 * Block '<Root>/Reshape1' : Reshape block reduction
 * Block '<Root>/Reshape2' : Reshape block reduction
 * Block '<Root>/Reshape3' : Reshape block reduction
 * Block '<Root>/Reshape4' : Reshape block reduction
 * Block '<Root>/Reshape5' : Reshape block reduction
 * Block '<Root>/Reshape6' : Reshape block reduction
 */

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
 * '<Root>' : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis'
 * '<S1>'   : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/Adaptive MPC Controller1'
 * '<S2>'   : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/Chart'
 * '<S3>'   : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/Leg Cor 2 Body Cor'
 * '<S4>'   : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/MATLAB Function'
 * '<S5>'   : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/MATLAB Function1'
 * '<S6>'   : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/MATLAB Function2'
 * '<S7>'   : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/MATLAB Function4'
 * '<S8>'   : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/MATLAB Function5'
 * '<S9>'   : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/MATLAB Function6'
 * '<S10>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/MATLAB Function7'
 * '<S11>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/MATLAB Function8'
 * '<S12>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/Norminal_to_ServoAngle'
 * '<S13>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/ServoAngle_to_Norminal'
 * '<S14>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/ServoCtr_V2'
 * '<S15>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/Subsystem'
 * '<S16>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/getSwitchStatus'
 * '<S17>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/online Constraints1'
 * '<S18>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/Adaptive MPC Controller1/MPC'
 * '<S19>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/Adaptive MPC Controller1/MPC/MPC Matrix Signal Check'
 * '<S20>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/Adaptive MPC Controller1/MPC/MPC Matrix Signal Check A'
 * '<S21>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/Adaptive MPC Controller1/MPC/MPC Matrix Signal Check B'
 * '<S22>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/Adaptive MPC Controller1/MPC/MPC Matrix Signal Check C'
 * '<S23>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/Adaptive MPC Controller1/MPC/MPC Matrix Signal Check D'
 * '<S24>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/Adaptive MPC Controller1/MPC/MPC Matrix Signal Check DX'
 * '<S25>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/Adaptive MPC Controller1/MPC/MPC Matrix Signal Check U'
 * '<S26>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/Adaptive MPC Controller1/MPC/MPC Matrix Signal Check X'
 * '<S27>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/Adaptive MPC Controller1/MPC/MPC Matrix Signal Check Y'
 * '<S28>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/Adaptive MPC Controller1/MPC/MPC Matrix Signal Check1'
 * '<S29>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/Adaptive MPC Controller1/MPC/MPC Matrix Signal Check2'
 * '<S30>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/Adaptive MPC Controller1/MPC/MPC Preview Signal Check'
 * '<S31>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/Adaptive MPC Controller1/MPC/MPC Preview Signal Check1'
 * '<S32>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/Adaptive MPC Controller1/MPC/MPC Preview Signal Check2'
 * '<S33>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/Adaptive MPC Controller1/MPC/MPC Preview Signal Check3'
 * '<S34>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/Adaptive MPC Controller1/MPC/MPC Preview Signal Check4'
 * '<S35>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/Adaptive MPC Controller1/MPC/MPC Preview Signal Check5'
 * '<S36>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/Adaptive MPC Controller1/MPC/MPC Preview Signal Check6'
 * '<S37>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/Adaptive MPC Controller1/MPC/MPC Preview Signal Check7'
 * '<S38>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/Adaptive MPC Controller1/MPC/MPC Preview Signal Check8'
 * '<S39>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/Adaptive MPC Controller1/MPC/MPC Scalar Signal Check'
 * '<S40>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/Adaptive MPC Controller1/MPC/MPC Scalar Signal Check1'
 * '<S41>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/Adaptive MPC Controller1/MPC/MPC Scalar Signal Check2'
 * '<S42>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/Adaptive MPC Controller1/MPC/MPC Vector Signal Check'
 * '<S43>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/Adaptive MPC Controller1/MPC/MPC Vector Signal Check1'
 * '<S44>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/Adaptive MPC Controller1/MPC/MPC Vector Signal Check11'
 * '<S45>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/Adaptive MPC Controller1/MPC/MPC Vector Signal Check6'
 * '<S46>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/Adaptive MPC Controller1/MPC/optimizer'
 * '<S47>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/Adaptive MPC Controller1/MPC/optimizer/FixedHorizonOptimizer'
 * '<S48>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/ServoCtr_V2/MATLAB Function'
 * '<S49>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/ServoCtr_V2/MATLAB Function1'
 * '<S50>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/Subsystem/MATLAB Function'
 * '<S51>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/Subsystem/MATLAB Function1'
 * '<S52>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/getSwitchStatus/S1'
 * '<S53>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/getSwitchStatus/S2'
 * '<S54>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/getSwitchStatus/S3'
 * '<S55>'  : 'raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis/getSwitchStatus/S4'
 */
#endif
      /* RTW_HEADER_raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis_h_ */
