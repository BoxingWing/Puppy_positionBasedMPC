/*
 * raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis_types.h
 *
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * Code generation for model "raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis".
 *
 * Model version              : 2.372
 * Simulink Coder version : 9.5 (R2021a) 14-Nov-2020
 * C source code generated on : Thu Oct 21 12:01:29 2021
 *
 * Target selection: ert.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis_types_h_
#define RTW_HEADER_raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis_types_h_
#include "rtwtypes.h"
#include "multiword_types.h"

/* Model Code Variants */
#include "rtw_linux.h"
#ifndef DEFINED_TYPEDEF_FOR_struct_qYRJtcce7MM7XuQ3AAWdMD_
#define DEFINED_TYPEDEF_FOR_struct_qYRJtcce7MM7XuQ3AAWdMD_

typedef struct {
  real_T MaxIterations;
  real_T ConstraintTolerance;
  boolean_T UseWarmStart;
} struct_qYRJtcce7MM7XuQ3AAWdMD;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_SmvKLCDySlKdToNTroAGyF_
#define DEFINED_TYPEDEF_FOR_struct_SmvKLCDySlKdToNTroAGyF_

typedef struct {
  real_T MaxIterations;
  real_T ConstraintTolerance;
  real_T OptimalityTolerance;
  real_T ComplementarityTolerance;
  real_T StepTolerance;
} struct_SmvKLCDySlKdToNTroAGyF;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_aH4cViuPz8aZIf26PeppuD_
#define DEFINED_TYPEDEF_FOR_struct_aH4cViuPz8aZIf26PeppuD_

typedef struct {
  real_T MaxIterations;
  real_T ConstraintTolerance;
  real_T DiscreteConstraintTolerance;
  boolean_T RoundingAtRootNode;
  real_T MaxPendingNodes;
} struct_aH4cViuPz8aZIf26PeppuD;

#endif

#ifndef struct_tag_OJDNKcWHoIkwfa6qcciyOC
#define struct_tag_OJDNKcWHoIkwfa6qcciyOC

struct tag_OJDNKcWHoIkwfa6qcciyOC
{
  real_T dataOld;
};

#endif                                 /* struct_tag_OJDNKcWHoIkwfa6qcciyOC */

#ifndef typedef_edgeDetector_raspberrypi_mult_T
#define typedef_edgeDetector_raspberrypi_mult_T

typedef struct tag_OJDNKcWHoIkwfa6qcciyOC edgeDetector_raspberrypi_mult_T;

#endif                             /* typedef_edgeDetector_raspberrypi_mult_T */

#ifndef struct_tag_SvZQAaSRdQgKLzSmghKMHB
#define struct_tag_SvZQAaSRdQgKLzSmghKMHB

struct tag_SvZQAaSRdQgKLzSmghKMHB
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  real_T SampleTime;
};

#endif                                 /* struct_tag_SvZQAaSRdQgKLzSmghKMHB */

#ifndef typedef_JoystickRead_raspberrypi_mult_T
#define typedef_JoystickRead_raspberrypi_mult_T

typedef struct tag_SvZQAaSRdQgKLzSmghKMHB JoystickRead_raspberrypi_mult_T;

#endif                             /* typedef_JoystickRead_raspberrypi_mult_T */

#ifndef struct_tag_HJ9hVWGSAwyxLu1xLHIoFF
#define struct_tag_HJ9hVWGSAwyxLu1xLHIoFF

struct tag_HJ9hVWGSAwyxLu1xLHIoFF
{
  real_T tCount;
};

#endif                                 /* struct_tag_HJ9hVWGSAwyxLu1xLHIoFF */

#ifndef typedef_PhaseOscillator_SG_raspberryp_T
#define typedef_PhaseOscillator_SG_raspberryp_T

typedef struct tag_HJ9hVWGSAwyxLu1xLHIoFF PhaseOscillator_SG_raspberryp_T;

#endif                             /* typedef_PhaseOscillator_SG_raspberryp_T */

#ifndef struct_tag_0EtcGaKcTiFf2U70KE8pM
#define struct_tag_0EtcGaKcTiFf2U70KE8pM

struct tag_0EtcGaKcTiFf2U70KE8pM
{
  real_T pArray_L_Old[12];
};

#endif                                 /* struct_tag_0EtcGaKcTiFf2U70KE8pM */

#ifndef typedef_footEndPos_protect_raspberryp_T
#define typedef_footEndPos_protect_raspberryp_T

typedef struct tag_0EtcGaKcTiFf2U70KE8pM footEndPos_protect_raspberryp_T;

#endif                             /* typedef_footEndPos_protect_raspberryp_T */

#ifndef struct_tag_rnygrDayOjkpi7zvZVk62D
#define struct_tag_rnygrDayOjkpi7zvZVk62D

struct tag_rnygrDayOjkpi7zvZVk62D
{
  real_T BC;
  real_T CDP;
  real_T DP;
  real_T OR;
};

#endif                                 /* struct_tag_rnygrDayOjkpi7zvZVk62D */

#ifndef typedef_FK_raspberrypi_multicore_MPCt_T
#define typedef_FK_raspberrypi_multicore_MPCt_T

typedef struct tag_rnygrDayOjkpi7zvZVk62D FK_raspberrypi_multicore_MPCt_T;

#endif                             /* typedef_FK_raspberrypi_multicore_MPCt_T */

#ifndef struct_tag_ZZn4ydEqPVDsseBsY0fjiD
#define struct_tag_ZZn4ydEqPVDsseBsY0fjiD

struct tag_ZZn4ydEqPVDsseBsY0fjiD
{
  int32_T isInitialized;
  real_T lateral_width;
  real_T sagetial_width;
  real_T roll_Off;
  real_T hIni;
  real_T pCoM_Old[3];
  real_T vCoM_Old[3];
  real_T RPYnew_Old[3];
  real_T OmegaW_Old[3];
  real_T SPLeg_Old[4];
  real_T SP_Old[12];
  real_T surVN_Old[3];
  real_T surV1_Old[3];
  real_T surV2_Old[3];
  real_T surP_Old[3];
  real_T headG_Old[3];
  real_T vxPercent_Old;
  real_T vyPercent_Old;
  real_T wzPercent_Old;
  real_T phi_Old;
  real_T estDis_Old[6];
};

#endif                                 /* struct_tag_ZZn4ydEqPVDsseBsY0fjiD */

#ifndef typedef_message_decoder_PC_raspberryp_T
#define typedef_message_decoder_PC_raspberryp_T

typedef struct tag_ZZn4ydEqPVDsseBsY0fjiD message_decoder_PC_raspberryp_T;

#endif                             /* typedef_message_decoder_PC_raspberryp_T */

#ifndef struct_tag_j4qwwDLdYxEHmNbDqyAMYC
#define struct_tag_j4qwwDLdYxEHmNbDqyAMYC

struct tag_j4qwwDLdYxEHmNbDqyAMYC
{
  int32_T isInitialized;
  real_T cons_Vel;
  real_T dt;
  real_T PosDesOld[12];
  real_T Ini_count;
};

#endif                                 /* struct_tag_j4qwwDLdYxEHmNbDqyAMYC */

#ifndef typedef_PosAssign_vel_raspberrypi_mul_T
#define typedef_PosAssign_vel_raspberrypi_mul_T

typedef struct tag_j4qwwDLdYxEHmNbDqyAMYC PosAssign_vel_raspberrypi_mul_T;

#endif                             /* typedef_PosAssign_vel_raspberrypi_mul_T */

#ifndef struct_tag_oxKlBSYXn8jAm1PP5X6Xp
#define struct_tag_oxKlBSYXn8jAm1PP5X6Xp

struct tag_oxKlBSYXn8jAm1PP5X6Xp
{
  real_T XOld;
  real_T YOld;
  real_T AOld;
  real_T BOld;
  real_T upOld;
  real_T downOld;
  real_T leftOld;
  real_T rightOld;
  real_T LeftAxis[2];
  real_T RightAxis[2];
  real_T count;
};

#endif                                 /* struct_tag_oxKlBSYXn8jAm1PP5X6Xp */

#ifndef typedef_buttonDecoder_2_raspberrypi_m_T
#define typedef_buttonDecoder_2_raspberrypi_m_T

typedef struct tag_oxKlBSYXn8jAm1PP5X6Xp buttonDecoder_2_raspberrypi_m_T;

#endif                             /* typedef_buttonDecoder_2_raspberrypi_m_T */

#ifndef struct_tag_BkJBeTFpBMhEhCzHdIUrhG
#define struct_tag_BkJBeTFpBMhEhCzHdIUrhG

struct tag_BkJBeTFpBMhEhCzHdIUrhG
{
  int32_T isInitialized;
  real_T vxMax;
  real_T vyMax;
  real_T wzMax;
};

#endif                                 /* struct_tag_BkJBeTFpBMhEhCzHdIUrhG */

#ifndef typedef_ref_ctr_raspberrypi_multicore_T
#define typedef_ref_ctr_raspberrypi_multicore_T

typedef struct tag_BkJBeTFpBMhEhCzHdIUrhG ref_ctr_raspberrypi_multicore_T;

#endif                             /* typedef_ref_ctr_raspberrypi_multicore_T */

#ifndef struct_tag_i43SvnPFqp0TolL88iyUj
#define struct_tag_i43SvnPFqp0TolL88iyUj

struct tag_i43SvnPFqp0TolL88iyUj
{
  real_T ESOld;
};

#endif                                 /* struct_tag_i43SvnPFqp0TolL88iyUj */

#ifndef typedef_ESgen_raspberrypi_multicore_M_T
#define typedef_ESgen_raspberrypi_multicore_M_T

typedef struct tag_i43SvnPFqp0TolL88iyUj ESgen_raspberrypi_multicore_M_T;

#endif                             /* typedef_ESgen_raspberrypi_multicore_M_T */

/* Custom Type definition for MATLABSystem: '<S14>/SPI Master Transfer' */
#include "MW_SVD.h"
#ifndef struct_tag_vB8gURpcbTUBlHcczw228B
#define struct_tag_vB8gURpcbTUBlHcczw228B

struct tag_vB8gURpcbTUBlHcczw228B
{
  int32_T __dummy;
};

#endif                                 /* struct_tag_vB8gURpcbTUBlHcczw228B */

#ifndef typedef_e_codertarget_raspi_internal__T
#define typedef_e_codertarget_raspi_internal__T

typedef struct tag_vB8gURpcbTUBlHcczw228B e_codertarget_raspi_internal__T;

#endif                             /* typedef_e_codertarget_raspi_internal__T */

#ifndef struct_tag_q4V738Rbj3JAyZLmtF7H2G
#define struct_tag_q4V738Rbj3JAyZLmtF7H2G

struct tag_q4V738Rbj3JAyZLmtF7H2G
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  e_codertarget_raspi_internal__T Hw;
  MW_Handle_Type MW_SPI_HANDLE;
};

#endif                                 /* struct_tag_q4V738Rbj3JAyZLmtF7H2G */

#ifndef typedef_codertarget_raspi_internal_SP_T
#define typedef_codertarget_raspi_internal_SP_T

typedef struct tag_q4V738Rbj3JAyZLmtF7H2G codertarget_raspi_internal_SP_T;

#endif                             /* typedef_codertarget_raspi_internal_SP_T */

#ifndef struct_tag_L039fPpwOJyNiyi09wU2GE
#define struct_tag_L039fPpwOJyNiyi09wU2GE

struct tag_L039fPpwOJyNiyi09wU2GE
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  real_T SampleTime;
};

#endif                                 /* struct_tag_L039fPpwOJyNiyi09wU2GE */

#ifndef typedef_codertarget_linux_blocks_Digi_T
#define typedef_codertarget_linux_blocks_Digi_T

typedef struct tag_L039fPpwOJyNiyi09wU2GE codertarget_linux_blocks_Digi_T;

#endif                             /* typedef_codertarget_linux_blocks_Digi_T */

#ifndef struct_tag_G81DOTaAOeYk2z0hIYglZH
#define struct_tag_G81DOTaAOeYk2z0hIYglZH

struct tag_G81DOTaAOeYk2z0hIYglZH
{
  real_T Roff[9];
  real_T accOff[3];
  real_T sitaOld[3];
  real_T yawOff;
  real_T accStore[1200];
};

#endif                                 /* struct_tag_G81DOTaAOeYk2z0hIYglZH */

#ifndef typedef_IMU_regulation_raspberrypi_mu_T
#define typedef_IMU_regulation_raspberrypi_mu_T

typedef struct tag_G81DOTaAOeYk2z0hIYglZH IMU_regulation_raspberrypi_mu_T;

#endif                             /* typedef_IMU_regulation_raspberrypi_mu_T */

#ifndef struct_tag_l6D6mQx3SthYPXH3T0e5wE
#define struct_tag_l6D6mQx3SthYPXH3T0e5wE

struct tag_l6D6mQx3SthYPXH3T0e5wE
{
  int32_T isInitialized;
  real_T Inorm[9];
  real_T m;
  real_T hIni;
  real_T lateral_width;
  real_T sagetial_width;
  real_T roll_Off;
  real_T Inow_Old[9];
  real_T U_Old[12];
  real_T X_mpc_Old[13];
  real_T refP_Old[13];
  real_T SP_MPC_Old[12];
  real_T MPC_Count_Old;
  real_T LegStateMPC_Old[4];
  real_T phiSlow_Old;
};

#endif                                 /* struct_tag_l6D6mQx3SthYPXH3T0e5wE */

#ifndef typedef_UDP_decoder_raspi_raspberrypi_T
#define typedef_UDP_decoder_raspi_raspberrypi_T

typedef struct tag_l6D6mQx3SthYPXH3T0e5wE UDP_decoder_raspi_raspberrypi_T;

#endif                             /* typedef_UDP_decoder_raspi_raspberrypi_T */

#ifndef struct_tag_Dw3M1dfI2b6XP3rqDbUWLB
#define struct_tag_Dw3M1dfI2b6XP3rqDbUWLB

struct tag_Dw3M1dfI2b6XP3rqDbUWLB
{
  int32_T isInitialized;
  real_T LF_Off[3];
  real_T RF_Off[3];
  real_T LH_Off[3];
  real_T RH_Off[3];
  real_T AB;
  real_T BC;
  real_T CDP;
  real_T DP;
  real_T OR;
  real_T PxLim[2];
  real_T PyLim[2];
  real_T PzLim[2];
  real_T LastInput[12];
  real_T AngleOff[12];
};

#endif                                 /* struct_tag_Dw3M1dfI2b6XP3rqDbUWLB */

#ifndef typedef_IK_raspberrypi_multicore_MPCt_T
#define typedef_IK_raspberrypi_multicore_MPCt_T

typedef struct tag_Dw3M1dfI2b6XP3rqDbUWLB IK_raspberrypi_multicore_MPCt_T;

#endif                             /* typedef_IK_raspberrypi_multicore_MPCt_T */

#ifndef struct_tag_KCsQzNs3pIl19Bg9MNXqVF
#define struct_tag_KCsQzNs3pIl19Bg9MNXqVF

struct tag_KCsQzNs3pIl19Bg9MNXqVF
{
  int32_T isInitialized;
  real_T r0[3];
  real_T theta0[3];
  real_T dr0[3];
  real_T omega0[3];
  real_T lateral_width;
  real_T sagetial_width;
  real_T roll_Off;
  real_T m;
  real_T kx;
  real_T ky;
  real_T kRz;
  real_T T;
  real_T StepH;
  real_T SampleTime;
  real_T OffsetTime;
  real_T TickTime;
  real_T startPhase;
  real_T pL_LS[12];
  real_T PendAllLocalOld[12];
  real_T MPC_legStateOld[4];
  real_T pArray_L_Adm_Old[12];
  real_T interpol_Count;
  real_T pArray_L_Adm_Now[12];
  real_T MPC_Count_Old;
  real_T pLnorm[12];
  real_T vNowN[60];
};

#endif                                 /* struct_tag_KCsQzNs3pIl19Bg9MNXqVF */

#ifndef typedef_LegSequence_RT_v3_raspberrypi_T
#define typedef_LegSequence_RT_v3_raspberrypi_T

typedef struct tag_KCsQzNs3pIl19Bg9MNXqVF LegSequence_RT_v3_raspberrypi_T;

#endif                             /* typedef_LegSequence_RT_v3_raspberrypi_T */

#ifndef struct_tag_EPcQJG41OLoMDBQ6mZ5T5G
#define struct_tag_EPcQJG41OLoMDBQ6mZ5T5G

struct tag_EPcQJG41OLoMDBQ6mZ5T5G
{
  int32_T isInitialized;
  real_T lateral_width;
  real_T sagetial_width;
  real_T roll_Off;
  real_T LegState_Old[4];
  real_T pW_LS[12];
  real_T pWnorm[12];
};

#endif                                 /* struct_tag_EPcQJG41OLoMDBQ6mZ5T5G */

#ifndef typedef_groundAdaptionP1_raspberrypi__T
#define typedef_groundAdaptionP1_raspberrypi__T

typedef struct tag_EPcQJG41OLoMDBQ6mZ5T5G groundAdaptionP1_raspberrypi__T;

#endif                             /* typedef_groundAdaptionP1_raspberrypi__T */

#ifndef struct_tag_UN5JRbyCwE1SFrDcP2jdrF
#define struct_tag_UN5JRbyCwE1SFrDcP2jdrF

struct tag_UN5JRbyCwE1SFrDcP2jdrF
{
  int32_T isInitialized;
  real_T lateral_width;
  real_T sagetial_width;
  real_T roll_Off;
  real_T hIni;
  real_T m;
  real_T ks1;
  real_T ks2;
  real_T ks3;
  real_T AB;
  real_T BC;
  real_T CDP;
  real_T DP;
  real_T OR;
  real_T PendAllnorm[12];
};

#endif                                 /* struct_tag_UN5JRbyCwE1SFrDcP2jdrF */

#ifndef typedef_AdmittanceCtr_raspberrypi_mul_T
#define typedef_AdmittanceCtr_raspberrypi_mul_T

typedef struct tag_UN5JRbyCwE1SFrDcP2jdrF AdmittanceCtr_raspberrypi_mul_T;

#endif                             /* typedef_AdmittanceCtr_raspberrypi_mul_T */

#ifndef struct_tag_rRyTfTxtWo25QxHZmEmNjC
#define struct_tag_rRyTfTxtWo25QxHZmEmNjC

struct tag_rRyTfTxtWo25QxHZmEmNjC
{
  int32_T isInitialized;
  real_T Ts;
  real_T m;
  real_T Inorm[9];
  real_T G[361];
  real_T XOld[19];
  real_T POld[361];
  real_T count;
  real_T P0[361];
};

#endif                                 /* struct_tag_rRyTfTxtWo25QxHZmEmNjC */

#ifndef typedef_KalmanFilter_DIY_MPCDis_raspb_T
#define typedef_KalmanFilter_DIY_MPCDis_raspb_T

typedef struct tag_rRyTfTxtWo25QxHZmEmNjC KalmanFilter_DIY_MPCDis_raspb_T;

#endif                             /* typedef_KalmanFilter_DIY_MPCDis_raspb_T */

#ifndef struct_tag_7Df0vFfPsmwslC6MPptBcE
#define struct_tag_7Df0vFfPsmwslC6MPptBcE

struct tag_7Df0vFfPsmwslC6MPptBcE
{
  int32_T isInitialized;
  real_T Inorm[9];
  real_T m;
  real_T Ts;
  real_T hIni;
};

#endif                                 /* struct_tag_7Df0vFfPsmwslC6MPptBcE */

#ifndef typedef_ssModelgen_estDis_raspberrypi_T
#define typedef_ssModelgen_estDis_raspberrypi_T

typedef struct tag_7Df0vFfPsmwslC6MPptBcE ssModelgen_estDis_raspberrypi_T;

#endif                             /* typedef_ssModelgen_estDis_raspberrypi_T */

#ifndef struct_tag_tu2eCFDm2qezACZMxEOEYF
#define struct_tag_tu2eCFDm2qezACZMxEOEYF

struct tag_tu2eCFDm2qezACZMxEOEYF
{
  int32_T isInitialized;
  real_T r0[3];
  real_T theta0[3];
  real_T dr0[3];
  real_T omega0[3];
  real_T desHeight;
  real_T sitaErr_K[3];
  real_T dt;
  real_T refSeqOld[78];
  real_T sitaErrOld[3];
  real_T sitaZOld;
};

#endif                                 /* struct_tag_tu2eCFDm2qezACZMxEOEYF */

#ifndef typedef_refTrajectory_v4_raspberrypi__T
#define typedef_refTrajectory_v4_raspberrypi__T

typedef struct tag_tu2eCFDm2qezACZMxEOEYF refTrajectory_v4_raspberrypi__T;

#endif                             /* typedef_refTrajectory_v4_raspberrypi__T */

#ifndef struct_tag_i5ObSLC60TZVDCRMArf1j
#define struct_tag_i5ObSLC60TZVDCRMArf1j

struct tag_i5ObSLC60TZVDCRMArf1j
{
  real_T yWidth;
  real_T xWidth;
  real_T pArrayOld[12];
  real_T vCoMRec[63];
  real_T ymOld[6];
  real_T iniCount;
};

#endif                                 /* struct_tag_i5ObSLC60TZVDCRMArf1j */

#ifndef typedef_estX_byJoint_raspberrypi_mult_T
#define typedef_estX_byJoint_raspberrypi_mult_T

typedef struct tag_i5ObSLC60TZVDCRMArf1j estX_byJoint_raspberrypi_mult_T;

#endif                             /* typedef_estX_byJoint_raspberrypi_mult_T */

#ifndef struct_tag_TNNIvevZVZiKmr8oXLxJiB
#define struct_tag_TNNIvevZVZiKmr8oXLxJiB

struct tag_TNNIvevZVZiKmr8oXLxJiB
{
  int32_T isInitialized;
  real_T Ts;
  real_T A[81];
  real_T B[27];
  real_T C[54];
  real_T G[81];
  real_T XOld[9];
  real_T POld[81];
  real_T count;
};

#endif                                 /* struct_tag_TNNIvevZVZiKmr8oXLxJiB */

#ifndef typedef_KalmanFilter_DIY_Offrm_raspbe_T
#define typedef_KalmanFilter_DIY_Offrm_raspbe_T

typedef struct tag_TNNIvevZVZiKmr8oXLxJiB KalmanFilter_DIY_Offrm_raspbe_T;

#endif                             /* typedef_KalmanFilter_DIY_Offrm_raspbe_T */

#ifndef struct_tag_egWwGDcOAGN6JFv14K7bk
#define struct_tag_egWwGDcOAGN6JFv14K7bk

struct tag_egWwGDcOAGN6JFv14K7bk
{
  real_T SWOld[4];
  real_T SPOld[12];
  real_T count;
};

#endif                                 /* struct_tag_egWwGDcOAGN6JFv14K7bk */

#ifndef typedef_LegStateIndicator_raspberrypi_T
#define typedef_LegStateIndicator_raspberrypi_T

typedef struct tag_egWwGDcOAGN6JFv14K7bk LegStateIndicator_raspberrypi_T;

#endif                             /* typedef_LegStateIndicator_raspberrypi_T */

/* Parameters (default storage) */
typedef struct P_raspberrypi_multicore_MPCte_T_ P_raspberrypi_multicore_MPCte_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_raspberrypi_multicore_T RT_MODEL_raspberrypi_multicor_T;

#endif
/* RTW_HEADER_raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis_types_h_ */
