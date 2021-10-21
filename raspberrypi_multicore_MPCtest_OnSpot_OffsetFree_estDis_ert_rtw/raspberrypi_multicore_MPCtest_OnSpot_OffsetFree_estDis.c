/*
 * raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis.c
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

#include "raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis.h"
#include "raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis_private.h"

/* Named constants for MATLAB Function: '<S46>/FixedHorizonOptimizer' */
#define raspberrypi_multic_enable_value (0.0)
#define raspberrypi_multicore_MPCtes_nu (12.0)
#define raspberrypi_multicore_MPCtes_ny (13.0)
#define raspberrypi_multicore_MPCtest_p (6.0)
#define raspberrypi_multicore_M_degrees (25.0)

/* Named constants for Chart: '<Root>/Chart' */
#define raspberrypi_mu_IN_MPC_enable_p1 (1U)
#define raspberrypi_mu_IN_MPC_enable_p2 (2U)
#define raspberrypi_mult_IN_step_enable (4U)
#define raspberrypi_multicor_IN_StandBy (3U)

/* Block signals (default storage) */
B_raspberrypi_multicore_MPCte_T raspberrypi_multicore_MPCtest_B;

/* Block states (default storage) */
DW_raspberrypi_multicore_MPCt_T raspberrypi_multicore_MPCtes_DW;

/* Real-time model */
static RT_MODEL_raspberrypi_multicor_T raspberrypi_multicore_MPCtes_M_;
RT_MODEL_raspberrypi_multicor_T *const raspberrypi_multicore_MPCtes_M =
  &raspberrypi_multicore_MPCtes_M_;

/* Forward declaration for local functions */
static void ras_UDP_decoder_raspi_setupImpl(UDP_decoder_raspi_raspberrypi_T *obj);
static IK_raspberrypi_multicore_MPCt_T *raspberrypi_multicore_MPC_IK_IK
  (IK_raspberrypi_multicore_MPCt_T *obj);
static void raspberrypi_mu_SystemCore_setup(codertarget_raspi_internal_SP_T *obj);
static void KalmanFilter_DIY_Offrm_setupImp(KalmanFilter_DIY_Offrm_raspbe_T *obj);
static message_decoder_PC_raspberryp_T *message_decoder_PC_message_deco
  (message_decoder_PC_raspberryp_T *obj);
static real_T raspberrypi_multicore_M_xnrm2_l(int32_T n, const real_T x[9],
  int32_T ix0);
static void raspberrypi_multicore_MPC_xscal(int32_T n, real_T a, real_T x[9],
  int32_T ix0);
static real_T raspberrypi_multicore__xnrm2_lk(const real_T x[3], int32_T ix0);
static void raspberrypi_multicore_M_xscal_p(real_T a, real_T x[3], int32_T ix0);
static void raspberrypi_multicore__xaxpy_ml(int32_T n, real_T a, const real_T x
  [3], int32_T ix0, real_T y[9], int32_T iy0);
static void raspberrypi_multicore_M_xaxpy_m(int32_T n, real_T a, const real_T x
  [9], int32_T ix0, real_T y[3], int32_T iy0);
static real_T raspberrypi_multicore_MPC_xdotc(int32_T n, const real_T x[9],
  int32_T ix0, const real_T y[9], int32_T iy0);
static void raspberrypi_multicore_MPC_xaxpy(int32_T n, real_T a, int32_T ix0,
  real_T y[9], int32_T iy0);
static void raspberrypi_multicore__xscal_pn(real_T a, real_T x[9], int32_T ix0);
static void raspberrypi_multicore_MPC_xswap(real_T x[9], int32_T ix0, int32_T
  iy0);
static void raspberrypi_multicore_MPC_xrotg(real_T *a, real_T *b, real_T *c,
  real_T *s);
static void raspberrypi_multicore_MPCt_xrot(real_T x[9], int32_T ix0, int32_T
  iy0, real_T c, real_T s);
static void raspberrypi_multicore_MPCte_svd(const real_T A[9], real_T U[9],
  real_T s[3], real_T V[9]);
static real_T raspberrypi_multicore_MP_norm_k(const real_T x[3]);
static void raspberrypi_multico_Get1From2_a(real_T x1, real_T b_y1, real_T x2,
  real_T y2, real_T l1, real_T l2, real_T limit, real_T *xout, real_T *yout,
  real_T *Flag);
static void raspberryp_AdmittanceCtr_IK_one(const
  AdmittanceCtr_raspberrypi_mul_T *obj, const real_T p[3], real_T LegNum, real_T
  Angle[3], real_T *Flag);
static boolean_T raspberrypi_multicore_M_isequal(const real_T varargin_1[3],
  const real_T varargin_2[3]);
static real_T raspberrypi_multicore_factorial(real_T n);
static real_T raspberrypi_multicore_MP_median(const real_T x[3]);
static void raspberrypi_multicore_Get1From2(real_T x2, real_T y2, real_T l1,
  real_T l2, real_T limit, real_T *xout, real_T *yout, real_T *Flag);
static void raspberrypi_multicore_IK_IK_one(const
  IK_raspberrypi_multicore_MPCt_T *obj, const real_T p[3], real_T LegNum, real_T
  Angle[3], real_T *Flag);
static void raspberrypi_multicore_FK_FK_one(const
  FK_raspberrypi_multicore_MPCt_T *obj, const real_T Angle[3], real_T LegNum,
  real_T pP[3], real_T *Flag);
static void raspberrypi_multicore_M_mrdiv_a(const real_T A[54], const real_T B
  [36], real_T Y[54]);
static void KalmanFilter_DIY_Offrm_stepImpl(KalmanFilter_DIY_Offrm_raspbe_T *obj,
  const real_T u[3], const real_T y[6], const real_T x0[9], const real_T p0[81],
  const real_T Q[81], const real_T R[36], real_T Reset, real_T updateEN, real_T
  xhat[9], real_T P[81]);
static void raspberrypi_multicore_MP_DS_gen(real_T Ts, real_T m, const real_T
  in3[3], const real_T in4[9], const real_T in5[3], const real_T in6[3], const
  real_T in7[3], const real_T in8[3], real_T Ad[169], real_T Bd[156]);
static void raspberrypi_multicore_MP_xgetrf(const real_T A[169], real_T b_A[169],
  int32_T ipiv[13], int32_T *info);
static void raspberrypi_multicore_MPC_mrdiv(const real_T A[247], const real_T B
  [169], real_T Y[247]);
static void KalmanFilter_DIY_MPCDis_stepImp(KalmanFilter_DIY_MPCDis_raspb_T *obj,
  const real_T U_MPC[12], const real_T xFB[13], const real_T pW[12], const
  real_T Q[361], const real_T R[169], real_T Reset, real_T estXbar[19], real_T
  P[361]);
static void raspberrypi_mul_SystemCore_step(KalmanFilter_DIY_MPCDis_raspb_T *obj,
  const real_T varargin_1[12], const real_T varargin_2[13], const real_T
  varargin_3[12], const real_T varargin_4[361], const real_T varargin_5[169],
  real_T varargin_6, real_T varargout_1[19], real_T varargout_2[361]);
static real_T raspberrypi_multicore__norm_amt(const real_T x[3]);
static void raspberrypi__SystemCore_step_am(refTrajectory_v4_raspberrypi__T *obj,
  real_T varargin_1, real_T varargin_2, real_T varargin_3, const real_T
  varargin_4[3], const real_T varargin_5[3], const real_T varargin_6[13], real_T
  varargin_7, real_T varargout_1[78], real_T varargout_2[13], real_T
  varargout_3[78], real_T varargout_4[3], real_T varargout_5[3]);
static void ra_ssModelgen_estDis_stepImpl_a(const
  ssModelgen_estDis_raspberrypi_T *obj, const real_T PendAll[12], const real_T
  xFB[13], const real_T SPLeg[4], real_T Anew[169], real_T Bnew[234], real_T U
  [18], real_T Y[13], real_T DX[13], real_T Inow[9]);
static void raspberrypi_m_SystemCore_step_a(const
  ssModelgen_estDis_raspberrypi_T *obj, const real_T varargin_1[12], const
  real_T varargin_2[13], const real_T varargin_3[4], real_T varargout_1[169],
  real_T varargout_2[234], real_T varargout_5[13], real_T varargout_6[18],
  real_T varargout_7[13], real_T varargout_8[13], real_T varargout_9[9]);
static void raspberrypi_mul_mpc_plantupdate(const real_T a[169], real_T b[234],
  real_T c[169], real_T d[234], real_T b_A[169], real_T b_B[416], real_T b_C[169],
  real_T b_D[416], const real_T b_mvindex[12], const real_T b_mdindex[6], const
  real_T b_myindex[13], const real_T b_Uscale[18], const real_T b_Yscale[13],
  real_T Bu[156], real_T Bv[91], real_T Cm[169], real_T Dv[91], real_T Dvm[91],
  real_T QQ[169], real_T RR[169], real_T NN[169]);
static real_T raspberrypi_multicore_MPC_mod_p(real_T x);
static real_T raspberrypi_multicore_MPCte_mod(real_T x);
static void raspberry_mpc_updateFromNominal(real_T b_Mlim[252], const real_T
  b_Mrows[28], const real_T U0[18], const real_T b_Uscale[18], const real_T
  old_mvoff[12], const real_T b_mvindex[12], const real_T b_mdindex[6], real_T
  b_utarget[72], const real_T Y0[13], const real_T b_Yscale[13], const real_T
  old_yoff[13], const real_T b_myindex[13], const real_T X0[13], real_T b_xoff
  [13], const real_T DX0[13], real_T Bv[637], real_T new_mvoff[12], real_T
  new_mdoff[6], real_T new_yoff[13], real_T new_myoff[13]);
static void raspberrypi__mpc_constraintcoef(const real_T b_A[169], const real_T
  Bu[156], const real_T Bv[91], const real_T b_C[169], const real_T Dv[91],
  const real_T b_Jm[1728], real_T b_SuJm[1872], real_T b_Sx[1014], real_T b_Su1
  [936], real_T b_Hv[3822]);
static void raspbe_mpc_customconstraintcoef(const real_T b_SuJm[1872], const
  real_T b_Sx[1014], const real_T b_Su1[936], const real_T b_Hv[3822], const
  real_T b_C[169], const real_T Dv[637], const real_T b_Jm[1728], const real_T
  E[384], const real_T F[416], const real_T S[192], const real_T G[32], const
  real_T mvoff[12], const real_T mdoff[6], const real_T b_yoff[13], real_T Mu
  [5376], real_T b_Mv[10976], real_T b_Mu1[2688], real_T b_Mx[2912], real_T
  b_Mlim[224]);
static void raspberrypi_multicore_MPCt_kron(const real_T b_A[36], const real_T
  b_B[144], real_T K[5184]);
static void raspberrypi_multicore_MP_WtMult(const real_T W[12], const real_T M
  [1728], real_T nwt, real_T WM[1728]);
static void raspberryp_mpc_calculatehessian(const real_T b_Wy[78], const real_T
  b_Wu[12], const real_T b_Wdu[12], const real_T b_SuJm[1872], const real_T
  I2Jm[1728], const real_T b_Jm[1728], const real_T b_I1[864], const real_T
  b_Su1[936], const real_T b_Sx[1014], const real_T b_Hv[3822], real_T nmv,
  real_T b_ny, real_T b_H[576], real_T b_Ku1[288], real_T b_Kut[1728], real_T
  b_Kx[312], real_T b_Kv[1176], real_T b_Kr[1872]);
static int32_T raspberrypi_multicore_MP_xpotrf(real_T b_A[625]);
static real_T raspberrypi_multicore_M_minimum(const real_T x[25]);
static void raspberrypi_mu_mpc_checkhessian(real_T b_H[625], real_T L[625],
  real_T *BadH);
static void raspberrypi_multicore__trisolve(const real_T b_A[625], real_T b_B
  [625]);
static void raspberrypi_multi_Unconstrained(const real_T b_Hinv[625], const
  real_T f[25], real_T x[25], int16_T n);
static real_T raspberrypi_multicore_MPCt_norm(const real_T x[25]);
static void raspberrypi_multicore_MPCte_abs(const real_T x[25], real_T y[25]);
static real_T raspberrypi_multicore_M_maximum(const real_T x[25]);
static void raspberrypi_multicore_MPC_abs_d(const real_T x[252], real_T y[252]);
static void raspberrypi_multicore__maximum2(const real_T x[252], real_T y,
  real_T ex[252]);
static real_T raspberrypi_multicore_MPC_xnrm2(int32_T n, const real_T x[625],
  int32_T ix0);
static void raspberrypi_multicore_MPC_xgemv(int32_T b_m, int32_T n, const real_T
  b_A[625], int32_T ia0, const real_T x[625], int32_T ix0, real_T y[25]);
static void raspberrypi_multicore_MPC_xgerc(int32_T b_m, int32_T n, real_T
  alpha1, int32_T ix0, const real_T y[25], real_T b_A[625], int32_T ia0);
static void raspberrypi_multicore_MP_xzlarf(int32_T b_m, int32_T n, int32_T iv0,
  real_T tau, real_T b_C[625], int32_T ic0, real_T work[25]);
static void raspberrypi_multicore_MPCte_qrf(real_T b_A[625], int32_T ia0,
  int32_T b_m, int32_T n, int32_T nfxd, real_T tau[25]);
static void raspberrypi_multicore_MP_xgeqrf(real_T b_A[625], real_T tau[25]);
static void raspberrypi_multicore_MP_xorgqr(int32_T b_m, int32_T n, int32_T k,
  real_T b_A[625], int32_T ia0, const real_T tau[25], int32_T itau0);
static void raspberrypi_multicore_MPCtes_qr(const real_T b_A[625], real_T Q[625],
  real_T R[625]);
static real_T raspberrypi_multicor_KWIKfactor(const real_T b_Ac[6300], const
  int16_T iC[252], int16_T nA, const real_T b_Linv[625], real_T RLinv[625],
  real_T b_D[625], real_T b_H[625], int16_T n);
static real_T raspberrypi_multicore_MP_mtimes(const real_T b_A[25], const real_T
  b_B[25]);
static void raspberrypi_mult_DropConstraint(int16_T kDrop, int16_T iA[252],
  int16_T *nA, int16_T iC[252]);
static void raspberrypi_multicore_MP_qpkwik(const real_T b_Linv[625], const
  real_T b_Hinv[625], const real_T f[25], const real_T b_Ac[6300], const real_T
  b[252], int16_T iA[252], int16_T maxiter, real_T FeasTol, real_T x[25], real_T
  lambda[252], real_T *status);
static void raspberrypi_multico_mpc_solveQP(const real_T xQP[13], const real_T
  b_Kx[312], const real_T b_Kr[1872], const real_T rseq[78], const real_T b_Ku1
  [288], const real_T old_u[12], const real_T b_Kv[1176], const real_T vseq[49],
  const real_T b_Kut[1728], const real_T b_utarget[72], const real_T b_Linv[625],
  const real_T b_Hinv[625], const real_T b_Ac[6300], const real_T Bc[252],
  boolean_T iA[252], real_T zopt[25], real_T f[25], real_T *status);
static void raspberrypi_multic_mpc_clipping(const real_T uk[12], const real_T
  uk1[12], const real_T b_uoff[12], const real_T b_Mlim[252], const real_T
  b_Mrows[28], real_T b_p, real_T b_ny, real_T b_degrees, real_T uk_clipped[12]);
static void raspberrypi__mpcblock_optimizer(const real_T rseq[78], const real_T
  vseq[49], real_T switch_in, const real_T x[13], const real_T old_u[12], const
  boolean_T iA[252], real_T b_Mlim[252], real_T b_Mx[3276], real_T b_Mu1[3024],
  real_T b_Mv[12348], const real_T b_utarget[72], const real_T b_uoff[12], const
  real_T b_voff[6], const real_T b_yoff[13], real_T b_enable_value, real_T b_H
  [625], real_T b_Ac[6300], const real_T b_Wy[78], const real_T b_Wdu[12], const
  real_T b_Jm[1728], const real_T b_Wu[12], const real_T b_I1[864], const real_T
  b_A[169], const real_T Bu[1092], const real_T Bv[637], const real_T b_C[169],
  const real_T Dv[637], const real_T b_Mrows[28], const real_T b_Ecc[384], const
  real_T b_Fcc[416], const real_T b_Scc[192], const real_T b_Gcc[32], real_T u
  [12], real_T useq[84], real_T *status, boolean_T iAout[252]);
static void raspberrypi_multicore_MP_repmat(const real_T a[13], real_T b[91]);
static void raspberrypi_mpc_computeSequence(const real_T x[13], const real_T
  uopt[84], const real_T v[49], const real_T mvoff[12], const real_T b_yoff[13],
  const real_T b_xoff[13], const real_T b_A[169], const real_T Bu[1092], const
  real_T Bv[637], const real_T b_C[169], const real_T Dv[637], real_T yopt[91],
  real_T xopt[91]);

/*
 * Writes out MAT-file header.  Returns success or failure.
 * Returns:
 *      0 - success
 *      1 - failure
 */
int_T rt_WriteMat4FileHeader(FILE *fp, int32_T m, int32_T n, const char *name)
{
  typedef enum { ELITTLE_ENDIAN, EBIG_ENDIAN } ByteOrder;

  int16_T one = 1;
  ByteOrder byteOrder = (*((int8_T *)&one)==1) ? ELITTLE_ENDIAN : EBIG_ENDIAN;
  int32_T type = (byteOrder == ELITTLE_ENDIAN) ? 0: 1000;
  int32_T imagf = 0;
  int32_T name_len = (int32_T)strlen(name) + 1;
  if ((fwrite(&type, sizeof(int32_T), 1, fp) == 0) ||
      (fwrite(&m, sizeof(int32_T), 1, fp) == 0) ||
      (fwrite(&n, sizeof(int32_T), 1, fp) == 0) ||
      (fwrite(&imagf, sizeof(int32_T), 1, fp) == 0) ||
      (fwrite(&name_len, sizeof(int32_T), 1, fp) == 0) ||
      (fwrite(name, sizeof(char), name_len, fp) == 0)) {
    return(1);
  } else {
    return(0);
  }
}                                      /* end rt_WriteMat4FileHeader */

/*
 * Set which subrates need to run this base step (base rate always runs).
 * This function must be called prior to calling the model step function
 * in order to "remember" which rates need to run this base step.  The
 * buffering of events allows for overlapping preemption.
 */
void
  raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis_SetEventsForThisBaseStep
  (boolean_T *eventFlags)
{
  /* Task runs when its counter is zero, computed via rtmStepTask macro */
  eventFlags[1] = ((boolean_T)rtmStepTask(raspberrypi_multicore_MPCtes_M, 1));
}

/*
 *   This function updates active task flag for each subrate.
 * The function is called in the model base rate function.
 * It maintains SampleHit information to allow scheduling
 * of the subrates from the base rate function.
 */
void rate_scheduler(void)
{
  /* Compute which subrates run during the next base time step.  Subrates
   * are an integer multiple of the base rate counter.  Therefore, the subtask
   * counter is reset when it reaches its limit (zero means run).
   */
  (raspberrypi_multicore_MPCtes_M->Timing.TaskCounters.TID[1])++;
  if ((raspberrypi_multicore_MPCtes_M->Timing.TaskCounters.TID[1]) > 4) {/* Sample time: [0.025s, 0.0s] */
    raspberrypi_multicore_MPCtes_M->Timing.TaskCounters.TID[1] = 0;
  }
}

/*
 * Start for atomic system:
 *    synthesized block
 *    synthesized block
 */
void raspber_MATLABSystem4_Start(DW_MATLABSystem4_raspberrypi__T *localDW)
{
  /* Start for MATLABSystem: '<Root>/MATLAB System4' */
  localDW->objisempty = true;

  /*  Perform one-time calculations, such as computing constants */
  localDW->obj.dataOld = 0.0;
}

/*
 * Output and update for atomic system:
 *    synthesized block
 *    synthesized block
 */
void raspberrypi_m_MATLABSystem4(real_T rtu_0, B_MATLABSystem4_raspberrypi_m_T
  *localB, DW_MATLABSystem4_raspberrypi__T *localDW)
{
  /* MATLABSystem: '<Root>/MATLAB System4' */
  /*  NOTE: do NOT add vpxAdd and vpyAdd during start and stop */
  localB->MATLABSystem4 = (rtu_0 > localDW->obj.dataOld);

  /* MATLABSystem: '<Root>/MATLAB System4' */
  localDW->obj.dataOld = rtu_0;
}

static void ras_UDP_decoder_raspi_setupImpl(UDP_decoder_raspi_raspberrypi_T *obj)
{
  real_T PendAlltmp_tmp;
  real_T PendAlltmp_tmp_0;
  real_T PendAlltmp_tmp_1;
  int32_T i;
  static const int8_T tmp[12] = { 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1 };

  static const int8_T tmp_0[12] = { 0, 1, 0, 0, -1, 0, 0, 1, 0, 0, -1, 0 };

  memcpy(&obj->Inow_Old[0], &obj->Inorm[0], 9U * sizeof(real_T));

  /*              obj.PendAllnorm=[0.1075,0.1075,-0.1075,-0.1075; */
  /*                  0.0750,-0.0750,0.0750,-0.0750; */
  /*                  0,0,0,0]; */
  PendAlltmp_tmp = obj->lateral_width / 2.0;
  raspberrypi_multicore_MPCtest_B.PendAlltmp[0] = PendAlltmp_tmp;
  PendAlltmp_tmp_0 = obj->sagetial_width / 2.0;
  raspberrypi_multicore_MPCtest_B.PendAlltmp[1] = PendAlltmp_tmp_0;
  raspberrypi_multicore_MPCtest_B.PendAlltmp[2] = 0.0;
  raspberrypi_multicore_MPCtest_B.PendAlltmp[3] = PendAlltmp_tmp;
  PendAlltmp_tmp = -obj->sagetial_width / 2.0;
  raspberrypi_multicore_MPCtest_B.PendAlltmp[4] = PendAlltmp_tmp;
  raspberrypi_multicore_MPCtest_B.PendAlltmp[5] = 0.0;
  PendAlltmp_tmp_1 = -obj->lateral_width / 2.0;
  raspberrypi_multicore_MPCtest_B.PendAlltmp[6] = PendAlltmp_tmp_1;
  raspberrypi_multicore_MPCtest_B.PendAlltmp[7] = PendAlltmp_tmp_0;
  raspberrypi_multicore_MPCtest_B.PendAlltmp[8] = 0.0;
  raspberrypi_multicore_MPCtest_B.PendAlltmp[9] = PendAlltmp_tmp_1;
  raspberrypi_multicore_MPCtest_B.PendAlltmp[10] = PendAlltmp_tmp;
  raspberrypi_multicore_MPCtest_B.PendAlltmp[11] = 0.0;
  for (i = 0; i < 12; i++) {
    obj->U_Old[i] = (real_T)tmp[i] * obj->m / 4.0 * 9.8;
    obj->SP_MPC_Old[i] = (real_T)tmp_0[i] * obj->roll_Off +
      raspberrypi_multicore_MPCtest_B.PendAlltmp[i];
  }

  obj->X_mpc_Old[2] = obj->hIni;
  obj->refP_Old[2] = obj->hIni;
  obj->MPC_Count_Old = 0.0;
}

static IK_raspberrypi_multicore_MPCt_T *raspberrypi_multicore_MPC_IK_IK
  (IK_raspberrypi_multicore_MPCt_T *obj)
{
  int32_T i;
  static const int16_T tmp[12] = { 0, 0, -190, 0, 0, -190, 0, 0, -190, 0, 0,
    -190 };

  obj->AB = 44.5;
  obj->BC = 120.0;
  obj->CDP = 2.8770007389874528;
  obj->DP = 139.063;
  obj->OR = 37.0;
  obj->PxLim[0] = -70.0;
  obj->PxLim[1] = 70.0;
  obj->PyLim[0] = -20.0;
  obj->PyLim[1] = 100.0;
  obj->PzLim[0] = -140.0;
  obj->PzLim[1] = -230.0;
  for (i = 0; i < 12; i++) {
    obj->LastInput[i] = tmp[i];
  }

  for (i = 0; i < 12; i++) {
    obj->AngleOff[i] = 0.0;
  }

  obj->isInitialized = 0;
  return obj;
}

static void raspberrypi_mu_SystemCore_setup(codertarget_raspi_internal_SP_T *obj)
{
  uint32_T MISOPinLoc;
  uint32_T MOSIPinLoc;
  uint32_T SCKPinLoc;
  uint32_T SSPinNameLoc;
  obj->isSetupComplete = false;
  obj->isInitialized = 1;
  SETSPI0CE0SPEED();
  SETSPI0CE1SPEED();
  SSPinNameLoc = SPI0_CE0;
  MOSIPinLoc = MW_UNDEFINED_VALUE;
  MISOPinLoc = MW_UNDEFINED_VALUE;
  SCKPinLoc = MW_UNDEFINED_VALUE;
  obj->MW_SPI_HANDLE = MW_SPI_Open(0U, MOSIPinLoc, MISOPinLoc, SCKPinLoc,
    SSPinNameLoc, true, 0);
  MW_SPI_SetBusSpeed(obj->MW_SPI_HANDLE, 500000U);
  obj->isSetupComplete = true;
}

static void KalmanFilter_DIY_Offrm_setupImp(KalmanFilter_DIY_Offrm_raspbe_T *obj)
{
  real_T a_tmp;
  int32_T b_k;
  int32_T i;
  int32_T tmp;
  int32_T tmp_0;
  int32_T tmp_1;
  int32_T tmp_2;
  int32_T tmp_3;
  int32_T tmp_4;
  static const int8_T tmp_5[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  real_T tmp_6;

  /*  Perform one-time calculations, such as computing constants */
  a_tmp = obj->Ts * obj->Ts * 0.5;
  for (i = 0; i < 9; i++) {
    raspberrypi_multicore_MPCtest_B.b_I_h[i] = 0;
  }

  raspberrypi_multicore_MPCtest_B.b_I_h[0] = 1;
  raspberrypi_multicore_MPCtest_B.b_I_h[4] = 1;
  raspberrypi_multicore_MPCtest_B.b_I_h[8] = 1;
  for (i = 0; i < 9; i++) {
    raspberrypi_multicore_MPCtest_B.b_I_c[i] = 0;
  }

  raspberrypi_multicore_MPCtest_B.b_I_c[0] = 1;
  raspberrypi_multicore_MPCtest_B.b_I_c[4] = 1;
  raspberrypi_multicore_MPCtest_B.b_I_c[8] = 1;
  for (i = 0; i < 9; i++) {
    raspberrypi_multicore_MPCtest_B.b_I_k[i] = 0;
  }

  raspberrypi_multicore_MPCtest_B.b_I_k[0] = 1;
  raspberrypi_multicore_MPCtest_B.b_I_k[4] = 1;
  raspberrypi_multicore_MPCtest_B.b_I_k[8] = 1;
  for (i = 0; i < 9; i++) {
    raspberrypi_multicore_MPCtest_B.dv25[i] = (real_T)tmp_5[i] * obj->Ts;
  }

  for (i = 0; i < 3; i++) {
    tmp = tmp_5[3 * i];
    raspberrypi_multicore_MPCtest_B.d2 = raspberrypi_multicore_MPCtest_B.dv25[3 *
      i];
    obj->A[9 * i] = raspberrypi_multicore_MPCtest_B.b_I_h[3 * i];
    tmp_0 = 9 * (i + 3);
    obj->A[tmp_0] = raspberrypi_multicore_MPCtest_B.d2;
    tmp_1 = 9 * (i + 6);
    obj->A[tmp_1] = a_tmp * (real_T)tmp;
    b_k = 9 * i + 3;
    obj->A[b_k] = 0.0;
    obj->A[tmp_0 + 3] = raspberrypi_multicore_MPCtest_B.b_I_c[3 * i];
    obj->A[tmp_1 + 3] = raspberrypi_multicore_MPCtest_B.d2;
    tmp_2 = 9 * i + 6;
    obj->A[tmp_2] = 0.0;
    obj->A[tmp_0 + 6] = 0.0;
    obj->A[tmp_1 + 6] = raspberrypi_multicore_MPCtest_B.b_I_k[3 * i];
    obj->B[9 * i] = a_tmp * (real_T)tmp;
    obj->B[b_k] = obj->Ts * (real_T)tmp;
    obj->B[tmp_2] = 0.0;
    b_k = 3 * i + 1;
    tmp = tmp_5[b_k];
    raspberrypi_multicore_MPCtest_B.d2 =
      raspberrypi_multicore_MPCtest_B.dv25[b_k];
    tmp_2 = 9 * i + 1;
    obj->A[tmp_2] = raspberrypi_multicore_MPCtest_B.b_I_h[b_k];
    obj->A[tmp_0 + 1] = raspberrypi_multicore_MPCtest_B.d2;
    tmp_6 = a_tmp * (real_T)tmp;
    obj->A[tmp_1 + 1] = tmp_6;
    tmp_3 = 9 * i + 4;
    obj->A[tmp_3] = 0.0;
    obj->A[tmp_0 + 4] = raspberrypi_multicore_MPCtest_B.b_I_c[b_k];
    obj->A[tmp_1 + 4] = raspberrypi_multicore_MPCtest_B.d2;
    tmp_4 = 9 * i + 7;
    obj->A[tmp_4] = 0.0;
    obj->A[tmp_0 + 7] = 0.0;
    obj->A[tmp_1 + 7] = raspberrypi_multicore_MPCtest_B.b_I_k[b_k];
    obj->B[tmp_2] = tmp_6;
    obj->B[tmp_3] = obj->Ts * (real_T)tmp;
    obj->B[tmp_4] = 0.0;
    b_k = 3 * i + 2;
    tmp = tmp_5[b_k];
    raspberrypi_multicore_MPCtest_B.d2 =
      raspberrypi_multicore_MPCtest_B.dv25[b_k];
    tmp_2 = 9 * i + 2;
    obj->A[tmp_2] = raspberrypi_multicore_MPCtest_B.b_I_h[b_k];
    obj->A[tmp_0 + 2] = raspberrypi_multicore_MPCtest_B.d2;
    tmp_6 = a_tmp * (real_T)tmp;
    obj->A[tmp_1 + 2] = tmp_6;
    tmp_3 = 9 * i + 5;
    obj->A[tmp_3] = 0.0;
    obj->A[tmp_0 + 5] = raspberrypi_multicore_MPCtest_B.b_I_c[b_k];
    obj->A[tmp_1 + 5] = raspberrypi_multicore_MPCtest_B.d2;
    tmp_4 = 9 * i + 8;
    obj->A[tmp_4] = 0.0;
    obj->A[tmp_0 + 8] = 0.0;
    obj->A[tmp_1 + 8] = raspberrypi_multicore_MPCtest_B.b_I_k[b_k];
    obj->B[tmp_2] = tmp_6;
    obj->B[tmp_3] = obj->Ts * (real_T)tmp;
    obj->B[tmp_4] = 0.0;
  }

  for (i = 0; i < 9; i++) {
    raspberrypi_multicore_MPCtest_B.b_I_h[i] = 0;
  }

  raspberrypi_multicore_MPCtest_B.b_I_h[0] = 1;
  raspberrypi_multicore_MPCtest_B.b_I_h[4] = 1;
  raspberrypi_multicore_MPCtest_B.b_I_h[8] = 1;
  for (i = 0; i < 9; i++) {
    raspberrypi_multicore_MPCtest_B.b_I_c[i] = 0;
  }

  for (b_k = 0; b_k < 3; b_k++) {
    raspberrypi_multicore_MPCtest_B.b_I_c[b_k + 3 * b_k] = 1;
    obj->C[6 * b_k] = raspberrypi_multicore_MPCtest_B.b_I_h[3 * b_k];
    i = 6 * (b_k + 3);
    obj->C[i] = 0.0;
    tmp = 6 * (b_k + 6);
    obj->C[tmp] = 0.0;
    obj->C[6 * b_k + 3] = 0.0;
    tmp_0 = 3 * b_k + 1;
    obj->C[6 * b_k + 1] = raspberrypi_multicore_MPCtest_B.b_I_h[tmp_0];
    obj->C[i + 1] = 0.0;
    obj->C[tmp + 1] = 0.0;
    obj->C[6 * b_k + 4] = 0.0;
    tmp_1 = 3 * b_k + 2;
    obj->C[6 * b_k + 2] = raspberrypi_multicore_MPCtest_B.b_I_h[tmp_1];
    obj->C[i + 2] = 0.0;
    obj->C[tmp + 2] = 0.0;
    obj->C[6 * b_k + 5] = 0.0;
    obj->C[i + 3] = raspberrypi_multicore_MPCtest_B.b_I_c[3 * b_k];
    obj->C[tmp + 3] = 0.0;
    obj->C[i + 4] = raspberrypi_multicore_MPCtest_B.b_I_c[tmp_0];
    obj->C[tmp + 4] = 0.0;
    obj->C[i + 5] = raspberrypi_multicore_MPCtest_B.b_I_c[tmp_1];
    obj->C[tmp + 5] = 0.0;
  }

  obj->count = 0.0;
  memset(&obj->XOld[0], 0, 9U * sizeof(real_T));
  memset(&obj->POld[0], 0, 81U * sizeof(real_T));
  for (b_k = 0; b_k < 9; b_k++) {
    obj->POld[b_k + 9 * b_k] = 1.0;
  }

  memset(&obj->G[0], 0, 81U * sizeof(real_T));
  for (b_k = 0; b_k < 9; b_k++) {
    obj->G[b_k + 9 * b_k] = 1.0;
  }
}

static message_decoder_PC_raspberryp_T *message_decoder_PC_message_deco
  (message_decoder_PC_raspberryp_T *obj)
{
  int32_T i;
  obj->vCoM_Old[0] = 0.0;
  obj->vCoM_Old[1] = 0.0;
  obj->vCoM_Old[2] = 0.0;
  obj->RPYnew_Old[0] = 0.0;
  obj->RPYnew_Old[1] = 0.0;
  obj->RPYnew_Old[2] = 0.0;
  obj->OmegaW_Old[0] = 0.0;
  obj->OmegaW_Old[1] = 0.0;
  obj->OmegaW_Old[2] = 0.0;
  obj->surVN_Old[0] = 0.0;
  obj->surVN_Old[1] = 0.0;
  obj->surVN_Old[2] = 0.0;
  obj->surV1_Old[0] = 0.0;
  obj->surV1_Old[1] = 0.0;
  obj->surV1_Old[2] = 0.0;
  obj->surV2_Old[0] = 0.0;
  obj->surV2_Old[1] = 0.0;
  obj->surV2_Old[2] = 0.0;
  obj->surP_Old[0] = 0.0;
  obj->surP_Old[1] = 0.0;
  obj->surP_Old[2] = 0.0;
  obj->headG_Old[0] = 0.0;
  obj->headG_Old[1] = 0.0;
  obj->headG_Old[2] = 0.0;
  obj->vxPercent_Old = 0.0;
  obj->vyPercent_Old = 0.0;
  obj->wzPercent_Old = 0.0;
  obj->phi_Old = 0.0;
  for (i = 0; i < 6; i++) {
    obj->estDis_Old[i] = 0.0;
  }

  obj->isInitialized = 0;
  return obj;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function2' */
static real_T raspberrypi_multicore_M_xnrm2_l(int32_T n, const real_T x[9],
  int32_T ix0)
{
  real_T t;
  real_T y;
  int32_T k;
  int32_T kend;
  y = 0.0;
  raspberrypi_multicore_MPCtest_B.scale_h = 3.3121686421112381E-170;
  kend = (ix0 + n) - 1;
  for (k = ix0; k <= kend; k++) {
    raspberrypi_multicore_MPCtest_B.absxk_h = fabs(x[k - 1]);
    if (raspberrypi_multicore_MPCtest_B.absxk_h >
        raspberrypi_multicore_MPCtest_B.scale_h) {
      t = raspberrypi_multicore_MPCtest_B.scale_h /
        raspberrypi_multicore_MPCtest_B.absxk_h;
      y = y * t * t + 1.0;
      raspberrypi_multicore_MPCtest_B.scale_h =
        raspberrypi_multicore_MPCtest_B.absxk_h;
    } else {
      t = raspberrypi_multicore_MPCtest_B.absxk_h /
        raspberrypi_multicore_MPCtest_B.scale_h;
      y += t * t;
    }
  }

  return raspberrypi_multicore_MPCtest_B.scale_h * sqrt(y);
}

/* Function for MATLAB Function: '<Root>/MATLAB Function2' */
static void raspberrypi_multicore_MPC_xscal(int32_T n, real_T a, real_T x[9],
  int32_T ix0)
{
  int32_T b;
  int32_T k;
  b = (ix0 + n) - 1;
  for (k = ix0; k <= b; k++) {
    x[k - 1] *= a;
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function2' */
static real_T raspberrypi_multicore__xnrm2_lk(const real_T x[3], int32_T ix0)
{
  real_T absxk;
  real_T t;
  real_T y;
  int32_T k;
  y = 0.0;
  raspberrypi_multicore_MPCtest_B.scale_f = 3.3121686421112381E-170;
  for (k = ix0; k <= ix0 + 1; k++) {
    absxk = fabs(x[k - 1]);
    if (absxk > raspberrypi_multicore_MPCtest_B.scale_f) {
      t = raspberrypi_multicore_MPCtest_B.scale_f / absxk;
      y = y * t * t + 1.0;
      raspberrypi_multicore_MPCtest_B.scale_f = absxk;
    } else {
      t = absxk / raspberrypi_multicore_MPCtest_B.scale_f;
      y += t * t;
    }
  }

  return raspberrypi_multicore_MPCtest_B.scale_f * sqrt(y);
}

/* Function for MATLAB Function: '<Root>/MATLAB Function2' */
static void raspberrypi_multicore_M_xscal_p(real_T a, real_T x[3], int32_T ix0)
{
  int32_T k;
  for (k = ix0; k <= ix0 + 1; k++) {
    x[k - 1] *= a;
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function2' */
static void raspberrypi_multicore__xaxpy_ml(int32_T n, real_T a, const real_T x
  [3], int32_T ix0, real_T y[9], int32_T iy0)
{
  int32_T ix;
  int32_T iy;
  int32_T k;
  if (!(a == 0.0)) {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * x[ix];
      ix++;
      iy++;
    }
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function2' */
static void raspberrypi_multicore_M_xaxpy_m(int32_T n, real_T a, const real_T x
  [9], int32_T ix0, real_T y[3], int32_T iy0)
{
  int32_T ix;
  int32_T iy;
  int32_T k;
  if (!(a == 0.0)) {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * x[ix];
      ix++;
      iy++;
    }
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function2' */
static real_T raspberrypi_multicore_MPC_xdotc(int32_T n, const real_T x[9],
  int32_T ix0, const real_T y[9], int32_T iy0)
{
  real_T d;
  int32_T ix;
  int32_T iy;
  int32_T k;
  d = 0.0;
  ix = ix0;
  iy = iy0;
  for (k = 0; k < n; k++) {
    d += x[ix - 1] * y[iy - 1];
    ix++;
    iy++;
  }

  return d;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function2' */
static void raspberrypi_multicore_MPC_xaxpy(int32_T n, real_T a, int32_T ix0,
  real_T y[9], int32_T iy0)
{
  int32_T ix;
  int32_T iy;
  int32_T k;
  if (!(a == 0.0)) {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * y[ix];
      ix++;
      iy++;
    }
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function2' */
static void raspberrypi_multicore__xscal_pn(real_T a, real_T x[9], int32_T ix0)
{
  int32_T k;
  for (k = ix0; k <= ix0 + 2; k++) {
    x[k - 1] *= a;
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function2' */
static void raspberrypi_multicore_MPC_xswap(real_T x[9], int32_T ix0, int32_T
  iy0)
{
  real_T temp;
  temp = x[ix0 - 1];
  x[ix0 - 1] = x[iy0 - 1];
  x[iy0 - 1] = temp;
  temp = x[ix0];
  x[ix0] = x[iy0];
  x[iy0] = temp;
  temp = x[ix0 + 1];
  x[ix0 + 1] = x[iy0 + 1];
  x[iy0 + 1] = temp;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function2' */
static void raspberrypi_multicore_MPC_xrotg(real_T *a, real_T *b, real_T *c,
  real_T *s)
{
  real_T ads;
  real_T bds;
  raspberrypi_multicore_MPCtest_B.roe = *b;
  raspberrypi_multicore_MPCtest_B.absa = fabs(*a);
  raspberrypi_multicore_MPCtest_B.absb = fabs(*b);
  if (raspberrypi_multicore_MPCtest_B.absa >
      raspberrypi_multicore_MPCtest_B.absb) {
    raspberrypi_multicore_MPCtest_B.roe = *a;
  }

  raspberrypi_multicore_MPCtest_B.scale_k = raspberrypi_multicore_MPCtest_B.absa
    + raspberrypi_multicore_MPCtest_B.absb;
  if (raspberrypi_multicore_MPCtest_B.scale_k == 0.0) {
    *s = 0.0;
    *c = 1.0;
    raspberrypi_multicore_MPCtest_B.scale_k = 0.0;
    *b = 0.0;
  } else {
    ads = raspberrypi_multicore_MPCtest_B.absa /
      raspberrypi_multicore_MPCtest_B.scale_k;
    bds = raspberrypi_multicore_MPCtest_B.absb /
      raspberrypi_multicore_MPCtest_B.scale_k;
    raspberrypi_multicore_MPCtest_B.scale_k *= sqrt(ads * ads + bds * bds);
    if (raspberrypi_multicore_MPCtest_B.roe < 0.0) {
      raspberrypi_multicore_MPCtest_B.scale_k =
        -raspberrypi_multicore_MPCtest_B.scale_k;
    }

    *c = *a / raspberrypi_multicore_MPCtest_B.scale_k;
    *s = *b / raspberrypi_multicore_MPCtest_B.scale_k;
    if (raspberrypi_multicore_MPCtest_B.absa >
        raspberrypi_multicore_MPCtest_B.absb) {
      *b = *s;
    } else if (*c != 0.0) {
      *b = 1.0 / *c;
    } else {
      *b = 1.0;
    }
  }

  *a = raspberrypi_multicore_MPCtest_B.scale_k;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function2' */
static void raspberrypi_multicore_MPCt_xrot(real_T x[9], int32_T ix0, int32_T
  iy0, real_T c, real_T s)
{
  real_T temp;
  real_T temp_tmp;
  temp = x[iy0 - 1];
  temp_tmp = x[ix0 - 1];
  x[iy0 - 1] = temp * c - temp_tmp * s;
  x[ix0 - 1] = temp_tmp * c + temp * s;
  temp = x[ix0] * c + x[iy0] * s;
  x[iy0] = x[iy0] * c - x[ix0] * s;
  x[ix0] = temp;
  temp = x[iy0 + 1];
  temp_tmp = x[ix0 + 1];
  x[iy0 + 1] = temp * c - temp_tmp * s;
  x[ix0 + 1] = temp_tmp * c + temp * s;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function2' */
static void raspberrypi_multicore_MPCte_svd(const real_T A[9], real_T U[9],
  real_T s[3], real_T V[9])
{
  int32_T c_q;
  int32_T kase;
  int32_T m;
  int32_T qjj;
  int32_T qq;
  boolean_T apply_transform;
  boolean_T exitg1;
  raspberrypi_multicore_MPCtest_B.e[0] = 0.0;
  raspberrypi_multicore_MPCtest_B.work_h[0] = 0.0;
  raspberrypi_multicore_MPCtest_B.e[1] = 0.0;
  raspberrypi_multicore_MPCtest_B.work_h[1] = 0.0;
  raspberrypi_multicore_MPCtest_B.e[2] = 0.0;
  raspberrypi_multicore_MPCtest_B.work_h[2] = 0.0;
  for (m = 0; m < 9; m++) {
    raspberrypi_multicore_MPCtest_B.b_A_i[m] = A[m];
    U[m] = 0.0;
    raspberrypi_multicore_MPCtest_B.Vf[m] = 0.0;
  }

  apply_transform = false;
  raspberrypi_multicore_MPCtest_B.nrm = raspberrypi_multicore_M_xnrm2_l(3,
    raspberrypi_multicore_MPCtest_B.b_A_i, 1);
  if (raspberrypi_multicore_MPCtest_B.nrm > 0.0) {
    apply_transform = true;
    if (raspberrypi_multicore_MPCtest_B.b_A_i[0] < 0.0) {
      raspberrypi_multicore_MPCtest_B.b_s[0] =
        -raspberrypi_multicore_MPCtest_B.nrm;
    } else {
      raspberrypi_multicore_MPCtest_B.b_s[0] =
        raspberrypi_multicore_MPCtest_B.nrm;
    }

    if (fabs(raspberrypi_multicore_MPCtest_B.b_s[0]) >= 1.0020841800044864E-292)
    {
      raspberrypi_multicore_MPC_xscal(3, 1.0 /
        raspberrypi_multicore_MPCtest_B.b_s[0],
        raspberrypi_multicore_MPCtest_B.b_A_i, 1);
    } else {
      for (m = 0; m < 3; m++) {
        raspberrypi_multicore_MPCtest_B.b_A_i[m] /=
          raspberrypi_multicore_MPCtest_B.b_s[0];
      }
    }

    raspberrypi_multicore_MPCtest_B.b_A_i[0]++;
    raspberrypi_multicore_MPCtest_B.b_s[0] =
      -raspberrypi_multicore_MPCtest_B.b_s[0];
  } else {
    raspberrypi_multicore_MPCtest_B.b_s[0] = 0.0;
  }

  for (m = 1; m + 1 < 4; m++) {
    qjj = 3 * m;
    if (apply_transform) {
      raspberrypi_multicore_MPC_xaxpy(3, -(raspberrypi_multicore_MPC_xdotc(3,
        raspberrypi_multicore_MPCtest_B.b_A_i, 1,
        raspberrypi_multicore_MPCtest_B.b_A_i, qjj + 1) /
        raspberrypi_multicore_MPCtest_B.b_A_i[0]), 1,
        raspberrypi_multicore_MPCtest_B.b_A_i, qjj + 1);
    }

    raspberrypi_multicore_MPCtest_B.e[m] =
      raspberrypi_multicore_MPCtest_B.b_A_i[qjj];
  }

  for (m = 0; m + 1 < 4; m++) {
    U[m] = raspberrypi_multicore_MPCtest_B.b_A_i[m];
  }

  raspberrypi_multicore_MPCtest_B.nrm = raspberrypi_multicore__xnrm2_lk
    (raspberrypi_multicore_MPCtest_B.e, 2);
  if (raspberrypi_multicore_MPCtest_B.nrm == 0.0) {
    raspberrypi_multicore_MPCtest_B.e[0] = 0.0;
  } else {
    if (raspberrypi_multicore_MPCtest_B.e[1] < 0.0) {
      raspberrypi_multicore_MPCtest_B.nrm = -raspberrypi_multicore_MPCtest_B.nrm;
    }

    raspberrypi_multicore_MPCtest_B.e[0] = raspberrypi_multicore_MPCtest_B.nrm;
    if (fabs(raspberrypi_multicore_MPCtest_B.nrm) >= 1.0020841800044864E-292) {
      raspberrypi_multicore_M_xscal_p(1.0 / raspberrypi_multicore_MPCtest_B.nrm,
        raspberrypi_multicore_MPCtest_B.e, 2);
    } else {
      for (m = 1; m < 3; m++) {
        raspberrypi_multicore_MPCtest_B.e[m] /=
          raspberrypi_multicore_MPCtest_B.nrm;
      }
    }

    raspberrypi_multicore_MPCtest_B.e[1]++;
    raspberrypi_multicore_MPCtest_B.e[0] = -raspberrypi_multicore_MPCtest_B.e[0];
    for (m = 2; m < 4; m++) {
      raspberrypi_multicore_MPCtest_B.work_h[m - 1] = 0.0;
    }

    for (m = 1; m + 1 < 4; m++) {
      raspberrypi_multicore_M_xaxpy_m(2, raspberrypi_multicore_MPCtest_B.e[m],
        raspberrypi_multicore_MPCtest_B.b_A_i, 3 * m + 2,
        raspberrypi_multicore_MPCtest_B.work_h, 2);
    }

    for (m = 1; m + 1 < 4; m++) {
      raspberrypi_multicore__xaxpy_ml(2, -raspberrypi_multicore_MPCtest_B.e[m] /
        raspberrypi_multicore_MPCtest_B.e[1],
        raspberrypi_multicore_MPCtest_B.work_h, 2,
        raspberrypi_multicore_MPCtest_B.b_A_i, 3 * m + 2);
    }
  }

  for (m = 1; m + 1 < 4; m++) {
    raspberrypi_multicore_MPCtest_B.Vf[m] = raspberrypi_multicore_MPCtest_B.e[m];
  }

  apply_transform = false;
  raspberrypi_multicore_MPCtest_B.nrm = raspberrypi_multicore_M_xnrm2_l(2,
    raspberrypi_multicore_MPCtest_B.b_A_i, 5);
  if (raspberrypi_multicore_MPCtest_B.nrm > 0.0) {
    apply_transform = true;
    if (raspberrypi_multicore_MPCtest_B.b_A_i[4] < 0.0) {
      raspberrypi_multicore_MPCtest_B.b_s[1] =
        -raspberrypi_multicore_MPCtest_B.nrm;
    } else {
      raspberrypi_multicore_MPCtest_B.b_s[1] =
        raspberrypi_multicore_MPCtest_B.nrm;
    }

    if (fabs(raspberrypi_multicore_MPCtest_B.b_s[1]) >= 1.0020841800044864E-292)
    {
      raspberrypi_multicore_MPC_xscal(2, 1.0 /
        raspberrypi_multicore_MPCtest_B.b_s[1],
        raspberrypi_multicore_MPCtest_B.b_A_i, 5);
    } else {
      for (m = 4; m < 6; m++) {
        raspberrypi_multicore_MPCtest_B.b_A_i[m] /=
          raspberrypi_multicore_MPCtest_B.b_s[1];
      }
    }

    raspberrypi_multicore_MPCtest_B.b_A_i[4]++;
    raspberrypi_multicore_MPCtest_B.b_s[1] =
      -raspberrypi_multicore_MPCtest_B.b_s[1];
  } else {
    raspberrypi_multicore_MPCtest_B.b_s[1] = 0.0;
  }

  for (m = 2; m + 1 < 4; m++) {
    qjj = 3 * m + 1;
    if (apply_transform) {
      raspberrypi_multicore_MPC_xaxpy(2, -(raspberrypi_multicore_MPC_xdotc(2,
        raspberrypi_multicore_MPCtest_B.b_A_i, 5,
        raspberrypi_multicore_MPCtest_B.b_A_i, qjj + 1) /
        raspberrypi_multicore_MPCtest_B.b_A_i[4]), 5,
        raspberrypi_multicore_MPCtest_B.b_A_i, qjj + 1);
    }

    raspberrypi_multicore_MPCtest_B.e[m] =
      raspberrypi_multicore_MPCtest_B.b_A_i[qjj];
  }

  for (m = 1; m + 1 < 4; m++) {
    U[m + 3] = raspberrypi_multicore_MPCtest_B.b_A_i[m + 3];
  }

  m = 1;
  raspberrypi_multicore_MPCtest_B.b_s[2] =
    raspberrypi_multicore_MPCtest_B.b_A_i[8];
  U[6] = 0.0;
  U[7] = 0.0;
  U[8] = 1.0;
  for (c_q = 1; c_q >= 0; c_q--) {
    qq = 3 * c_q + c_q;
    if (raspberrypi_multicore_MPCtest_B.b_s[c_q] != 0.0) {
      for (kase = c_q + 1; kase + 1 < 4; kase++) {
        qjj = (3 * kase + c_q) + 1;
        raspberrypi_multicore_MPC_xaxpy(3 - c_q,
          -(raspberrypi_multicore_MPC_xdotc(3 - c_q, U, qq + 1, U, qjj) / U[qq]),
          qq + 1, U, qjj);
      }

      for (qjj = c_q; qjj + 1 < 4; qjj++) {
        kase = 3 * c_q + qjj;
        U[kase] = -U[kase];
      }

      U[qq]++;
      if (0 <= c_q - 1) {
        U[3 * c_q] = 0.0;
      }
    } else {
      U[3 * c_q] = 0.0;
      U[3 * c_q + 1] = 0.0;
      U[3 * c_q + 2] = 0.0;
      U[qq] = 1.0;
    }
  }

  for (c_q = 2; c_q >= 0; c_q--) {
    if ((c_q + 1 <= 1) && (raspberrypi_multicore_MPCtest_B.e[0] != 0.0)) {
      raspberrypi_multicore_MPC_xaxpy(2, -(raspberrypi_multicore_MPC_xdotc(2,
        raspberrypi_multicore_MPCtest_B.Vf, 2,
        raspberrypi_multicore_MPCtest_B.Vf, 5) /
        raspberrypi_multicore_MPCtest_B.Vf[1]), 2,
        raspberrypi_multicore_MPCtest_B.Vf, 5);
      raspberrypi_multicore_MPC_xaxpy(2, -(raspberrypi_multicore_MPC_xdotc(2,
        raspberrypi_multicore_MPCtest_B.Vf, 2,
        raspberrypi_multicore_MPCtest_B.Vf, 8) /
        raspberrypi_multicore_MPCtest_B.Vf[1]), 2,
        raspberrypi_multicore_MPCtest_B.Vf, 8);
    }

    raspberrypi_multicore_MPCtest_B.Vf[3 * c_q] = 0.0;
    raspberrypi_multicore_MPCtest_B.Vf[3 * c_q + 1] = 0.0;
    raspberrypi_multicore_MPCtest_B.Vf[3 * c_q + 2] = 0.0;
    raspberrypi_multicore_MPCtest_B.Vf[c_q + 3 * c_q] = 1.0;
  }

  raspberrypi_multicore_MPCtest_B.ztest0 = raspberrypi_multicore_MPCtest_B.e[0];
  if (raspberrypi_multicore_MPCtest_B.b_s[0] != 0.0) {
    raspberrypi_multicore_MPCtest_B.rt = fabs
      (raspberrypi_multicore_MPCtest_B.b_s[0]);
    raspberrypi_multicore_MPCtest_B.nrm = raspberrypi_multicore_MPCtest_B.b_s[0]
      / raspberrypi_multicore_MPCtest_B.rt;
    raspberrypi_multicore_MPCtest_B.b_s[0] = raspberrypi_multicore_MPCtest_B.rt;
    raspberrypi_multicore_MPCtest_B.ztest0 = raspberrypi_multicore_MPCtest_B.e[0]
      / raspberrypi_multicore_MPCtest_B.nrm;
    raspberrypi_multicore__xscal_pn(raspberrypi_multicore_MPCtest_B.nrm, U, 1);
  }

  if (raspberrypi_multicore_MPCtest_B.ztest0 != 0.0) {
    raspberrypi_multicore_MPCtest_B.rt = fabs
      (raspberrypi_multicore_MPCtest_B.ztest0);
    raspberrypi_multicore_MPCtest_B.nrm = raspberrypi_multicore_MPCtest_B.rt /
      raspberrypi_multicore_MPCtest_B.ztest0;
    raspberrypi_multicore_MPCtest_B.ztest0 = raspberrypi_multicore_MPCtest_B.rt;
    raspberrypi_multicore_MPCtest_B.b_s[1] *=
      raspberrypi_multicore_MPCtest_B.nrm;
    raspberrypi_multicore__xscal_pn(raspberrypi_multicore_MPCtest_B.nrm,
      raspberrypi_multicore_MPCtest_B.Vf, 4);
  }

  raspberrypi_multicore_MPCtest_B.e[0] = raspberrypi_multicore_MPCtest_B.ztest0;
  raspberrypi_multicore_MPCtest_B.ztest0 =
    raspberrypi_multicore_MPCtest_B.b_A_i[7];
  if (raspberrypi_multicore_MPCtest_B.b_s[1] != 0.0) {
    raspberrypi_multicore_MPCtest_B.rt = fabs
      (raspberrypi_multicore_MPCtest_B.b_s[1]);
    raspberrypi_multicore_MPCtest_B.nrm = raspberrypi_multicore_MPCtest_B.b_s[1]
      / raspberrypi_multicore_MPCtest_B.rt;
    raspberrypi_multicore_MPCtest_B.b_s[1] = raspberrypi_multicore_MPCtest_B.rt;
    raspberrypi_multicore_MPCtest_B.ztest0 =
      raspberrypi_multicore_MPCtest_B.b_A_i[7] /
      raspberrypi_multicore_MPCtest_B.nrm;
    raspberrypi_multicore__xscal_pn(raspberrypi_multicore_MPCtest_B.nrm, U, 4);
  }

  if (raspberrypi_multicore_MPCtest_B.ztest0 != 0.0) {
    raspberrypi_multicore_MPCtest_B.rt = fabs
      (raspberrypi_multicore_MPCtest_B.ztest0);
    raspberrypi_multicore_MPCtest_B.nrm = raspberrypi_multicore_MPCtest_B.rt /
      raspberrypi_multicore_MPCtest_B.ztest0;
    raspberrypi_multicore_MPCtest_B.ztest0 = raspberrypi_multicore_MPCtest_B.rt;
    raspberrypi_multicore_MPCtest_B.b_s[2] =
      raspberrypi_multicore_MPCtest_B.b_A_i[8] *
      raspberrypi_multicore_MPCtest_B.nrm;
    raspberrypi_multicore__xscal_pn(raspberrypi_multicore_MPCtest_B.nrm,
      raspberrypi_multicore_MPCtest_B.Vf, 7);
  }

  raspberrypi_multicore_MPCtest_B.e[1] = raspberrypi_multicore_MPCtest_B.ztest0;
  if (raspberrypi_multicore_MPCtest_B.b_s[2] != 0.0) {
    raspberrypi_multicore_MPCtest_B.rt = fabs
      (raspberrypi_multicore_MPCtest_B.b_s[2]);
    raspberrypi_multicore_MPCtest_B.nrm = raspberrypi_multicore_MPCtest_B.b_s[2]
      / raspberrypi_multicore_MPCtest_B.rt;
    raspberrypi_multicore_MPCtest_B.b_s[2] = raspberrypi_multicore_MPCtest_B.rt;
    raspberrypi_multicore__xscal_pn(raspberrypi_multicore_MPCtest_B.nrm, U, 7);
  }

  raspberrypi_multicore_MPCtest_B.e[2] = 0.0;
  qq = 0;
  if ((raspberrypi_multicore_MPCtest_B.b_s[0] >
       raspberrypi_multicore_MPCtest_B.e[0]) || rtIsNaN
      (raspberrypi_multicore_MPCtest_B.e[0])) {
    raspberrypi_multicore_MPCtest_B.nrm = raspberrypi_multicore_MPCtest_B.b_s[0];
  } else {
    raspberrypi_multicore_MPCtest_B.nrm = raspberrypi_multicore_MPCtest_B.e[0];
  }

  if ((raspberrypi_multicore_MPCtest_B.b_s[1] >
       raspberrypi_multicore_MPCtest_B.ztest0) || rtIsNaN
      (raspberrypi_multicore_MPCtest_B.ztest0)) {
    raspberrypi_multicore_MPCtest_B.ztest0 =
      raspberrypi_multicore_MPCtest_B.b_s[1];
  }

  if ((!(raspberrypi_multicore_MPCtest_B.nrm >
         raspberrypi_multicore_MPCtest_B.ztest0)) && (!rtIsNaN
       (raspberrypi_multicore_MPCtest_B.ztest0))) {
    raspberrypi_multicore_MPCtest_B.nrm = raspberrypi_multicore_MPCtest_B.ztest0;
  }

  if (raspberrypi_multicore_MPCtest_B.b_s[2] > 0.0) {
    raspberrypi_multicore_MPCtest_B.ztest0 =
      raspberrypi_multicore_MPCtest_B.b_s[2];
  } else {
    raspberrypi_multicore_MPCtest_B.ztest0 = 0.0;
  }

  if ((!(raspberrypi_multicore_MPCtest_B.nrm >
         raspberrypi_multicore_MPCtest_B.ztest0)) && (!rtIsNaN
       (raspberrypi_multicore_MPCtest_B.ztest0))) {
    raspberrypi_multicore_MPCtest_B.nrm = raspberrypi_multicore_MPCtest_B.ztest0;
  }

  while ((m + 2 > 0) && (qq < 75)) {
    c_q = m + 1;
    exitg1 = false;
    while (!(exitg1 || (c_q == 0))) {
      raspberrypi_multicore_MPCtest_B.ztest0 = fabs
        (raspberrypi_multicore_MPCtest_B.e[c_q - 1]);
      if ((raspberrypi_multicore_MPCtest_B.ztest0 <= (fabs
            (raspberrypi_multicore_MPCtest_B.b_s[c_q - 1]) + fabs
            (raspberrypi_multicore_MPCtest_B.b_s[c_q])) * 2.2204460492503131E-16)
          || ((raspberrypi_multicore_MPCtest_B.ztest0 <= 1.0020841800044864E-292)
              || ((qq > 20) && (raspberrypi_multicore_MPCtest_B.ztest0 <=
             2.2204460492503131E-16 * raspberrypi_multicore_MPCtest_B.nrm)))) {
        raspberrypi_multicore_MPCtest_B.e[c_q - 1] = 0.0;
        exitg1 = true;
      } else {
        c_q--;
      }
    }

    if (m + 1 == c_q) {
      kase = 4;
    } else {
      qjj = m + 2;
      kase = m + 2;
      exitg1 = false;
      while ((!exitg1) && (kase >= c_q)) {
        qjj = kase;
        if (kase == c_q) {
          exitg1 = true;
        } else {
          raspberrypi_multicore_MPCtest_B.ztest0 = 0.0;
          if (kase < m + 2) {
            raspberrypi_multicore_MPCtest_B.ztest0 = fabs
              (raspberrypi_multicore_MPCtest_B.e[kase - 1]);
          }

          if (kase > c_q + 1) {
            raspberrypi_multicore_MPCtest_B.ztest0 += fabs
              (raspberrypi_multicore_MPCtest_B.e[kase - 2]);
          }

          raspberrypi_multicore_MPCtest_B.rt = fabs
            (raspberrypi_multicore_MPCtest_B.b_s[kase - 1]);
          if ((raspberrypi_multicore_MPCtest_B.rt <= 2.2204460492503131E-16 *
               raspberrypi_multicore_MPCtest_B.ztest0) ||
              (raspberrypi_multicore_MPCtest_B.rt <= 1.0020841800044864E-292)) {
            raspberrypi_multicore_MPCtest_B.b_s[kase - 1] = 0.0;
            exitg1 = true;
          } else {
            kase--;
          }
        }
      }

      if (qjj == c_q) {
        kase = 3;
      } else if (m + 2 == qjj) {
        kase = 1;
      } else {
        kase = 2;
        c_q = qjj;
      }
    }

    switch (kase) {
     case 1:
      raspberrypi_multicore_MPCtest_B.ztest0 =
        raspberrypi_multicore_MPCtest_B.e[m];
      raspberrypi_multicore_MPCtest_B.e[m] = 0.0;
      for (qjj = m; qjj + 1 >= c_q + 1; qjj--) {
        raspberrypi_multicore_MPC_xrotg(&raspberrypi_multicore_MPCtest_B.b_s[qjj],
          &raspberrypi_multicore_MPCtest_B.ztest0,
          &raspberrypi_multicore_MPCtest_B.rt,
          &raspberrypi_multicore_MPCtest_B.sqds);
        if (qjj + 1 > c_q + 1) {
          raspberrypi_multicore_MPCtest_B.ztest0 =
            -raspberrypi_multicore_MPCtest_B.sqds *
            raspberrypi_multicore_MPCtest_B.e[0];
          raspberrypi_multicore_MPCtest_B.e[0] *=
            raspberrypi_multicore_MPCtest_B.rt;
        }

        raspberrypi_multicore_MPCt_xrot(raspberrypi_multicore_MPCtest_B.Vf, 3 *
          qjj + 1, 3 * (m + 1) + 1, raspberrypi_multicore_MPCtest_B.rt,
          raspberrypi_multicore_MPCtest_B.sqds);
      }
      break;

     case 2:
      raspberrypi_multicore_MPCtest_B.ztest0 =
        raspberrypi_multicore_MPCtest_B.e[c_q - 1];
      raspberrypi_multicore_MPCtest_B.e[c_q - 1] = 0.0;
      for (qjj = c_q; qjj < m + 2; qjj++) {
        raspberrypi_multicore_MPC_xrotg(&raspberrypi_multicore_MPCtest_B.b_s[qjj],
          &raspberrypi_multicore_MPCtest_B.ztest0,
          &raspberrypi_multicore_MPCtest_B.rt,
          &raspberrypi_multicore_MPCtest_B.sqds);
        raspberrypi_multicore_MPCtest_B.ztest0 =
          -raspberrypi_multicore_MPCtest_B.sqds *
          raspberrypi_multicore_MPCtest_B.e[qjj];
        raspberrypi_multicore_MPCtest_B.e[qjj] *=
          raspberrypi_multicore_MPCtest_B.rt;
        raspberrypi_multicore_MPCt_xrot(U, 3 * qjj + 1, 3 * (c_q - 1) + 1,
          raspberrypi_multicore_MPCtest_B.rt,
          raspberrypi_multicore_MPCtest_B.sqds);
      }
      break;

     case 3:
      raspberrypi_multicore_MPCtest_B.ztest0 =
        raspberrypi_multicore_MPCtest_B.b_s[m + 1];
      raspberrypi_multicore_MPCtest_B.rt = fabs
        (raspberrypi_multicore_MPCtest_B.ztest0);
      raspberrypi_multicore_MPCtest_B.sqds = fabs
        (raspberrypi_multicore_MPCtest_B.b_s[m]);
      if ((raspberrypi_multicore_MPCtest_B.rt >
           raspberrypi_multicore_MPCtest_B.sqds) || rtIsNaN
          (raspberrypi_multicore_MPCtest_B.sqds)) {
        raspberrypi_multicore_MPCtest_B.sqds =
          raspberrypi_multicore_MPCtest_B.rt;
      }

      raspberrypi_multicore_MPCtest_B.rt = fabs
        (raspberrypi_multicore_MPCtest_B.e[m]);
      if ((raspberrypi_multicore_MPCtest_B.sqds >
           raspberrypi_multicore_MPCtest_B.rt) || rtIsNaN
          (raspberrypi_multicore_MPCtest_B.rt)) {
        raspberrypi_multicore_MPCtest_B.rt =
          raspberrypi_multicore_MPCtest_B.sqds;
      }

      raspberrypi_multicore_MPCtest_B.sqds = fabs
        (raspberrypi_multicore_MPCtest_B.b_s[c_q]);
      if ((raspberrypi_multicore_MPCtest_B.rt >
           raspberrypi_multicore_MPCtest_B.sqds) || rtIsNaN
          (raspberrypi_multicore_MPCtest_B.sqds)) {
        raspberrypi_multicore_MPCtest_B.sqds =
          raspberrypi_multicore_MPCtest_B.rt;
      }

      raspberrypi_multicore_MPCtest_B.rt = fabs
        (raspberrypi_multicore_MPCtest_B.e[c_q]);
      if ((raspberrypi_multicore_MPCtest_B.sqds >
           raspberrypi_multicore_MPCtest_B.rt) || rtIsNaN
          (raspberrypi_multicore_MPCtest_B.rt)) {
        raspberrypi_multicore_MPCtest_B.rt =
          raspberrypi_multicore_MPCtest_B.sqds;
      }

      raspberrypi_multicore_MPCtest_B.ztest0 /=
        raspberrypi_multicore_MPCtest_B.rt;
      raspberrypi_multicore_MPCtest_B.smm1 =
        raspberrypi_multicore_MPCtest_B.b_s[m] /
        raspberrypi_multicore_MPCtest_B.rt;
      raspberrypi_multicore_MPCtest_B.emm1 = raspberrypi_multicore_MPCtest_B.e[m]
        / raspberrypi_multicore_MPCtest_B.rt;
      raspberrypi_multicore_MPCtest_B.sqds =
        raspberrypi_multicore_MPCtest_B.b_s[c_q] /
        raspberrypi_multicore_MPCtest_B.rt;
      raspberrypi_multicore_MPCtest_B.smm1 =
        ((raspberrypi_multicore_MPCtest_B.smm1 +
          raspberrypi_multicore_MPCtest_B.ztest0) *
         (raspberrypi_multicore_MPCtest_B.smm1 -
          raspberrypi_multicore_MPCtest_B.ztest0) +
         raspberrypi_multicore_MPCtest_B.emm1 *
         raspberrypi_multicore_MPCtest_B.emm1) / 2.0;
      raspberrypi_multicore_MPCtest_B.emm1 *=
        raspberrypi_multicore_MPCtest_B.ztest0;
      raspberrypi_multicore_MPCtest_B.emm1 *=
        raspberrypi_multicore_MPCtest_B.emm1;
      if ((raspberrypi_multicore_MPCtest_B.smm1 != 0.0) ||
          (raspberrypi_multicore_MPCtest_B.emm1 != 0.0)) {
        raspberrypi_multicore_MPCtest_B.shift = sqrt
          (raspberrypi_multicore_MPCtest_B.smm1 *
           raspberrypi_multicore_MPCtest_B.smm1 +
           raspberrypi_multicore_MPCtest_B.emm1);
        if (raspberrypi_multicore_MPCtest_B.smm1 < 0.0) {
          raspberrypi_multicore_MPCtest_B.shift =
            -raspberrypi_multicore_MPCtest_B.shift;
        }

        raspberrypi_multicore_MPCtest_B.shift =
          raspberrypi_multicore_MPCtest_B.emm1 /
          (raspberrypi_multicore_MPCtest_B.smm1 +
           raspberrypi_multicore_MPCtest_B.shift);
      } else {
        raspberrypi_multicore_MPCtest_B.shift = 0.0;
      }

      raspberrypi_multicore_MPCtest_B.ztest0 =
        (raspberrypi_multicore_MPCtest_B.sqds +
         raspberrypi_multicore_MPCtest_B.ztest0) *
        (raspberrypi_multicore_MPCtest_B.sqds -
         raspberrypi_multicore_MPCtest_B.ztest0) +
        raspberrypi_multicore_MPCtest_B.shift;
      raspberrypi_multicore_MPCtest_B.smm1 =
        raspberrypi_multicore_MPCtest_B.e[c_q] /
        raspberrypi_multicore_MPCtest_B.rt *
        raspberrypi_multicore_MPCtest_B.sqds;
      for (qjj = c_q + 1; qjj <= m + 1; qjj++) {
        raspberrypi_multicore_MPC_xrotg(&raspberrypi_multicore_MPCtest_B.ztest0,
          &raspberrypi_multicore_MPCtest_B.smm1,
          &raspberrypi_multicore_MPCtest_B.rt,
          &raspberrypi_multicore_MPCtest_B.sqds);
        if (qjj > c_q + 1) {
          raspberrypi_multicore_MPCtest_B.e[0] =
            raspberrypi_multicore_MPCtest_B.ztest0;
        }

        raspberrypi_multicore_MPCtest_B.smm1 =
          raspberrypi_multicore_MPCtest_B.e[qjj - 1];
        raspberrypi_multicore_MPCtest_B.emm1 =
          raspberrypi_multicore_MPCtest_B.b_s[qjj - 1];
        raspberrypi_multicore_MPCtest_B.ztest0 =
          raspberrypi_multicore_MPCtest_B.emm1 *
          raspberrypi_multicore_MPCtest_B.rt +
          raspberrypi_multicore_MPCtest_B.smm1 *
          raspberrypi_multicore_MPCtest_B.sqds;
        raspberrypi_multicore_MPCtest_B.e[qjj - 1] =
          raspberrypi_multicore_MPCtest_B.smm1 *
          raspberrypi_multicore_MPCtest_B.rt -
          raspberrypi_multicore_MPCtest_B.emm1 *
          raspberrypi_multicore_MPCtest_B.sqds;
        raspberrypi_multicore_MPCtest_B.smm1 =
          raspberrypi_multicore_MPCtest_B.sqds *
          raspberrypi_multicore_MPCtest_B.b_s[qjj];
        raspberrypi_multicore_MPCtest_B.b_s[qjj] *=
          raspberrypi_multicore_MPCtest_B.rt;
        raspberrypi_multicore_MPCt_xrot(raspberrypi_multicore_MPCtest_B.Vf, 3 *
          (qjj - 1) + 1, 3 * qjj + 1, raspberrypi_multicore_MPCtest_B.rt,
          raspberrypi_multicore_MPCtest_B.sqds);
        raspberrypi_multicore_MPC_xrotg(&raspberrypi_multicore_MPCtest_B.ztest0,
          &raspberrypi_multicore_MPCtest_B.smm1,
          &raspberrypi_multicore_MPCtest_B.rt,
          &raspberrypi_multicore_MPCtest_B.sqds);
        raspberrypi_multicore_MPCtest_B.b_s[qjj - 1] =
          raspberrypi_multicore_MPCtest_B.ztest0;
        raspberrypi_multicore_MPCtest_B.ztest0 =
          raspberrypi_multicore_MPCtest_B.e[qjj - 1] *
          raspberrypi_multicore_MPCtest_B.rt +
          raspberrypi_multicore_MPCtest_B.sqds *
          raspberrypi_multicore_MPCtest_B.b_s[qjj];
        raspberrypi_multicore_MPCtest_B.b_s[qjj] =
          raspberrypi_multicore_MPCtest_B.e[qjj - 1] *
          -raspberrypi_multicore_MPCtest_B.sqds +
          raspberrypi_multicore_MPCtest_B.rt *
          raspberrypi_multicore_MPCtest_B.b_s[qjj];
        raspberrypi_multicore_MPCtest_B.smm1 =
          raspberrypi_multicore_MPCtest_B.sqds *
          raspberrypi_multicore_MPCtest_B.e[qjj];
        raspberrypi_multicore_MPCtest_B.e[qjj] *=
          raspberrypi_multicore_MPCtest_B.rt;
        raspberrypi_multicore_MPCt_xrot(U, 3 * (qjj - 1) + 1, 3 * qjj + 1,
          raspberrypi_multicore_MPCtest_B.rt,
          raspberrypi_multicore_MPCtest_B.sqds);
      }

      raspberrypi_multicore_MPCtest_B.e[m] =
        raspberrypi_multicore_MPCtest_B.ztest0;
      qq++;
      break;

     default:
      if (raspberrypi_multicore_MPCtest_B.b_s[c_q] < 0.0) {
        raspberrypi_multicore_MPCtest_B.b_s[c_q] =
          -raspberrypi_multicore_MPCtest_B.b_s[c_q];
        raspberrypi_multicore__xscal_pn(-1.0, raspberrypi_multicore_MPCtest_B.Vf,
          3 * c_q + 1);
      }

      qq = c_q + 1;
      while ((c_q + 1 < 3) && (raspberrypi_multicore_MPCtest_B.b_s[c_q] <
              raspberrypi_multicore_MPCtest_B.b_s[qq])) {
        raspberrypi_multicore_MPCtest_B.rt =
          raspberrypi_multicore_MPCtest_B.b_s[c_q];
        raspberrypi_multicore_MPCtest_B.b_s[c_q] =
          raspberrypi_multicore_MPCtest_B.b_s[qq];
        raspberrypi_multicore_MPCtest_B.b_s[qq] =
          raspberrypi_multicore_MPCtest_B.rt;
        raspberrypi_multicore_MPC_xswap(raspberrypi_multicore_MPCtest_B.Vf, 3 *
          c_q + 1, 3 * (c_q + 1) + 1);
        raspberrypi_multicore_MPC_xswap(U, 3 * c_q + 1, 3 * (c_q + 1) + 1);
        c_q = qq;
        qq++;
      }

      qq = 0;
      m--;
      break;
    }
  }

  for (m = 0; m < 3; m++) {
    s[m] = raspberrypi_multicore_MPCtest_B.b_s[m];
    V[3 * m] = raspberrypi_multicore_MPCtest_B.Vf[3 * m];
    c_q = 3 * m + 1;
    V[c_q] = raspberrypi_multicore_MPCtest_B.Vf[c_q];
    c_q = 3 * m + 2;
    V[c_q] = raspberrypi_multicore_MPCtest_B.Vf[c_q];
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function2' */
static real_T raspberrypi_multicore_MP_norm_k(const real_T x[3])
{
  real_T absxk;
  real_T scale;
  real_T t;
  real_T y;
  scale = 3.3121686421112381E-170;
  absxk = fabs(x[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  absxk = fabs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = fabs(x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  return scale * sqrt(y);
}

real_T rt_powd_snf(real_T u0, real_T u1)
{
  real_T tmp;
  real_T tmp_0;
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else {
    tmp = fabs(u0);
    tmp_0 = fabs(u1);
    if (rtIsInf(u1)) {
      if (tmp == 1.0) {
        y = 1.0;
      } else if (tmp > 1.0) {
        if (u1 > 0.0) {
          y = (rtInf);
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = (rtInf);
      }
    } else if (tmp_0 == 0.0) {
      y = 1.0;
    } else if (tmp_0 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = (rtNaN);
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

static void raspberrypi_multico_Get1From2_a(real_T x1, real_T b_y1, real_T x2,
  real_T y2, real_T l1, real_T l2, real_T limit, real_T *xout, real_T *yout,
  real_T *Flag)
{
  real_T D_tmp;
  real_T D_tmp_tmp;
  real_T E_tmp;
  boolean_T guard1 = false;

  /*    INPUT: */
  /*    limit: */
  /*    0: y bigger */
  /*    1: y smaller */
  /*    2: x smaller */
  /*    3: x bigger */
  /*  OUTPUT: */
  /*  Flag:   0  normal */
  /*            1  infeasible */
  /*            2  limit intput error */
  *Flag = 0.0;
  raspberrypi_multicore_MPCtest_B.a_tmp = x1 - x2;
  raspberrypi_multicore_MPCtest_B.yo1 = b_y1 - y2;
  D_tmp = raspberrypi_multicore_MPCtest_B.a_tmp *
    raspberrypi_multicore_MPCtest_B.a_tmp;
  D_tmp_tmp = raspberrypi_multicore_MPCtest_B.yo1 *
    raspberrypi_multicore_MPCtest_B.yo1;
  raspberrypi_multicore_MPCtest_B.F_h = D_tmp + D_tmp_tmp;
  raspberrypi_multicore_MPCtest_B.D = sqrt(raspberrypi_multicore_MPCtest_B.F_h);
  raspberrypi_multicore_MPCtest_B.temp_e = l1 + l2;
  guard1 = false;
  if (raspberrypi_multicore_MPCtest_B.temp_e > raspberrypi_multicore_MPCtest_B.D)
  {
    raspberrypi_multicore_MPCtest_B.a_tmp = l1 - l2;
    if (fabs(raspberrypi_multicore_MPCtest_B.a_tmp) <
        raspberrypi_multicore_MPCtest_B.D) {
      raspberrypi_multicore_MPCtest_B.D = l1 * l1;
      E_tmp = l2 * l2;
      raspberrypi_multicore_MPCtest_B.E_b =
        (((-raspberrypi_multicore_MPCtest_B.D + E_tmp) *
          raspberrypi_multicore_MPCtest_B.yo1 + (D_tmp - b_y1 * y2) * (b_y1 + y2))
         + rt_powd_snf(b_y1, 3.0)) + rt_powd_snf(y2, 3.0);
      raspberrypi_multicore_MPCtest_B.F_h *= 2.0;
      raspberrypi_multicore_MPCtest_B.temp_e = sqrt
        (((raspberrypi_multicore_MPCtest_B.temp_e *
           raspberrypi_multicore_MPCtest_B.temp_e - D_tmp) - D_tmp_tmp) *
         ((-(raspberrypi_multicore_MPCtest_B.a_tmp *
             raspberrypi_multicore_MPCtest_B.a_tmp) + D_tmp) + D_tmp_tmp));
      raspberrypi_multicore_MPCtest_B.a_tmp = x2 *
        raspberrypi_multicore_MPCtest_B.temp_e;
      raspberrypi_multicore_MPCtest_B.temp_e *= x1;
      raspberrypi_multicore_MPCtest_B.yo1 =
        ((raspberrypi_multicore_MPCtest_B.temp_e -
          raspberrypi_multicore_MPCtest_B.a_tmp) +
         raspberrypi_multicore_MPCtest_B.E_b) /
        raspberrypi_multicore_MPCtest_B.F_h;
      *yout = ((raspberrypi_multicore_MPCtest_B.a_tmp -
                raspberrypi_multicore_MPCtest_B.temp_e) +
               raspberrypi_multicore_MPCtest_B.E_b) /
        raspberrypi_multicore_MPCtest_B.F_h;
      raspberrypi_multicore_MPCtest_B.a_tmp =
        raspberrypi_multicore_MPCtest_B.yo1 - b_y1;
      D_tmp = sqrt(raspberrypi_multicore_MPCtest_B.D -
                   raspberrypi_multicore_MPCtest_B.a_tmp *
                   raspberrypi_multicore_MPCtest_B.a_tmp);
      raspberrypi_multicore_MPCtest_B.E_b = D_tmp + x1;
      raspberrypi_multicore_MPCtest_B.temp_e =
        raspberrypi_multicore_MPCtest_B.E_b - x2;
      raspberrypi_multicore_MPCtest_B.a_tmp =
        raspberrypi_multicore_MPCtest_B.yo1 - y2;
      if (fabs((raspberrypi_multicore_MPCtest_B.temp_e *
                raspberrypi_multicore_MPCtest_B.temp_e +
                raspberrypi_multicore_MPCtest_B.a_tmp *
                raspberrypi_multicore_MPCtest_B.a_tmp) - E_tmp) > 0.0001) {
        raspberrypi_multicore_MPCtest_B.E_b = -D_tmp + x1;
      }

      raspberrypi_multicore_MPCtest_B.a_tmp = *yout - b_y1;
      raspberrypi_multicore_MPCtest_B.D = sqrt(raspberrypi_multicore_MPCtest_B.D
        - raspberrypi_multicore_MPCtest_B.a_tmp *
        raspberrypi_multicore_MPCtest_B.a_tmp);
      *xout = raspberrypi_multicore_MPCtest_B.D + x1;
      raspberrypi_multicore_MPCtest_B.temp_e = *xout - x2;
      raspberrypi_multicore_MPCtest_B.a_tmp = *yout - y2;
      if (fabs((raspberrypi_multicore_MPCtest_B.temp_e *
                raspberrypi_multicore_MPCtest_B.temp_e +
                raspberrypi_multicore_MPCtest_B.a_tmp *
                raspberrypi_multicore_MPCtest_B.a_tmp) - E_tmp) > 0.0001) {
        *xout = -raspberrypi_multicore_MPCtest_B.D + x1;
      }

      if (fabs(raspberrypi_multicore_MPCtest_B.yo1 - *yout) < 0.0001) {
        *xout = 2.0 * x1 - raspberrypi_multicore_MPCtest_B.E_b;
      }

      switch ((int32_T)limit) {
       case 0:
        if (raspberrypi_multicore_MPCtest_B.yo1 > *yout) {
          *yout = raspberrypi_multicore_MPCtest_B.yo1;
          *xout = raspberrypi_multicore_MPCtest_B.E_b;
        }
        break;

       case 1:
        if (raspberrypi_multicore_MPCtest_B.yo1 < *yout) {
          *yout = raspberrypi_multicore_MPCtest_B.yo1;
          *xout = raspberrypi_multicore_MPCtest_B.E_b;
        }
        break;

       case 2:
        if (raspberrypi_multicore_MPCtest_B.E_b < *xout) {
          *yout = raspberrypi_multicore_MPCtest_B.yo1;
          *xout = raspberrypi_multicore_MPCtest_B.E_b;
        }
        break;

       case 3:
        if (raspberrypi_multicore_MPCtest_B.E_b > *xout) {
          *yout = raspberrypi_multicore_MPCtest_B.yo1;
          *xout = raspberrypi_multicore_MPCtest_B.E_b;
        }
        break;

       default:
        *xout = 99.0;
        *yout = 99.0;
        *Flag = 2.0;
        break;
      }
    } else {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    *xout = 99.0;
    *yout = 99.0;
    *Flag = 1.0;
  }
}

static void raspberryp_AdmittanceCtr_IK_one(const
  AdmittanceCtr_raspberrypi_mul_T *obj, const real_T p[3], real_T LegNum, real_T
  Angle[3], real_T *Flag)
{
  boolean_T guard1 = false;

  /*  calculate one leg IK according to different leg index */
  /*  Flag: */
  /*      0  normal */
  /*      1  roll calculation error */
  /*      2  pX, pY out of workspace */
  /*      3  AD collides with DP */
  /*      4  elbow touches the ground */
  /*      5  leg index assignment error */
  /*  all error state will triger a three-zeros output */
  *Flag = 0.0;
  Angle[0] = 0.0;
  Angle[1] = 0.0;
  Angle[2] = 0.0;
  guard1 = false;
  switch ((int32_T)LegNum) {
   case 1:
   case 3:
    raspberrypi_multicore_MPCtest_B.RP = sqrt((p[1] * p[1] + p[2] * p[2]) -
      obj->OR * obj->OR);
    raspberrypi_multico_Get1From2_a(0.0, 0.0, p[1], p[2], obj->OR,
      raspberrypi_multicore_MPCtest_B.RP, 3.0,
      &raspberrypi_multicore_MPCtest_B.pRy, &raspberrypi_multicore_MPCtest_B.pRz,
      &raspberrypi_multicore_MPCtest_B.errFlag);

    /*  angle for 13 angle */
    if (raspberrypi_multicore_MPCtest_B.errFlag != 0.0) {
      *Flag = 1.0;
    } else {
      raspberrypi_multico_Get1From2_a(0.0, 0.0, p[0],
        -raspberrypi_multicore_MPCtest_B.RP, obj->BC, obj->DP, 2.0,
        &raspberrypi_multicore_MPCtest_B.xout,
        &raspberrypi_multicore_MPCtest_B.yout,
        &raspberrypi_multicore_MPCtest_B.errFlag);
      if (raspberrypi_multicore_MPCtest_B.errFlag != 0.0) {
        *Flag = 2.0;
      } else {
        raspberrypi_multicore_MPCtest_B.errFlag = sin(-obj->CDP);
        raspberrypi_multicore_MPCtest_B.pC_tmp = cos(-obj->CDP);
        raspberrypi_multicore_MPCtest_B.pP_idx_0 = p[0] -
          raspberrypi_multicore_MPCtest_B.xout;
        raspberrypi_multicore_MPCtest_B.pP_idx_1 =
          -raspberrypi_multicore_MPCtest_B.RP -
          raspberrypi_multicore_MPCtest_B.yout;
        raspberrypi_multicore_MPCtest_B.pC =
          (raspberrypi_multicore_MPCtest_B.errFlag *
           raspberrypi_multicore_MPCtest_B.pP_idx_0 +
           raspberrypi_multicore_MPCtest_B.pC_tmp *
           raspberrypi_multicore_MPCtest_B.pP_idx_1) / obj->DP * obj->AB +
          raspberrypi_multicore_MPCtest_B.yout;
        raspberrypi_multicore_MPCtest_B.pB_idx_1 =
          raspberrypi_multicore_MPCtest_B.pC -
          raspberrypi_multicore_MPCtest_B.yout;

        /*  angle for 11 angle */
        /*  angle for 12 angle */
        if (-raspberrypi_multicore_MPCtest_B.RP >
            raspberrypi_multicore_MPCtest_B.yout) {
          *Flag = 3.0;

          /*  AD collides with DP */
        } else if (-raspberrypi_multicore_MPCtest_B.RP >
                   raspberrypi_multicore_MPCtest_B.pC) {
          *Flag = 4.0;

          /*  elbow touches the ground */
        } else {
          if (raspberrypi_multicore_MPCtest_B.yout < 0.0) {
            raspberrypi_multicore_MPCtest_B.RP = -1.0;
          } else if (raspberrypi_multicore_MPCtest_B.yout > 0.0) {
            raspberrypi_multicore_MPCtest_B.RP = 1.0;
          } else if (raspberrypi_multicore_MPCtest_B.yout == 0.0) {
            raspberrypi_multicore_MPCtest_B.RP = 0.0;
          } else {
            raspberrypi_multicore_MPCtest_B.RP = (rtNaN);
          }

          Angle[0] = acos((0.0 * raspberrypi_multicore_MPCtest_B.yout +
                           -raspberrypi_multicore_MPCtest_B.xout) / obj->BC) *
            raspberrypi_multicore_MPCtest_B.RP;
          if (-raspberrypi_multicore_MPCtest_B.pB_idx_1 < 0.0) {
            raspberrypi_multicore_MPCtest_B.RP = -1.0;
          } else if (-raspberrypi_multicore_MPCtest_B.pB_idx_1 > 0.0) {
            raspberrypi_multicore_MPCtest_B.RP = 1.0;
          } else if (-raspberrypi_multicore_MPCtest_B.pB_idx_1 == 0.0) {
            raspberrypi_multicore_MPCtest_B.RP = 0.0;
          } else {
            raspberrypi_multicore_MPCtest_B.RP = (rtNaN);
          }

          Angle[1] = acos((-(((raspberrypi_multicore_MPCtest_B.pC_tmp *
                               raspberrypi_multicore_MPCtest_B.pP_idx_0 +
                               -raspberrypi_multicore_MPCtest_B.errFlag *
                               raspberrypi_multicore_MPCtest_B.pP_idx_1) /
                              obj->DP * obj->AB +
                              raspberrypi_multicore_MPCtest_B.xout) -
                             raspberrypi_multicore_MPCtest_B.xout) + 0.0 *
                           raspberrypi_multicore_MPCtest_B.pB_idx_1) / obj->AB) *
            raspberrypi_multicore_MPCtest_B.RP;
          if (raspberrypi_multicore_MPCtest_B.pRz < 0.0) {
            raspberrypi_multicore_MPCtest_B.RP = -1.0;
          } else if (raspberrypi_multicore_MPCtest_B.pRz > 0.0) {
            raspberrypi_multicore_MPCtest_B.RP = 1.0;
          } else if (raspberrypi_multicore_MPCtest_B.pRz == 0.0) {
            raspberrypi_multicore_MPCtest_B.RP = 0.0;
          } else {
            raspberrypi_multicore_MPCtest_B.RP = (rtNaN);
          }

          Angle[2] = acos((0.0 * raspberrypi_multicore_MPCtest_B.pRz +
                           raspberrypi_multicore_MPCtest_B.pRy) / obj->OR) *
            raspberrypi_multicore_MPCtest_B.RP;
          guard1 = true;
        }
      }
    }
    break;

   case 2:
   case 4:
    raspberrypi_multicore_MPCtest_B.RP = sqrt((p[1] * p[1] + p[2] * p[2]) -
      obj->OR * obj->OR);
    raspberrypi_multico_Get1From2_a(0.0, 0.0, p[1], p[2], obj->OR,
      raspberrypi_multicore_MPCtest_B.RP, 2.0,
      &raspberrypi_multicore_MPCtest_B.pRy, &raspberrypi_multicore_MPCtest_B.pRz,
      &raspberrypi_multicore_MPCtest_B.errFlag);

    /*  angle for 13 servo */
    if (raspberrypi_multicore_MPCtest_B.errFlag != 0.0) {
      *Flag = 1.0;
    } else {
      raspberrypi_multico_Get1From2_a(0.0, 0.0, p[0],
        -raspberrypi_multicore_MPCtest_B.RP, obj->BC, obj->DP, 2.0,
        &raspberrypi_multicore_MPCtest_B.xout,
        &raspberrypi_multicore_MPCtest_B.yout,
        &raspberrypi_multicore_MPCtest_B.errFlag);
      if (raspberrypi_multicore_MPCtest_B.errFlag != 0.0) {
        *Flag = 2.0;
      } else {
        raspberrypi_multicore_MPCtest_B.pP_idx_0 = p[0] -
          raspberrypi_multicore_MPCtest_B.xout;
        raspberrypi_multicore_MPCtest_B.pP_idx_1 =
          -raspberrypi_multicore_MPCtest_B.RP -
          raspberrypi_multicore_MPCtest_B.yout;
        raspberrypi_multicore_MPCtest_B.errFlag = cos(-obj->CDP);
        raspberrypi_multicore_MPCtest_B.pC_tmp = sin(-obj->CDP);
        raspberrypi_multicore_MPCtest_B.pC =
          (raspberrypi_multicore_MPCtest_B.pC_tmp *
           raspberrypi_multicore_MPCtest_B.pP_idx_0 +
           raspberrypi_multicore_MPCtest_B.errFlag *
           raspberrypi_multicore_MPCtest_B.pP_idx_1) / obj->DP * obj->AB +
          raspberrypi_multicore_MPCtest_B.yout;
        raspberrypi_multicore_MPCtest_B.pB_idx_1 =
          raspberrypi_multicore_MPCtest_B.pC -
          raspberrypi_multicore_MPCtest_B.yout;

        /*  angle for 11 servo */
        /*  angle for 12 servo */
        if (-raspberrypi_multicore_MPCtest_B.RP >
            raspberrypi_multicore_MPCtest_B.yout) {
          *Flag = 3.0;

          /*  AD collides with DP */
        } else if (-raspberrypi_multicore_MPCtest_B.RP >
                   raspberrypi_multicore_MPCtest_B.pC) {
          *Flag = 4.0;

          /*  elbow touches the ground */
        } else {
          if (-raspberrypi_multicore_MPCtest_B.yout < 0.0) {
            raspberrypi_multicore_MPCtest_B.RP = -1.0;
          } else if (-raspberrypi_multicore_MPCtest_B.yout > 0.0) {
            raspberrypi_multicore_MPCtest_B.RP = 1.0;
          } else if (-raspberrypi_multicore_MPCtest_B.yout == 0.0) {
            raspberrypi_multicore_MPCtest_B.RP = 0.0;
          } else {
            raspberrypi_multicore_MPCtest_B.RP = (rtNaN);
          }

          Angle[0] = acos((0.0 * raspberrypi_multicore_MPCtest_B.yout +
                           -raspberrypi_multicore_MPCtest_B.xout) / obj->BC) *
            raspberrypi_multicore_MPCtest_B.RP;
          if (raspberrypi_multicore_MPCtest_B.pB_idx_1 < 0.0) {
            raspberrypi_multicore_MPCtest_B.RP = -1.0;
          } else if (raspberrypi_multicore_MPCtest_B.pB_idx_1 > 0.0) {
            raspberrypi_multicore_MPCtest_B.RP = 1.0;
          } else if (raspberrypi_multicore_MPCtest_B.pB_idx_1 == 0.0) {
            raspberrypi_multicore_MPCtest_B.RP = 0.0;
          } else {
            raspberrypi_multicore_MPCtest_B.RP = (rtNaN);
          }

          Angle[1] = acos((-(((raspberrypi_multicore_MPCtest_B.errFlag *
                               raspberrypi_multicore_MPCtest_B.pP_idx_0 +
                               -raspberrypi_multicore_MPCtest_B.pC_tmp *
                               raspberrypi_multicore_MPCtest_B.pP_idx_1) /
                              obj->DP * obj->AB +
                              raspberrypi_multicore_MPCtest_B.xout) -
                             raspberrypi_multicore_MPCtest_B.xout) + 0.0 *
                           raspberrypi_multicore_MPCtest_B.pB_idx_1) / obj->AB) *
            raspberrypi_multicore_MPCtest_B.RP;
          if (-raspberrypi_multicore_MPCtest_B.pRz < 0.0) {
            raspberrypi_multicore_MPCtest_B.RP = -1.0;
          } else if (-raspberrypi_multicore_MPCtest_B.pRz > 0.0) {
            raspberrypi_multicore_MPCtest_B.RP = 1.0;
          } else if (-raspberrypi_multicore_MPCtest_B.pRz == 0.0) {
            raspberrypi_multicore_MPCtest_B.RP = 0.0;
          } else {
            raspberrypi_multicore_MPCtest_B.RP = (rtNaN);
          }

          Angle[2] = acos((0.0 * raspberrypi_multicore_MPCtest_B.pRz +
                           -raspberrypi_multicore_MPCtest_B.pRy) / obj->OR) *
            raspberrypi_multicore_MPCtest_B.RP;
          guard1 = true;
        }
      }
    }
    break;

   default:
    *Flag = 5.0;
    guard1 = true;
    break;
  }

  if (guard1) {
    if ((fabs(LegNum - 3.0) < 0.1) || (fabs(LegNum - 4.0) < 0.1)) {
      Angle[2] = -Angle[2];
    }
  }
}

static boolean_T raspberrypi_multicore_M_isequal(const real_T varargin_1[3],
  const real_T varargin_2[3])
{
  int32_T b_k;
  boolean_T exitg1;
  boolean_T p;
  boolean_T p_0;
  p = false;
  p_0 = true;
  b_k = 0;
  exitg1 = false;
  while ((!exitg1) && (b_k < 3)) {
    if (!(varargin_1[b_k] == varargin_2[b_k])) {
      p_0 = false;
      exitg1 = true;
    } else {
      b_k++;
    }
  }

  if (p_0) {
    p = true;
  }

  return p;
}

static real_T raspberrypi_multicore_factorial(real_T n)
{
  real_T b_n;
  static const real_T tmp[170] = { 1.0, 2.0, 6.0, 24.0, 120.0, 720.0, 5040.0,
    40320.0, 362880.0, 3.6288E+6, 3.99168E+7, 4.790016E+8, 6.2270208E+9,
    8.71782912E+10, 1.307674368E+12, 2.0922789888E+13, 3.55687428096E+14,
    6.402373705728E+15, 1.21645100408832E+17, 2.43290200817664E+18,
    5.109094217170944E+19, 1.1240007277776077E+21, 2.5852016738884978E+22,
    6.2044840173323941E+23, 1.5511210043330986E+25, 4.0329146112660565E+26,
    1.0888869450418352E+28, 3.0488834461171384E+29, 8.8417619937397008E+30,
    2.6525285981219103E+32, 8.2228386541779224E+33, 2.6313083693369352E+35,
    8.6833176188118859E+36, 2.9523279903960412E+38, 1.0333147966386144E+40,
    3.7199332678990118E+41, 1.3763753091226343E+43, 5.23022617466601E+44,
    2.0397882081197442E+46, 8.1591528324789768E+47, 3.3452526613163803E+49,
    1.4050061177528798E+51, 6.0415263063373834E+52, 2.6582715747884485E+54,
    1.1962222086548019E+56, 5.5026221598120885E+57, 2.5862324151116818E+59,
    1.2413915592536073E+61, 6.0828186403426752E+62, 3.0414093201713376E+64,
    1.5511187532873822E+66, 8.0658175170943877E+67, 4.2748832840600255E+69,
    2.3084369733924138E+71, 1.2696403353658276E+73, 7.1099858780486348E+74,
    4.0526919504877221E+76, 2.3505613312828789E+78, 1.3868311854568986E+80,
    8.3209871127413916E+81, 5.0758021387722484E+83, 3.1469973260387939E+85,
    1.98260831540444E+87, 1.2688693218588417E+89, 8.2476505920824715E+90,
    5.4434493907744307E+92, 3.6471110918188683E+94, 2.4800355424368305E+96,
    1.711224524281413E+98, 1.197857166996989E+100, 8.5047858856786218E+101,
    6.1234458376886077E+103, 4.4701154615126834E+105, 3.3078854415193856E+107,
    2.4809140811395391E+109, 1.8854947016660498E+111, 1.4518309202828584E+113,
    1.1324281178206295E+115, 8.9461821307829729E+116, 7.1569457046263779E+118,
    5.7971260207473655E+120, 4.75364333701284E+122, 3.9455239697206569E+124,
    3.314240134565352E+126, 2.8171041143805494E+128, 2.4227095383672724E+130,
    2.1077572983795269E+132, 1.8548264225739836E+134, 1.6507955160908452E+136,
    1.4857159644817607E+138, 1.3520015276784023E+140, 1.24384140546413E+142,
    1.1567725070816409E+144, 1.0873661566567424E+146, 1.0329978488239052E+148,
    9.916779348709491E+149, 9.6192759682482062E+151, 9.426890448883242E+153,
    9.33262154439441E+155, 9.33262154439441E+157, 9.4259477598383536E+159,
    9.6144667150351211E+161, 9.9029007164861754E+163, 1.0299016745145622E+166,
    1.0813967582402903E+168, 1.1462805637347078E+170, 1.2265202031961373E+172,
    1.3246418194518284E+174, 1.4438595832024928E+176, 1.5882455415227421E+178,
    1.7629525510902437E+180, 1.9745068572210728E+182, 2.2311927486598123E+184,
    2.5435597334721862E+186, 2.9250936934930141E+188, 3.3931086844518965E+190,
    3.969937160808719E+192, 4.6845258497542883E+194, 5.5745857612076033E+196,
    6.6895029134491239E+198, 8.09429852527344E+200, 9.8750442008335976E+202,
    1.2146304367025325E+205, 1.5061417415111404E+207, 1.8826771768889254E+209,
    2.3721732428800459E+211, 3.0126600184576582E+213, 3.8562048236258025E+215,
    4.9745042224772855E+217, 6.4668554892204716E+219, 8.4715806908788174E+221,
    1.1182486511960039E+224, 1.4872707060906852E+226, 1.9929427461615181E+228,
    2.6904727073180495E+230, 3.6590428819525472E+232, 5.01288874827499E+234,
    6.9177864726194859E+236, 9.6157231969410859E+238, 1.346201247571752E+241,
    1.89814375907617E+243, 2.6953641378881614E+245, 3.8543707171800706E+247,
    5.5502938327393013E+249, 8.0479260574719866E+251, 1.17499720439091E+254,
    1.7272458904546376E+256, 2.5563239178728637E+258, 3.8089226376305671E+260,
    5.7133839564458505E+262, 8.6272097742332346E+264, 1.3113358856834518E+267,
    2.0063439050956811E+269, 3.0897696138473489E+271, 4.7891429014633912E+273,
    7.47106292628289E+275, 1.1729568794264138E+278, 1.8532718694937338E+280,
    2.9467022724950369E+282, 4.714723635992059E+284, 7.5907050539472148E+286,
    1.2296942187394488E+289, 2.0044015765453015E+291, 3.2872185855342945E+293,
    5.423910666131586E+295, 9.0036917057784329E+297, 1.5036165148649983E+300,
    2.5260757449731969E+302, 4.2690680090047027E+304, 7.257415615307994E+306 };

  if ((n < 0.0) || (floor(n) != n) || rtIsInf(n)) {
    b_n = (rtNaN);
  } else if (n > 170.0) {
    b_n = (rtInf);
  } else if (n < 1.0) {
    b_n = 1.0;
  } else {
    b_n = tmp[(int32_T)n - 1];
  }

  return b_n;
}

static real_T raspberrypi_multicore_MP_median(const real_T x[3])
{
  real_T y;
  int32_T b_k;
  int32_T exitg1;
  b_k = 0;
  do {
    exitg1 = 0;
    if (b_k < 3) {
      if (rtIsNaN(x[b_k])) {
        y = (rtNaN);
        exitg1 = 1;
      } else {
        b_k++;
      }
    } else {
      if (x[0] < x[1]) {
        if (x[1] < x[2]) {
          b_k = 2;
        } else if (x[0] < x[2]) {
          b_k = 3;
        } else {
          b_k = 1;
        }
      } else if (x[0] < x[2]) {
        b_k = 1;
      } else if (x[1] < x[2]) {
        b_k = 3;
      } else {
        b_k = 2;
      }

      y = x[b_k - 1];
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return y;
}

static void raspberrypi_multicore_Get1From2(real_T x2, real_T y2, real_T l1,
  real_T l2, real_T limit, real_T *xout, real_T *yout, real_T *Flag)
{
  real_T D_tmp_tmp;
  real_T E_tmp;
  real_T a;
  boolean_T guard1 = false;

  /*    INPUT: */
  /*    limit: */
  /*    0: y bigger */
  /*    1: y smaller */
  /*    2: x smaller */
  /*    3: x bigger */
  /*  OUTPUT: */
  /*  Flag:   0  normal */
  /*            1  infeasible */
  /*            2  limit intput error */
  *Flag = 0.0;
  raspberrypi_multicore_MPCtest_B.yo1_n = (0.0 - x2) * (0.0 - x2);
  D_tmp_tmp = (0.0 - y2) * (0.0 - y2);
  raspberrypi_multicore_MPCtest_B.F_f = raspberrypi_multicore_MPCtest_B.yo1_n +
    D_tmp_tmp;
  raspberrypi_multicore_MPCtest_B.D_i = sqrt(raspberrypi_multicore_MPCtest_B.F_f);
  raspberrypi_multicore_MPCtest_B.temp_ew = l1 + l2;
  guard1 = false;
  if (raspberrypi_multicore_MPCtest_B.temp_ew >
      raspberrypi_multicore_MPCtest_B.D_i) {
    a = l1 - l2;
    if (fabs(a) < raspberrypi_multicore_MPCtest_B.D_i) {
      E_tmp = l1 * l1;
      raspberrypi_multicore_MPCtest_B.D_i = l2 * l2;
      raspberrypi_multicore_MPCtest_B.E_p =
        ((raspberrypi_multicore_MPCtest_B.yo1_n - 0.0 * y2) * y2 + (-E_tmp +
          raspberrypi_multicore_MPCtest_B.D_i) * (0.0 - y2)) + rt_powd_snf(y2,
        3.0);
      raspberrypi_multicore_MPCtest_B.F_f *= 2.0;
      raspberrypi_multicore_MPCtest_B.temp_ew = sqrt
        (((raspberrypi_multicore_MPCtest_B.temp_ew *
           raspberrypi_multicore_MPCtest_B.temp_ew -
           raspberrypi_multicore_MPCtest_B.yo1_n) - D_tmp_tmp) *
         ((raspberrypi_multicore_MPCtest_B.yo1_n + -(a * a)) + D_tmp_tmp));
      a = x2 * raspberrypi_multicore_MPCtest_B.temp_ew;
      raspberrypi_multicore_MPCtest_B.yo1_n = ((0.0 *
        raspberrypi_multicore_MPCtest_B.temp_ew - a) +
        raspberrypi_multicore_MPCtest_B.E_p) /
        raspberrypi_multicore_MPCtest_B.F_f;
      *yout = ((a - 0.0 * raspberrypi_multicore_MPCtest_B.temp_ew) +
               raspberrypi_multicore_MPCtest_B.E_p) /
        raspberrypi_multicore_MPCtest_B.F_f;
      raspberrypi_multicore_MPCtest_B.E_p = sqrt(E_tmp -
        raspberrypi_multicore_MPCtest_B.yo1_n *
        raspberrypi_multicore_MPCtest_B.yo1_n);
      raspberrypi_multicore_MPCtest_B.temp_ew =
        raspberrypi_multicore_MPCtest_B.E_p - x2;
      a = raspberrypi_multicore_MPCtest_B.yo1_n - y2;
      if (fabs((raspberrypi_multicore_MPCtest_B.temp_ew *
                raspberrypi_multicore_MPCtest_B.temp_ew + a * a) -
               raspberrypi_multicore_MPCtest_B.D_i) > 0.0001) {
        raspberrypi_multicore_MPCtest_B.E_p =
          -raspberrypi_multicore_MPCtest_B.E_p;
      }

      *xout = sqrt(E_tmp - *yout * *yout);
      raspberrypi_multicore_MPCtest_B.temp_ew = *xout - x2;
      a = *yout - y2;
      if (fabs((raspberrypi_multicore_MPCtest_B.temp_ew *
                raspberrypi_multicore_MPCtest_B.temp_ew + a * a) -
               raspberrypi_multicore_MPCtest_B.D_i) > 0.0001) {
        *xout = -*xout;
      }

      if (fabs(raspberrypi_multicore_MPCtest_B.yo1_n - *yout) < 0.0001) {
        *xout = 0.0 - raspberrypi_multicore_MPCtest_B.E_p;
      }

      if ((int32_T)limit == 2) {
        if (raspberrypi_multicore_MPCtest_B.E_p < *xout) {
          *yout = raspberrypi_multicore_MPCtest_B.yo1_n;
          *xout = raspberrypi_multicore_MPCtest_B.E_p;
        }
      } else if (raspberrypi_multicore_MPCtest_B.E_p > *xout) {
        *yout = raspberrypi_multicore_MPCtest_B.yo1_n;
        *xout = raspberrypi_multicore_MPCtest_B.E_p;
      }
    } else {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    *xout = 99.0;
    *yout = 99.0;
    *Flag = 1.0;
  }
}

static void raspberrypi_multicore_IK_IK_one(const
  IK_raspberrypi_multicore_MPCt_T *obj, const real_T p[3], real_T LegNum, real_T
  Angle[3], real_T *Flag)
{
  /*  calculate one leg IK according to different leg index */
  /*  Flag: */
  /*      0  normal */
  /*      1  roll calculation error */
  /*      2  pX, pY out of workspace */
  /*      3  AD collides with DP */
  /*      4  elbow touches the ground */
  /*      5  leg index assignment error */
  /*  all error state will triger a three-zeros output */
  *Flag = 0.0;
  Angle[0] = 0.0;
  Angle[1] = 0.0;
  Angle[2] = 0.0;
  switch ((int32_T)LegNum) {
   case 1:
   case 3:
    raspberrypi_multicore_MPCtest_B.RP_h = sqrt((p[1] * p[1] + p[2] * p[2]) -
      obj->OR * obj->OR);
    raspberrypi_multicore_Get1From2(p[1], p[2], obj->OR,
      raspberrypi_multicore_MPCtest_B.RP_h, 3.0,
      &raspberrypi_multicore_MPCtest_B.b_pRy,
      &raspberrypi_multicore_MPCtest_B.b_pRz,
      &raspberrypi_multicore_MPCtest_B.b_errFlag);

    /*  angle for 13 angle */
    if (raspberrypi_multicore_MPCtest_B.b_errFlag != 0.0) {
      *Flag = 1.0;
    } else {
      raspberrypi_multicore_Get1From2(p[0],
        -raspberrypi_multicore_MPCtest_B.RP_h, obj->BC, obj->DP, 2.0,
        &raspberrypi_multicore_MPCtest_B.b_xout,
        &raspberrypi_multicore_MPCtest_B.b_yout,
        &raspberrypi_multicore_MPCtest_B.b_errFlag);
      if (raspberrypi_multicore_MPCtest_B.b_errFlag != 0.0) {
        *Flag = 2.0;
      } else {
        raspberrypi_multicore_MPCtest_B.pP_idx_0_c = p[0] -
          raspberrypi_multicore_MPCtest_B.b_xout;
        raspberrypi_multicore_MPCtest_B.pP_idx_1_h =
          -raspberrypi_multicore_MPCtest_B.RP_h -
          raspberrypi_multicore_MPCtest_B.b_yout;
        raspberrypi_multicore_MPCtest_B.b_errFlag = cos(-obj->CDP);
        raspberrypi_multicore_MPCtest_B.pC_tmp_k = sin(-obj->CDP);
        raspberrypi_multicore_MPCtest_B.pC_j =
          (raspberrypi_multicore_MPCtest_B.pC_tmp_k *
           raspberrypi_multicore_MPCtest_B.pP_idx_0_c +
           raspberrypi_multicore_MPCtest_B.b_errFlag *
           raspberrypi_multicore_MPCtest_B.pP_idx_1_h) / obj->DP * obj->AB +
          raspberrypi_multicore_MPCtest_B.b_yout;
        raspberrypi_multicore_MPCtest_B.pB_idx_1_o =
          raspberrypi_multicore_MPCtest_B.pC_j -
          raspberrypi_multicore_MPCtest_B.b_yout;

        /*  angle for 11 angle */
        /*  angle for 12 angle */
        if (-raspberrypi_multicore_MPCtest_B.RP_h >
            raspberrypi_multicore_MPCtest_B.b_yout) {
          *Flag = 3.0;

          /*  AD collides with DP */
        } else if (-raspberrypi_multicore_MPCtest_B.RP_h >
                   raspberrypi_multicore_MPCtest_B.pC_j) {
          *Flag = 4.0;

          /*  elbow touches the ground */
        } else {
          if (raspberrypi_multicore_MPCtest_B.b_yout < 0.0) {
            raspberrypi_multicore_MPCtest_B.RP_h = -1.0;
          } else if (raspberrypi_multicore_MPCtest_B.b_yout > 0.0) {
            raspberrypi_multicore_MPCtest_B.RP_h = 1.0;
          } else if (raspberrypi_multicore_MPCtest_B.b_yout == 0.0) {
            raspberrypi_multicore_MPCtest_B.RP_h = 0.0;
          } else {
            raspberrypi_multicore_MPCtest_B.RP_h = (rtNaN);
          }

          Angle[0] = acos((0.0 * raspberrypi_multicore_MPCtest_B.b_yout +
                           -raspberrypi_multicore_MPCtest_B.b_xout) / obj->BC) *
            raspberrypi_multicore_MPCtest_B.RP_h;
          if (-raspberrypi_multicore_MPCtest_B.pB_idx_1_o < 0.0) {
            raspberrypi_multicore_MPCtest_B.RP_h = -1.0;
          } else if (-raspberrypi_multicore_MPCtest_B.pB_idx_1_o > 0.0) {
            raspberrypi_multicore_MPCtest_B.RP_h = 1.0;
          } else if (-raspberrypi_multicore_MPCtest_B.pB_idx_1_o == 0.0) {
            raspberrypi_multicore_MPCtest_B.RP_h = 0.0;
          } else {
            raspberrypi_multicore_MPCtest_B.RP_h = (rtNaN);
          }

          Angle[1] = acos((-(((raspberrypi_multicore_MPCtest_B.b_errFlag *
                               raspberrypi_multicore_MPCtest_B.pP_idx_0_c +
                               -raspberrypi_multicore_MPCtest_B.pC_tmp_k *
                               raspberrypi_multicore_MPCtest_B.pP_idx_1_h) /
                              obj->DP * obj->AB +
                              raspberrypi_multicore_MPCtest_B.b_xout) -
                             raspberrypi_multicore_MPCtest_B.b_xout) + 0.0 *
                           raspberrypi_multicore_MPCtest_B.pB_idx_1_o) / obj->AB)
            * raspberrypi_multicore_MPCtest_B.RP_h;
          if (raspberrypi_multicore_MPCtest_B.b_pRz < 0.0) {
            raspberrypi_multicore_MPCtest_B.RP_h = -1.0;
          } else if (raspberrypi_multicore_MPCtest_B.b_pRz > 0.0) {
            raspberrypi_multicore_MPCtest_B.RP_h = 1.0;
          } else if (raspberrypi_multicore_MPCtest_B.b_pRz == 0.0) {
            raspberrypi_multicore_MPCtest_B.RP_h = 0.0;
          } else {
            raspberrypi_multicore_MPCtest_B.RP_h = (rtNaN);
          }

          Angle[2] = acos((0.0 * raspberrypi_multicore_MPCtest_B.b_pRz +
                           raspberrypi_multicore_MPCtest_B.b_pRy) / obj->OR) *
            raspberrypi_multicore_MPCtest_B.RP_h;
        }
      }
    }
    break;

   default:
    raspberrypi_multicore_MPCtest_B.RP_h = sqrt((p[1] * p[1] + p[2] * p[2]) -
      obj->OR * obj->OR);
    raspberrypi_multicore_Get1From2(p[1], p[2], obj->OR,
      raspberrypi_multicore_MPCtest_B.RP_h, 2.0,
      &raspberrypi_multicore_MPCtest_B.b_pRy,
      &raspberrypi_multicore_MPCtest_B.b_pRz,
      &raspberrypi_multicore_MPCtest_B.b_errFlag);

    /*  angle for 13 servo */
    if (raspberrypi_multicore_MPCtest_B.b_errFlag != 0.0) {
      *Flag = 1.0;
    } else {
      raspberrypi_multicore_Get1From2(p[0],
        -raspberrypi_multicore_MPCtest_B.RP_h, obj->BC, obj->DP, 2.0,
        &raspberrypi_multicore_MPCtest_B.b_xout,
        &raspberrypi_multicore_MPCtest_B.b_yout,
        &raspberrypi_multicore_MPCtest_B.b_errFlag);
      if (raspberrypi_multicore_MPCtest_B.b_errFlag != 0.0) {
        *Flag = 2.0;
      } else {
        raspberrypi_multicore_MPCtest_B.b_errFlag = sin(-obj->CDP);
        raspberrypi_multicore_MPCtest_B.pC_tmp_k = cos(-obj->CDP);
        raspberrypi_multicore_MPCtest_B.pP_idx_0_c = p[0] -
          raspberrypi_multicore_MPCtest_B.b_xout;
        raspberrypi_multicore_MPCtest_B.pP_idx_1_h =
          -raspberrypi_multicore_MPCtest_B.RP_h -
          raspberrypi_multicore_MPCtest_B.b_yout;
        raspberrypi_multicore_MPCtest_B.pC_j =
          (raspberrypi_multicore_MPCtest_B.b_errFlag *
           raspberrypi_multicore_MPCtest_B.pP_idx_0_c +
           raspberrypi_multicore_MPCtest_B.pC_tmp_k *
           raspberrypi_multicore_MPCtest_B.pP_idx_1_h) / obj->DP * obj->AB +
          raspberrypi_multicore_MPCtest_B.b_yout;
        raspberrypi_multicore_MPCtest_B.pB_idx_1_o =
          raspberrypi_multicore_MPCtest_B.pC_j -
          raspberrypi_multicore_MPCtest_B.b_yout;

        /*  angle for 11 servo */
        /*  angle for 12 servo */
        if (-raspberrypi_multicore_MPCtest_B.RP_h >
            raspberrypi_multicore_MPCtest_B.b_yout) {
          *Flag = 3.0;

          /*  AD collides with DP */
        } else if (-raspberrypi_multicore_MPCtest_B.RP_h >
                   raspberrypi_multicore_MPCtest_B.pC_j) {
          *Flag = 4.0;

          /*  elbow touches the ground */
        } else {
          if (-raspberrypi_multicore_MPCtest_B.b_yout < 0.0) {
            raspberrypi_multicore_MPCtest_B.RP_h = -1.0;
          } else if (-raspberrypi_multicore_MPCtest_B.b_yout > 0.0) {
            raspberrypi_multicore_MPCtest_B.RP_h = 1.0;
          } else if (-raspberrypi_multicore_MPCtest_B.b_yout == 0.0) {
            raspberrypi_multicore_MPCtest_B.RP_h = 0.0;
          } else {
            raspberrypi_multicore_MPCtest_B.RP_h = (rtNaN);
          }

          Angle[0] = acos((0.0 * raspberrypi_multicore_MPCtest_B.b_yout +
                           -raspberrypi_multicore_MPCtest_B.b_xout) / obj->BC) *
            raspberrypi_multicore_MPCtest_B.RP_h;
          if (raspberrypi_multicore_MPCtest_B.pB_idx_1_o < 0.0) {
            raspberrypi_multicore_MPCtest_B.RP_h = -1.0;
          } else if (raspberrypi_multicore_MPCtest_B.pB_idx_1_o > 0.0) {
            raspberrypi_multicore_MPCtest_B.RP_h = 1.0;
          } else if (raspberrypi_multicore_MPCtest_B.pB_idx_1_o == 0.0) {
            raspberrypi_multicore_MPCtest_B.RP_h = 0.0;
          } else {
            raspberrypi_multicore_MPCtest_B.RP_h = (rtNaN);
          }

          Angle[1] = acos((-(((raspberrypi_multicore_MPCtest_B.pC_tmp_k *
                               raspberrypi_multicore_MPCtest_B.pP_idx_0_c +
                               -raspberrypi_multicore_MPCtest_B.b_errFlag *
                               raspberrypi_multicore_MPCtest_B.pP_idx_1_h) /
                              obj->DP * obj->AB +
                              raspberrypi_multicore_MPCtest_B.b_xout) -
                             raspberrypi_multicore_MPCtest_B.b_xout) + 0.0 *
                           raspberrypi_multicore_MPCtest_B.pB_idx_1_o) / obj->AB)
            * raspberrypi_multicore_MPCtest_B.RP_h;
          if (-raspberrypi_multicore_MPCtest_B.b_pRz < 0.0) {
            raspberrypi_multicore_MPCtest_B.RP_h = -1.0;
          } else if (-raspberrypi_multicore_MPCtest_B.b_pRz > 0.0) {
            raspberrypi_multicore_MPCtest_B.RP_h = 1.0;
          } else if (-raspberrypi_multicore_MPCtest_B.b_pRz == 0.0) {
            raspberrypi_multicore_MPCtest_B.RP_h = 0.0;
          } else {
            raspberrypi_multicore_MPCtest_B.RP_h = (rtNaN);
          }

          Angle[2] = acos((0.0 * raspberrypi_multicore_MPCtest_B.b_pRz +
                           -raspberrypi_multicore_MPCtest_B.b_pRy) / obj->OR) *
            raspberrypi_multicore_MPCtest_B.RP_h;
        }
      }
    }
    break;
  }
}

real_T rt_roundd_snf(real_T u)
{
  real_T y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  int32_T u0_0;
  int32_T u1_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      u0_0 = 1;
    } else {
      u0_0 = -1;
    }

    if (u1 > 0.0) {
      u1_0 = 1;
    } else {
      u1_0 = -1;
    }

    y = atan2(u0_0, u1_0);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

static void raspberrypi_multicore_FK_FK_one(const
  FK_raspberrypi_multicore_MPCt_T *obj, const real_T Angle[3], real_T LegNum,
  real_T pP[3], real_T *Flag)
{
  real_T ORnow;
  real_T pP_tmp;
  real_T pP_tmp_0;
  real_T roll;
  real_T sita;
  int32_T i;
  boolean_T guard1 = false;

  /*  calculate one leg IK according to different leg index */
  /*  Flag: */
  /*      0  normal */
  /*      1  leg index assignment error */
  /*  all error state will triger a three-zeros output */
  *Flag = 0.0;
  guard1 = false;
  switch ((int32_T)LegNum) {
   case 1:
   case 3:
    raspberrypi_multicore_MPCtest_B.alpha = Angle[0];
    raspberrypi_multicore_MPCtest_B.beta = -Angle[1];
    roll = Angle[2];
    ORnow = obj->OR;
    guard1 = true;
    break;

   case 2:
   case 4:
    raspberrypi_multicore_MPCtest_B.alpha = -Angle[0];
    raspberrypi_multicore_MPCtest_B.beta = Angle[1];
    roll = Angle[2];
    ORnow = -obj->OR;
    guard1 = true;
    break;

   default:
    pP[0] = 0.0;
    pP[1] = 0.0;
    pP[2] = 0.0;
    *Flag = 1.0;
    break;
  }

  if (guard1) {
    /*  3D rotation matrix, from world to body */
    sita = ((-(3.1415926535897931 - fabs(raspberrypi_multicore_MPCtest_B.alpha -
               raspberrypi_multicore_MPCtest_B.beta)) +
             raspberrypi_multicore_MPCtest_B.alpha) + 3.1415926535897931) -
      obj->CDP;

    /*  3D rotation matrix, from world to body */
    /*  3D rotation matrix, from world to body */
    raspberrypi_multicore_MPCtest_B.beta = sin(roll);
    roll = cos(roll);
    pP_tmp = sin(raspberrypi_multicore_MPCtest_B.alpha);
    raspberrypi_multicore_MPCtest_B.alpha = cos
      (raspberrypi_multicore_MPCtest_B.alpha);
    pP_tmp_0 = sin(sita);
    sita = cos(sita);
    raspberrypi_multicore_MPCtest_B.pP_tmp[0] =
      raspberrypi_multicore_MPCtest_B.alpha;
    raspberrypi_multicore_MPCtest_B.pP_tmp[3] = 0.0;
    raspberrypi_multicore_MPCtest_B.pP_tmp[6] = pP_tmp;
    raspberrypi_multicore_MPCtest_B.pP_tmp[2] = -pP_tmp;
    raspberrypi_multicore_MPCtest_B.pP_tmp[5] = 0.0;
    raspberrypi_multicore_MPCtest_B.pP_tmp[8] =
      raspberrypi_multicore_MPCtest_B.alpha;
    raspberrypi_multicore_MPCtest_B.pP_tmp_f[0] = sita;
    raspberrypi_multicore_MPCtest_B.pP_tmp_f[3] = 0.0;
    raspberrypi_multicore_MPCtest_B.pP_tmp_f[6] = pP_tmp_0;
    raspberrypi_multicore_MPCtest_B.pP_tmp[1] = 0.0;
    raspberrypi_multicore_MPCtest_B.pP_tmp_f[1] = 0.0;
    raspberrypi_multicore_MPCtest_B.pP_tmp[4] = 1.0;
    raspberrypi_multicore_MPCtest_B.pP_tmp_f[4] = 1.0;
    raspberrypi_multicore_MPCtest_B.pP_tmp[7] = 0.0;
    raspberrypi_multicore_MPCtest_B.pP_tmp_f[7] = 0.0;
    raspberrypi_multicore_MPCtest_B.pP_tmp_f[2] = -pP_tmp_0;
    raspberrypi_multicore_MPCtest_B.pP_tmp_f[5] = 0.0;
    raspberrypi_multicore_MPCtest_B.pP_tmp_f[8] = sita;
    for (i = 0; i < 3; i++) {
      raspberrypi_multicore_MPCtest_B.pP_tmp_m[i] =
        raspberrypi_multicore_MPCtest_B.pP_tmp[i + 6] * 0.0 +
        raspberrypi_multicore_MPCtest_B.pP_tmp[i] * -obj->BC;
      raspberrypi_multicore_MPCtest_B.pP_tmp_mc[i] =
        raspberrypi_multicore_MPCtest_B.pP_tmp_f[i + 6] * 0.0 +
        raspberrypi_multicore_MPCtest_B.pP_tmp_f[i] * -obj->DP;
    }

    raspberrypi_multicore_MPCtest_B.pP_tmp[1] = 0.0;
    raspberrypi_multicore_MPCtest_B.pP_tmp[4] = roll;
    raspberrypi_multicore_MPCtest_B.pP_tmp[7] =
      -raspberrypi_multicore_MPCtest_B.beta;
    raspberrypi_multicore_MPCtest_B.pP_tmp[2] = 0.0;
    raspberrypi_multicore_MPCtest_B.pP_tmp[5] =
      raspberrypi_multicore_MPCtest_B.beta;
    raspberrypi_multicore_MPCtest_B.pP_tmp[8] = roll;
    raspberrypi_multicore_MPCtest_B.pP_tmp[0] = 1.0;
    raspberrypi_multicore_MPCtest_B.beta =
      raspberrypi_multicore_MPCtest_B.pP_tmp_m[0] +
      raspberrypi_multicore_MPCtest_B.pP_tmp_mc[0];
    raspberrypi_multicore_MPCtest_B.pP_tmp[3] = 0.0;
    ORnow += raspberrypi_multicore_MPCtest_B.pP_tmp_m[1] +
      raspberrypi_multicore_MPCtest_B.pP_tmp_mc[1];
    raspberrypi_multicore_MPCtest_B.pP_tmp[6] = 0.0;
    sita = raspberrypi_multicore_MPCtest_B.pP_tmp_m[2] +
      raspberrypi_multicore_MPCtest_B.pP_tmp_mc[2];
    for (i = 0; i < 3; i++) {
      pP[i] = raspberrypi_multicore_MPCtest_B.pP_tmp[i + 6] * sita +
        (raspberrypi_multicore_MPCtest_B.pP_tmp[i + 3] * ORnow +
         raspberrypi_multicore_MPCtest_B.pP_tmp[i] *
         raspberrypi_multicore_MPCtest_B.beta);
    }
  }
}

static void raspberrypi_multicore_M_mrdiv_a(const real_T A[54], const real_T B
  [36], real_T Y[54])
{
  int32_T a;
  int32_T b_j;
  int32_T c;
  int32_T c_0;
  int32_T ijA;
  int32_T jA;
  int32_T jj;
  int32_T k;
  int32_T kBcol;
  int8_T b_ipiv;
  memcpy(&raspberrypi_multicore_MPCtest_B.c_A_o[0], &B[0], 36U * sizeof(real_T));
  for (b_j = 0; b_j < 6; b_j++) {
    raspberrypi_multicore_MPCtest_B.b_ipiv[b_j] = (int8_T)(b_j + 1);
  }

  for (b_j = 0; b_j < 5; b_j++) {
    c = b_j * 7 + 2;
    jj = b_j * 7;
    c_0 = 6 - b_j;
    a = 1;
    raspberrypi_multicore_MPCtest_B.smax_p = fabs
      (raspberrypi_multicore_MPCtest_B.c_A_o[jj]);
    for (k = 2; k <= c_0; k++) {
      raspberrypi_multicore_MPCtest_B.s_b = fabs
        (raspberrypi_multicore_MPCtest_B.c_A_o[(c + k) - 3]);
      if (raspberrypi_multicore_MPCtest_B.s_b >
          raspberrypi_multicore_MPCtest_B.smax_p) {
        a = k;
        raspberrypi_multicore_MPCtest_B.smax_p =
          raspberrypi_multicore_MPCtest_B.s_b;
      }
    }

    if (raspberrypi_multicore_MPCtest_B.c_A_o[(c + a) - 3] != 0.0) {
      if (a - 1 != 0) {
        a += b_j;
        raspberrypi_multicore_MPCtest_B.b_ipiv[b_j] = (int8_T)a;
        for (k = 0; k < 6; k++) {
          jA = k * 6 + b_j;
          raspberrypi_multicore_MPCtest_B.smax_p =
            raspberrypi_multicore_MPCtest_B.c_A_o[jA];
          c_0 = (k * 6 + a) - 1;
          raspberrypi_multicore_MPCtest_B.c_A_o[jA] =
            raspberrypi_multicore_MPCtest_B.c_A_o[c_0];
          raspberrypi_multicore_MPCtest_B.c_A_o[c_0] =
            raspberrypi_multicore_MPCtest_B.smax_p;
        }
      }

      k = c - b_j;
      for (a = c; a <= k + 4; a++) {
        raspberrypi_multicore_MPCtest_B.c_A_o[a - 1] /=
          raspberrypi_multicore_MPCtest_B.c_A_o[jj];
      }
    }

    c_0 = 5 - b_j;
    jA = jj;
    for (a = 0; a < c_0; a++) {
      raspberrypi_multicore_MPCtest_B.smax_p =
        raspberrypi_multicore_MPCtest_B.c_A_o[(jj + a * 6) + 6];
      if (raspberrypi_multicore_MPCtest_B.smax_p != 0.0) {
        k = jA + 8;
        kBcol = jA - b_j;
        for (ijA = k; ijA <= kBcol + 12; ijA++) {
          raspberrypi_multicore_MPCtest_B.c_A_o[ijA - 1] +=
            raspberrypi_multicore_MPCtest_B.c_A_o[((c + ijA) - jA) - 9] *
            -raspberrypi_multicore_MPCtest_B.smax_p;
        }
      }

      jA += 6;
    }
  }

  memcpy(&Y[0], &A[0], 54U * sizeof(real_T));
  for (b_j = 0; b_j < 6; b_j++) {
    jj = 9 * b_j - 1;
    jA = 6 * b_j - 1;
    k = b_j - 1;
    for (a = 0; a <= k; a++) {
      kBcol = 9 * a - 1;
      raspberrypi_multicore_MPCtest_B.smax_p =
        raspberrypi_multicore_MPCtest_B.c_A_o[(a + jA) + 1];
      if (raspberrypi_multicore_MPCtest_B.smax_p != 0.0) {
        for (c_0 = 0; c_0 < 9; c_0++) {
          c = (c_0 + jj) + 1;
          Y[c] -= raspberrypi_multicore_MPCtest_B.smax_p * Y[(c_0 + kBcol) + 1];
        }
      }
    }

    raspberrypi_multicore_MPCtest_B.smax_p = 1.0 /
      raspberrypi_multicore_MPCtest_B.c_A_o[(b_j + jA) + 1];
    for (k = 0; k < 9; k++) {
      c = (k + jj) + 1;
      Y[c] *= raspberrypi_multicore_MPCtest_B.smax_p;
    }
  }

  for (a = 5; a >= 0; a--) {
    jj = 9 * a - 1;
    jA = 6 * a - 1;
    for (k = a + 2; k < 7; k++) {
      kBcol = (k - 1) * 9 - 1;
      raspberrypi_multicore_MPCtest_B.smax_p =
        raspberrypi_multicore_MPCtest_B.c_A_o[k + jA];
      if (raspberrypi_multicore_MPCtest_B.smax_p != 0.0) {
        for (c_0 = 0; c_0 < 9; c_0++) {
          c = (c_0 + jj) + 1;
          Y[c] -= Y[(c_0 + kBcol) + 1] * raspberrypi_multicore_MPCtest_B.smax_p;
        }
      }
    }
  }

  for (a = 4; a >= 0; a--) {
    b_ipiv = raspberrypi_multicore_MPCtest_B.b_ipiv[a];
    if (a + 1 != b_ipiv) {
      for (c_0 = 0; c_0 < 9; c_0++) {
        jA = 9 * a + c_0;
        raspberrypi_multicore_MPCtest_B.smax_p = Y[jA];
        c = 9 * (b_ipiv - 1) + c_0;
        Y[jA] = Y[c];
        Y[c] = raspberrypi_multicore_MPCtest_B.smax_p;
      }
    }
  }
}

static void KalmanFilter_DIY_Offrm_stepImpl(KalmanFilter_DIY_Offrm_raspbe_T *obj,
  const real_T u[3], const real_T y[6], const real_T x0[9], const real_T p0[81],
  const real_T Q[81], const real_T R[36], real_T Reset, real_T updateEN, real_T
  xhat[9], real_T P[81])
{
  int32_T obj_tmp;

  /*  note Q, R must be matrixes */
  /*  Q for process noise */
  /*  R for measurement noise */
  if (obj->count < 0.5) {
    memcpy(&obj->XOld[0], &x0[0], 9U * sizeof(real_T));
    memcpy(&obj->POld[0], &p0[0], 81U * sizeof(real_T));
    obj->count++;
  }

  for (raspberrypi_multicore_MPCtest_B.i_kg = 0;
       raspberrypi_multicore_MPCtest_B.i_kg < 9;
       raspberrypi_multicore_MPCtest_B.i_kg++) {
    raspberrypi_multicore_MPCtest_B.obj_o[raspberrypi_multicore_MPCtest_B.i_kg] =
      0.0;
    for (raspberrypi_multicore_MPCtest_B.i3 = 0;
         raspberrypi_multicore_MPCtest_B.i3 < 9;
         raspberrypi_multicore_MPCtest_B.i3++) {
      raspberrypi_multicore_MPCtest_B.obj_d[raspberrypi_multicore_MPCtest_B.i3 +
        9 * raspberrypi_multicore_MPCtest_B.i_kg] = 0.0;
      raspberrypi_multicore_MPCtest_B.obj_o[raspberrypi_multicore_MPCtest_B.i_kg]
        += obj->A[9 * raspberrypi_multicore_MPCtest_B.i3 +
        raspberrypi_multicore_MPCtest_B.i_kg] * obj->
        XOld[raspberrypi_multicore_MPCtest_B.i3];
    }

    raspberrypi_multicore_MPCtest_B.Xpre_o[raspberrypi_multicore_MPCtest_B.i_kg]
      =
      raspberrypi_multicore_MPCtest_B.obj_o[raspberrypi_multicore_MPCtest_B.i_kg]
      + (obj->B[raspberrypi_multicore_MPCtest_B.i_kg + 18] * u[2] + (obj->
          B[raspberrypi_multicore_MPCtest_B.i_kg + 9] * u[1] + obj->
          B[raspberrypi_multicore_MPCtest_B.i_kg] * u[0]));
  }

  for (raspberrypi_multicore_MPCtest_B.i_kg = 0;
       raspberrypi_multicore_MPCtest_B.i_kg < 9;
       raspberrypi_multicore_MPCtest_B.i_kg++) {
    for (raspberrypi_multicore_MPCtest_B.i3 = 0;
         raspberrypi_multicore_MPCtest_B.i3 < 9;
         raspberrypi_multicore_MPCtest_B.i3++) {
      for (raspberrypi_multicore_MPCtest_B.i4 = 0;
           raspberrypi_multicore_MPCtest_B.i4 < 9;
           raspberrypi_multicore_MPCtest_B.i4++) {
        obj_tmp = 9 * raspberrypi_multicore_MPCtest_B.i3 +
          raspberrypi_multicore_MPCtest_B.i_kg;
        raspberrypi_multicore_MPCtest_B.obj_d[obj_tmp] += obj->A[9 *
          raspberrypi_multicore_MPCtest_B.i4 +
          raspberrypi_multicore_MPCtest_B.i_kg] * obj->POld[9 *
          raspberrypi_multicore_MPCtest_B.i3 +
          raspberrypi_multicore_MPCtest_B.i4];
      }

      raspberrypi_multicore_MPCtest_B.obj_e[raspberrypi_multicore_MPCtest_B.i3 +
        9 * raspberrypi_multicore_MPCtest_B.i_kg] = 0.0;
    }
  }

  for (raspberrypi_multicore_MPCtest_B.i_kg = 0;
       raspberrypi_multicore_MPCtest_B.i_kg < 9;
       raspberrypi_multicore_MPCtest_B.i_kg++) {
    for (raspberrypi_multicore_MPCtest_B.i3 = 0;
         raspberrypi_multicore_MPCtest_B.i3 < 9;
         raspberrypi_multicore_MPCtest_B.i3++) {
      for (raspberrypi_multicore_MPCtest_B.i4 = 0;
           raspberrypi_multicore_MPCtest_B.i4 < 9;
           raspberrypi_multicore_MPCtest_B.i4++) {
        obj_tmp = 9 * raspberrypi_multicore_MPCtest_B.i3 +
          raspberrypi_multicore_MPCtest_B.i_kg;
        raspberrypi_multicore_MPCtest_B.obj_e[obj_tmp] += obj->G[9 *
          raspberrypi_multicore_MPCtest_B.i4 +
          raspberrypi_multicore_MPCtest_B.i_kg] * Q[9 *
          raspberrypi_multicore_MPCtest_B.i3 +
          raspberrypi_multicore_MPCtest_B.i4];
      }

      raspberrypi_multicore_MPCtest_B.obj_b[raspberrypi_multicore_MPCtest_B.i3 +
        9 * raspberrypi_multicore_MPCtest_B.i_kg] = 0.0;
    }
  }

  for (raspberrypi_multicore_MPCtest_B.i_kg = 0;
       raspberrypi_multicore_MPCtest_B.i_kg < 9;
       raspberrypi_multicore_MPCtest_B.i_kg++) {
    for (raspberrypi_multicore_MPCtest_B.i3 = 0;
         raspberrypi_multicore_MPCtest_B.i3 < 9;
         raspberrypi_multicore_MPCtest_B.i3++) {
      for (raspberrypi_multicore_MPCtest_B.i4 = 0;
           raspberrypi_multicore_MPCtest_B.i4 < 9;
           raspberrypi_multicore_MPCtest_B.i4++) {
        obj_tmp = 9 * raspberrypi_multicore_MPCtest_B.i3 +
          raspberrypi_multicore_MPCtest_B.i_kg;
        raspberrypi_multicore_MPCtest_B.obj_b[obj_tmp] +=
          raspberrypi_multicore_MPCtest_B.obj_d[9 *
          raspberrypi_multicore_MPCtest_B.i4 +
          raspberrypi_multicore_MPCtest_B.i_kg] * obj->A[9 *
          raspberrypi_multicore_MPCtest_B.i4 +
          raspberrypi_multicore_MPCtest_B.i3];
      }

      raspberrypi_multicore_MPCtest_B.obj_j[raspberrypi_multicore_MPCtest_B.i3 +
        9 * raspberrypi_multicore_MPCtest_B.i_kg] = 0.0;
    }
  }

  for (raspberrypi_multicore_MPCtest_B.i_kg = 0;
       raspberrypi_multicore_MPCtest_B.i_kg < 9;
       raspberrypi_multicore_MPCtest_B.i_kg++) {
    for (raspberrypi_multicore_MPCtest_B.i3 = 0;
         raspberrypi_multicore_MPCtest_B.i3 < 9;
         raspberrypi_multicore_MPCtest_B.i3++) {
      for (raspberrypi_multicore_MPCtest_B.i4 = 0;
           raspberrypi_multicore_MPCtest_B.i4 < 9;
           raspberrypi_multicore_MPCtest_B.i4++) {
        obj_tmp = 9 * raspberrypi_multicore_MPCtest_B.i3 +
          raspberrypi_multicore_MPCtest_B.i_kg;
        raspberrypi_multicore_MPCtest_B.obj_j[obj_tmp] +=
          raspberrypi_multicore_MPCtest_B.obj_e[9 *
          raspberrypi_multicore_MPCtest_B.i4 +
          raspberrypi_multicore_MPCtest_B.i_kg] * obj->G[9 *
          raspberrypi_multicore_MPCtest_B.i4 +
          raspberrypi_multicore_MPCtest_B.i3];
      }
    }
  }

  for (raspberrypi_multicore_MPCtest_B.i_kg = 0;
       raspberrypi_multicore_MPCtest_B.i_kg < 81;
       raspberrypi_multicore_MPCtest_B.i_kg++) {
    P[raspberrypi_multicore_MPCtest_B.i_kg] =
      raspberrypi_multicore_MPCtest_B.obj_b[raspberrypi_multicore_MPCtest_B.i_kg]
      + raspberrypi_multicore_MPCtest_B.obj_j[raspberrypi_multicore_MPCtest_B.i_kg];
  }

  for (raspberrypi_multicore_MPCtest_B.i_kg = 0;
       raspberrypi_multicore_MPCtest_B.i_kg < 6;
       raspberrypi_multicore_MPCtest_B.i_kg++) {
    for (raspberrypi_multicore_MPCtest_B.i3 = 0;
         raspberrypi_multicore_MPCtest_B.i3 < 9;
         raspberrypi_multicore_MPCtest_B.i3++) {
      raspberrypi_multicore_MPCtest_B.K_o[raspberrypi_multicore_MPCtest_B.i3 + 9
        * raspberrypi_multicore_MPCtest_B.i_kg] = obj->C[6 *
        raspberrypi_multicore_MPCtest_B.i3 +
        raspberrypi_multicore_MPCtest_B.i_kg];
    }
  }

  for (raspberrypi_multicore_MPCtest_B.i_kg = 0;
       raspberrypi_multicore_MPCtest_B.i_kg < 9;
       raspberrypi_multicore_MPCtest_B.i_kg++) {
    for (raspberrypi_multicore_MPCtest_B.i3 = 0;
         raspberrypi_multicore_MPCtest_B.i3 < 6;
         raspberrypi_multicore_MPCtest_B.i3++) {
      raspberrypi_multicore_MPCtest_B.obj_n[raspberrypi_multicore_MPCtest_B.i3 +
        6 * raspberrypi_multicore_MPCtest_B.i_kg] = 0.0;
    }
  }

  for (raspberrypi_multicore_MPCtest_B.i3 = 0;
       raspberrypi_multicore_MPCtest_B.i3 < 6;
       raspberrypi_multicore_MPCtest_B.i3++) {
    for (raspberrypi_multicore_MPCtest_B.i_kg = 0;
         raspberrypi_multicore_MPCtest_B.i_kg < 9;
         raspberrypi_multicore_MPCtest_B.i_kg++) {
      for (raspberrypi_multicore_MPCtest_B.i4 = 0;
           raspberrypi_multicore_MPCtest_B.i4 < 9;
           raspberrypi_multicore_MPCtest_B.i4++) {
        obj_tmp = 6 * raspberrypi_multicore_MPCtest_B.i_kg +
          raspberrypi_multicore_MPCtest_B.i3;
        raspberrypi_multicore_MPCtest_B.obj_n[obj_tmp] += obj->C[6 *
          raspberrypi_multicore_MPCtest_B.i4 +
          raspberrypi_multicore_MPCtest_B.i3] * P[9 *
          raspberrypi_multicore_MPCtest_B.i_kg +
          raspberrypi_multicore_MPCtest_B.i4];
      }

      raspberrypi_multicore_MPCtest_B.P[raspberrypi_multicore_MPCtest_B.i_kg + 9
        * raspberrypi_multicore_MPCtest_B.i3] = 0.0;
    }

    for (raspberrypi_multicore_MPCtest_B.i_kg = 0;
         raspberrypi_multicore_MPCtest_B.i_kg < 9;
         raspberrypi_multicore_MPCtest_B.i_kg++) {
      for (raspberrypi_multicore_MPCtest_B.i4 = 0;
           raspberrypi_multicore_MPCtest_B.i4 < 9;
           raspberrypi_multicore_MPCtest_B.i4++) {
        obj_tmp = 9 * raspberrypi_multicore_MPCtest_B.i3 +
          raspberrypi_multicore_MPCtest_B.i4;
        raspberrypi_multicore_MPCtest_B.P[obj_tmp] += P[9 *
          raspberrypi_multicore_MPCtest_B.i_kg +
          raspberrypi_multicore_MPCtest_B.i4] *
          raspberrypi_multicore_MPCtest_B.K_o[9 *
          raspberrypi_multicore_MPCtest_B.i3 +
          raspberrypi_multicore_MPCtest_B.i_kg];
      }
    }

    for (raspberrypi_multicore_MPCtest_B.i_kg = 0;
         raspberrypi_multicore_MPCtest_B.i_kg < 6;
         raspberrypi_multicore_MPCtest_B.i_kg++) {
      raspberrypi_multicore_MPCtest_B.d_k = 0.0;
      for (raspberrypi_multicore_MPCtest_B.i4 = 0;
           raspberrypi_multicore_MPCtest_B.i4 < 9;
           raspberrypi_multicore_MPCtest_B.i4++) {
        raspberrypi_multicore_MPCtest_B.d_k +=
          raspberrypi_multicore_MPCtest_B.obj_n[6 *
          raspberrypi_multicore_MPCtest_B.i4 +
          raspberrypi_multicore_MPCtest_B.i3] *
          raspberrypi_multicore_MPCtest_B.K_o[9 *
          raspberrypi_multicore_MPCtest_B.i_kg +
          raspberrypi_multicore_MPCtest_B.i4];
      }

      obj_tmp = 6 * raspberrypi_multicore_MPCtest_B.i_kg +
        raspberrypi_multicore_MPCtest_B.i3;
      raspberrypi_multicore_MPCtest_B.obj_i[obj_tmp] = R[obj_tmp] +
        raspberrypi_multicore_MPCtest_B.d_k;
    }
  }

  raspberrypi_multicore_M_mrdiv_a(raspberrypi_multicore_MPCtest_B.P,
    raspberrypi_multicore_MPCtest_B.obj_i, raspberrypi_multicore_MPCtest_B.K_o);
  if (updateEN < 0.5) {
    for (raspberrypi_multicore_MPCtest_B.i_kg = 0;
         raspberrypi_multicore_MPCtest_B.i_kg < 54;
         raspberrypi_multicore_MPCtest_B.i_kg++) {
      raspberrypi_multicore_MPCtest_B.K_o[raspberrypi_multicore_MPCtest_B.i_kg] *=
        0.0;
    }
  } else {
    memset(&raspberrypi_multicore_MPCtest_B.b_I_a[0], 0, 81U * sizeof(int8_T));
    for (raspberrypi_multicore_MPCtest_B.i_kg = 0;
         raspberrypi_multicore_MPCtest_B.i_kg < 9;
         raspberrypi_multicore_MPCtest_B.i_kg++) {
      raspberrypi_multicore_MPCtest_B.b_I_a[raspberrypi_multicore_MPCtest_B.i_kg
        + 9 * raspberrypi_multicore_MPCtest_B.i_kg] = 1;
    }

    for (raspberrypi_multicore_MPCtest_B.i_kg = 0;
         raspberrypi_multicore_MPCtest_B.i_kg < 9;
         raspberrypi_multicore_MPCtest_B.i_kg++) {
      for (raspberrypi_multicore_MPCtest_B.i3 = 0;
           raspberrypi_multicore_MPCtest_B.i3 < 9;
           raspberrypi_multicore_MPCtest_B.i3++) {
        raspberrypi_multicore_MPCtest_B.d_k = 0.0;
        for (raspberrypi_multicore_MPCtest_B.i4 = 0;
             raspberrypi_multicore_MPCtest_B.i4 < 6;
             raspberrypi_multicore_MPCtest_B.i4++) {
          raspberrypi_multicore_MPCtest_B.d_k +=
            raspberrypi_multicore_MPCtest_B.K_o[9 *
            raspberrypi_multicore_MPCtest_B.i4 +
            raspberrypi_multicore_MPCtest_B.i_kg] * obj->C[6 *
            raspberrypi_multicore_MPCtest_B.i3 +
            raspberrypi_multicore_MPCtest_B.i4];
        }

        obj_tmp = 9 * raspberrypi_multicore_MPCtest_B.i3 +
          raspberrypi_multicore_MPCtest_B.i_kg;
        raspberrypi_multicore_MPCtest_B.obj_b[obj_tmp] = (real_T)
          raspberrypi_multicore_MPCtest_B.b_I_a[obj_tmp] -
          raspberrypi_multicore_MPCtest_B.d_k;
        raspberrypi_multicore_MPCtest_B.obj_j[raspberrypi_multicore_MPCtest_B.i3
          + 9 * raspberrypi_multicore_MPCtest_B.i_kg] = 0.0;
      }
    }

    for (raspberrypi_multicore_MPCtest_B.i_kg = 0;
         raspberrypi_multicore_MPCtest_B.i_kg < 9;
         raspberrypi_multicore_MPCtest_B.i_kg++) {
      for (raspberrypi_multicore_MPCtest_B.i3 = 0;
           raspberrypi_multicore_MPCtest_B.i3 < 9;
           raspberrypi_multicore_MPCtest_B.i3++) {
        for (raspberrypi_multicore_MPCtest_B.i4 = 0;
             raspberrypi_multicore_MPCtest_B.i4 < 9;
             raspberrypi_multicore_MPCtest_B.i4++) {
          obj_tmp = 9 * raspberrypi_multicore_MPCtest_B.i_kg +
            raspberrypi_multicore_MPCtest_B.i4;
          raspberrypi_multicore_MPCtest_B.obj_j[obj_tmp] +=
            raspberrypi_multicore_MPCtest_B.obj_b[9 *
            raspberrypi_multicore_MPCtest_B.i3 +
            raspberrypi_multicore_MPCtest_B.i4] * P[9 *
            raspberrypi_multicore_MPCtest_B.i_kg +
            raspberrypi_multicore_MPCtest_B.i3];
        }
      }
    }

    memcpy(&P[0], &raspberrypi_multicore_MPCtest_B.obj_j[0], 81U * sizeof(real_T));
  }

  for (raspberrypi_multicore_MPCtest_B.i_kg = 0;
       raspberrypi_multicore_MPCtest_B.i_kg < 6;
       raspberrypi_multicore_MPCtest_B.i_kg++) {
    raspberrypi_multicore_MPCtest_B.d_k = 0.0;
    for (raspberrypi_multicore_MPCtest_B.i3 = 0;
         raspberrypi_multicore_MPCtest_B.i3 < 9;
         raspberrypi_multicore_MPCtest_B.i3++) {
      raspberrypi_multicore_MPCtest_B.d_k += obj->C[6 *
        raspberrypi_multicore_MPCtest_B.i3 +
        raspberrypi_multicore_MPCtest_B.i_kg] *
        raspberrypi_multicore_MPCtest_B.Xpre_o[raspberrypi_multicore_MPCtest_B.i3];
    }

    raspberrypi_multicore_MPCtest_B.y_c[raspberrypi_multicore_MPCtest_B.i_kg] =
      y[raspberrypi_multicore_MPCtest_B.i_kg] -
      raspberrypi_multicore_MPCtest_B.d_k;
  }

  for (raspberrypi_multicore_MPCtest_B.i_kg = 0;
       raspberrypi_multicore_MPCtest_B.i_kg < 9;
       raspberrypi_multicore_MPCtest_B.i_kg++) {
    raspberrypi_multicore_MPCtest_B.d_k = 0.0;
    for (raspberrypi_multicore_MPCtest_B.i3 = 0;
         raspberrypi_multicore_MPCtest_B.i3 < 6;
         raspberrypi_multicore_MPCtest_B.i3++) {
      raspberrypi_multicore_MPCtest_B.d_k +=
        raspberrypi_multicore_MPCtest_B.K_o[9 *
        raspberrypi_multicore_MPCtest_B.i3 +
        raspberrypi_multicore_MPCtest_B.i_kg] *
        raspberrypi_multicore_MPCtest_B.y_c[raspberrypi_multicore_MPCtest_B.i3];
    }

    xhat[raspberrypi_multicore_MPCtest_B.i_kg] =
      raspberrypi_multicore_MPCtest_B.Xpre_o[raspberrypi_multicore_MPCtest_B.i_kg]
      + raspberrypi_multicore_MPCtest_B.d_k;
  }

  if (Reset > 0.5) {
    memcpy(&xhat[0], &x0[0], 9U * sizeof(real_T));
    memcpy(&P[0], &p0[0], 81U * sizeof(real_T));
  }

  memcpy(&obj->XOld[0], &xhat[0], 9U * sizeof(real_T));
  memcpy(&obj->POld[0], &P[0], 81U * sizeof(real_T));
}

static void raspberrypi_multicore_MP_DS_gen(real_T Ts, real_T m, const real_T
  in3[3], const real_T in4[9], const real_T in5[3], const real_T in6[3], const
  real_T in7[3], const real_T in8[3], real_T Ad[169], real_T Bd[156])
{
  real_T t147_tmp;
  real_T t147_tmp_0;
  real_T t153_tmp;
  real_T t153_tmp_0;
  real_T t154_tmp;

  /*  subfunction */
  /* DS_GEN */
  /*     [AD,BD] = DS_GEN(TS,M,IN3,IN4,IN5,IN6,IN7,IN8) */
  /*     This function was generated by the Symbolic Math Toolbox version 8.7. */
  /*     29-Sep-2021 16:12:07 */
  raspberrypi_multicore_MPCtest_B.t2 = cos(in3[2]);
  raspberrypi_multicore_MPCtest_B.t3 = sin(in3[2]);
  raspberrypi_multicore_MPCtest_B.t28 = Ts * Ts;
  raspberrypi_multicore_MPCtest_B.t31 = 1.0 / m;
  raspberrypi_multicore_MPCtest_B.t92 = Ts * raspberrypi_multicore_MPCtest_B.t31;
  raspberrypi_multicore_MPCtest_B.t31 = raspberrypi_multicore_MPCtest_B.t28 *
    raspberrypi_multicore_MPCtest_B.t31 / 2.0;
  raspberrypi_multicore_MPCtest_B.t117 = in5[1] * in4[2] + -(in5[0] * in4[5]);
  raspberrypi_multicore_MPCtest_B.t118 = in4[2] * in5[2] + -(in5[0] * in4[8]);
  raspberrypi_multicore_MPCtest_B.t119 = in5[2] * in4[5] + -(in5[1] * in4[8]);
  raspberrypi_multicore_MPCtest_B.t120 = in6[1] * in4[2] + -(in6[0] * in4[5]);
  raspberrypi_multicore_MPCtest_B.t121 = in4[2] * in6[2] + -(in6[0] * in4[8]);
  raspberrypi_multicore_MPCtest_B.t122 = in6[2] * in4[5] + -(in6[1] * in4[8]);
  raspberrypi_multicore_MPCtest_B.t123 = in7[1] * in4[2] + -(in7[0] * in4[5]);
  raspberrypi_multicore_MPCtest_B.t124 = in4[2] * in7[2] + -(in7[0] * in4[8]);
  raspberrypi_multicore_MPCtest_B.t125 = in7[2] * in4[5] + -(in7[1] * in4[8]);
  raspberrypi_multicore_MPCtest_B.t126 = in8[1] * in4[2] + -(in8[0] * in4[5]);
  raspberrypi_multicore_MPCtest_B.t127 = in4[2] * in8[2] + -(in8[0] * in4[8]);
  raspberrypi_multicore_MPCtest_B.t128 = in8[2] * in4[5] + -(in8[1] * in4[8]);
  raspberrypi_multicore_MPCtest_B.t132_tmp = in4[0] * in5[1];
  raspberrypi_multicore_MPCtest_B.t132_tmp_m = in5[0] * in4[3];
  raspberrypi_multicore_MPCtest_B.t132 =
    raspberrypi_multicore_MPCtest_B.t132_tmp / 2.0 +
    -(raspberrypi_multicore_MPCtest_B.t132_tmp_m / 2.0);
  raspberrypi_multicore_MPCtest_B.t133_tmp = in4[0] * in5[2];
  raspberrypi_multicore_MPCtest_B.t133_tmp_e = in5[0] * in4[6];
  raspberrypi_multicore_MPCtest_B.t133 =
    raspberrypi_multicore_MPCtest_B.t133_tmp / 2.0 +
    -(raspberrypi_multicore_MPCtest_B.t133_tmp_e / 2.0);
  raspberrypi_multicore_MPCtest_B.t134_tmp = in5[2] * in4[3];
  raspberrypi_multicore_MPCtest_B.t134_tmp_m = in5[1] * in4[6];
  raspberrypi_multicore_MPCtest_B.t134 =
    raspberrypi_multicore_MPCtest_B.t134_tmp / 2.0 +
    -(raspberrypi_multicore_MPCtest_B.t134_tmp_m / 2.0);
  raspberrypi_multicore_MPCtest_B.t135_tmp = in4[0] * in6[1];
  raspberrypi_multicore_MPCtest_B.t135_tmp_d = in6[0] * in4[3];
  raspberrypi_multicore_MPCtest_B.t135 =
    raspberrypi_multicore_MPCtest_B.t135_tmp / 2.0 +
    -(raspberrypi_multicore_MPCtest_B.t135_tmp_d / 2.0);
  raspberrypi_multicore_MPCtest_B.t136_tmp = in4[1] * in5[1];
  raspberrypi_multicore_MPCtest_B.t136_tmp_j = in5[0] * in4[4];
  raspberrypi_multicore_MPCtest_B.t136 =
    raspberrypi_multicore_MPCtest_B.t136_tmp / 2.0 +
    -(raspberrypi_multicore_MPCtest_B.t136_tmp_j / 2.0);
  raspberrypi_multicore_MPCtest_B.t137_tmp = in4[0] * in6[2];
  raspberrypi_multicore_MPCtest_B.t137_tmp_g = in6[0] * in4[6];
  raspberrypi_multicore_MPCtest_B.t137 =
    raspberrypi_multicore_MPCtest_B.t137_tmp / 2.0 +
    -(raspberrypi_multicore_MPCtest_B.t137_tmp_g / 2.0);
  raspberrypi_multicore_MPCtest_B.t138_tmp = in4[1] * in5[2];
  raspberrypi_multicore_MPCtest_B.t138_tmp_m = in5[0] * in4[7];
  raspberrypi_multicore_MPCtest_B.t138 =
    raspberrypi_multicore_MPCtest_B.t138_tmp / 2.0 +
    -(raspberrypi_multicore_MPCtest_B.t138_tmp_m / 2.0);
  raspberrypi_multicore_MPCtest_B.t139_tmp = in6[2] * in4[3];
  raspberrypi_multicore_MPCtest_B.t139_tmp_f = in6[1] * in4[6];
  raspberrypi_multicore_MPCtest_B.t139 =
    raspberrypi_multicore_MPCtest_B.t139_tmp / 2.0 +
    -(raspberrypi_multicore_MPCtest_B.t139_tmp_f / 2.0);
  raspberrypi_multicore_MPCtest_B.t140_tmp = in5[2] * in4[4];
  raspberrypi_multicore_MPCtest_B.t140_tmp_j = in5[1] * in4[7];
  raspberrypi_multicore_MPCtest_B.t140 =
    raspberrypi_multicore_MPCtest_B.t140_tmp / 2.0 +
    -(raspberrypi_multicore_MPCtest_B.t140_tmp_j / 2.0);
  raspberrypi_multicore_MPCtest_B.t141_tmp = in4[0] * in7[1];
  raspberrypi_multicore_MPCtest_B.t141_tmp_n = in7[0] * in4[3];
  raspberrypi_multicore_MPCtest_B.t141 =
    raspberrypi_multicore_MPCtest_B.t141_tmp / 2.0 +
    -(raspberrypi_multicore_MPCtest_B.t141_tmp_n / 2.0);
  raspberrypi_multicore_MPCtest_B.t142_tmp = in4[1] * in6[1];
  raspberrypi_multicore_MPCtest_B.t142_tmp_n = in6[0] * in4[4];
  raspberrypi_multicore_MPCtest_B.t142 =
    raspberrypi_multicore_MPCtest_B.t142_tmp / 2.0 +
    -(raspberrypi_multicore_MPCtest_B.t142_tmp_n / 2.0);
  raspberrypi_multicore_MPCtest_B.t143_tmp = in4[0] * in7[2];
  raspberrypi_multicore_MPCtest_B.t143_tmp_p = in7[0] * in4[6];
  raspberrypi_multicore_MPCtest_B.t143 =
    raspberrypi_multicore_MPCtest_B.t143_tmp / 2.0 +
    -(raspberrypi_multicore_MPCtest_B.t143_tmp_p / 2.0);
  raspberrypi_multicore_MPCtest_B.t144_tmp = in4[1] * in6[2];
  raspberrypi_multicore_MPCtest_B.t144_tmp_n = in6[0] * in4[7];
  raspberrypi_multicore_MPCtest_B.t144 =
    raspberrypi_multicore_MPCtest_B.t144_tmp / 2.0 +
    -(raspberrypi_multicore_MPCtest_B.t144_tmp_n / 2.0);
  raspberrypi_multicore_MPCtest_B.t145_tmp = in7[2] * in4[3];
  raspberrypi_multicore_MPCtest_B.t145_tmp_c = in7[1] * in4[6];
  raspberrypi_multicore_MPCtest_B.t145 =
    raspberrypi_multicore_MPCtest_B.t145_tmp / 2.0 +
    -(raspberrypi_multicore_MPCtest_B.t145_tmp_c / 2.0);
  raspberrypi_multicore_MPCtest_B.t146_tmp = in6[2] * in4[4];
  raspberrypi_multicore_MPCtest_B.t146_tmp_a = in6[1] * in4[7];
  raspberrypi_multicore_MPCtest_B.t146 =
    raspberrypi_multicore_MPCtest_B.t146_tmp / 2.0 +
    -(raspberrypi_multicore_MPCtest_B.t146_tmp_a / 2.0);
  t147_tmp = in4[0] * in8[1];
  t147_tmp_0 = in8[0] * in4[3];
  raspberrypi_multicore_MPCtest_B.t147 = t147_tmp / 2.0 + -(t147_tmp_0 / 2.0);
  raspberrypi_multicore_MPCtest_B.t148_tmp = in4[1] * in7[1];
  raspberrypi_multicore_MPCtest_B.t148_tmp_k = in7[0] * in4[4];
  raspberrypi_multicore_MPCtest_B.t148 =
    raspberrypi_multicore_MPCtest_B.t148_tmp / 2.0 +
    -(raspberrypi_multicore_MPCtest_B.t148_tmp_k / 2.0);
  raspberrypi_multicore_MPCtest_B.t149_tmp = in4[0] * in8[2];
  raspberrypi_multicore_MPCtest_B.t149_tmp_g = in8[0] * in4[6];
  raspberrypi_multicore_MPCtest_B.t149 =
    raspberrypi_multicore_MPCtest_B.t149_tmp / 2.0 +
    -(raspberrypi_multicore_MPCtest_B.t149_tmp_g / 2.0);
  raspberrypi_multicore_MPCtest_B.t150_tmp = in4[1] * in7[2];
  raspberrypi_multicore_MPCtest_B.t150_tmp_p = in7[0] * in4[7];
  raspberrypi_multicore_MPCtest_B.t150 =
    raspberrypi_multicore_MPCtest_B.t150_tmp / 2.0 +
    -(raspberrypi_multicore_MPCtest_B.t150_tmp_p / 2.0);
  raspberrypi_multicore_MPCtest_B.t151_tmp = in8[2] * in4[3];
  raspberrypi_multicore_MPCtest_B.t151_tmp_n = in8[1] * in4[6];
  raspberrypi_multicore_MPCtest_B.t151 =
    raspberrypi_multicore_MPCtest_B.t151_tmp / 2.0 +
    -(raspberrypi_multicore_MPCtest_B.t151_tmp_n / 2.0);
  raspberrypi_multicore_MPCtest_B.t152_tmp = in7[2] * in4[4];
  raspberrypi_multicore_MPCtest_B.t152_tmp_f = in7[1] * in4[7];
  raspberrypi_multicore_MPCtest_B.t152 =
    raspberrypi_multicore_MPCtest_B.t152_tmp / 2.0 +
    -(raspberrypi_multicore_MPCtest_B.t152_tmp_f / 2.0);
  t153_tmp = in4[1] * in8[1];
  t153_tmp_0 = in8[0] * in4[4];
  raspberrypi_multicore_MPCtest_B.t153 = t153_tmp / 2.0 + -(t153_tmp_0 / 2.0);
  raspberrypi_multicore_MPCtest_B.t154_tmp = in4[1] * in8[2];
  t154_tmp = in8[0] * in4[7];
  raspberrypi_multicore_MPCtest_B.t154 =
    raspberrypi_multicore_MPCtest_B.t154_tmp / 2.0 + -(t154_tmp / 2.0);
  raspberrypi_multicore_MPCtest_B.t155_tmp = in8[2] * in4[4];
  raspberrypi_multicore_MPCtest_B.t155_tmp_o = in8[1] * in4[7];
  raspberrypi_multicore_MPCtest_B.t155 =
    raspberrypi_multicore_MPCtest_B.t155_tmp / 2.0 +
    -(raspberrypi_multicore_MPCtest_B.t155_tmp_o / 2.0);
  raspberrypi_multicore_MPCtest_B.t131 = 1.0 /
    (raspberrypi_multicore_MPCtest_B.t2 * raspberrypi_multicore_MPCtest_B.t2 +
     raspberrypi_multicore_MPCtest_B.t3 * raspberrypi_multicore_MPCtest_B.t3);
  raspberrypi_multicore_MPCtest_B.t156 = Ts * raspberrypi_multicore_MPCtest_B.t2
    * raspberrypi_multicore_MPCtest_B.t131;
  raspberrypi_multicore_MPCtest_B.t131 *= Ts *
    raspberrypi_multicore_MPCtest_B.t3;
  Ad[0] = 1.0;
  memset(&Ad[1], 0, 13U * sizeof(real_T));
  Ad[14] = 1.0;
  memset(&Ad[15], 0, 13U * sizeof(real_T));
  Ad[28] = 1.0;
  memset(&Ad[29], 0, 13U * sizeof(real_T));
  Ad[42] = 1.0;
  memset(&Ad[43], 0, 13U * sizeof(real_T));
  Ad[56] = 1.0;
  memset(&Ad[57], 0, 13U * sizeof(real_T));
  Ad[70] = 1.0;
  Ad[71] = 0.0;
  Ad[72] = 0.0;
  Ad[73] = 0.0;
  Ad[74] = 0.0;
  Ad[75] = 0.0;
  Ad[76] = 0.0;
  Ad[77] = 0.0;
  Ad[78] = Ts;
  Ad[79] = 0.0;
  Ad[80] = 0.0;
  Ad[81] = 0.0;
  Ad[82] = 0.0;
  Ad[83] = 0.0;
  Ad[84] = 1.0;
  Ad[85] = 0.0;
  Ad[86] = 0.0;
  Ad[87] = 0.0;
  Ad[88] = 0.0;
  Ad[89] = 0.0;
  Ad[90] = 0.0;
  Ad[91] = 0.0;
  Ad[92] = Ts;
  Ad[93] = 0.0;
  Ad[94] = 0.0;
  Ad[95] = 0.0;
  Ad[96] = 0.0;
  Ad[97] = 0.0;
  Ad[98] = 1.0;
  Ad[99] = 0.0;
  Ad[100] = 0.0;
  Ad[101] = 0.0;
  Ad[102] = 0.0;
  Ad[103] = 0.0;
  Ad[104] = 0.0;
  Ad[105] = 0.0;
  Ad[106] = Ts;
  Ad[107] = 0.0;
  Ad[108] = 0.0;
  Ad[109] = 0.0;
  Ad[110] = 0.0;
  Ad[111] = 0.0;
  Ad[112] = 1.0;
  Ad[113] = 0.0;
  Ad[114] = 0.0;
  Ad[115] = 0.0;
  Ad[116] = 0.0;
  Ad[117] = 0.0;
  Ad[118] = 0.0;
  Ad[119] = 0.0;
  Ad[120] = raspberrypi_multicore_MPCtest_B.t156;
  Ad[121] = -raspberrypi_multicore_MPCtest_B.t131;
  Ad[122] = 0.0;
  Ad[123] = 0.0;
  Ad[124] = 0.0;
  Ad[125] = 0.0;
  Ad[126] = 1.0;
  Ad[127] = 0.0;
  Ad[128] = 0.0;
  Ad[129] = 0.0;
  Ad[130] = 0.0;
  Ad[131] = 0.0;
  Ad[132] = 0.0;
  Ad[133] = raspberrypi_multicore_MPCtest_B.t131;
  Ad[134] = raspberrypi_multicore_MPCtest_B.t156;
  Ad[135] = 0.0;
  Ad[136] = 0.0;
  Ad[137] = 0.0;
  Ad[138] = 0.0;
  Ad[139] = 0.0;
  Ad[140] = 1.0;
  Ad[141] = 0.0;
  Ad[142] = 0.0;
  Ad[143] = 0.0;
  Ad[144] = 0.0;
  Ad[145] = 0.0;
  Ad[146] = 0.0;
  Ad[147] = 0.0;
  Ad[148] = Ts;
  Ad[149] = 0.0;
  Ad[150] = 0.0;
  Ad[151] = 0.0;
  Ad[152] = 0.0;
  Ad[153] = 0.0;
  Ad[154] = 1.0;
  Ad[155] = 0.0;
  Ad[156] = 0.0;
  Ad[157] = 0.0;
  Ad[158] = raspberrypi_multicore_MPCtest_B.t28 * -0.5;
  Ad[159] = 0.0;
  Ad[160] = 0.0;
  Ad[161] = 0.0;
  Ad[162] = 0.0;
  Ad[163] = 0.0;
  Ad[164] = -Ts;
  Ad[165] = 0.0;
  Ad[166] = 0.0;
  Ad[167] = 0.0;
  Ad[168] = 1.0;
  Bd[0] = raspberrypi_multicore_MPCtest_B.t31;
  Bd[1] = 0.0;
  Bd[2] = 0.0;
  raspberrypi_multicore_MPCtest_B.t156 = raspberrypi_multicore_MPCtest_B.t2 *
    raspberrypi_multicore_MPCtest_B.t28;
  raspberrypi_multicore_MPCtest_B.t131 = raspberrypi_multicore_MPCtest_B.t3 *
    raspberrypi_multicore_MPCtest_B.t28;
  Bd[3] = raspberrypi_multicore_MPCtest_B.t156 *
    raspberrypi_multicore_MPCtest_B.t134 + raspberrypi_multicore_MPCtest_B.t131 *
    raspberrypi_multicore_MPCtest_B.t140;
  raspberrypi_multicore_MPCtest_B.t3 = -raspberrypi_multicore_MPCtest_B.t3 *
    raspberrypi_multicore_MPCtest_B.t28;
  Bd[4] = raspberrypi_multicore_MPCtest_B.t3 *
    raspberrypi_multicore_MPCtest_B.t134 + raspberrypi_multicore_MPCtest_B.t156 *
    raspberrypi_multicore_MPCtest_B.t140;
  Bd[5] = raspberrypi_multicore_MPCtest_B.t28 *
    raspberrypi_multicore_MPCtest_B.t119 / 2.0;
  Bd[6] = raspberrypi_multicore_MPCtest_B.t92;
  Bd[7] = 0.0;
  Bd[8] = 0.0;
  Bd[9] = (raspberrypi_multicore_MPCtest_B.t134_tmp -
           raspberrypi_multicore_MPCtest_B.t134_tmp_m) * Ts;
  Bd[10] = (raspberrypi_multicore_MPCtest_B.t140_tmp -
            raspberrypi_multicore_MPCtest_B.t140_tmp_j) * Ts;
  Bd[11] = Ts * raspberrypi_multicore_MPCtest_B.t119;
  Bd[12] = 0.0;
  Bd[13] = 0.0;
  Bd[14] = raspberrypi_multicore_MPCtest_B.t31;
  Bd[15] = 0.0;
  raspberrypi_multicore_MPCtest_B.t2 = -raspberrypi_multicore_MPCtest_B.t2 *
    raspberrypi_multicore_MPCtest_B.t28;
  Bd[16] = raspberrypi_multicore_MPCtest_B.t2 *
    raspberrypi_multicore_MPCtest_B.t133 - raspberrypi_multicore_MPCtest_B.t131 *
    raspberrypi_multicore_MPCtest_B.t138;
  Bd[17] = raspberrypi_multicore_MPCtest_B.t131 *
    raspberrypi_multicore_MPCtest_B.t133 - raspberrypi_multicore_MPCtest_B.t156 *
    raspberrypi_multicore_MPCtest_B.t138;
  Bd[18] = raspberrypi_multicore_MPCtest_B.t28 *
    raspberrypi_multicore_MPCtest_B.t118 * -0.5;
  Bd[19] = 0.0;
  Bd[20] = raspberrypi_multicore_MPCtest_B.t92;
  Bd[21] = 0.0;
  Bd[22] = (raspberrypi_multicore_MPCtest_B.t133_tmp -
            raspberrypi_multicore_MPCtest_B.t133_tmp_e) * -Ts;
  Bd[23] = (raspberrypi_multicore_MPCtest_B.t138_tmp -
            raspberrypi_multicore_MPCtest_B.t138_tmp_m) * -Ts;
  Bd[24] = -Ts * raspberrypi_multicore_MPCtest_B.t118;
  Bd[25] = 0.0;
  Bd[26] = 0.0;
  Bd[27] = 0.0;
  Bd[28] = raspberrypi_multicore_MPCtest_B.t31;
  Bd[29] = raspberrypi_multicore_MPCtest_B.t156 *
    raspberrypi_multicore_MPCtest_B.t132 + raspberrypi_multicore_MPCtest_B.t131 *
    raspberrypi_multicore_MPCtest_B.t136;
  Bd[30] = raspberrypi_multicore_MPCtest_B.t3 *
    raspberrypi_multicore_MPCtest_B.t132 + raspberrypi_multicore_MPCtest_B.t156 *
    raspberrypi_multicore_MPCtest_B.t136;
  Bd[31] = raspberrypi_multicore_MPCtest_B.t28 *
    raspberrypi_multicore_MPCtest_B.t117 / 2.0;
  Bd[32] = 0.0;
  Bd[33] = 0.0;
  Bd[34] = raspberrypi_multicore_MPCtest_B.t92;
  Bd[35] = (raspberrypi_multicore_MPCtest_B.t132_tmp -
            raspberrypi_multicore_MPCtest_B.t132_tmp_m) * Ts;
  Bd[36] = (raspberrypi_multicore_MPCtest_B.t136_tmp -
            raspberrypi_multicore_MPCtest_B.t136_tmp_j) * Ts;
  Bd[37] = Ts * raspberrypi_multicore_MPCtest_B.t117;
  Bd[38] = 0.0;
  Bd[39] = raspberrypi_multicore_MPCtest_B.t31;
  Bd[40] = 0.0;
  Bd[41] = 0.0;
  Bd[42] = raspberrypi_multicore_MPCtest_B.t156 *
    raspberrypi_multicore_MPCtest_B.t139 + raspberrypi_multicore_MPCtest_B.t131 *
    raspberrypi_multicore_MPCtest_B.t146;
  Bd[43] = raspberrypi_multicore_MPCtest_B.t3 *
    raspberrypi_multicore_MPCtest_B.t139 + raspberrypi_multicore_MPCtest_B.t156 *
    raspberrypi_multicore_MPCtest_B.t146;
  Bd[44] = raspberrypi_multicore_MPCtest_B.t28 *
    raspberrypi_multicore_MPCtest_B.t122 / 2.0;
  Bd[45] = raspberrypi_multicore_MPCtest_B.t92;
  Bd[46] = 0.0;
  Bd[47] = 0.0;
  Bd[48] = (raspberrypi_multicore_MPCtest_B.t139_tmp -
            raspberrypi_multicore_MPCtest_B.t139_tmp_f) * Ts;
  Bd[49] = (raspberrypi_multicore_MPCtest_B.t146_tmp -
            raspberrypi_multicore_MPCtest_B.t146_tmp_a) * Ts;
  Bd[50] = Ts * raspberrypi_multicore_MPCtest_B.t122;
  Bd[51] = 0.0;
  Bd[52] = 0.0;
  Bd[53] = raspberrypi_multicore_MPCtest_B.t31;
  Bd[54] = 0.0;
  Bd[55] = raspberrypi_multicore_MPCtest_B.t2 *
    raspberrypi_multicore_MPCtest_B.t137 - raspberrypi_multicore_MPCtest_B.t131 *
    raspberrypi_multicore_MPCtest_B.t144;
  Bd[56] = raspberrypi_multicore_MPCtest_B.t131 *
    raspberrypi_multicore_MPCtest_B.t137 - raspberrypi_multicore_MPCtest_B.t156 *
    raspberrypi_multicore_MPCtest_B.t144;
  Bd[57] = raspberrypi_multicore_MPCtest_B.t28 *
    raspberrypi_multicore_MPCtest_B.t121 * -0.5;
  Bd[58] = 0.0;
  Bd[59] = raspberrypi_multicore_MPCtest_B.t92;
  Bd[60] = 0.0;
  Bd[61] = (raspberrypi_multicore_MPCtest_B.t137_tmp -
            raspberrypi_multicore_MPCtest_B.t137_tmp_g) * -Ts;
  Bd[62] = (raspberrypi_multicore_MPCtest_B.t144_tmp -
            raspberrypi_multicore_MPCtest_B.t144_tmp_n) * -Ts;
  Bd[63] = -Ts * raspberrypi_multicore_MPCtest_B.t121;
  Bd[64] = 0.0;
  Bd[65] = 0.0;
  Bd[66] = 0.0;
  Bd[67] = raspberrypi_multicore_MPCtest_B.t31;
  Bd[68] = raspberrypi_multicore_MPCtest_B.t156 *
    raspberrypi_multicore_MPCtest_B.t135 + raspberrypi_multicore_MPCtest_B.t131 *
    raspberrypi_multicore_MPCtest_B.t142;
  Bd[69] = raspberrypi_multicore_MPCtest_B.t3 *
    raspberrypi_multicore_MPCtest_B.t135 + raspberrypi_multicore_MPCtest_B.t156 *
    raspberrypi_multicore_MPCtest_B.t142;
  Bd[70] = raspberrypi_multicore_MPCtest_B.t28 *
    raspberrypi_multicore_MPCtest_B.t120 / 2.0;
  Bd[71] = 0.0;
  Bd[72] = 0.0;
  Bd[73] = raspberrypi_multicore_MPCtest_B.t92;
  Bd[74] = (raspberrypi_multicore_MPCtest_B.t135_tmp -
            raspberrypi_multicore_MPCtest_B.t135_tmp_d) * Ts;
  Bd[75] = (raspberrypi_multicore_MPCtest_B.t142_tmp -
            raspberrypi_multicore_MPCtest_B.t142_tmp_n) * Ts;
  Bd[76] = Ts * raspberrypi_multicore_MPCtest_B.t120;
  Bd[77] = 0.0;
  Bd[78] = raspberrypi_multicore_MPCtest_B.t31;
  Bd[79] = 0.0;
  Bd[80] = 0.0;
  Bd[81] = raspberrypi_multicore_MPCtest_B.t156 *
    raspberrypi_multicore_MPCtest_B.t145 + raspberrypi_multicore_MPCtest_B.t131 *
    raspberrypi_multicore_MPCtest_B.t152;
  Bd[82] = raspberrypi_multicore_MPCtest_B.t3 *
    raspberrypi_multicore_MPCtest_B.t145 + raspberrypi_multicore_MPCtest_B.t156 *
    raspberrypi_multicore_MPCtest_B.t152;
  Bd[83] = raspberrypi_multicore_MPCtest_B.t28 *
    raspberrypi_multicore_MPCtest_B.t125 / 2.0;
  Bd[84] = raspberrypi_multicore_MPCtest_B.t92;
  Bd[85] = 0.0;
  Bd[86] = 0.0;
  Bd[87] = (raspberrypi_multicore_MPCtest_B.t145_tmp -
            raspberrypi_multicore_MPCtest_B.t145_tmp_c) * Ts;
  Bd[88] = (raspberrypi_multicore_MPCtest_B.t152_tmp -
            raspberrypi_multicore_MPCtest_B.t152_tmp_f) * Ts;
  Bd[89] = Ts * raspberrypi_multicore_MPCtest_B.t125;
  Bd[90] = 0.0;
  Bd[91] = 0.0;
  Bd[92] = raspberrypi_multicore_MPCtest_B.t31;
  Bd[93] = 0.0;
  Bd[94] = raspberrypi_multicore_MPCtest_B.t2 *
    raspberrypi_multicore_MPCtest_B.t143 - raspberrypi_multicore_MPCtest_B.t131 *
    raspberrypi_multicore_MPCtest_B.t150;
  Bd[95] = raspberrypi_multicore_MPCtest_B.t131 *
    raspberrypi_multicore_MPCtest_B.t143 - raspberrypi_multicore_MPCtest_B.t156 *
    raspberrypi_multicore_MPCtest_B.t150;
  Bd[96] = raspberrypi_multicore_MPCtest_B.t28 *
    raspberrypi_multicore_MPCtest_B.t124 * -0.5;
  Bd[97] = 0.0;
  Bd[98] = raspberrypi_multicore_MPCtest_B.t92;
  Bd[99] = 0.0;
  Bd[100] = (raspberrypi_multicore_MPCtest_B.t143_tmp -
             raspberrypi_multicore_MPCtest_B.t143_tmp_p) * -Ts;
  Bd[101] = (raspberrypi_multicore_MPCtest_B.t150_tmp -
             raspberrypi_multicore_MPCtest_B.t150_tmp_p) * -Ts;
  Bd[102] = -Ts * raspberrypi_multicore_MPCtest_B.t124;
  Bd[103] = 0.0;
  Bd[104] = 0.0;
  Bd[105] = 0.0;
  Bd[106] = raspberrypi_multicore_MPCtest_B.t31;
  Bd[107] = raspberrypi_multicore_MPCtest_B.t156 *
    raspberrypi_multicore_MPCtest_B.t141 + raspberrypi_multicore_MPCtest_B.t131 *
    raspberrypi_multicore_MPCtest_B.t148;
  Bd[108] = raspberrypi_multicore_MPCtest_B.t3 *
    raspberrypi_multicore_MPCtest_B.t141 + raspberrypi_multicore_MPCtest_B.t156 *
    raspberrypi_multicore_MPCtest_B.t148;
  Bd[109] = raspberrypi_multicore_MPCtest_B.t28 *
    raspberrypi_multicore_MPCtest_B.t123 / 2.0;
  Bd[110] = 0.0;
  Bd[111] = 0.0;
  Bd[112] = raspberrypi_multicore_MPCtest_B.t92;
  Bd[113] = (raspberrypi_multicore_MPCtest_B.t141_tmp -
             raspberrypi_multicore_MPCtest_B.t141_tmp_n) * Ts;
  Bd[114] = (raspberrypi_multicore_MPCtest_B.t148_tmp -
             raspberrypi_multicore_MPCtest_B.t148_tmp_k) * Ts;
  Bd[115] = Ts * raspberrypi_multicore_MPCtest_B.t123;
  Bd[116] = 0.0;
  Bd[117] = raspberrypi_multicore_MPCtest_B.t31;
  Bd[118] = 0.0;
  Bd[119] = 0.0;
  Bd[120] = raspberrypi_multicore_MPCtest_B.t156 *
    raspberrypi_multicore_MPCtest_B.t151 + raspberrypi_multicore_MPCtest_B.t131 *
    raspberrypi_multicore_MPCtest_B.t155;
  Bd[121] = raspberrypi_multicore_MPCtest_B.t3 *
    raspberrypi_multicore_MPCtest_B.t151 + raspberrypi_multicore_MPCtest_B.t156 *
    raspberrypi_multicore_MPCtest_B.t155;
  Bd[122] = raspberrypi_multicore_MPCtest_B.t28 *
    raspberrypi_multicore_MPCtest_B.t128 / 2.0;
  Bd[123] = raspberrypi_multicore_MPCtest_B.t92;
  Bd[124] = 0.0;
  Bd[125] = 0.0;
  Bd[126] = (raspberrypi_multicore_MPCtest_B.t151_tmp -
             raspberrypi_multicore_MPCtest_B.t151_tmp_n) * Ts;
  Bd[127] = (raspberrypi_multicore_MPCtest_B.t155_tmp -
             raspberrypi_multicore_MPCtest_B.t155_tmp_o) * Ts;
  Bd[128] = Ts * raspberrypi_multicore_MPCtest_B.t128;
  Bd[129] = 0.0;
  Bd[130] = 0.0;
  Bd[131] = raspberrypi_multicore_MPCtest_B.t31;
  Bd[132] = 0.0;
  Bd[133] = raspberrypi_multicore_MPCtest_B.t2 *
    raspberrypi_multicore_MPCtest_B.t149 - raspberrypi_multicore_MPCtest_B.t131 *
    raspberrypi_multicore_MPCtest_B.t154;
  Bd[134] = raspberrypi_multicore_MPCtest_B.t131 *
    raspberrypi_multicore_MPCtest_B.t149 - raspberrypi_multicore_MPCtest_B.t156 *
    raspberrypi_multicore_MPCtest_B.t154;
  Bd[135] = raspberrypi_multicore_MPCtest_B.t28 *
    raspberrypi_multicore_MPCtest_B.t127 * -0.5;
  Bd[136] = 0.0;
  Bd[137] = raspberrypi_multicore_MPCtest_B.t92;
  Bd[138] = 0.0;
  Bd[139] = (raspberrypi_multicore_MPCtest_B.t149_tmp -
             raspberrypi_multicore_MPCtest_B.t149_tmp_g) * -Ts;
  Bd[140] = (raspberrypi_multicore_MPCtest_B.t154_tmp - t154_tmp) * -Ts;
  Bd[141] = -Ts * raspberrypi_multicore_MPCtest_B.t127;
  Bd[142] = 0.0;
  Bd[143] = 0.0;
  Bd[144] = 0.0;
  Bd[145] = raspberrypi_multicore_MPCtest_B.t31;
  Bd[146] = raspberrypi_multicore_MPCtest_B.t156 *
    raspberrypi_multicore_MPCtest_B.t147 + raspberrypi_multicore_MPCtest_B.t131 *
    raspberrypi_multicore_MPCtest_B.t153;
  Bd[147] = raspberrypi_multicore_MPCtest_B.t3 *
    raspberrypi_multicore_MPCtest_B.t147 + raspberrypi_multicore_MPCtest_B.t156 *
    raspberrypi_multicore_MPCtest_B.t153;
  Bd[148] = raspberrypi_multicore_MPCtest_B.t28 *
    raspberrypi_multicore_MPCtest_B.t126 / 2.0;
  Bd[149] = 0.0;
  Bd[150] = 0.0;
  Bd[151] = raspberrypi_multicore_MPCtest_B.t92;
  Bd[152] = (t147_tmp - t147_tmp_0) * Ts;
  Bd[153] = (t153_tmp - t153_tmp_0) * Ts;
  Bd[154] = Ts * raspberrypi_multicore_MPCtest_B.t126;
  Bd[155] = 0.0;
}

static void raspberrypi_multicore_MP_xgetrf(const real_T A[169], real_T b_A[169],
  int32_T ipiv[13], int32_T *info)
{
  int32_T b_A_tmp;
  int32_T b_j;
  int32_T c;
  int32_T c_0;
  int32_T c_1;
  int32_T ijA;
  int32_T jA;
  int32_T jj;
  int32_T k;
  memcpy(&b_A[0], &A[0], 169U * sizeof(real_T));
  for (b_j = 0; b_j < 13; b_j++) {
    ipiv[b_j] = b_j + 1;
  }

  *info = 0;
  for (b_j = 0; b_j < 12; b_j++) {
    c = b_j * 14 + 2;
    jj = b_j * 14;
    c_0 = 13 - b_j;
    jA = 1;
    raspberrypi_multicore_MPCtest_B.smax = fabs(b_A[jj]);
    for (k = 2; k <= c_0; k++) {
      raspberrypi_multicore_MPCtest_B.s_a = fabs(b_A[(c + k) - 3]);
      if (raspberrypi_multicore_MPCtest_B.s_a >
          raspberrypi_multicore_MPCtest_B.smax) {
        jA = k;
        raspberrypi_multicore_MPCtest_B.smax =
          raspberrypi_multicore_MPCtest_B.s_a;
      }
    }

    if (b_A[(c + jA) - 3] != 0.0) {
      if (jA - 1 != 0) {
        c_0 = b_j + jA;
        ipiv[b_j] = c_0;
        for (k = 0; k < 13; k++) {
          jA = k * 13 + b_j;
          raspberrypi_multicore_MPCtest_B.smax = b_A[jA];
          b_A_tmp = (k * 13 + c_0) - 1;
          b_A[jA] = b_A[b_A_tmp];
          b_A[b_A_tmp] = raspberrypi_multicore_MPCtest_B.smax;
        }
      }

      k = c - b_j;
      for (c_0 = c; c_0 <= k + 11; c_0++) {
        b_A[c_0 - 1] /= b_A[jj];
      }
    } else {
      *info = b_j + 1;
    }

    c_0 = 12 - b_j;
    jA = jj;
    for (b_A_tmp = 0; b_A_tmp < c_0; b_A_tmp++) {
      raspberrypi_multicore_MPCtest_B.smax = b_A[(jj + b_A_tmp * 13) + 13];
      if (raspberrypi_multicore_MPCtest_B.smax != 0.0) {
        k = jA + 15;
        c_1 = jA - b_j;
        for (ijA = k; ijA <= c_1 + 26; ijA++) {
          b_A[ijA - 1] += b_A[((c + ijA) - jA) - 16] *
            -raspberrypi_multicore_MPCtest_B.smax;
        }
      }

      jA += 13;
    }
  }

  if ((*info == 0) && (!(b_A[168] != 0.0))) {
    *info = 13;
  }
}

static void raspberrypi_multicore_MPC_mrdiv(const real_T A[247], const real_T B
  [169], real_T Y[247])
{
  int32_T b_i;
  int32_T b_k;
  raspberrypi_multicore_MP_xgetrf(B, raspberrypi_multicore_MPCtest_B.c_A_d,
    raspberrypi_multicore_MPCtest_B.ipiv,
    &raspberrypi_multicore_MPCtest_B.b_info);
  memcpy(&Y[0], &A[0], 247U * sizeof(real_T));
  for (raspberrypi_multicore_MPCtest_B.b_j_d = 0;
       raspberrypi_multicore_MPCtest_B.b_j_d < 13;
       raspberrypi_multicore_MPCtest_B.b_j_d++) {
    raspberrypi_multicore_MPCtest_B.jBcol = 19 *
      raspberrypi_multicore_MPCtest_B.b_j_d - 1;
    raspberrypi_multicore_MPCtest_B.jAcol = 13 *
      raspberrypi_multicore_MPCtest_B.b_j_d - 1;
    raspberrypi_multicore_MPCtest_B.b_l = raspberrypi_multicore_MPCtest_B.b_j_d
      - 1;
    for (b_k = 0; b_k <= raspberrypi_multicore_MPCtest_B.b_l; b_k++) {
      raspberrypi_multicore_MPCtest_B.kBcol = 19 * b_k - 1;
      raspberrypi_multicore_MPCtest_B.temp_m =
        raspberrypi_multicore_MPCtest_B.c_A_d[(b_k +
        raspberrypi_multicore_MPCtest_B.jAcol) + 1];
      if (raspberrypi_multicore_MPCtest_B.temp_m != 0.0) {
        for (b_i = 0; b_i < 19; b_i++) {
          raspberrypi_multicore_MPCtest_B.b_info = (b_i +
            raspberrypi_multicore_MPCtest_B.jBcol) + 1;
          Y[raspberrypi_multicore_MPCtest_B.b_info] -=
            raspberrypi_multicore_MPCtest_B.temp_m * Y[(b_i +
            raspberrypi_multicore_MPCtest_B.kBcol) + 1];
        }
      }
    }

    raspberrypi_multicore_MPCtest_B.temp_m = 1.0 /
      raspberrypi_multicore_MPCtest_B.c_A_d
      [(raspberrypi_multicore_MPCtest_B.b_j_d +
        raspberrypi_multicore_MPCtest_B.jAcol) + 1];
    for (b_i = 0; b_i < 19; b_i++) {
      raspberrypi_multicore_MPCtest_B.b_info = (b_i +
        raspberrypi_multicore_MPCtest_B.jBcol) + 1;
      Y[raspberrypi_multicore_MPCtest_B.b_info] *=
        raspberrypi_multicore_MPCtest_B.temp_m;
    }
  }

  for (raspberrypi_multicore_MPCtest_B.b_j_d = 12;
       raspberrypi_multicore_MPCtest_B.b_j_d >= 0;
       raspberrypi_multicore_MPCtest_B.b_j_d--) {
    raspberrypi_multicore_MPCtest_B.jBcol = 19 *
      raspberrypi_multicore_MPCtest_B.b_j_d - 1;
    raspberrypi_multicore_MPCtest_B.jAcol = 13 *
      raspberrypi_multicore_MPCtest_B.b_j_d - 1;
    for (raspberrypi_multicore_MPCtest_B.b_l =
         raspberrypi_multicore_MPCtest_B.b_j_d + 2;
         raspberrypi_multicore_MPCtest_B.b_l < 14;
         raspberrypi_multicore_MPCtest_B.b_l++) {
      raspberrypi_multicore_MPCtest_B.kBcol =
        (raspberrypi_multicore_MPCtest_B.b_l - 1) * 19 - 1;
      raspberrypi_multicore_MPCtest_B.temp_m =
        raspberrypi_multicore_MPCtest_B.c_A_d[raspberrypi_multicore_MPCtest_B.b_l
        + raspberrypi_multicore_MPCtest_B.jAcol];
      if (raspberrypi_multicore_MPCtest_B.temp_m != 0.0) {
        for (b_i = 0; b_i < 19; b_i++) {
          raspberrypi_multicore_MPCtest_B.b_info = (b_i +
            raspberrypi_multicore_MPCtest_B.jBcol) + 1;
          Y[raspberrypi_multicore_MPCtest_B.b_info] -= Y[(b_i +
            raspberrypi_multicore_MPCtest_B.kBcol) + 1] *
            raspberrypi_multicore_MPCtest_B.temp_m;
        }
      }
    }
  }

  for (raspberrypi_multicore_MPCtest_B.b_j_d = 11;
       raspberrypi_multicore_MPCtest_B.b_j_d >= 0;
       raspberrypi_multicore_MPCtest_B.b_j_d--) {
    raspberrypi_multicore_MPCtest_B.jBcol =
      raspberrypi_multicore_MPCtest_B.ipiv[raspberrypi_multicore_MPCtest_B.b_j_d];
    if (raspberrypi_multicore_MPCtest_B.b_j_d + 1 !=
        raspberrypi_multicore_MPCtest_B.jBcol) {
      for (b_i = 0; b_i < 19; b_i++) {
        raspberrypi_multicore_MPCtest_B.jAcol = 19 *
          raspberrypi_multicore_MPCtest_B.b_j_d + b_i;
        raspberrypi_multicore_MPCtest_B.temp_m =
          Y[raspberrypi_multicore_MPCtest_B.jAcol];
        raspberrypi_multicore_MPCtest_B.b_info = 19 *
          (raspberrypi_multicore_MPCtest_B.jBcol - 1) + b_i;
        Y[raspberrypi_multicore_MPCtest_B.jAcol] =
          Y[raspberrypi_multicore_MPCtest_B.b_info];
        Y[raspberrypi_multicore_MPCtest_B.b_info] =
          raspberrypi_multicore_MPCtest_B.temp_m;
      }
    }
  }
}

static void KalmanFilter_DIY_MPCDis_stepImp(KalmanFilter_DIY_MPCDis_raspb_T *obj,
  const real_T U_MPC[12], const real_T xFB[13], const real_T pW[12], const
  real_T Q[361], const real_T R[169], real_T Reset, real_T estXbar[19], real_T
  P[361])
{
  static const int8_T tmp[36] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

  static const int8_T tmp_0[114] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

  static const int8_T tmp_1[247] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  /*  note Q, R must be matrixes */
  /*  Q for process noise */
  /*  R for measurement noise */
  raspberrypi_multicore_MPCtest_B.absx11 = sin(xFB[5]);
  raspberrypi_multicore_MPCtest_B.absx21 = cos(xFB[5]);
  raspberrypi_multicore_MPCtest_B.Rz[0] = raspberrypi_multicore_MPCtest_B.absx21;
  raspberrypi_multicore_MPCtest_B.Rz[3] =
    -raspberrypi_multicore_MPCtest_B.absx11;
  raspberrypi_multicore_MPCtest_B.Rz[6] = 0.0;
  raspberrypi_multicore_MPCtest_B.Rz[1] = raspberrypi_multicore_MPCtest_B.absx11;
  raspberrypi_multicore_MPCtest_B.Rz[4] = raspberrypi_multicore_MPCtest_B.absx21;
  raspberrypi_multicore_MPCtest_B.Rz[7] = 0.0;
  raspberrypi_multicore_MPCtest_B.Rz[2] = 0.0;
  raspberrypi_multicore_MPCtest_B.Rz[5] = 0.0;
  raspberrypi_multicore_MPCtest_B.Rz[8] = 1.0;
  for (raspberrypi_multicore_MPCtest_B.p1 = 0;
       raspberrypi_multicore_MPCtest_B.p1 < 3;
       raspberrypi_multicore_MPCtest_B.p1++) {
    for (raspberrypi_multicore_MPCtest_B.p2 = 0;
         raspberrypi_multicore_MPCtest_B.p2 < 3;
         raspberrypi_multicore_MPCtest_B.p2++) {
      raspberrypi_multicore_MPCtest_B.p3 = raspberrypi_multicore_MPCtest_B.p1 +
        3 * raspberrypi_multicore_MPCtest_B.p2;
      raspberrypi_multicore_MPCtest_B.Rz_l[raspberrypi_multicore_MPCtest_B.p3] =
        0.0;
      raspberrypi_multicore_MPCtest_B.Rz_l[raspberrypi_multicore_MPCtest_B.p3] +=
        obj->Inorm[3 * raspberrypi_multicore_MPCtest_B.p2] *
        raspberrypi_multicore_MPCtest_B.Rz[raspberrypi_multicore_MPCtest_B.p1];
      raspberrypi_multicore_MPCtest_B.Rz_l[raspberrypi_multicore_MPCtest_B.p3] +=
        obj->Inorm[3 * raspberrypi_multicore_MPCtest_B.p2 + 1] *
        raspberrypi_multicore_MPCtest_B.Rz[raspberrypi_multicore_MPCtest_B.p1 +
        3];
      raspberrypi_multicore_MPCtest_B.Rz_l[raspberrypi_multicore_MPCtest_B.p3] +=
        obj->Inorm[3 * raspberrypi_multicore_MPCtest_B.p2 + 2] *
        raspberrypi_multicore_MPCtest_B.Rz[raspberrypi_multicore_MPCtest_B.p1 +
        6];
    }

    for (raspberrypi_multicore_MPCtest_B.p2 = 0;
         raspberrypi_multicore_MPCtest_B.p2 < 3;
         raspberrypi_multicore_MPCtest_B.p2++) {
      raspberrypi_multicore_MPCtest_B.p3 = raspberrypi_multicore_MPCtest_B.p1 +
        3 * raspberrypi_multicore_MPCtest_B.p2;
      raspberrypi_multicore_MPCtest_B.Inow[raspberrypi_multicore_MPCtest_B.p3] =
        0.0;
      raspberrypi_multicore_MPCtest_B.Inow[raspberrypi_multicore_MPCtest_B.p3] +=
        raspberrypi_multicore_MPCtest_B.Rz_l[raspberrypi_multicore_MPCtest_B.p1]
        * raspberrypi_multicore_MPCtest_B.Rz[raspberrypi_multicore_MPCtest_B.p2];
      raspberrypi_multicore_MPCtest_B.Inow[raspberrypi_multicore_MPCtest_B.p3] +=
        raspberrypi_multicore_MPCtest_B.Rz_l[raspberrypi_multicore_MPCtest_B.p1
        + 3] *
        raspberrypi_multicore_MPCtest_B.Rz[raspberrypi_multicore_MPCtest_B.p2 +
        3];
      raspberrypi_multicore_MPCtest_B.Inow[raspberrypi_multicore_MPCtest_B.p3] +=
        raspberrypi_multicore_MPCtest_B.Rz_l[raspberrypi_multicore_MPCtest_B.p1
        + 6] *
        raspberrypi_multicore_MPCtest_B.Rz[raspberrypi_multicore_MPCtest_B.p2 +
        6];
    }
  }

  memcpy(&raspberrypi_multicore_MPCtest_B.Rz[0],
         &raspberrypi_multicore_MPCtest_B.Inow[0], 9U * sizeof(real_T));
  raspberrypi_multicore_MPCtest_B.p1 = 1;
  raspberrypi_multicore_MPCtest_B.p2 = 3;
  raspberrypi_multicore_MPCtest_B.p3 = 6;
  raspberrypi_multicore_MPCtest_B.absx11 = fabs
    (raspberrypi_multicore_MPCtest_B.Inow[0]);
  raspberrypi_multicore_MPCtest_B.absx21 = fabs
    (raspberrypi_multicore_MPCtest_B.Inow[1]);
  raspberrypi_multicore_MPCtest_B.absx31 = fabs
    (raspberrypi_multicore_MPCtest_B.Inow[2]);
  if ((raspberrypi_multicore_MPCtest_B.absx21 >
       raspberrypi_multicore_MPCtest_B.absx11) &&
      (raspberrypi_multicore_MPCtest_B.absx21 >
       raspberrypi_multicore_MPCtest_B.absx31)) {
    raspberrypi_multicore_MPCtest_B.p1 = 4;
    raspberrypi_multicore_MPCtest_B.p2 = 0;
    raspberrypi_multicore_MPCtest_B.Rz[0] =
      raspberrypi_multicore_MPCtest_B.Inow[1];
    raspberrypi_multicore_MPCtest_B.Rz[1] =
      raspberrypi_multicore_MPCtest_B.Inow[0];
    raspberrypi_multicore_MPCtest_B.Rz[3] =
      raspberrypi_multicore_MPCtest_B.Inow[4];
    raspberrypi_multicore_MPCtest_B.Rz[4] =
      raspberrypi_multicore_MPCtest_B.Inow[3];
    raspberrypi_multicore_MPCtest_B.Rz[6] =
      raspberrypi_multicore_MPCtest_B.Inow[7];
    raspberrypi_multicore_MPCtest_B.Rz[7] =
      raspberrypi_multicore_MPCtest_B.Inow[6];
  } else if (raspberrypi_multicore_MPCtest_B.absx31 >
             raspberrypi_multicore_MPCtest_B.absx11) {
    raspberrypi_multicore_MPCtest_B.p1 = 7;
    raspberrypi_multicore_MPCtest_B.p3 = 0;
    raspberrypi_multicore_MPCtest_B.Rz[0] =
      raspberrypi_multicore_MPCtest_B.Inow[2];
    raspberrypi_multicore_MPCtest_B.Rz[2] =
      raspberrypi_multicore_MPCtest_B.Inow[0];
    raspberrypi_multicore_MPCtest_B.Rz[3] =
      raspberrypi_multicore_MPCtest_B.Inow[5];
    raspberrypi_multicore_MPCtest_B.Rz[5] =
      raspberrypi_multicore_MPCtest_B.Inow[3];
    raspberrypi_multicore_MPCtest_B.Rz[6] =
      raspberrypi_multicore_MPCtest_B.Inow[8];
    raspberrypi_multicore_MPCtest_B.Rz[8] =
      raspberrypi_multicore_MPCtest_B.Inow[6];
  }

  raspberrypi_multicore_MPCtest_B.absx11 = raspberrypi_multicore_MPCtest_B.Rz[1]
    / raspberrypi_multicore_MPCtest_B.Rz[0];
  raspberrypi_multicore_MPCtest_B.Rz[1] = raspberrypi_multicore_MPCtest_B.absx11;
  raspberrypi_multicore_MPCtest_B.absx21 = raspberrypi_multicore_MPCtest_B.Rz[2]
    / raspberrypi_multicore_MPCtest_B.Rz[0];
  raspberrypi_multicore_MPCtest_B.Rz[2] = raspberrypi_multicore_MPCtest_B.absx21;
  raspberrypi_multicore_MPCtest_B.Rz[4] -=
    raspberrypi_multicore_MPCtest_B.absx11 * raspberrypi_multicore_MPCtest_B.Rz
    [3];
  raspberrypi_multicore_MPCtest_B.Rz[5] -=
    raspberrypi_multicore_MPCtest_B.absx21 * raspberrypi_multicore_MPCtest_B.Rz
    [3];
  raspberrypi_multicore_MPCtest_B.Rz[7] -=
    raspberrypi_multicore_MPCtest_B.absx11 * raspberrypi_multicore_MPCtest_B.Rz
    [6];
  raspberrypi_multicore_MPCtest_B.Rz[8] -=
    raspberrypi_multicore_MPCtest_B.absx21 * raspberrypi_multicore_MPCtest_B.Rz
    [6];
  if (fabs(raspberrypi_multicore_MPCtest_B.Rz[5]) > fabs
      (raspberrypi_multicore_MPCtest_B.Rz[4])) {
    raspberrypi_multicore_MPCtest_B.itmp = raspberrypi_multicore_MPCtest_B.p2;
    raspberrypi_multicore_MPCtest_B.p2 = raspberrypi_multicore_MPCtest_B.p3;
    raspberrypi_multicore_MPCtest_B.p3 = raspberrypi_multicore_MPCtest_B.itmp;
    raspberrypi_multicore_MPCtest_B.Rz[1] =
      raspberrypi_multicore_MPCtest_B.absx21;
    raspberrypi_multicore_MPCtest_B.Rz[2] =
      raspberrypi_multicore_MPCtest_B.absx11;
    raspberrypi_multicore_MPCtest_B.absx11 = raspberrypi_multicore_MPCtest_B.Rz
      [4];
    raspberrypi_multicore_MPCtest_B.Rz[4] = raspberrypi_multicore_MPCtest_B.Rz[5];
    raspberrypi_multicore_MPCtest_B.Rz[5] =
      raspberrypi_multicore_MPCtest_B.absx11;
    raspberrypi_multicore_MPCtest_B.absx11 = raspberrypi_multicore_MPCtest_B.Rz
      [7];
    raspberrypi_multicore_MPCtest_B.Rz[7] = raspberrypi_multicore_MPCtest_B.Rz[8];
    raspberrypi_multicore_MPCtest_B.Rz[8] =
      raspberrypi_multicore_MPCtest_B.absx11;
  }

  raspberrypi_multicore_MPCtest_B.absx11 = raspberrypi_multicore_MPCtest_B.Rz[5]
    / raspberrypi_multicore_MPCtest_B.Rz[4];
  raspberrypi_multicore_MPCtest_B.Rz[8] -=
    raspberrypi_multicore_MPCtest_B.absx11 * raspberrypi_multicore_MPCtest_B.Rz
    [7];
  raspberrypi_multicore_MPCtest_B.absx21 = (raspberrypi_multicore_MPCtest_B.Rz[1]
    * raspberrypi_multicore_MPCtest_B.absx11 -
    raspberrypi_multicore_MPCtest_B.Rz[2]) / raspberrypi_multicore_MPCtest_B.Rz
    [8];
  raspberrypi_multicore_MPCtest_B.absx31 = -(raspberrypi_multicore_MPCtest_B.Rz
    [7] * raspberrypi_multicore_MPCtest_B.absx21 +
    raspberrypi_multicore_MPCtest_B.Rz[1]) / raspberrypi_multicore_MPCtest_B.Rz
    [4];
  raspberrypi_multicore_MPCtest_B.Inow[raspberrypi_multicore_MPCtest_B.p1 - 1] =
    ((1.0 - raspberrypi_multicore_MPCtest_B.Rz[3] *
      raspberrypi_multicore_MPCtest_B.absx31) -
     raspberrypi_multicore_MPCtest_B.Rz[6] *
     raspberrypi_multicore_MPCtest_B.absx21) /
    raspberrypi_multicore_MPCtest_B.Rz[0];
  raspberrypi_multicore_MPCtest_B.Inow[raspberrypi_multicore_MPCtest_B.p1] =
    raspberrypi_multicore_MPCtest_B.absx31;
  raspberrypi_multicore_MPCtest_B.Inow[raspberrypi_multicore_MPCtest_B.p1 + 1] =
    raspberrypi_multicore_MPCtest_B.absx21;
  raspberrypi_multicore_MPCtest_B.absx21 =
    -raspberrypi_multicore_MPCtest_B.absx11 /
    raspberrypi_multicore_MPCtest_B.Rz[8];
  raspberrypi_multicore_MPCtest_B.absx31 = (1.0 -
    raspberrypi_multicore_MPCtest_B.Rz[7] *
    raspberrypi_multicore_MPCtest_B.absx21) /
    raspberrypi_multicore_MPCtest_B.Rz[4];
  raspberrypi_multicore_MPCtest_B.Inow[raspberrypi_multicore_MPCtest_B.p2] =
    -(raspberrypi_multicore_MPCtest_B.Rz[3] *
      raspberrypi_multicore_MPCtest_B.absx31 +
      raspberrypi_multicore_MPCtest_B.Rz[6] *
      raspberrypi_multicore_MPCtest_B.absx21) /
    raspberrypi_multicore_MPCtest_B.Rz[0];
  raspberrypi_multicore_MPCtest_B.Inow[raspberrypi_multicore_MPCtest_B.p2 + 1] =
    raspberrypi_multicore_MPCtest_B.absx31;
  raspberrypi_multicore_MPCtest_B.Inow[raspberrypi_multicore_MPCtest_B.p2 + 2] =
    raspberrypi_multicore_MPCtest_B.absx21;
  raspberrypi_multicore_MPCtest_B.absx21 = 1.0 /
    raspberrypi_multicore_MPCtest_B.Rz[8];
  raspberrypi_multicore_MPCtest_B.absx31 = -raspberrypi_multicore_MPCtest_B.Rz[7]
    * raspberrypi_multicore_MPCtest_B.absx21 /
    raspberrypi_multicore_MPCtest_B.Rz[4];
  raspberrypi_multicore_MPCtest_B.Inow[raspberrypi_multicore_MPCtest_B.p3] =
    -(raspberrypi_multicore_MPCtest_B.Rz[3] *
      raspberrypi_multicore_MPCtest_B.absx31 +
      raspberrypi_multicore_MPCtest_B.Rz[6] *
      raspberrypi_multicore_MPCtest_B.absx21) /
    raspberrypi_multicore_MPCtest_B.Rz[0];
  raspberrypi_multicore_MPCtest_B.Inow[raspberrypi_multicore_MPCtest_B.p3 + 1] =
    raspberrypi_multicore_MPCtest_B.absx31;
  raspberrypi_multicore_MPCtest_B.Inow[raspberrypi_multicore_MPCtest_B.p3 + 2] =
    raspberrypi_multicore_MPCtest_B.absx21;
  raspberrypi_multicore_MPCtest_B.pW[0] = pW[0] - xFB[0];
  raspberrypi_multicore_MPCtest_B.pW_f[0] = pW[3] - xFB[0];
  raspberrypi_multicore_MPCtest_B.pW_p[0] = pW[6] - xFB[0];
  raspberrypi_multicore_MPCtest_B.pW_e[0] = pW[9] - xFB[0];
  raspberrypi_multicore_MPCtest_B.pW[1] = pW[1] - xFB[1];
  raspberrypi_multicore_MPCtest_B.pW_f[1] = pW[4] - xFB[1];
  raspberrypi_multicore_MPCtest_B.pW_p[1] = pW[7] - xFB[1];
  raspberrypi_multicore_MPCtest_B.pW_e[1] = pW[10] - xFB[1];
  raspberrypi_multicore_MPCtest_B.pW[2] = pW[2] - xFB[2];
  raspberrypi_multicore_MPCtest_B.pW_f[2] = pW[5] - xFB[2];
  raspberrypi_multicore_MPCtest_B.pW_p[2] = pW[8] - xFB[2];
  raspberrypi_multicore_MPCtest_B.pW_e[2] = pW[11] - xFB[2];
  raspberrypi_multicore_MP_DS_gen(obj->Ts, obj->m, &xFB[3],
    raspberrypi_multicore_MPCtest_B.Inow, raspberrypi_multicore_MPCtest_B.pW,
    raspberrypi_multicore_MPCtest_B.pW_f, raspberrypi_multicore_MPCtest_B.pW_p,
    raspberrypi_multicore_MPCtest_B.pW_e, raspberrypi_multicore_MPCtest_B.A,
    raspberrypi_multicore_MPCtest_B.B_o);

  /* Bdis=zeros(13,6); */
  /* Bdis=[0.5*obj.Ts^2*eye(6);obj.Ts*eye(6);zeros(1,6)]; */
  for (raspberrypi_multicore_MPCtest_B.p1 = 0;
       raspberrypi_multicore_MPCtest_B.p1 < 13;
       raspberrypi_multicore_MPCtest_B.p1++) {
    memcpy
      (&raspberrypi_multicore_MPCtest_B.Anew[raspberrypi_multicore_MPCtest_B.p1 *
       19],
       &raspberrypi_multicore_MPCtest_B.A[raspberrypi_multicore_MPCtest_B.p1 *
       13], 13U * sizeof(real_T));
  }

  for (raspberrypi_multicore_MPCtest_B.p1 = 0;
       raspberrypi_multicore_MPCtest_B.p1 < 6;
       raspberrypi_multicore_MPCtest_B.p1++) {
    for (raspberrypi_multicore_MPCtest_B.p2 = 0;
         raspberrypi_multicore_MPCtest_B.p2 < 6;
         raspberrypi_multicore_MPCtest_B.p2++) {
      raspberrypi_multicore_MPCtest_B.p3 = tmp[6 *
        raspberrypi_multicore_MPCtest_B.p1 + raspberrypi_multicore_MPCtest_B.p2];
      raspberrypi_multicore_MPCtest_B.itmp = raspberrypi_multicore_MPCtest_B.p2
        + 19 * (raspberrypi_multicore_MPCtest_B.p1 + 13);
      raspberrypi_multicore_MPCtest_B.Anew[raspberrypi_multicore_MPCtest_B.itmp]
        = obj->Ts * (real_T)raspberrypi_multicore_MPCtest_B.p3;
      raspberrypi_multicore_MPCtest_B.Anew[raspberrypi_multicore_MPCtest_B.itmp
        + 6] = raspberrypi_multicore_MPCtest_B.p3;
    }

    raspberrypi_multicore_MPCtest_B.Anew[19 *
      (raspberrypi_multicore_MPCtest_B.p1 + 13) + 12] = 0.0;
  }

  for (raspberrypi_multicore_MPCtest_B.p1 = 0;
       raspberrypi_multicore_MPCtest_B.p1 < 19;
       raspberrypi_multicore_MPCtest_B.p1++) {
    for (raspberrypi_multicore_MPCtest_B.p2 = 0;
         raspberrypi_multicore_MPCtest_B.p2 < 6;
         raspberrypi_multicore_MPCtest_B.p2++) {
      raspberrypi_multicore_MPCtest_B.Anew[(raspberrypi_multicore_MPCtest_B.p2 +
        19 * raspberrypi_multicore_MPCtest_B.p1) + 13] = tmp_0[6 *
        raspberrypi_multicore_MPCtest_B.p1 + raspberrypi_multicore_MPCtest_B.p2];
    }
  }

  if (obj->count < 0.5) {
    memcpy(&obj->XOld[0], &xFB[0], 13U * sizeof(real_T));
    for (raspberrypi_multicore_MPCtest_B.p1 = 0;
         raspberrypi_multicore_MPCtest_B.p1 < 6;
         raspberrypi_multicore_MPCtest_B.p1++) {
      obj->XOld[raspberrypi_multicore_MPCtest_B.p1 + 13] = 0.0;
    }

    memcpy(&obj->POld[0], &obj->P0[0], 361U * sizeof(real_T));
    obj->count++;
  }

  for (raspberrypi_multicore_MPCtest_B.p1 = 0;
       raspberrypi_multicore_MPCtest_B.p1 < 12;
       raspberrypi_multicore_MPCtest_B.p1++) {
    memcpy(&raspberrypi_multicore_MPCtest_B.B[raspberrypi_multicore_MPCtest_B.p1
           * 19],
           &raspberrypi_multicore_MPCtest_B.B_o[raspberrypi_multicore_MPCtest_B.p1
           * 13], 13U * sizeof(real_T));
  }

  for (raspberrypi_multicore_MPCtest_B.p1 = 0;
       raspberrypi_multicore_MPCtest_B.p1 < 12;
       raspberrypi_multicore_MPCtest_B.p1++) {
    for (raspberrypi_multicore_MPCtest_B.p2 = 0;
         raspberrypi_multicore_MPCtest_B.p2 < 6;
         raspberrypi_multicore_MPCtest_B.p2++) {
      raspberrypi_multicore_MPCtest_B.B[(raspberrypi_multicore_MPCtest_B.p2 + 19
        * raspberrypi_multicore_MPCtest_B.p1) + 13] = 0.0;
    }
  }

  for (raspberrypi_multicore_MPCtest_B.p1 = 0;
       raspberrypi_multicore_MPCtest_B.p1 < 19;
       raspberrypi_multicore_MPCtest_B.p1++) {
    raspberrypi_multicore_MPCtest_B.Anew_md[raspberrypi_multicore_MPCtest_B.p1] =
      0.0;
    for (raspberrypi_multicore_MPCtest_B.p2 = 0;
         raspberrypi_multicore_MPCtest_B.p2 < 19;
         raspberrypi_multicore_MPCtest_B.p2++) {
      raspberrypi_multicore_MPCtest_B.Anew_md[raspberrypi_multicore_MPCtest_B.p1]
        += raspberrypi_multicore_MPCtest_B.Anew[19 *
        raspberrypi_multicore_MPCtest_B.p2 + raspberrypi_multicore_MPCtest_B.p1]
        * obj->XOld[raspberrypi_multicore_MPCtest_B.p2];
    }

    raspberrypi_multicore_MPCtest_B.B_m[raspberrypi_multicore_MPCtest_B.p1] =
      0.0;
  }

  for (raspberrypi_multicore_MPCtest_B.p1 = 0;
       raspberrypi_multicore_MPCtest_B.p1 < 12;
       raspberrypi_multicore_MPCtest_B.p1++) {
    for (raspberrypi_multicore_MPCtest_B.p2 = 0;
         raspberrypi_multicore_MPCtest_B.p2 < 19;
         raspberrypi_multicore_MPCtest_B.p2++) {
      raspberrypi_multicore_MPCtest_B.B_m[raspberrypi_multicore_MPCtest_B.p2] +=
        raspberrypi_multicore_MPCtest_B.B[19 *
        raspberrypi_multicore_MPCtest_B.p1 + raspberrypi_multicore_MPCtest_B.p2]
        * U_MPC[raspberrypi_multicore_MPCtest_B.p1];
    }
  }

  for (raspberrypi_multicore_MPCtest_B.p1 = 0;
       raspberrypi_multicore_MPCtest_B.p1 < 19;
       raspberrypi_multicore_MPCtest_B.p1++) {
    raspberrypi_multicore_MPCtest_B.Xpre[raspberrypi_multicore_MPCtest_B.p1] =
      raspberrypi_multicore_MPCtest_B.Anew_md[raspberrypi_multicore_MPCtest_B.p1]
      + raspberrypi_multicore_MPCtest_B.B_m[raspberrypi_multicore_MPCtest_B.p1];
    memset
      (&raspberrypi_multicore_MPCtest_B.Anew_m[raspberrypi_multicore_MPCtest_B.p1
       * 19], 0, 19U * sizeof(real_T));
  }

  for (raspberrypi_multicore_MPCtest_B.p2 = 0;
       raspberrypi_multicore_MPCtest_B.p2 < 19;
       raspberrypi_multicore_MPCtest_B.p2++) {
    for (raspberrypi_multicore_MPCtest_B.p1 = 0;
         raspberrypi_multicore_MPCtest_B.p1 < 19;
         raspberrypi_multicore_MPCtest_B.p1++) {
      for (raspberrypi_multicore_MPCtest_B.p3 = 0;
           raspberrypi_multicore_MPCtest_B.p3 < 19;
           raspberrypi_multicore_MPCtest_B.p3++) {
        raspberrypi_multicore_MPCtest_B.itmp = 19 *
          raspberrypi_multicore_MPCtest_B.p1 +
          raspberrypi_multicore_MPCtest_B.p2;
        raspberrypi_multicore_MPCtest_B.Anew_m[raspberrypi_multicore_MPCtest_B.itmp]
          += raspberrypi_multicore_MPCtest_B.Anew[19 *
          raspberrypi_multicore_MPCtest_B.p3 +
          raspberrypi_multicore_MPCtest_B.p2] * obj->POld[19 *
          raspberrypi_multicore_MPCtest_B.p1 +
          raspberrypi_multicore_MPCtest_B.p3];
      }

      raspberrypi_multicore_MPCtest_B.Ppre[raspberrypi_multicore_MPCtest_B.p1 +
        19 * raspberrypi_multicore_MPCtest_B.p2] = 0.0;
    }
  }

  for (raspberrypi_multicore_MPCtest_B.p1 = 0;
       raspberrypi_multicore_MPCtest_B.p1 < 19;
       raspberrypi_multicore_MPCtest_B.p1++) {
    for (raspberrypi_multicore_MPCtest_B.p2 = 0;
         raspberrypi_multicore_MPCtest_B.p2 < 19;
         raspberrypi_multicore_MPCtest_B.p2++) {
      for (raspberrypi_multicore_MPCtest_B.p3 = 0;
           raspberrypi_multicore_MPCtest_B.p3 < 19;
           raspberrypi_multicore_MPCtest_B.p3++) {
        raspberrypi_multicore_MPCtest_B.itmp = 19 *
          raspberrypi_multicore_MPCtest_B.p2 +
          raspberrypi_multicore_MPCtest_B.p1;
        raspberrypi_multicore_MPCtest_B.Ppre[raspberrypi_multicore_MPCtest_B.itmp]
          += obj->G[19 * raspberrypi_multicore_MPCtest_B.p3 +
          raspberrypi_multicore_MPCtest_B.p1] * Q[19 *
          raspberrypi_multicore_MPCtest_B.p2 +
          raspberrypi_multicore_MPCtest_B.p3];
      }

      raspberrypi_multicore_MPCtest_B.Anew_n[raspberrypi_multicore_MPCtest_B.p2
        + 19 * raspberrypi_multicore_MPCtest_B.p1] = 0.0;
    }
  }

  for (raspberrypi_multicore_MPCtest_B.p1 = 0;
       raspberrypi_multicore_MPCtest_B.p1 < 19;
       raspberrypi_multicore_MPCtest_B.p1++) {
    for (raspberrypi_multicore_MPCtest_B.p2 = 0;
         raspberrypi_multicore_MPCtest_B.p2 < 19;
         raspberrypi_multicore_MPCtest_B.p2++) {
      for (raspberrypi_multicore_MPCtest_B.p3 = 0;
           raspberrypi_multicore_MPCtest_B.p3 < 19;
           raspberrypi_multicore_MPCtest_B.p3++) {
        raspberrypi_multicore_MPCtest_B.itmp = 19 *
          raspberrypi_multicore_MPCtest_B.p2 +
          raspberrypi_multicore_MPCtest_B.p3;
        raspberrypi_multicore_MPCtest_B.Anew_n[raspberrypi_multicore_MPCtest_B.itmp]
          += raspberrypi_multicore_MPCtest_B.Anew_m[19 *
          raspberrypi_multicore_MPCtest_B.p1 +
          raspberrypi_multicore_MPCtest_B.p3] *
          raspberrypi_multicore_MPCtest_B.Anew[19 *
          raspberrypi_multicore_MPCtest_B.p1 +
          raspberrypi_multicore_MPCtest_B.p2];
      }

      raspberrypi_multicore_MPCtest_B.obj[raspberrypi_multicore_MPCtest_B.p2 +
        19 * raspberrypi_multicore_MPCtest_B.p1] = 0.0;
    }
  }

  for (raspberrypi_multicore_MPCtest_B.p1 = 0;
       raspberrypi_multicore_MPCtest_B.p1 < 19;
       raspberrypi_multicore_MPCtest_B.p1++) {
    for (raspberrypi_multicore_MPCtest_B.p2 = 0;
         raspberrypi_multicore_MPCtest_B.p2 < 19;
         raspberrypi_multicore_MPCtest_B.p2++) {
      for (raspberrypi_multicore_MPCtest_B.p3 = 0;
           raspberrypi_multicore_MPCtest_B.p3 < 19;
           raspberrypi_multicore_MPCtest_B.p3++) {
        raspberrypi_multicore_MPCtest_B.itmp = 19 *
          raspberrypi_multicore_MPCtest_B.p2 +
          raspberrypi_multicore_MPCtest_B.p1;
        raspberrypi_multicore_MPCtest_B.obj[raspberrypi_multicore_MPCtest_B.itmp]
          += raspberrypi_multicore_MPCtest_B.Ppre[19 *
          raspberrypi_multicore_MPCtest_B.p3 +
          raspberrypi_multicore_MPCtest_B.p1] * obj->G[19 *
          raspberrypi_multicore_MPCtest_B.p3 +
          raspberrypi_multicore_MPCtest_B.p2];
      }
    }
  }

  for (raspberrypi_multicore_MPCtest_B.p1 = 0;
       raspberrypi_multicore_MPCtest_B.p1 < 361;
       raspberrypi_multicore_MPCtest_B.p1++) {
    raspberrypi_multicore_MPCtest_B.Ppre[raspberrypi_multicore_MPCtest_B.p1] =
      raspberrypi_multicore_MPCtest_B.Anew_n[raspberrypi_multicore_MPCtest_B.p1]
      + raspberrypi_multicore_MPCtest_B.obj[raspberrypi_multicore_MPCtest_B.p1];
  }

  for (raspberrypi_multicore_MPCtest_B.p1 = 0;
       raspberrypi_multicore_MPCtest_B.p1 < 13;
       raspberrypi_multicore_MPCtest_B.p1++) {
    for (raspberrypi_multicore_MPCtest_B.p2 = 0;
         raspberrypi_multicore_MPCtest_B.p2 < 19;
         raspberrypi_multicore_MPCtest_B.p2++) {
      raspberrypi_multicore_MPCtest_B.K[raspberrypi_multicore_MPCtest_B.p2 + 19 *
        raspberrypi_multicore_MPCtest_B.p1] = tmp_1[13 *
        raspberrypi_multicore_MPCtest_B.p2 + raspberrypi_multicore_MPCtest_B.p1];
    }
  }

  for (raspberrypi_multicore_MPCtest_B.p1 = 0;
       raspberrypi_multicore_MPCtest_B.p1 < 19;
       raspberrypi_multicore_MPCtest_B.p1++) {
    memset
      (&raspberrypi_multicore_MPCtest_B.dv8[raspberrypi_multicore_MPCtest_B.p1 *
       13], 0, 13U * sizeof(real_T));
    for (raspberrypi_multicore_MPCtest_B.p3 = 0;
         raspberrypi_multicore_MPCtest_B.p3 < 19;
         raspberrypi_multicore_MPCtest_B.p3++) {
      for (raspberrypi_multicore_MPCtest_B.p2 = 0;
           raspberrypi_multicore_MPCtest_B.p2 < 13;
           raspberrypi_multicore_MPCtest_B.p2++) {
        raspberrypi_multicore_MPCtest_B.itmp = 13 *
          raspberrypi_multicore_MPCtest_B.p1 +
          raspberrypi_multicore_MPCtest_B.p2;
        raspberrypi_multicore_MPCtest_B.dv8[raspberrypi_multicore_MPCtest_B.itmp]
          += (real_T)tmp_1[13 * raspberrypi_multicore_MPCtest_B.p3 +
          raspberrypi_multicore_MPCtest_B.p2] *
          raspberrypi_multicore_MPCtest_B.Ppre[19 *
          raspberrypi_multicore_MPCtest_B.p1 +
          raspberrypi_multicore_MPCtest_B.p3];
      }
    }
  }

  for (raspberrypi_multicore_MPCtest_B.p1 = 0;
       raspberrypi_multicore_MPCtest_B.p1 < 13;
       raspberrypi_multicore_MPCtest_B.p1++) {
    memset
      (&raspberrypi_multicore_MPCtest_B.Ppre_j[raspberrypi_multicore_MPCtest_B.p1
       * 19], 0, 19U * sizeof(real_T));
    for (raspberrypi_multicore_MPCtest_B.p3 = 0;
         raspberrypi_multicore_MPCtest_B.p3 < 19;
         raspberrypi_multicore_MPCtest_B.p3++) {
      for (raspberrypi_multicore_MPCtest_B.p2 = 0;
           raspberrypi_multicore_MPCtest_B.p2 < 19;
           raspberrypi_multicore_MPCtest_B.p2++) {
        raspberrypi_multicore_MPCtest_B.itmp = 19 *
          raspberrypi_multicore_MPCtest_B.p1 +
          raspberrypi_multicore_MPCtest_B.p2;
        raspberrypi_multicore_MPCtest_B.Ppre_j[raspberrypi_multicore_MPCtest_B.itmp]
          += raspberrypi_multicore_MPCtest_B.Ppre[19 *
          raspberrypi_multicore_MPCtest_B.p3 +
          raspberrypi_multicore_MPCtest_B.p2] *
          raspberrypi_multicore_MPCtest_B.K[19 *
          raspberrypi_multicore_MPCtest_B.p1 +
          raspberrypi_multicore_MPCtest_B.p3];
      }
    }

    for (raspberrypi_multicore_MPCtest_B.p2 = 0;
         raspberrypi_multicore_MPCtest_B.p2 < 13;
         raspberrypi_multicore_MPCtest_B.p2++) {
      raspberrypi_multicore_MPCtest_B.absx11 = 0.0;
      for (raspberrypi_multicore_MPCtest_B.p3 = 0;
           raspberrypi_multicore_MPCtest_B.p3 < 19;
           raspberrypi_multicore_MPCtest_B.p3++) {
        raspberrypi_multicore_MPCtest_B.absx11 +=
          raspberrypi_multicore_MPCtest_B.dv8[13 *
          raspberrypi_multicore_MPCtest_B.p3 +
          raspberrypi_multicore_MPCtest_B.p1] *
          raspberrypi_multicore_MPCtest_B.K[19 *
          raspberrypi_multicore_MPCtest_B.p2 +
          raspberrypi_multicore_MPCtest_B.p3];
      }

      raspberrypi_multicore_MPCtest_B.p3 = 13 *
        raspberrypi_multicore_MPCtest_B.p2 + raspberrypi_multicore_MPCtest_B.p1;
      raspberrypi_multicore_MPCtest_B.A[raspberrypi_multicore_MPCtest_B.p3] =
        R[raspberrypi_multicore_MPCtest_B.p3] +
        raspberrypi_multicore_MPCtest_B.absx11;
    }
  }

  raspberrypi_multicore_MPC_mrdiv(raspberrypi_multicore_MPCtest_B.Ppre_j,
    raspberrypi_multicore_MPCtest_B.A, raspberrypi_multicore_MPCtest_B.K);

  /* P=Ppre; */
  memset(&raspberrypi_multicore_MPCtest_B.Anew[0], 0, 361U * sizeof(real_T));
  for (raspberrypi_multicore_MPCtest_B.p1 = 0;
       raspberrypi_multicore_MPCtest_B.p1 < 19;
       raspberrypi_multicore_MPCtest_B.p1++) {
    raspberrypi_multicore_MPCtest_B.Anew[raspberrypi_multicore_MPCtest_B.p1 + 19
      * raspberrypi_multicore_MPCtest_B.p1] = 1.0;
  }

  for (raspberrypi_multicore_MPCtest_B.p1 = 0;
       raspberrypi_multicore_MPCtest_B.p1 < 19;
       raspberrypi_multicore_MPCtest_B.p1++) {
    for (raspberrypi_multicore_MPCtest_B.p2 = 0;
         raspberrypi_multicore_MPCtest_B.p2 < 19;
         raspberrypi_multicore_MPCtest_B.p2++) {
      raspberrypi_multicore_MPCtest_B.absx11 = 0.0;
      for (raspberrypi_multicore_MPCtest_B.p3 = 0;
           raspberrypi_multicore_MPCtest_B.p3 < 13;
           raspberrypi_multicore_MPCtest_B.p3++) {
        raspberrypi_multicore_MPCtest_B.absx11 +=
          raspberrypi_multicore_MPCtest_B.K[19 *
          raspberrypi_multicore_MPCtest_B.p3 +
          raspberrypi_multicore_MPCtest_B.p1] * (real_T)tmp_1[13 *
          raspberrypi_multicore_MPCtest_B.p2 +
          raspberrypi_multicore_MPCtest_B.p3];
      }

      raspberrypi_multicore_MPCtest_B.itmp = 19 *
        raspberrypi_multicore_MPCtest_B.p2 + raspberrypi_multicore_MPCtest_B.p1;
      raspberrypi_multicore_MPCtest_B.Anew_m[raspberrypi_multicore_MPCtest_B.itmp]
        =
        raspberrypi_multicore_MPCtest_B.Anew[raspberrypi_multicore_MPCtest_B.itmp]
        - raspberrypi_multicore_MPCtest_B.absx11;
      P[raspberrypi_multicore_MPCtest_B.p2 + 19 *
        raspberrypi_multicore_MPCtest_B.p1] = 0.0;
    }
  }

  for (raspberrypi_multicore_MPCtest_B.p1 = 0;
       raspberrypi_multicore_MPCtest_B.p1 < 19;
       raspberrypi_multicore_MPCtest_B.p1++) {
    for (raspberrypi_multicore_MPCtest_B.p2 = 0;
         raspberrypi_multicore_MPCtest_B.p2 < 19;
         raspberrypi_multicore_MPCtest_B.p2++) {
      for (raspberrypi_multicore_MPCtest_B.p3 = 0;
           raspberrypi_multicore_MPCtest_B.p3 < 19;
           raspberrypi_multicore_MPCtest_B.p3++) {
        raspberrypi_multicore_MPCtest_B.itmp = 19 *
          raspberrypi_multicore_MPCtest_B.p1 +
          raspberrypi_multicore_MPCtest_B.p3;
        P[raspberrypi_multicore_MPCtest_B.itmp] +=
          raspberrypi_multicore_MPCtest_B.Anew_m[19 *
          raspberrypi_multicore_MPCtest_B.p2 +
          raspberrypi_multicore_MPCtest_B.p3] *
          raspberrypi_multicore_MPCtest_B.Ppre[19 *
          raspberrypi_multicore_MPCtest_B.p1 +
          raspberrypi_multicore_MPCtest_B.p2];
      }
    }
  }

  /*              if updateEN<0.5 */
  /*                  K=K*0; */
  /*              else */
  /*                  P=(eye(length(x0))-K*obj.C)*Ppre; */
  /*              end */
  for (raspberrypi_multicore_MPCtest_B.p1 = 0;
       raspberrypi_multicore_MPCtest_B.p1 < 13;
       raspberrypi_multicore_MPCtest_B.p1++) {
    raspberrypi_multicore_MPCtest_B.absx11 = 0.0;
    for (raspberrypi_multicore_MPCtest_B.p2 = 0;
         raspberrypi_multicore_MPCtest_B.p2 < 19;
         raspberrypi_multicore_MPCtest_B.p2++) {
      raspberrypi_multicore_MPCtest_B.absx11 += (real_T)tmp_1[13 *
        raspberrypi_multicore_MPCtest_B.p2 + raspberrypi_multicore_MPCtest_B.p1]
        * raspberrypi_multicore_MPCtest_B.Xpre[raspberrypi_multicore_MPCtest_B.p2];
    }

    raspberrypi_multicore_MPCtest_B.xFB[raspberrypi_multicore_MPCtest_B.p1] =
      xFB[raspberrypi_multicore_MPCtest_B.p1] -
      raspberrypi_multicore_MPCtest_B.absx11;
  }

  for (raspberrypi_multicore_MPCtest_B.p1 = 0;
       raspberrypi_multicore_MPCtest_B.p1 < 19;
       raspberrypi_multicore_MPCtest_B.p1++) {
    raspberrypi_multicore_MPCtest_B.absx11 = 0.0;
    for (raspberrypi_multicore_MPCtest_B.p2 = 0;
         raspberrypi_multicore_MPCtest_B.p2 < 13;
         raspberrypi_multicore_MPCtest_B.p2++) {
      raspberrypi_multicore_MPCtest_B.absx11 +=
        raspberrypi_multicore_MPCtest_B.K[19 *
        raspberrypi_multicore_MPCtest_B.p2 + raspberrypi_multicore_MPCtest_B.p1]
        * raspberrypi_multicore_MPCtest_B.xFB[raspberrypi_multicore_MPCtest_B.p2];
    }

    estXbar[raspberrypi_multicore_MPCtest_B.p1] =
      raspberrypi_multicore_MPCtest_B.Xpre[raspberrypi_multicore_MPCtest_B.p1] +
      raspberrypi_multicore_MPCtest_B.absx11;
  }

  if (Reset > 0.5) {
    memcpy(&estXbar[0], &xFB[0], 13U * sizeof(real_T));
    for (raspberrypi_multicore_MPCtest_B.p1 = 0;
         raspberrypi_multicore_MPCtest_B.p1 < 6;
         raspberrypi_multicore_MPCtest_B.p1++) {
      estXbar[raspberrypi_multicore_MPCtest_B.p1 + 13] = 0.0;
    }

    memcpy(&P[0], &obj->P0[0], 361U * sizeof(real_T));
  }

  memcpy(&obj->XOld[0], &estXbar[0], 19U * sizeof(real_T));
  memcpy(&obj->POld[0], &P[0], 361U * sizeof(real_T));
}

static void raspberrypi_mul_SystemCore_step(KalmanFilter_DIY_MPCDis_raspb_T *obj,
  const real_T varargin_1[12], const real_T varargin_2[13], const real_T
  varargin_3[12], const real_T varargin_4[361], const real_T varargin_5[169],
  real_T varargin_6, real_T varargout_1[19], real_T varargout_2[361])
{
  KalmanFilter_DIY_MPCDis_stepImp(obj, varargin_1, varargin_2, varargin_3,
    varargin_4, varargin_5, varargin_6, varargout_1, varargout_2);
}

static real_T raspberrypi_multicore__norm_amt(const real_T x[3])
{
  real_T absxk;
  real_T scale;
  real_T t;
  real_T y;
  scale = 3.3121686421112381E-170;
  absxk = fabs(x[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  absxk = fabs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = fabs(x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  return scale * sqrt(y);
}

static void raspberrypi__SystemCore_step_am(refTrajectory_v4_raspberrypi__T *obj,
  real_T varargin_1, real_T varargin_2, real_T varargin_3, const real_T
  varargin_4[3], const real_T varargin_5[3], const real_T varargin_6[13], real_T
  varargin_7, real_T varargout_1[78], real_T varargout_2[13], real_T
  varargout_3[78], real_T varargout_4[3], real_T varargout_5[3])
{
  real_T headX_tmp;
  real_T headX_tmp_0;
  real_T varargout_4_0;
  int32_T b_i;
  int32_T headX_tmp_tmp;
  int32_T i;
  static const int8_T tmp[3] = { 1, 0, 0 };

  /*  refSeqOut: ref sequence including sitaErr compensation */
  /*  refSeq: ref sequence excluding sitaErr compensation */
  /*  refP: last element of the refSeq */
  /*  disable: sitaErr will not be accumulated, ref will not be updated */
  memcpy(&raspberrypi_multicore_MPCtest_B.ref[0], &varargin_6[0], 13U * sizeof
         (real_T));
  raspberrypi_multicore_MPCtest_B.ref[12] = 9.8;
  memset(&varargout_3[0], 0, 78U * sizeof(real_T));
  for (i = 0; i < 13; i++) {
    varargout_3[6 * i] = raspberrypi_multicore_MPCtest_B.ref[i];
  }

  raspberrypi_multicore_MPCtest_B.desH = obj->desHeight;
  varargout_4[0] = obj->refSeqOld[19] - varargin_6[3];
  varargout_4[1] = obj->refSeqOld[25] - varargin_6[4];
  varargout_4[2] = obj->refSeqOld[31] - varargin_6[5];

  /*  deadzone */
  if ((varargout_4[0] > 0.0087266462599716477) || (varargout_4[0] <
       -0.0087266462599716477)) {
    if (varargout_4[0] < 0.0) {
      varargout_4_0 = -1.0;
    } else if (varargout_4[0] > 0.0) {
      varargout_4_0 = 1.0;
    } else {
      varargout_4_0 = 0.0;
    }

    varargout_4[0] -= varargout_4_0 * 0.0087266462599716477;
  } else {
    varargout_4[0] = 0.0;
  }

  if ((varargout_4[1] > 0.0087266462599716477) || (varargout_4[1] <
       -0.0087266462599716477)) {
    if (varargout_4[1] < 0.0) {
      varargout_4_0 = -1.0;
    } else if (varargout_4[1] > 0.0) {
      varargout_4_0 = 1.0;
    } else {
      varargout_4_0 = 0.0;
    }

    varargout_4[1] -= varargout_4_0 * 0.0087266462599716477;
  } else {
    varargout_4[1] = 0.0;
  }

  if ((varargout_4[2] > 0.017453292519943295) || (varargout_4[2] <
       -0.017453292519943295)) {
    if (varargout_4[2] < 0.0) {
      varargout_4_0 = -1.0;
    } else if (varargout_4[2] > 0.0) {
      varargout_4_0 = 1.0;
    } else {
      varargout_4_0 = 0.0;
    }

    varargout_4[2] -= varargout_4_0 * 0.017453292519943295;
  } else {
    varargout_4[2] = 0.0;
  }

  if (varargin_7 < 0.5) {
    memset(&raspberrypi_multicore_MPCtest_B.Rsur[0], 0, 9U * sizeof(real_T));
    raspberrypi_multicore_MPCtest_B.Rsur[0] = obj->sitaErr_K[0];
    raspberrypi_multicore_MPCtest_B.Rsur[4] = obj->sitaErr_K[1];
    raspberrypi_multicore_MPCtest_B.Rsur[8] = obj->sitaErr_K[2];
    for (i = 0; i < 3; i++) {
      obj->sitaErrOld[i] += (raspberrypi_multicore_MPCtest_B.Rsur[i + 3] *
        varargout_4[1] + raspberrypi_multicore_MPCtest_B.Rsur[i] * varargout_4[0])
        + raspberrypi_multicore_MPCtest_B.Rsur[i + 6] * varargout_4[2];
    }
  }

  /*              surVN=[0;0;1]; */
  /*              sura=[0;0;0]; */
  obj->sitaZOld += varargin_3 * obj->dt;

  /* Rbody=Rz(X_FB(6))*Ry(X_FB(5))*Rx(X_FB(4)); */
  /*  3D rotation matrix, vb=M*v: */
  /*  rotate a vector in one frame, */
  /*  or change the vector 'v' in rotated frame to 'vb' in world frame */
  /*  subfunction */
  /*  3D rotation matrix, vb=M*v: */
  /*  rotate a vector in one frame, */
  /*  or change the vector 'v' in rotated frame to 'vb' in world frame */
  /*  3D rotation matrix, vb=M*v: */
  /*  rotate a vector in one frame, */
  /*  or change the vector 'v' in rotated frame to 'vb' in world frame */
  varargout_4_0 = sin(obj->sitaZOld);
  raspberrypi_multicore_MPCtest_B.sy = cos(obj->sitaZOld);
  raspberrypi_multicore_MPCtest_B.headX_tmp_fn = sin(varargin_6[4]);
  raspberrypi_multicore_MPCtest_B.headX_tmp_j = cos(varargin_6[4]);
  headX_tmp = sin(varargin_6[3]);
  headX_tmp_0 = cos(varargin_6[3]);
  raspberrypi_multicore_MPCtest_B.Rsur[0] =
    raspberrypi_multicore_MPCtest_B.headX_tmp_j;
  raspberrypi_multicore_MPCtest_B.Rsur[3] = 0.0;
  raspberrypi_multicore_MPCtest_B.Rsur[6] =
    raspberrypi_multicore_MPCtest_B.headX_tmp_fn;
  raspberrypi_multicore_MPCtest_B.Rsur[2] =
    -raspberrypi_multicore_MPCtest_B.headX_tmp_fn;
  raspberrypi_multicore_MPCtest_B.Rsur[5] = 0.0;
  raspberrypi_multicore_MPCtest_B.Rsur[8] =
    raspberrypi_multicore_MPCtest_B.headX_tmp_j;
  raspberrypi_multicore_MPCtest_B.dv24[1] = 0.0;
  raspberrypi_multicore_MPCtest_B.dv24[4] = headX_tmp_0;
  raspberrypi_multicore_MPCtest_B.dv24[7] = -headX_tmp;
  raspberrypi_multicore_MPCtest_B.dv24[2] = 0.0;
  raspberrypi_multicore_MPCtest_B.dv24[5] = headX_tmp;
  raspberrypi_multicore_MPCtest_B.dv24[8] = headX_tmp_0;
  raspberrypi_multicore_MPCtest_B.headX_tmp[0] =
    raspberrypi_multicore_MPCtest_B.sy;
  raspberrypi_multicore_MPCtest_B.headX_tmp[3] = -varargout_4_0;
  raspberrypi_multicore_MPCtest_B.headX_tmp[6] = 0.0;
  raspberrypi_multicore_MPCtest_B.headX_tmp[1] = varargout_4_0;
  raspberrypi_multicore_MPCtest_B.headX_tmp[4] =
    raspberrypi_multicore_MPCtest_B.sy;
  raspberrypi_multicore_MPCtest_B.headX_tmp[7] = 0.0;
  raspberrypi_multicore_MPCtest_B.Rsur[1] = 0.0;
  raspberrypi_multicore_MPCtest_B.dv24[0] = 1.0;
  raspberrypi_multicore_MPCtest_B.headX_tmp[2] = 0.0;
  raspberrypi_multicore_MPCtest_B.Rsur[4] = 1.0;
  raspberrypi_multicore_MPCtest_B.dv24[3] = 0.0;
  raspberrypi_multicore_MPCtest_B.headX_tmp[5] = 0.0;
  raspberrypi_multicore_MPCtest_B.Rsur[7] = 0.0;
  raspberrypi_multicore_MPCtest_B.dv24[6] = 0.0;
  raspberrypi_multicore_MPCtest_B.headX_tmp[8] = 1.0;
  for (i = 0; i < 3; i++) {
    for (b_i = 0; b_i < 3; b_i++) {
      headX_tmp_tmp = b_i + 3 * i;
      raspberrypi_multicore_MPCtest_B.headX_tmp_f[headX_tmp_tmp] = 0.0;
      raspberrypi_multicore_MPCtest_B.headX_tmp_f[headX_tmp_tmp] +=
        raspberrypi_multicore_MPCtest_B.dv24[3 * i] *
        raspberrypi_multicore_MPCtest_B.Rsur[b_i];
      raspberrypi_multicore_MPCtest_B.headX_tmp_f[headX_tmp_tmp] +=
        raspberrypi_multicore_MPCtest_B.dv24[3 * i + 1] *
        raspberrypi_multicore_MPCtest_B.Rsur[b_i + 3];
      raspberrypi_multicore_MPCtest_B.headX_tmp_f[headX_tmp_tmp] +=
        raspberrypi_multicore_MPCtest_B.dv24[3 * i + 2] *
        raspberrypi_multicore_MPCtest_B.Rsur[b_i + 6];
    }
  }

  raspberrypi_multicore_MPCtest_B.sy = raspberrypi_multicore__norm_amt
    (varargin_4);
  varargout_4_0 = 0.0;
  for (i = 0; i < 3; i++) {
    raspberrypi_multicore_MPCtest_B.headX[i] = 0.0;
    for (b_i = 0; b_i < 3; b_i++) {
      headX_tmp_tmp = i + 3 * b_i;
      raspberrypi_multicore_MPCtest_B.Rsur[headX_tmp_tmp] = 0.0;
      raspberrypi_multicore_MPCtest_B.Rsur[headX_tmp_tmp] +=
        raspberrypi_multicore_MPCtest_B.headX_tmp_f[3 * b_i] *
        raspberrypi_multicore_MPCtest_B.headX_tmp[i];
      raspberrypi_multicore_MPCtest_B.Rsur[headX_tmp_tmp] +=
        raspberrypi_multicore_MPCtest_B.headX_tmp_f[3 * b_i + 1] *
        raspberrypi_multicore_MPCtest_B.headX_tmp[i + 3];
      raspberrypi_multicore_MPCtest_B.Rsur[headX_tmp_tmp] +=
        raspberrypi_multicore_MPCtest_B.headX_tmp_f[3 * b_i + 2] *
        raspberrypi_multicore_MPCtest_B.headX_tmp[i + 6];
      raspberrypi_multicore_MPCtest_B.headX[i] +=
        raspberrypi_multicore_MPCtest_B.Rsur[headX_tmp_tmp] * (real_T)tmp[b_i];
    }

    raspberrypi_multicore_MPCtest_B.headX_tmp_fn = varargin_4[i] /
      raspberrypi_multicore_MPCtest_B.sy;
    varargout_4_0 += raspberrypi_multicore_MPCtest_B.headX_tmp_fn *
      raspberrypi_multicore_MPCtest_B.headX[i];
    raspberrypi_multicore_MPCtest_B.surVN[i] =
      raspberrypi_multicore_MPCtest_B.headX_tmp_fn;
  }

  raspberrypi_multicore_MPCtest_B.headX[0] -= varargout_4_0 *
    raspberrypi_multicore_MPCtest_B.surVN[0];
  raspberrypi_multicore_MPCtest_B.headX[1] -= varargout_4_0 *
    raspberrypi_multicore_MPCtest_B.surVN[1];
  raspberrypi_multicore_MPCtest_B.headX[2] -= varargout_4_0 *
    raspberrypi_multicore_MPCtest_B.surVN[2];
  raspberrypi_multicore_MPCtest_B.sy = raspberrypi_multicore__norm_amt
    (raspberrypi_multicore_MPCtest_B.headX);
  varargout_4_0 = raspberrypi_multicore_MPCtest_B.headX[0] /
    raspberrypi_multicore_MPCtest_B.sy;
  raspberrypi_multicore_MPCtest_B.Rsur[0] = varargout_4_0;
  raspberrypi_multicore_MPCtest_B.Rsur[6] =
    raspberrypi_multicore_MPCtest_B.surVN[0];
  raspberrypi_multicore_MPCtest_B.headX[0] = varargout_4_0;
  varargout_4_0 = raspberrypi_multicore_MPCtest_B.headX[1] /
    raspberrypi_multicore_MPCtest_B.sy;
  raspberrypi_multicore_MPCtest_B.Rsur[1] = varargout_4_0;
  raspberrypi_multicore_MPCtest_B.Rsur[7] =
    raspberrypi_multicore_MPCtest_B.surVN[1];
  raspberrypi_multicore_MPCtest_B.headX[1] = varargout_4_0;
  varargout_4_0 = raspberrypi_multicore_MPCtest_B.headX[2] /
    raspberrypi_multicore_MPCtest_B.sy;
  raspberrypi_multicore_MPCtest_B.Rsur[2] = varargout_4_0;
  raspberrypi_multicore_MPCtest_B.Rsur[8] =
    raspberrypi_multicore_MPCtest_B.surVN[2];
  raspberrypi_multicore_MPCtest_B.Rsur[3] =
    raspberrypi_multicore_MPCtest_B.surVN[1] * varargout_4_0 -
    raspberrypi_multicore_MPCtest_B.headX[1] *
    raspberrypi_multicore_MPCtest_B.surVN[2];
  raspberrypi_multicore_MPCtest_B.Rsur[4] =
    raspberrypi_multicore_MPCtest_B.headX[0] *
    raspberrypi_multicore_MPCtest_B.surVN[2] -
    raspberrypi_multicore_MPCtest_B.surVN[0] * varargout_4_0;
  raspberrypi_multicore_MPCtest_B.Rsur[5] =
    raspberrypi_multicore_MPCtest_B.surVN[0] *
    raspberrypi_multicore_MPCtest_B.headX[1] -
    raspberrypi_multicore_MPCtest_B.headX[0] *
    raspberrypi_multicore_MPCtest_B.surVN[1];

  /* %% leave the first step ref to the current state */
  /*              for i=2:1:obj.numP */
  /*                  desTheta=Rot2Eul(Rsur); */
  /*                  if i<2.5 */
  /*                      desr=Rsur*[vxL;vyL;0]*obj.dt+obj.refSeqOld(3,1:3)'; */
  /*                  else */
  /*                      desr=Rsur*[vxL;vyL;0]*obj.dt+refSeq(i-1,1:3)'; */
  /*                  end */
  /*                  desdr=Rsur*[vxL;vyL;0]; */
  /*                  desdtheta=Rsur*[0;0;omegaZ]; */
  /*                  pSur=[0;0;sura(1)]; % get the point on the surface whose x and y coordinates are zero */
  /*                  h=(desr-pSur)'*surVN; */
  /*                  desr=desr+(desH-h)*surVN; % guarantee that the distance between the height and the robot are the desired height */
  /*                   */
  /*                  refSeq(i,1:3)=desr; */
  /*                  refSeq(i,4:6)=desTheta; */
  /*                  refSeq(i,7:9)=desdr; */
  /*                  refSeq(i,10:12)=desdtheta; */
  /*                  refSeq(i,13)=9.8; */
  /*              end */
  /* %% override the first step ref to the desired ref */
  raspberrypi_multicore_MPCtest_B.sy = sqrt
    (raspberrypi_multicore_MPCtest_B.Rsur[0] *
     raspberrypi_multicore_MPCtest_B.Rsur[0] +
     raspberrypi_multicore_MPCtest_B.Rsur[1] *
     raspberrypi_multicore_MPCtest_B.Rsur[1]);
  for (b_i = 0; b_i < 6; b_i++) {
    /*  Rotation matrix to euler angles, ZYX intrinsic order */
    raspberrypi_multicore_MPCtest_B.headX[2] = 0.0;
    if (raspberrypi_multicore_MPCtest_B.sy < 1.0E-6) {
      raspberrypi_multicore_MPCtest_B.headX[0] = rt_atan2d_snf
        (-raspberrypi_multicore_MPCtest_B.Rsur[7],
         raspberrypi_multicore_MPCtest_B.Rsur[4]);
      raspberrypi_multicore_MPCtest_B.headX[1] = rt_atan2d_snf(-varargout_4_0,
        raspberrypi_multicore_MPCtest_B.sy);
    } else {
      raspberrypi_multicore_MPCtest_B.headX[0] = rt_atan2d_snf
        (raspberrypi_multicore_MPCtest_B.Rsur[5],
         raspberrypi_multicore_MPCtest_B.Rsur[8]);
      raspberrypi_multicore_MPCtest_B.headX[1] = rt_atan2d_snf(-varargout_4_0,
        raspberrypi_multicore_MPCtest_B.sy);
      raspberrypi_multicore_MPCtest_B.headX[2] = rt_atan2d_snf
        (raspberrypi_multicore_MPCtest_B.Rsur[1],
         raspberrypi_multicore_MPCtest_B.Rsur[0]);
    }

    if ((real_T)b_i + 1.0 < 1.5) {
      for (i = 0; i < 3; i++) {
        raspberrypi_multicore_MPCtest_B.desr[i] = obj->refSeqOld[6 * i + 1] +
          (raspberrypi_multicore_MPCtest_B.Rsur[i + 6] * 0.0 +
           (raspberrypi_multicore_MPCtest_B.Rsur[i + 3] * varargin_2 +
            raspberrypi_multicore_MPCtest_B.Rsur[i] * varargin_1)) * obj->dt;
      }
    } else {
      for (i = 0; i < 3; i++) {
        raspberrypi_multicore_MPCtest_B.desr[i] = varargout_3[(6 * i + b_i) - 1]
          + (raspberrypi_multicore_MPCtest_B.Rsur[i + 6] * 0.0 +
             (raspberrypi_multicore_MPCtest_B.Rsur[i + 3] * varargin_2 +
              raspberrypi_multicore_MPCtest_B.Rsur[i] * varargin_1)) * obj->dt;
      }
    }

    /*  get the point on the surface whose x and y coordinates are zero */
    raspberrypi_multicore_MPCtest_B.headX_tmp_fn =
      raspberrypi_multicore_MPCtest_B.desH -
      ((raspberrypi_multicore_MPCtest_B.desr[0] *
        raspberrypi_multicore_MPCtest_B.surVN[0] +
        raspberrypi_multicore_MPCtest_B.desr[1] *
        raspberrypi_multicore_MPCtest_B.surVN[1]) +
       (raspberrypi_multicore_MPCtest_B.desr[2] - varargin_5[0]) *
       raspberrypi_multicore_MPCtest_B.surVN[2]);

    /*  guarantee that the distance between the height and the robot are the desired height */
    for (i = 0; i < 3; i++) {
      varargout_3[b_i + 6 * i] = raspberrypi_multicore_MPCtest_B.headX_tmp_fn *
        raspberrypi_multicore_MPCtest_B.surVN[i] +
        raspberrypi_multicore_MPCtest_B.desr[i];
      varargout_3[b_i + 6 * ((i + 4) - 1)] =
        raspberrypi_multicore_MPCtest_B.headX[i];
      raspberrypi_multicore_MPCtest_B.headX_tmp_j =
        raspberrypi_multicore_MPCtest_B.Rsur[i];
      headX_tmp = raspberrypi_multicore_MPCtest_B.headX_tmp_j * varargin_1;
      headX_tmp_0 = raspberrypi_multicore_MPCtest_B.headX_tmp_j * 0.0;
      raspberrypi_multicore_MPCtest_B.headX_tmp_j =
        raspberrypi_multicore_MPCtest_B.Rsur[i + 3];
      headX_tmp += raspberrypi_multicore_MPCtest_B.headX_tmp_j * varargin_2;
      headX_tmp_0 += raspberrypi_multicore_MPCtest_B.headX_tmp_j * 0.0;
      raspberrypi_multicore_MPCtest_B.headX_tmp_j =
        raspberrypi_multicore_MPCtest_B.Rsur[i + 6];
      varargout_3[b_i + 6 * ((i + 7) - 1)] =
        raspberrypi_multicore_MPCtest_B.headX_tmp_j * 0.0 + headX_tmp;
      varargout_3[b_i + 6 * ((i + 10) - 1)] =
        raspberrypi_multicore_MPCtest_B.headX_tmp_j * varargin_3 + headX_tmp_0;
    }

    varargout_3[b_i + 72] = 9.8;
  }

  if (varargin_7 > 0.5) {
    memcpy(&varargout_3[0], &obj->refSeqOld[0], 78U * sizeof(real_T));
  }

  memcpy(&varargout_1[0], &varargout_3[0], 78U * sizeof(real_T));
  for (b_i = 0; b_i < 5; b_i++) {
    raspberrypi_multicore_MPCtest_B.ref[0] = 0.0;
    raspberrypi_multicore_MPCtest_B.ref[3] = obj->sitaErrOld[0];
    raspberrypi_multicore_MPCtest_B.ref[1] = 0.0;
    raspberrypi_multicore_MPCtest_B.ref[4] = obj->sitaErrOld[1];
    raspberrypi_multicore_MPCtest_B.ref[2] = 0.0;
    raspberrypi_multicore_MPCtest_B.ref[5] = obj->sitaErrOld[2];
    for (i = 0; i < 7; i++) {
      raspberrypi_multicore_MPCtest_B.ref[i + 6] = 0.0;
    }

    for (i = 0; i < 13; i++) {
      headX_tmp_tmp = (6 * i + b_i) + 1;
      varargout_1[headX_tmp_tmp] += raspberrypi_multicore_MPCtest_B.ref[i];
    }
  }

  if (varargin_7 > 0.5) {
    memcpy(&varargout_1[0], &varargout_3[0], 78U * sizeof(real_T));
  }

  memcpy(&obj->refSeqOld[0], &varargout_3[0], 78U * sizeof(real_T));
  for (i = 0; i < 13; i++) {
    varargout_2[i] = varargout_3[6 * i + 1];
  }

  varargout_5[0] = obj->sitaErrOld[0];
  varargout_5[1] = obj->sitaErrOld[1];
  varargout_5[2] = obj->sitaErrOld[2];
}

static void ra_ssModelgen_estDis_stepImpl_a(const
  ssModelgen_estDis_raspberrypi_T *obj, const real_T PendAll[12], const real_T
  xFB[13], const real_T SPLeg[4], real_T Anew[169], real_T Bnew[234], real_T U
  [18], real_T Y[13], real_T DX[13], real_T Inow[9])
{
  int32_T itmp;
  int32_T p1;
  int32_T p2;
  int32_T p3;
  static const int8_T tmp[36] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

  static const int8_T tmp_0[169] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

  /* Pc=XOld(1:3); */
  /*              if disable<0.5 */
  /*                  Unow=UOld; */
  /*              else */
  /*                  Unow=obj.U_stand; */
  /*              end */
  raspberrypi_multicore_MPCtest_B.absx11_c = sin(xFB[5]);
  raspberrypi_multicore_MPCtest_B.absx21_n = cos(xFB[5]);
  raspberrypi_multicore_MPCtest_B.Rz_i[0] =
    raspberrypi_multicore_MPCtest_B.absx21_n;
  raspberrypi_multicore_MPCtest_B.Rz_i[3] =
    -raspberrypi_multicore_MPCtest_B.absx11_c;
  raspberrypi_multicore_MPCtest_B.Rz_i[6] = 0.0;
  raspberrypi_multicore_MPCtest_B.Rz_i[1] =
    raspberrypi_multicore_MPCtest_B.absx11_c;
  raspberrypi_multicore_MPCtest_B.Rz_i[4] =
    raspberrypi_multicore_MPCtest_B.absx21_n;
  raspberrypi_multicore_MPCtest_B.Rz_i[7] = 0.0;
  raspberrypi_multicore_MPCtest_B.Rz_i[2] = 0.0;
  raspberrypi_multicore_MPCtest_B.Rz_i[5] = 0.0;
  raspberrypi_multicore_MPCtest_B.Rz_i[8] = 1.0;

  /*  Consider to replace this !!!! */
  for (p1 = 0; p1 < 3; p1++) {
    for (p2 = 0; p2 < 3; p2++) {
      p3 = p1 + 3 * p2;
      raspberrypi_multicore_MPCtest_B.Iinv[p3] = 0.0;
      raspberrypi_multicore_MPCtest_B.Iinv[p3] += obj->Inorm[3 * p2] *
        raspberrypi_multicore_MPCtest_B.Rz_i[p1];
      raspberrypi_multicore_MPCtest_B.Iinv[p3] += obj->Inorm[3 * p2 + 1] *
        raspberrypi_multicore_MPCtest_B.Rz_i[p1 + 3];
      raspberrypi_multicore_MPCtest_B.Iinv[p3] += obj->Inorm[3 * p2 + 2] *
        raspberrypi_multicore_MPCtest_B.Rz_i[p1 + 6];
    }

    for (p2 = 0; p2 < 3; p2++) {
      p3 = p1 + 3 * p2;
      Inow[p3] = 0.0;
      Inow[p3] += raspberrypi_multicore_MPCtest_B.Iinv[p1] *
        raspberrypi_multicore_MPCtest_B.Rz_i[p2];
      Inow[p3] += raspberrypi_multicore_MPCtest_B.Iinv[p1 + 3] *
        raspberrypi_multicore_MPCtest_B.Rz_i[p2 + 3];
      Inow[p3] += raspberrypi_multicore_MPCtest_B.Iinv[p1 + 6] *
        raspberrypi_multicore_MPCtest_B.Rz_i[p2 + 6];
    }
  }

  memcpy(&raspberrypi_multicore_MPCtest_B.Rz_i[0], &Inow[0], 9U * sizeof(real_T));
  p1 = 1;
  p2 = 3;
  p3 = 6;
  raspberrypi_multicore_MPCtest_B.absx11_c = fabs(Inow[0]);
  raspberrypi_multicore_MPCtest_B.absx21_n = fabs(Inow[1]);
  raspberrypi_multicore_MPCtest_B.absx31_i = fabs(Inow[2]);
  if ((raspberrypi_multicore_MPCtest_B.absx21_n >
       raspberrypi_multicore_MPCtest_B.absx11_c) &&
      (raspberrypi_multicore_MPCtest_B.absx21_n >
       raspberrypi_multicore_MPCtest_B.absx31_i)) {
    p1 = 4;
    p2 = 0;
    raspberrypi_multicore_MPCtest_B.Rz_i[0] = Inow[1];
    raspberrypi_multicore_MPCtest_B.Rz_i[1] = Inow[0];
    raspberrypi_multicore_MPCtest_B.Rz_i[3] = Inow[4];
    raspberrypi_multicore_MPCtest_B.Rz_i[4] = Inow[3];
    raspberrypi_multicore_MPCtest_B.Rz_i[6] = Inow[7];
    raspberrypi_multicore_MPCtest_B.Rz_i[7] = Inow[6];
  } else if (raspberrypi_multicore_MPCtest_B.absx31_i >
             raspberrypi_multicore_MPCtest_B.absx11_c) {
    p1 = 7;
    p3 = 0;
    raspberrypi_multicore_MPCtest_B.Rz_i[0] = Inow[2];
    raspberrypi_multicore_MPCtest_B.Rz_i[2] = Inow[0];
    raspberrypi_multicore_MPCtest_B.Rz_i[3] = Inow[5];
    raspberrypi_multicore_MPCtest_B.Rz_i[5] = Inow[3];
    raspberrypi_multicore_MPCtest_B.Rz_i[6] = Inow[8];
    raspberrypi_multicore_MPCtest_B.Rz_i[8] = Inow[6];
  }

  raspberrypi_multicore_MPCtest_B.absx11_c =
    raspberrypi_multicore_MPCtest_B.Rz_i[1] /
    raspberrypi_multicore_MPCtest_B.Rz_i[0];
  raspberrypi_multicore_MPCtest_B.Rz_i[1] =
    raspberrypi_multicore_MPCtest_B.absx11_c;
  raspberrypi_multicore_MPCtest_B.absx21_n =
    raspberrypi_multicore_MPCtest_B.Rz_i[2] /
    raspberrypi_multicore_MPCtest_B.Rz_i[0];
  raspberrypi_multicore_MPCtest_B.Rz_i[2] =
    raspberrypi_multicore_MPCtest_B.absx21_n;
  raspberrypi_multicore_MPCtest_B.Rz_i[4] -=
    raspberrypi_multicore_MPCtest_B.absx11_c *
    raspberrypi_multicore_MPCtest_B.Rz_i[3];
  raspberrypi_multicore_MPCtest_B.Rz_i[5] -=
    raspberrypi_multicore_MPCtest_B.absx21_n *
    raspberrypi_multicore_MPCtest_B.Rz_i[3];
  raspberrypi_multicore_MPCtest_B.Rz_i[7] -=
    raspberrypi_multicore_MPCtest_B.absx11_c *
    raspberrypi_multicore_MPCtest_B.Rz_i[6];
  raspberrypi_multicore_MPCtest_B.Rz_i[8] -=
    raspberrypi_multicore_MPCtest_B.absx21_n *
    raspberrypi_multicore_MPCtest_B.Rz_i[6];
  if (fabs(raspberrypi_multicore_MPCtest_B.Rz_i[5]) > fabs
      (raspberrypi_multicore_MPCtest_B.Rz_i[4])) {
    itmp = p2;
    p2 = p3;
    p3 = itmp;
    raspberrypi_multicore_MPCtest_B.Rz_i[1] =
      raspberrypi_multicore_MPCtest_B.absx21_n;
    raspberrypi_multicore_MPCtest_B.Rz_i[2] =
      raspberrypi_multicore_MPCtest_B.absx11_c;
    raspberrypi_multicore_MPCtest_B.absx11_c =
      raspberrypi_multicore_MPCtest_B.Rz_i[4];
    raspberrypi_multicore_MPCtest_B.Rz_i[4] =
      raspberrypi_multicore_MPCtest_B.Rz_i[5];
    raspberrypi_multicore_MPCtest_B.Rz_i[5] =
      raspberrypi_multicore_MPCtest_B.absx11_c;
    raspberrypi_multicore_MPCtest_B.absx11_c =
      raspberrypi_multicore_MPCtest_B.Rz_i[7];
    raspberrypi_multicore_MPCtest_B.Rz_i[7] =
      raspberrypi_multicore_MPCtest_B.Rz_i[8];
    raspberrypi_multicore_MPCtest_B.Rz_i[8] =
      raspberrypi_multicore_MPCtest_B.absx11_c;
  }

  raspberrypi_multicore_MPCtest_B.absx11_c =
    raspberrypi_multicore_MPCtest_B.Rz_i[5] /
    raspberrypi_multicore_MPCtest_B.Rz_i[4];
  raspberrypi_multicore_MPCtest_B.Rz_i[8] -=
    raspberrypi_multicore_MPCtest_B.absx11_c *
    raspberrypi_multicore_MPCtest_B.Rz_i[7];
  raspberrypi_multicore_MPCtest_B.absx21_n =
    (raspberrypi_multicore_MPCtest_B.Rz_i[1] *
     raspberrypi_multicore_MPCtest_B.absx11_c -
     raspberrypi_multicore_MPCtest_B.Rz_i[2]) /
    raspberrypi_multicore_MPCtest_B.Rz_i[8];
  raspberrypi_multicore_MPCtest_B.absx31_i =
    -(raspberrypi_multicore_MPCtest_B.Rz_i[7] *
      raspberrypi_multicore_MPCtest_B.absx21_n +
      raspberrypi_multicore_MPCtest_B.Rz_i[1]) /
    raspberrypi_multicore_MPCtest_B.Rz_i[4];
  raspberrypi_multicore_MPCtest_B.Iinv[p1 - 1] = ((1.0 -
    raspberrypi_multicore_MPCtest_B.Rz_i[3] *
    raspberrypi_multicore_MPCtest_B.absx31_i) -
    raspberrypi_multicore_MPCtest_B.Rz_i[6] *
    raspberrypi_multicore_MPCtest_B.absx21_n) /
    raspberrypi_multicore_MPCtest_B.Rz_i[0];
  raspberrypi_multicore_MPCtest_B.Iinv[p1] =
    raspberrypi_multicore_MPCtest_B.absx31_i;
  raspberrypi_multicore_MPCtest_B.Iinv[p1 + 1] =
    raspberrypi_multicore_MPCtest_B.absx21_n;
  raspberrypi_multicore_MPCtest_B.absx21_n =
    -raspberrypi_multicore_MPCtest_B.absx11_c /
    raspberrypi_multicore_MPCtest_B.Rz_i[8];
  raspberrypi_multicore_MPCtest_B.absx31_i = (1.0 -
    raspberrypi_multicore_MPCtest_B.Rz_i[7] *
    raspberrypi_multicore_MPCtest_B.absx21_n) /
    raspberrypi_multicore_MPCtest_B.Rz_i[4];
  raspberrypi_multicore_MPCtest_B.Iinv[p2] =
    -(raspberrypi_multicore_MPCtest_B.Rz_i[3] *
      raspberrypi_multicore_MPCtest_B.absx31_i +
      raspberrypi_multicore_MPCtest_B.Rz_i[6] *
      raspberrypi_multicore_MPCtest_B.absx21_n) /
    raspberrypi_multicore_MPCtest_B.Rz_i[0];
  raspberrypi_multicore_MPCtest_B.Iinv[p2 + 1] =
    raspberrypi_multicore_MPCtest_B.absx31_i;
  raspberrypi_multicore_MPCtest_B.Iinv[p2 + 2] =
    raspberrypi_multicore_MPCtest_B.absx21_n;
  raspberrypi_multicore_MPCtest_B.absx21_n = 1.0 /
    raspberrypi_multicore_MPCtest_B.Rz_i[8];
  raspberrypi_multicore_MPCtest_B.absx31_i =
    -raspberrypi_multicore_MPCtest_B.Rz_i[7] *
    raspberrypi_multicore_MPCtest_B.absx21_n /
    raspberrypi_multicore_MPCtest_B.Rz_i[4];
  raspberrypi_multicore_MPCtest_B.Iinv[p3] =
    -(raspberrypi_multicore_MPCtest_B.Rz_i[3] *
      raspberrypi_multicore_MPCtest_B.absx31_i +
      raspberrypi_multicore_MPCtest_B.Rz_i[6] *
      raspberrypi_multicore_MPCtest_B.absx21_n) /
    raspberrypi_multicore_MPCtest_B.Rz_i[0];
  raspberrypi_multicore_MPCtest_B.Iinv[p3 + 1] =
    raspberrypi_multicore_MPCtest_B.absx31_i;
  raspberrypi_multicore_MPCtest_B.Iinv[p3 + 2] =
    raspberrypi_multicore_MPCtest_B.absx21_n;
  raspberrypi_multicore_MPCtest_B.PendAll[0] = PendAll[0] - xFB[0];
  raspberrypi_multicore_MPCtest_B.PendAll_o[0] = PendAll[3] - xFB[0];
  raspberrypi_multicore_MPCtest_B.PendAll_h[0] = PendAll[6] - xFB[0];
  raspberrypi_multicore_MPCtest_B.PendAll_l[0] = PendAll[9] - xFB[0];
  raspberrypi_multicore_MPCtest_B.PendAll[1] = PendAll[1] - xFB[1];
  raspberrypi_multicore_MPCtest_B.PendAll_o[1] = PendAll[4] - xFB[1];
  raspberrypi_multicore_MPCtest_B.PendAll_h[1] = PendAll[7] - xFB[1];
  raspberrypi_multicore_MPCtest_B.PendAll_l[1] = PendAll[10] - xFB[1];
  raspberrypi_multicore_MPCtest_B.PendAll[2] = PendAll[2] - xFB[2];
  raspberrypi_multicore_MPCtest_B.PendAll_o[2] = PendAll[5] - xFB[2];
  raspberrypi_multicore_MPCtest_B.PendAll_h[2] = PendAll[8] - xFB[2];
  raspberrypi_multicore_MPCtest_B.PendAll_l[2] = PendAll[11] - xFB[2];
  raspberrypi_multicore_MP_DS_gen(obj->Ts, obj->m, &xFB[3],
    raspberrypi_multicore_MPCtest_B.Iinv,
    raspberrypi_multicore_MPCtest_B.PendAll,
    raspberrypi_multicore_MPCtest_B.PendAll_o,
    raspberrypi_multicore_MPCtest_B.PendAll_h,
    raspberrypi_multicore_MPCtest_B.PendAll_l, Anew,
    raspberrypi_multicore_MPCtest_B.B_n);
  for (p1 = 0; p1 < 12; p1++) {
    memcpy(&Bnew[p1 * 13], &raspberrypi_multicore_MPCtest_B.B_n[p1 * 13], 13U *
           sizeof(real_T));
  }

  for (p1 = 0; p1 < 6; p1++) {
    for (p2 = 0; p2 < 6; p2++) {
      p3 = tmp[6 * p1 + p2];
      itmp = p2 + 13 * (p1 + 12);
      Bnew[itmp] = obj->Ts * (real_T)p3;
      Bnew[itmp + 6] = p3;
    }

    Bnew[13 * (p1 + 12) + 12] = 0.0;
  }

  for (p1 = 0; p1 < 12; p1++) {
    raspberrypi_multicore_MPCtest_B.Unow[p1] = 0;
  }

  if (SPLeg[0] > 0.5) {
    raspberrypi_multicore_MPCtest_B.Unow[2] = 1;
  }

  if (SPLeg[1] > 0.5) {
    raspberrypi_multicore_MPCtest_B.Unow[5] = 1;
  }

  if (SPLeg[2] > 0.5) {
    raspberrypi_multicore_MPCtest_B.Unow[8] = 1;
  }

  if (SPLeg[3] > 0.5) {
    raspberrypi_multicore_MPCtest_B.Unow[11] = 1;
  }

  raspberrypi_multicore_MPCtest_B.absx11_c = ((SPLeg[0] + SPLeg[1]) + SPLeg[2])
    + SPLeg[3];
  for (p1 = 0; p1 < 12; p1++) {
    U[p1] = (real_T)raspberrypi_multicore_MPCtest_B.Unow[p1] * obj->m * 9.8 /
      raspberrypi_multicore_MPCtest_B.absx11_c;
  }

  for (p1 = 0; p1 < 6; p1++) {
    U[p1 + 12] = 0.0;
  }

  for (p1 = 0; p1 < 13; p1++) {
    raspberrypi_multicore_MPCtest_B.dv18[p1] = 0.0;
    for (p2 = 0; p2 < 13; p2++) {
      raspberrypi_multicore_MPCtest_B.dv18[p1] += (real_T)tmp_0[13 * p2 + p1] *
        xFB[p2];
    }

    raspberrypi_multicore_MPCtest_B.dv19[p1] = 0.0;
  }

  for (p1 = 0; p1 < 18; p1++) {
    for (p2 = 0; p2 < 13; p2++) {
      raspberrypi_multicore_MPCtest_B.dv19[p2] += 0.0 * U[p1];
    }
  }

  for (p1 = 0; p1 < 13; p1++) {
    Y[p1] = raspberrypi_multicore_MPCtest_B.dv18[p1] +
      raspberrypi_multicore_MPCtest_B.dv19[p1];
    raspberrypi_multicore_MPCtest_B.Anew_p[p1] = 0.0;
    for (p2 = 0; p2 < 13; p2++) {
      raspberrypi_multicore_MPCtest_B.Anew_p[p1] += Anew[13 * p2 + p1] * xFB[p2];
    }

    raspberrypi_multicore_MPCtest_B.Bnew[p1] = 0.0;
  }

  for (p1 = 0; p1 < 18; p1++) {
    for (p2 = 0; p2 < 13; p2++) {
      raspberrypi_multicore_MPCtest_B.Bnew[p2] += Bnew[13 * p1 + p2] * U[p1];
    }
  }

  for (p1 = 0; p1 < 13; p1++) {
    DX[p1] = (raspberrypi_multicore_MPCtest_B.Anew_p[p1] +
              raspberrypi_multicore_MPCtest_B.Bnew[p1]) - xFB[p1];
  }
}

static void raspberrypi_m_SystemCore_step_a(const
  ssModelgen_estDis_raspberrypi_T *obj, const real_T varargin_1[12], const
  real_T varargin_2[13], const real_T varargin_3[4], real_T varargout_1[169],
  real_T varargout_2[234], real_T varargout_5[13], real_T varargout_6[18],
  real_T varargout_7[13], real_T varargout_8[13], real_T varargout_9[9])
{
  memcpy(&varargout_5[0], &varargin_2[0], 13U * sizeof(real_T));
  ra_ssModelgen_estDis_stepImpl_a(obj, varargin_1, varargout_5, varargin_3,
    varargout_1, varargout_2, varargout_6, varargout_7, varargout_8, varargout_9);
}

/* Function for MATLAB Function: '<S46>/FixedHorizonOptimizer' */
static void raspberrypi_mul_mpc_plantupdate(const real_T a[169], real_T b[234],
  real_T c[169], real_T d[234], real_T b_A[169], real_T b_B[416], real_T b_C[169],
  real_T b_D[416], const real_T b_mvindex[12], const real_T b_mdindex[6], const
  real_T b_myindex[13], const real_T b_Uscale[18], const real_T b_Yscale[13],
  real_T Bu[156], real_T Bv[91], real_T Cm[169], real_T Dv[91], real_T Dvm[91],
  real_T QQ[169], real_T RR[169], real_T NN[169])
{
  real_T b_mdindex_0;
  int32_T b_B_tmp;
  int32_T b_tmp;
  int32_T i;
  int32_T i_0;
  int8_T UnknownIn[31];
  for (i_0 = 0; i_0 < 18; i_0++) {
    for (i = 0; i < 13; i++) {
      b_tmp = 13 * i_0 + i;
      b[b_tmp] *= b_Uscale[i_0];
    }
  }

  for (i_0 = 0; i_0 < 13; i_0++) {
    for (i = 0; i < 13; i++) {
      b_tmp = 13 * i_0 + i;
      c[b_tmp] /= b_Yscale[i];
    }
  }

  for (i_0 = 0; i_0 < 18; i_0++) {
    for (i = 0; i < 13; i++) {
      b_tmp = 13 * i_0 + i;
      d[b_tmp] = d[b_tmp] / b_Yscale[i] * b_Uscale[i_0];
    }
  }

  memcpy(&b_A[0], &a[0], 169U * sizeof(real_T));
  for (i_0 = 0; i_0 < 12; i_0++) {
    for (i = 0; i < 13; i++) {
      b_B[i + 13 * i_0] = b[((int32_T)b_mvindex[i_0] - 1) * 13 + i];
    }
  }

  memcpy(&b_C[0], &c[0], 169U * sizeof(real_T));
  for (i_0 = 0; i_0 < 6; i_0++) {
    b_mdindex_0 = b_mdindex[i_0];
    for (i = 0; i < 13; i++) {
      b_tmp = ((int32_T)b_mdindex_0 - 1) * 13 + i;
      b_B_tmp = i + 13 * (i_0 + 12);
      b_B[b_B_tmp] = b[b_tmp];
      b_D[b_B_tmp] = d[b_tmp];
    }
  }

  for (i_0 = 0; i_0 < 12; i_0++) {
    memcpy(&Bu[i_0 * 13], &b_B[i_0 * 13], 13U * sizeof(real_T));
  }

  for (i_0 = 0; i_0 < 7; i_0++) {
    memcpy(&Bv[i_0 * 13], &b_B[i_0 * 13 + 156], 13U * sizeof(real_T));
  }

  for (i_0 = 0; i_0 < 13; i_0++) {
    for (i = 0; i < 13; i++) {
      Cm[i + 13 * i_0] = c[(13 * i_0 + (int32_T)b_myindex[i]) - 1];
    }
  }

  for (i_0 = 0; i_0 < 7; i_0++) {
    for (i = 0; i < 13; i++) {
      b_tmp = (i_0 + 12) * 13;
      b_B_tmp = i + 13 * i_0;
      Dv[b_B_tmp] = b_D[b_tmp + i];
      Dvm[b_B_tmp] = b_D[(b_tmp + (int32_T)b_myindex[i]) - 1];
    }
  }

  for (i_0 = 0; i_0 < 31; i_0++) {
    UnknownIn[i_0] = 0;
  }

  for (i_0 = 0; i_0 < 18; i_0++) {
    UnknownIn[i_0] = (int8_T)(i_0 + 1);
  }

  for (i_0 = 0; i_0 < 13; i_0++) {
    UnknownIn[i_0 + 18] = (int8_T)(i_0 + 20);
  }

  for (i_0 = 0; i_0 < 31; i_0++) {
    for (i = 0; i < 13; i++) {
      b_tmp = (UnknownIn[i_0] - 1) * 13;
      b_B_tmp = i + 26 * i_0;
      raspberrypi_multicore_MPCtest_B.b_B[b_B_tmp] = b_B[b_tmp + i];
      raspberrypi_multicore_MPCtest_B.b_B[b_B_tmp + 13] = b_D[(b_tmp + (int32_T)
        b_myindex[i]) - 1];
    }
  }

  for (i_0 = 0; i_0 < 13; i_0++) {
    for (i = 0; i < 31; i++) {
      b_tmp = (UnknownIn[i] - 1) * 13;
      raspberrypi_multicore_MPCtest_B.b_B_f[i + 31 * i_0] = b_B[b_tmp + i_0];
      raspberrypi_multicore_MPCtest_B.b_B_f[i + 31 * (i_0 + 13)] = b_D[(b_tmp +
        (int32_T)b_myindex[i_0]) - 1];
    }
  }

  for (i_0 = 0; i_0 < 26; i_0++) {
    memset(&raspberrypi_multicore_MPCtest_B.CovMat[i_0 * 26], 0, 26U * sizeof
           (real_T));
    for (b_tmp = 0; b_tmp < 31; b_tmp++) {
      for (i = 0; i < 26; i++) {
        b_B_tmp = 26 * i_0 + i;
        raspberrypi_multicore_MPCtest_B.CovMat[b_B_tmp] +=
          raspberrypi_multicore_MPCtest_B.b_B[26 * b_tmp + i] *
          raspberrypi_multicore_MPCtest_B.b_B_f[31 * i_0 + b_tmp];
      }
    }
  }

  for (i_0 = 0; i_0 < 13; i_0++) {
    memcpy(&QQ[i_0 * 13], &raspberrypi_multicore_MPCtest_B.CovMat[i_0 * 26], 13U
           * sizeof(real_T));
    memcpy(&RR[i_0 * 13], &raspberrypi_multicore_MPCtest_B.CovMat[i_0 * 26 + 351],
           13U * sizeof(real_T));
    memcpy(&NN[i_0 * 13], &raspberrypi_multicore_MPCtest_B.CovMat[i_0 * 26 + 338],
           13U * sizeof(real_T));
  }
}

/* Function for MATLAB Function: '<S46>/FixedHorizonOptimizer' */
static real_T raspberrypi_multicore_MPC_mod_p(real_T x)
{
  real_T r;
  if (rtIsNaN(x) || rtIsInf(x)) {
    r = (rtNaN);
  } else if (x == 0.0) {
    r = 0.0;
  } else {
    r = fmod(x, raspberrypi_multicore_MPCtes_nu);
    if (r == 0.0) {
      r = 0.0;
    } else if (x < 0.0) {
      r += raspberrypi_multicore_MPCtes_nu;
    }
  }

  return r;
}

/* Function for MATLAB Function: '<S46>/FixedHorizonOptimizer' */
static real_T raspberrypi_multicore_MPCte_mod(real_T x)
{
  real_T r;
  if (rtIsNaN(x) || rtIsInf(x)) {
    r = (rtNaN);
  } else if (x == 0.0) {
    r = 0.0;
  } else {
    r = fmod(x, raspberrypi_multicore_MPCtes_ny);
    if (r == 0.0) {
      r = 0.0;
    } else if (x < 0.0) {
      r += raspberrypi_multicore_MPCtes_ny;
    }
  }

  return r;
}

/* Function for MATLAB Function: '<S46>/FixedHorizonOptimizer' */
static void raspberry_mpc_updateFromNominal(real_T b_Mlim[252], const real_T
  b_Mrows[28], const real_T U0[18], const real_T b_Uscale[18], const real_T
  old_mvoff[12], const real_T b_mvindex[12], const real_T b_mdindex[6], real_T
  b_utarget[72], const real_T Y0[13], const real_T b_Yscale[13], const real_T
  old_yoff[13], const real_T b_myindex[13], const real_T X0[13], real_T b_xoff
  [13], const real_T DX0[13], real_T Bv[637], real_T new_mvoff[12], real_T
  new_mdoff[6], real_T new_yoff[13], real_T new_myoff[13])
{
  real_T U;
  real_T k;
  int32_T b_utarget_tmp;
  int32_T i;
  int32_T i_0;
  for (i = 0; i < 18; i++) {
    raspberrypi_multicore_MPCtest_B.U_h[i] = U0[i] / b_Uscale[i];
  }

  for (i = 0; i < 13; i++) {
    new_yoff[i] = Y0[i] / b_Yscale[i];
  }

  for (i = 0; i < 12; i++) {
    new_mvoff[i] = raspberrypi_multicore_MPCtest_B.U_h[(int32_T)b_mvindex[i] - 1];
  }

  for (i = 0; i < 6; i++) {
    new_mdoff[i] = raspberrypi_multicore_MPCtest_B.U_h[(int32_T)b_mdindex[i] - 1];
  }

  for (i = 0; i < 13; i++) {
    new_myoff[i] = new_yoff[(int32_T)b_myindex[i] - 1];
  }

  for (i = 0; i < 28; i++) {
    k = b_Mrows[i];
    if (k <= 78.0) {
      k = raspberrypi_multicore_MPCte_mod(k - 1.0) + 1.0;
      b_Mlim[i] += old_yoff[(int32_T)k - 1] - new_yoff[(int32_T)k - 1];
    } else if (k <= 156.0) {
      k = raspberrypi_multicore_MPCte_mod((k - 78.0) - 1.0) + 1.0;
      b_Mlim[i] -= old_yoff[(int32_T)k - 1] - new_yoff[(int32_T)k - 1];
    } else if (k <= 228.0) {
      k = raspberrypi_multicore_MPC_mod_p((k - 156.0) - 1.0) + 1.0;
      b_Mlim[i] += old_mvoff[(int32_T)k - 1] -
        raspberrypi_multicore_MPCtest_B.U_h[(int32_T)b_mvindex[(int32_T)k - 1] -
        1];
    } else if (k <= 300.0) {
      k = raspberrypi_multicore_MPC_mod_p(((k - 156.0) - 72.0) - 1.0) + 1.0;
      b_Mlim[i] -= old_mvoff[(int32_T)k - 1] -
        raspberrypi_multicore_MPCtest_B.U_h[(int32_T)b_mvindex[(int32_T)k - 1] -
        1];
    }
  }

  for (i = 0; i < 12; i++) {
    k = old_mvoff[i];
    U = raspberrypi_multicore_MPCtest_B.U_h[(int32_T)b_mvindex[i] - 1];
    for (i_0 = 0; i_0 < 6; i_0++) {
      b_utarget_tmp = 12 * i_0 + i;
      b_utarget[b_utarget_tmp] = (b_utarget[b_utarget_tmp] + k) - U;
    }
  }

  memcpy(&b_xoff[0], &X0[0], 13U * sizeof(real_T));
  memcpy(&Bv[78], &DX0[0], 13U * sizeof(real_T));
}

/* Function for MATLAB Function: '<S46>/FixedHorizonOptimizer' */
static void raspberrypi__mpc_constraintcoef(const real_T b_A[169], const real_T
  Bu[156], const real_T Bv[91], const real_T b_C[169], const real_T Dv[91],
  const real_T b_Jm[1728], real_T b_SuJm[1872], real_T b_Sx[1014], real_T b_Su1
  [936], real_T b_Hv[3822])
{
  real_T Sum;
  int32_T CA_tmp;
  int32_T b_C_tmp;
  int32_T b_Hv_tmp;
  int32_T i_0;
  int32_T i_1;
  int8_T rows[13];
  int8_T i;
  for (i_1 = 0; i_1 < 13; i_1++) {
    memset(&raspberrypi_multicore_MPCtest_B.CA_d[i_1 * 13], 0, 13U * sizeof
           (real_T));
    for (b_Hv_tmp = 0; b_Hv_tmp < 13; b_Hv_tmp++) {
      for (i_0 = 0; i_0 < 13; i_0++) {
        CA_tmp = 13 * i_1 + i_0;
        raspberrypi_multicore_MPCtest_B.CA_d[CA_tmp] += b_C[13 * b_Hv_tmp + i_0]
          * b_A[13 * i_1 + b_Hv_tmp];
      }
    }
  }

  for (i_1 = 0; i_1 < 12; i_1++) {
    memset(&raspberrypi_multicore_MPCtest_B.Sum_b[i_1 * 13], 0, 13U * sizeof
           (real_T));
    for (b_Hv_tmp = 0; b_Hv_tmp < 13; b_Hv_tmp++) {
      for (i_0 = 0; i_0 < 13; i_0++) {
        CA_tmp = 13 * i_1 + i_0;
        raspberrypi_multicore_MPCtest_B.Sum_b[CA_tmp] += b_C[13 * b_Hv_tmp + i_0]
          * Bu[13 * i_1 + b_Hv_tmp];
      }
    }
  }

  for (i_1 = 0; i_1 < 7; i_1++) {
    memset(&raspberrypi_multicore_MPCtest_B.b_C_h[i_1 * 13], 0, 13U * sizeof
           (real_T));
    for (b_Hv_tmp = 0; b_Hv_tmp < 13; b_Hv_tmp++) {
      for (i_0 = 0; i_0 < 13; i_0++) {
        b_C_tmp = 13 * i_1 + i_0;
        raspberrypi_multicore_MPCtest_B.b_C_h[b_C_tmp] += b_C[13 * b_Hv_tmp +
          i_0] * Bv[13 * i_1 + b_Hv_tmp];
      }
    }

    for (i_0 = 0; i_0 < 13; i_0++) {
      b_Hv_tmp = 13 * i_1 + i_0;
      b_Hv[i_0 + 78 * i_1] = raspberrypi_multicore_MPCtest_B.b_C_h[b_Hv_tmp];
      b_Hv[i_0 + 78 * (i_1 + 7)] = Dv[b_Hv_tmp];
    }
  }

  for (i_1 = 0; i_1 < 35; i_1++) {
    memset(&b_Hv[i_1 * 78 + 1092], 0, 13U * sizeof(real_T));
  }

  for (i_1 = 0; i_1 < 49; i_1++) {
    memset(&b_Hv[i_1 * 78 + 13], 0, 65U * sizeof(real_T));
  }

  for (i_1 = 0; i_1 < 13; i_1++) {
    memcpy(&b_Sx[i_1 * 78], &raspberrypi_multicore_MPCtest_B.CA_d[i_1 * 13], 13U
           * sizeof(real_T));
    memset(&b_Sx[i_1 * 78 + 13], 0, 65U * sizeof(real_T));
  }

  for (i_1 = 0; i_1 < 12; i_1++) {
    memcpy(&b_Su1[i_1 * 78], &raspberrypi_multicore_MPCtest_B.Sum_b[i_1 * 13],
           13U * sizeof(real_T));
    memset(&b_Su1[i_1 * 78 + 13], 0, 65U * sizeof(real_T));
    memcpy(&raspberrypi_multicore_MPCtest_B.Su[i_1 * 78],
           &raspberrypi_multicore_MPCtest_B.Sum_b[i_1 * 13], 13U * sizeof(real_T));
  }

  for (i_1 = 0; i_1 < 60; i_1++) {
    memset(&raspberrypi_multicore_MPCtest_B.Su[i_1 * 78 + 936], 0, 13U * sizeof
           (real_T));
  }

  for (i_1 = 0; i_1 < 72; i_1++) {
    memset(&raspberrypi_multicore_MPCtest_B.Su[i_1 * 78 + 13], 0, 65U * sizeof
           (real_T));
  }

  for (b_C_tmp = 0; b_C_tmp < 5; b_C_tmp++) {
    i = (int8_T)((b_C_tmp + 1) * 13 + 1);
    for (i_1 = 0; i_1 < 13; i_1++) {
      rows[i_1] = (int8_T)(i_1 + i);
      for (i_0 = 0; i_0 < 12; i_0++) {
        Sum = 0.0;
        for (b_Hv_tmp = 0; b_Hv_tmp < 13; b_Hv_tmp++) {
          Sum += raspberrypi_multicore_MPCtest_B.CA_d[13 * b_Hv_tmp + i_1] * Bu
            [13 * i_0 + b_Hv_tmp];
        }

        CA_tmp = 13 * i_0 + i_1;
        raspberrypi_multicore_MPCtest_B.Sum_b[CA_tmp] += Sum;
      }
    }

    for (i_1 = 0; i_1 < 12; i_1++) {
      for (i_0 = 0; i_0 < 13; i_0++) {
        CA_tmp = 13 * i_1 + i_0;
        Sum = raspberrypi_multicore_MPCtest_B.Sum_b[CA_tmp];
        b_Su1[(rows[i_0] + 78 * i_1) - 1] = Sum;
        raspberrypi_multicore_MPCtest_B.Sum[CA_tmp] = Sum;
      }
    }

    for (i_1 = 0; i_1 < 60; i_1++) {
      for (i_0 = 0; i_0 < 13; i_0++) {
        raspberrypi_multicore_MPCtest_B.Sum[i_0 + 13 * (i_1 + 12)] =
          raspberrypi_multicore_MPCtest_B.Su[(78 * i_1 + rows[i_0]) - 14];
      }
    }

    for (i_1 = 0; i_1 < 72; i_1++) {
      for (i_0 = 0; i_0 < 13; i_0++) {
        raspberrypi_multicore_MPCtest_B.Su[(rows[i_0] + 78 * i_1) - 1] =
          raspberrypi_multicore_MPCtest_B.Sum[13 * i_1 + i_0];
      }
    }

    for (i_1 = 0; i_1 < 13; i_1++) {
      for (i_0 = 0; i_0 < 7; i_0++) {
        CA_tmp = i_1 + 13 * i_0;
        raspberrypi_multicore_MPCtest_B.b_C_h[CA_tmp] = 0.0;
        for (b_Hv_tmp = 0; b_Hv_tmp < 13; b_Hv_tmp++) {
          raspberrypi_multicore_MPCtest_B.b_C_h[CA_tmp] +=
            raspberrypi_multicore_MPCtest_B.CA_d[13 * b_Hv_tmp + i_1] * Bv[13 *
            i_0 + b_Hv_tmp];
        }
      }
    }

    for (i_1 = 0; i_1 < 7; i_1++) {
      memcpy(&raspberrypi_multicore_MPCtest_B.CA[i_1 * 13],
             &raspberrypi_multicore_MPCtest_B.b_C_h[i_1 * 13], 13U * sizeof
             (real_T));
    }

    for (i_1 = 0; i_1 < 42; i_1++) {
      for (i_0 = 0; i_0 < 13; i_0++) {
        raspberrypi_multicore_MPCtest_B.CA[i_0 + 13 * (i_1 + 7)] = b_Hv[(78 *
          i_1 + rows[i_0]) - 14];
      }
    }

    for (i_1 = 0; i_1 < 49; i_1++) {
      for (i_0 = 0; i_0 < 13; i_0++) {
        b_Hv[(rows[i_0] + 78 * i_1) - 1] = raspberrypi_multicore_MPCtest_B.CA[13
          * i_1 + i_0];
      }
    }

    for (i_1 = 0; i_1 < 13; i_1++) {
      for (i_0 = 0; i_0 < 13; i_0++) {
        CA_tmp = i_1 + 13 * i_0;
        raspberrypi_multicore_MPCtest_B.CA_l[CA_tmp] = 0.0;
        for (b_Hv_tmp = 0; b_Hv_tmp < 13; b_Hv_tmp++) {
          raspberrypi_multicore_MPCtest_B.CA_l[CA_tmp] +=
            raspberrypi_multicore_MPCtest_B.CA_d[13 * b_Hv_tmp + i_1] * b_A[13 *
            i_0 + b_Hv_tmp];
        }
      }
    }

    memcpy(&raspberrypi_multicore_MPCtest_B.CA_d[0],
           &raspberrypi_multicore_MPCtest_B.CA_l[0], 169U * sizeof(real_T));
    for (i_1 = 0; i_1 < 13; i_1++) {
      for (i_0 = 0; i_0 < 13; i_0++) {
        b_Sx[(rows[i_0] + 78 * i_1) - 1] = raspberrypi_multicore_MPCtest_B.CA_d
          [13 * i_1 + i_0];
      }
    }
  }

  for (i_1 = 0; i_1 < 24; i_1++) {
    memset(&b_SuJm[i_1 * 78], 0, 78U * sizeof(real_T));
    for (b_Hv_tmp = 0; b_Hv_tmp < 72; b_Hv_tmp++) {
      for (i_0 = 0; i_0 < 78; i_0++) {
        b_C_tmp = 78 * i_1 + i_0;
        b_SuJm[b_C_tmp] += raspberrypi_multicore_MPCtest_B.Su[78 * b_Hv_tmp +
          i_0] * b_Jm[72 * i_1 + b_Hv_tmp];
      }
    }
  }
}

/* Function for MATLAB Function: '<S46>/FixedHorizonOptimizer' */
static void raspbe_mpc_customconstraintcoef(const real_T b_SuJm[1872], const
  real_T b_Sx[1014], const real_T b_Su1[936], const real_T b_Hv[3822], const
  real_T b_C[169], const real_T Dv[637], const real_T b_Jm[1728], const real_T
  E[384], const real_T F[416], const real_T S[192], const real_T G[32], const
  real_T mvoff[12], const real_T mdoff[6], const real_T b_yoff[13], real_T Mu
  [5376], real_T b_Mv[10976], real_T b_Mu1[2688], real_T b_Mx[2912], real_T
  b_Mlim[224])
{
  real_T tmp;
  int32_T b_i2;
  int32_T i1;
  int32_T i2;
  int32_T j2;
  int32_T kidx;
  static const int8_T b_A[49] = { 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 0,
    1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1,
    0, 0, 0, 0, 0, 0, 1 };

  memset(&raspberrypi_multicore_MPCtest_B.b_I_j[0], 0, 144U * sizeof(int8_T));
  for (kidx = 0; kidx < 12; kidx++) {
    raspberrypi_multicore_MPCtest_B.b_I_j[kidx + 12 * kidx] = 1;
  }

  kidx = -1;
  for (j2 = 0; j2 < 12; j2++) {
    for (i1 = 0; i1 < 7; i1++) {
      for (i2 = 0; i2 < 12; i2++) {
        raspberrypi_multicore_MPCtest_B.b_I1_b[(kidx + i2) + 1] =
          raspberrypi_multicore_MPCtest_B.b_I_j[12 * j2 + i2];
      }

      kidx += 12;
    }
  }

  memset(&raspberrypi_multicore_MPCtest_B.b_I_j[0], 0, 144U * sizeof(int8_T));
  for (kidx = 0; kidx < 12; kidx++) {
    raspberrypi_multicore_MPCtest_B.b_I_j[kidx + 12 * kidx] = 1;
  }

  kidx = -1;
  for (j2 = 0; j2 < 7; j2++) {
    for (i1 = 0; i1 < 12; i1++) {
      for (i2 = 0; i2 < 7; i2++) {
        for (b_i2 = 0; b_i2 < 12; b_i2++) {
          raspberrypi_multicore_MPCtest_B.I2_c[(kidx + b_i2) + 1] = (int8_T)
            (b_A[7 * j2 + i2] * raspberrypi_multicore_MPCtest_B.b_I_j[12 * i1 +
             b_i2]);
        }

        kidx += 12;
      }
    }
  }

  for (kidx = 0; kidx < 49; kidx++) {
    raspberrypi_multicore_MPCtest_B.c_I_g[kidx] = 0;
  }

  for (kidx = 0; kidx < 7; kidx++) {
    raspberrypi_multicore_MPCtest_B.c_I_g[kidx + 7 * kidx] = 1;
  }

  kidx = -1;
  for (j2 = 0; j2 < 7; j2++) {
    for (i1 = 0; i1 < 12; i1++) {
      for (i2 = 0; i2 < 7; i2++) {
        for (b_i2 = 0; b_i2 < 32; b_i2++) {
          raspberrypi_multicore_MPCtest_B.Ep1[(kidx + b_i2) + 1] = (real_T)
            raspberrypi_multicore_MPCtest_B.c_I_g[7 * j2 + i2] * E[(i1 << 5) +
            b_i2];
        }

        kidx += 32;
      }
    }
  }

  for (kidx = 0; kidx < 49; kidx++) {
    raspberrypi_multicore_MPCtest_B.c_I_g[kidx] = 0;
  }

  for (kidx = 0; kidx < 7; kidx++) {
    raspberrypi_multicore_MPCtest_B.c_I_g[kidx + 7 * kidx] = 1;
  }

  kidx = -1;
  for (j2 = 0; j2 < 7; j2++) {
    for (i1 = 0; i1 < 13; i1++) {
      for (i2 = 0; i2 < 7; i2++) {
        for (b_i2 = 0; b_i2 < 32; b_i2++) {
          raspberrypi_multicore_MPCtest_B.Fp1[(kidx + b_i2) + 1] = (real_T)
            raspberrypi_multicore_MPCtest_B.c_I_g[7 * j2 + i2] * F[(i1 << 5) +
            b_i2];
        }

        kidx += 32;
      }
    }
  }

  for (kidx = 0; kidx < 49; kidx++) {
    raspberrypi_multicore_MPCtest_B.c_I_g[kidx] = 0;
  }

  for (kidx = 0; kidx < 7; kidx++) {
    raspberrypi_multicore_MPCtest_B.c_I_g[kidx + 7 * kidx] = 1;
  }

  memcpy(&raspberrypi_multicore_MPCtest_B.b_B_g[0], &S[0], 192U * sizeof(real_T));
  memset(&raspberrypi_multicore_MPCtest_B.b_B_g[192], 0, sizeof(real_T) << 5U);
  kidx = -1;
  for (j2 = 0; j2 < 7; j2++) {
    for (i1 = 0; i1 < 7; i1++) {
      for (i2 = 0; i2 < 7; i2++) {
        for (b_i2 = 0; b_i2 < 32; b_i2++) {
          raspberrypi_multicore_MPCtest_B.Sp1[(kidx + b_i2) + 1] = (real_T)
            raspberrypi_multicore_MPCtest_B.c_I_g[7 * j2 + i2] *
            raspberrypi_multicore_MPCtest_B.b_B_g[(i1 << 5) + b_i2];
        }

        kidx += 32;
      }
    }
  }

  for (kidx = 0; kidx < 24; kidx++) {
    memcpy(&raspberrypi_multicore_MPCtest_B.b_Jm[kidx * 84], &b_Jm[kidx * 72],
           72U * sizeof(real_T));
    memset(&raspberrypi_multicore_MPCtest_B.b_Jm[kidx * 84 + 72], 0, 12U *
           sizeof(real_T));
    memset(&raspberrypi_multicore_MPCtest_B.I2[kidx * 84], 0, 84U * sizeof
           (real_T));
    for (i1 = 0; i1 < 84; i1++) {
      for (j2 = 0; j2 < 84; j2++) {
        i2 = 84 * kidx + j2;
        raspberrypi_multicore_MPCtest_B.I2[i2] += (real_T)
          raspberrypi_multicore_MPCtest_B.I2_c[84 * i1 + j2] *
          raspberrypi_multicore_MPCtest_B.b_Jm[84 * kidx + i1];
      }
    }

    memset(&raspberrypi_multicore_MPCtest_B.dv1[kidx * 91], 0, 13U * sizeof
           (real_T));
    memcpy(&raspberrypi_multicore_MPCtest_B.dv1[kidx * 91 + 13], &b_SuJm[kidx *
           78], 78U * sizeof(real_T));
    memset(&raspberrypi_multicore_MPCtest_B.Ep1_k[kidx * 224], 0, 224U * sizeof
           (real_T));
    for (i1 = 0; i1 < 84; i1++) {
      for (j2 = 0; j2 < 224; j2++) {
        i2 = 224 * kidx + j2;
        raspberrypi_multicore_MPCtest_B.Ep1_k[i2] +=
          raspberrypi_multicore_MPCtest_B.Ep1[224 * i1 + j2] *
          raspberrypi_multicore_MPCtest_B.I2[84 * kidx + i1];
      }
    }

    memset(&raspberrypi_multicore_MPCtest_B.Fp1_c[kidx * 224], 0, 224U * sizeof
           (real_T));
    for (i1 = 0; i1 < 91; i1++) {
      for (j2 = 0; j2 < 224; j2++) {
        i2 = 224 * kidx + j2;
        raspberrypi_multicore_MPCtest_B.Fp1_c[i2] +=
          raspberrypi_multicore_MPCtest_B.Fp1[224 * i1 + j2] *
          raspberrypi_multicore_MPCtest_B.dv1[91 * kidx + i1];
      }
    }
  }

  for (kidx = 0; kidx < 5376; kidx++) {
    Mu[kidx] = raspberrypi_multicore_MPCtest_B.Ep1_k[kidx] +
      raspberrypi_multicore_MPCtest_B.Fp1_c[kidx];
  }

  for (kidx = 0; kidx < 20384; kidx++) {
    raspberrypi_multicore_MPCtest_B.Fp1_m[kidx] =
      -raspberrypi_multicore_MPCtest_B.Fp1[kidx];
  }

  for (kidx = 0; kidx < 7; kidx++) {
    memcpy(&raspberrypi_multicore_MPCtest_B.Dv[kidx * 91], &Dv[kidx * 13], 13U *
           sizeof(real_T));
  }

  for (kidx = 0; kidx < 42; kidx++) {
    memset(&raspberrypi_multicore_MPCtest_B.Dv[kidx * 91 + 637], 0, 13U * sizeof
           (real_T));
  }

  for (kidx = 0; kidx < 49; kidx++) {
    memcpy(&raspberrypi_multicore_MPCtest_B.Dv[kidx * 91 + 13], &b_Hv[kidx * 78],
           78U * sizeof(real_T));
  }

  for (kidx = 0; kidx < 224; kidx++) {
    for (j2 = 0; j2 < 49; j2++) {
      tmp = 0.0;
      for (i1 = 0; i1 < 91; i1++) {
        tmp += raspberrypi_multicore_MPCtest_B.Fp1_m[224 * i1 + kidx] *
          raspberrypi_multicore_MPCtest_B.Dv[91 * j2 + i1];
      }

      i1 = 224 * j2 + kidx;
      b_Mv[i1] = tmp - raspberrypi_multicore_MPCtest_B.Sp1[i1];
    }
  }

  for (kidx = 0; kidx < 20384; kidx++) {
    raspberrypi_multicore_MPCtest_B.Fp1_m[kidx] =
      -raspberrypi_multicore_MPCtest_B.Fp1[kidx];
  }

  for (kidx = 0; kidx < 13; kidx++) {
    memcpy(&raspberrypi_multicore_MPCtest_B.b_C[kidx * 91], &b_C[kidx * 13], 13U
           * sizeof(real_T));
    memcpy(&raspberrypi_multicore_MPCtest_B.b_C[kidx * 91 + 13], &b_Sx[kidx * 78],
           78U * sizeof(real_T));
    memset(&b_Mx[kidx * 224], 0, 224U * sizeof(real_T));
    for (i1 = 0; i1 < 91; i1++) {
      for (j2 = 0; j2 < 224; j2++) {
        i2 = 224 * kidx + j2;
        b_Mx[i2] += raspberrypi_multicore_MPCtest_B.Fp1_m[224 * i1 + j2] *
          raspberrypi_multicore_MPCtest_B.b_C[91 * kidx + i1];
      }
    }
  }

  for (kidx = 0; kidx < 18816; kidx++) {
    raspberrypi_multicore_MPCtest_B.Ep1_c[kidx] =
      -raspberrypi_multicore_MPCtest_B.Ep1[kidx];
  }

  for (kidx = 0; kidx < 12; kidx++) {
    memset(&raspberrypi_multicore_MPCtest_B.dv5[kidx * 91], 0, 13U * sizeof
           (real_T));
    memcpy(&raspberrypi_multicore_MPCtest_B.dv5[kidx * 91 + 13], &b_Su1[kidx *
           78], 78U * sizeof(real_T));
    memset(&raspberrypi_multicore_MPCtest_B.Ep1_b[kidx * 224], 0, 224U * sizeof
           (real_T));
    for (i1 = 0; i1 < 84; i1++) {
      for (j2 = 0; j2 < 224; j2++) {
        i2 = 224 * kidx + j2;
        raspberrypi_multicore_MPCtest_B.Ep1_b[i2] +=
          raspberrypi_multicore_MPCtest_B.Ep1_c[224 * i1 + j2] * (real_T)
          raspberrypi_multicore_MPCtest_B.b_I1_b[84 * kidx + i1];
      }
    }

    memset(&raspberrypi_multicore_MPCtest_B.Fp1_p[kidx * 224], 0, 224U * sizeof
           (real_T));
    for (i1 = 0; i1 < 91; i1++) {
      for (j2 = 0; j2 < 224; j2++) {
        i2 = 224 * kidx + j2;
        raspberrypi_multicore_MPCtest_B.Fp1_p[i2] +=
          raspberrypi_multicore_MPCtest_B.Fp1[224 * i1 + j2] *
          raspberrypi_multicore_MPCtest_B.dv5[91 * kidx + i1];
      }
    }
  }

  for (kidx = 0; kidx < 2688; kidx++) {
    b_Mu1[kidx] = raspberrypi_multicore_MPCtest_B.Ep1_b[kidx] -
      raspberrypi_multicore_MPCtest_B.Fp1_p[kidx];
  }

  memset(&raspberrypi_multicore_MPCtest_B.E[0], 0, sizeof(real_T) << 5U);
  for (j2 = 0; j2 < 12; j2++) {
    for (kidx = 0; kidx < 32; kidx++) {
      raspberrypi_multicore_MPCtest_B.E[kidx] += E[(j2 << 5) + kidx] * mvoff[j2];
    }
  }

  memset(&raspberrypi_multicore_MPCtest_B.F[0], 0, sizeof(real_T) << 5U);
  for (j2 = 0; j2 < 13; j2++) {
    for (kidx = 0; kidx < 32; kidx++) {
      raspberrypi_multicore_MPCtest_B.F[kidx] += F[(j2 << 5) + kidx] * b_yoff[j2];
    }
  }

  for (kidx = 0; kidx < 32; kidx++) {
    tmp = 0.0;
    for (j2 = 0; j2 < 6; j2++) {
      tmp += S[(j2 << 5) + kidx] * mdoff[j2];
    }

    raspberrypi_multicore_MPCtest_B.E_n[kidx] =
      ((raspberrypi_multicore_MPCtest_B.E[kidx] +
        raspberrypi_multicore_MPCtest_B.F[kidx]) - G[kidx]) + tmp;
  }

  for (kidx = 0; kidx < 7; kidx++) {
    memcpy(&raspberrypi_multicore_MPCtest_B.b_B_g[kidx << 5],
           &raspberrypi_multicore_MPCtest_B.E_n[0], sizeof(real_T) << 5U);
  }

  for (kidx = 0; kidx < 224; kidx++) {
    b_Mlim[kidx] = -raspberrypi_multicore_MPCtest_B.b_B_g[kidx];
  }
}

/* Function for MATLAB Function: '<S46>/FixedHorizonOptimizer' */
static void raspberrypi_multicore_MPCt_kron(const real_T b_A[36], const real_T
  b_B[144], real_T K[5184])
{
  int32_T b_j1;
  int32_T i1;
  int32_T i2;
  int32_T j2;
  int32_T kidx;
  kidx = -1;
  for (b_j1 = 0; b_j1 < 6; b_j1++) {
    for (j2 = 0; j2 < 12; j2++) {
      for (i1 = 0; i1 < 6; i1++) {
        for (i2 = 0; i2 < 12; i2++) {
          K[(kidx + i2) + 1] = b_A[6 * b_j1 + i1] * b_B[12 * j2 + i2];
        }

        kidx += 12;
      }
    }
  }
}

/* Function for MATLAB Function: '<S46>/FixedHorizonOptimizer' */
static void raspberrypi_multicore_MP_WtMult(const real_T W[12], const real_T M
  [1728], real_T nwt, real_T WM[1728])
{
  real_T W_0;
  int32_T WM_tmp;
  int32_T i;
  int32_T i_0;
  int16_T ixw;
  ixw = 1;
  for (i = 0; i < 72; i++) {
    W_0 = W[ixw - 1];
    for (i_0 = 0; i_0 < 24; i_0++) {
      WM_tmp = 72 * i_0 + i;
      WM[WM_tmp] = M[WM_tmp] * W_0;
    }

    i_0 = ixw + 1;
    if (ixw + 1 > 32767) {
      i_0 = 32767;
    }

    ixw = (int16_T)i_0;
    if ((int16_T)i_0 > 12) {
      W_0 = rt_roundd_snf(12.0 - nwt);
      if (W_0 < 32768.0) {
        if (W_0 >= -32768.0) {
          ixw = (int16_T)W_0;
        } else {
          ixw = MIN_int16_T;
        }
      } else {
        ixw = MAX_int16_T;
      }

      i_0 = ixw + 1;
      if (W_0 < 32768.0) {
        if (W_0 >= -32768.0) {
          ixw = (int16_T)W_0;
        } else {
          ixw = MIN_int16_T;
        }
      } else {
        ixw = MAX_int16_T;
      }

      if (ixw + 1 > 32767) {
        i_0 = 32767;
      }

      ixw = (int16_T)i_0;
    }
  }
}

/* Function for MATLAB Function: '<S46>/FixedHorizonOptimizer' */
static void raspberryp_mpc_calculatehessian(const real_T b_Wy[78], const real_T
  b_Wu[12], const real_T b_Wdu[12], const real_T b_SuJm[1872], const real_T
  I2Jm[1728], const real_T b_Jm[1728], const real_T b_I1[864], const real_T
  b_Su1[936], const real_T b_Sx[1014], const real_T b_Hv[3822], real_T nmv,
  real_T b_ny, real_T b_H[576], real_T b_Ku1[288], real_T b_Kut[1728], real_T
  b_Kx[312], real_T b_Kv[1176], real_T b_Kr[1872])
{
  real_T b_Wy_0;
  int32_T b_Kr_tmp;
  int32_T b_SuJm_tmp;
  int32_T i;
  int32_T i_0;
  int16_T ixw;
  ixw = 1;
  for (i = 0; i < 78; i++) {
    b_Wy_0 = b_Wy[ixw - 1];
    for (i_0 = 0; i_0 < 24; i_0++) {
      b_Kr_tmp = 78 * i_0 + i;
      b_Kr[b_Kr_tmp] = b_SuJm[b_Kr_tmp] * b_Wy_0;
    }

    i_0 = ixw + 1;
    if (ixw + 1 > 32767) {
      i_0 = 32767;
    }

    ixw = (int16_T)i_0;
    if ((int16_T)i_0 > 78) {
      b_Wy_0 = rt_roundd_snf(78.0 - b_ny);
      if (b_Wy_0 < 32768.0) {
        if (b_Wy_0 >= -32768.0) {
          ixw = (int16_T)b_Wy_0;
        } else {
          ixw = MIN_int16_T;
        }
      } else {
        ixw = MAX_int16_T;
      }

      i_0 = ixw + 1;
      if (b_Wy_0 < 32768.0) {
        if (b_Wy_0 >= -32768.0) {
          ixw = (int16_T)b_Wy_0;
        } else {
          ixw = MIN_int16_T;
        }
      } else {
        ixw = MAX_int16_T;
      }

      if (ixw + 1 > 32767) {
        i_0 = 32767;
      }

      ixw = (int16_T)i_0;
    }
  }

  raspberrypi_multicore_MP_WtMult(b_Wu, I2Jm, nmv, b_Kut);
  raspberrypi_multicore_MP_WtMult(b_Wdu, b_Jm, nmv,
    raspberrypi_multicore_MPCtest_B.dv3);
  for (i_0 = 0; i_0 < 24; i_0++) {
    for (i = 0; i < 24; i++) {
      b_SuJm_tmp = i + 24 * i_0;
      raspberrypi_multicore_MPCtest_B.b_SuJm[b_SuJm_tmp] = 0.0;
      for (b_Kr_tmp = 0; b_Kr_tmp < 78; b_Kr_tmp++) {
        raspberrypi_multicore_MPCtest_B.b_SuJm[b_SuJm_tmp] += b_SuJm[78 * i +
          b_Kr_tmp] * b_Kr[78 * i_0 + b_Kr_tmp];
      }

      raspberrypi_multicore_MPCtest_B.b_Jm_g[b_SuJm_tmp] = 0.0;
      for (b_Kr_tmp = 0; b_Kr_tmp < 72; b_Kr_tmp++) {
        raspberrypi_multicore_MPCtest_B.b_Jm_g[b_SuJm_tmp] += b_Jm[72 * i +
          b_Kr_tmp] * raspberrypi_multicore_MPCtest_B.dv3[72 * i_0 + b_Kr_tmp];
      }
    }
  }

  for (i_0 = 0; i_0 < 24; i_0++) {
    for (i = 0; i < 24; i++) {
      b_Wy_0 = 0.0;
      for (b_Kr_tmp = 0; b_Kr_tmp < 72; b_Kr_tmp++) {
        b_Wy_0 += I2Jm[72 * i_0 + b_Kr_tmp] * b_Kut[72 * i + b_Kr_tmp];
      }

      b_Kr_tmp = 24 * i + i_0;
      b_H[b_Kr_tmp] = (raspberrypi_multicore_MPCtest_B.b_SuJm[b_Kr_tmp] +
                       raspberrypi_multicore_MPCtest_B.b_Jm_g[b_Kr_tmp]) +
        b_Wy_0;
    }

    for (i = 0; i < 12; i++) {
      b_SuJm_tmp = i + 12 * i_0;
      raspberrypi_multicore_MPCtest_B.b_Su1[b_SuJm_tmp] = 0.0;
      for (b_Kr_tmp = 0; b_Kr_tmp < 78; b_Kr_tmp++) {
        raspberrypi_multicore_MPCtest_B.b_Su1[b_SuJm_tmp] += b_Su1[78 * i +
          b_Kr_tmp] * b_Kr[78 * i_0 + b_Kr_tmp];
      }

      raspberrypi_multicore_MPCtest_B.b_I1[b_SuJm_tmp] = 0.0;
      for (b_Kr_tmp = 0; b_Kr_tmp < 72; b_Kr_tmp++) {
        raspberrypi_multicore_MPCtest_B.b_I1[b_SuJm_tmp] += b_I1[72 * i +
          b_Kr_tmp] * b_Kut[72 * i_0 + b_Kr_tmp];
      }
    }
  }

  for (i_0 = 0; i_0 < 288; i_0++) {
    b_Ku1[i_0] = raspberrypi_multicore_MPCtest_B.b_Su1[i_0] +
      raspberrypi_multicore_MPCtest_B.b_I1[i_0];
  }

  for (i_0 = 0; i_0 < 1728; i_0++) {
    b_Kut[i_0] = -b_Kut[i_0];
  }

  for (i_0 = 0; i_0 < 24; i_0++) {
    for (i = 0; i < 13; i++) {
      b_SuJm_tmp = i + 13 * i_0;
      b_Kx[b_SuJm_tmp] = 0.0;
      for (b_Kr_tmp = 0; b_Kr_tmp < 78; b_Kr_tmp++) {
        b_Kx[b_SuJm_tmp] += b_Sx[78 * i + b_Kr_tmp] * b_Kr[78 * i_0 + b_Kr_tmp];
      }
    }

    for (i = 0; i < 49; i++) {
      b_SuJm_tmp = i + 49 * i_0;
      b_Kv[b_SuJm_tmp] = 0.0;
      for (b_Kr_tmp = 0; b_Kr_tmp < 78; b_Kr_tmp++) {
        b_Kv[b_SuJm_tmp] += b_Hv[78 * i + b_Kr_tmp] * b_Kr[78 * i_0 + b_Kr_tmp];
      }
    }
  }

  for (i_0 = 0; i_0 < 1872; i_0++) {
    b_Kr[i_0] = -b_Kr[i_0];
  }
}

/* Function for MATLAB Function: '<S46>/FixedHorizonOptimizer' */
static int32_T raspberrypi_multicore_MP_xpotrf(real_T b_A[625])
{
  real_T c;
  real_T ssq;
  int32_T b_ix;
  int32_T b_iy;
  int32_T b_k;
  int32_T d;
  int32_T ia;
  int32_T idxAjj;
  int32_T info;
  int32_T iy;
  int32_T j;
  boolean_T exitg1;
  info = 0;
  j = 0;
  exitg1 = false;
  while ((!exitg1) && (j < 25)) {
    idxAjj = j * 25 + j;
    ssq = 0.0;
    if (j >= 1) {
      b_ix = j;
      b_iy = j;
      for (b_k = 0; b_k < j; b_k++) {
        ssq += b_A[b_ix] * b_A[b_iy];
        b_ix += 25;
        b_iy += 25;
      }
    }

    ssq = b_A[idxAjj] - ssq;
    if (ssq > 0.0) {
      ssq = sqrt(ssq);
      b_A[idxAjj] = ssq;
      if (j + 1 < 25) {
        if (j != 0) {
          b_ix = j;
          b_iy = ((j - 1) * 25 + j) + 2;
          for (b_k = j + 2; b_k <= b_iy; b_k += 25) {
            c = -b_A[b_ix];
            iy = idxAjj + 1;
            d = (b_k - j) + 23;
            for (ia = b_k; ia <= d; ia++) {
              b_A[iy] += b_A[ia - 1] * c;
              iy++;
            }

            b_ix += 25;
          }
        }

        ssq = 1.0 / ssq;
        b_ix = (idxAjj - j) + 25;
        for (idxAjj++; idxAjj < b_ix; idxAjj++) {
          b_A[idxAjj] *= ssq;
        }
      }

      j++;
    } else {
      b_A[idxAjj] = ssq;
      info = j + 1;
      exitg1 = true;
    }
  }

  return info;
}

/* Function for MATLAB Function: '<S46>/FixedHorizonOptimizer' */
static real_T raspberrypi_multicore_M_minimum(const real_T x[25])
{
  real_T ex;
  int32_T idx;
  int32_T k;
  boolean_T exitg1;
  if (!rtIsNaN(x[0])) {
    idx = 1;
  } else {
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 26)) {
      if (!rtIsNaN(x[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    ex = x[0];
  } else {
    ex = x[idx - 1];
    while (idx + 1 <= 25) {
      if (ex > x[idx]) {
        ex = x[idx];
      }

      idx++;
    }
  }

  return ex;
}

/* Function for MATLAB Function: '<S46>/FixedHorizonOptimizer' */
static void raspberrypi_mu_mpc_checkhessian(real_T b_H[625], real_T L[625],
  real_T *BadH)
{
  int32_T Tries;
  int32_T j;
  boolean_T exitg1;
  boolean_T exitg2;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  *BadH = 0.0;
  memcpy(&L[0], &b_H[0], 625U * sizeof(real_T));
  Tries = raspberrypi_multicore_MP_xpotrf(L);
  guard1 = false;
  if (Tries == 0) {
    for (Tries = 0; Tries < 25; Tries++) {
      raspberrypi_multicore_MPCtest_B.varargin_1[Tries] = L[25 * Tries + Tries];
    }

    if (raspberrypi_multicore_M_minimum
        (raspberrypi_multicore_MPCtest_B.varargin_1) > 1.4901161193847656E-7) {
    } else {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    raspberrypi_multicore_MPCtest_B.normH = 0.0;
    Tries = 0;
    exitg2 = false;
    while ((!exitg2) && (Tries < 25)) {
      raspberrypi_multicore_MPCtest_B.s_c = 0.0;
      for (j = 0; j < 25; j++) {
        raspberrypi_multicore_MPCtest_B.s_c += fabs(b_H[25 * j + Tries]);
      }

      if (rtIsNaN(raspberrypi_multicore_MPCtest_B.s_c)) {
        raspberrypi_multicore_MPCtest_B.normH = (rtNaN);
        exitg2 = true;
      } else {
        if (raspberrypi_multicore_MPCtest_B.s_c >
            raspberrypi_multicore_MPCtest_B.normH) {
          raspberrypi_multicore_MPCtest_B.normH =
            raspberrypi_multicore_MPCtest_B.s_c;
        }

        Tries++;
      }
    }

    if (raspberrypi_multicore_MPCtest_B.normH >= 1.0E+10) {
      *BadH = 2.0;
    } else {
      Tries = 0;
      exitg1 = false;
      while ((!exitg1) && (Tries <= 4)) {
        raspberrypi_multicore_MPCtest_B.normH = rt_powd_snf(10.0, (real_T)Tries)
          * 1.4901161193847656E-7;
        memset(&raspberrypi_multicore_MPCtest_B.b_f[0], 0, 625U * sizeof(int8_T));
        for (j = 0; j < 25; j++) {
          raspberrypi_multicore_MPCtest_B.b_f[j + 25 * j] = 1;
        }

        for (j = 0; j < 625; j++) {
          raspberrypi_multicore_MPCtest_B.s_c =
            raspberrypi_multicore_MPCtest_B.normH * (real_T)
            raspberrypi_multicore_MPCtest_B.b_f[j] + b_H[j];
          L[j] = raspberrypi_multicore_MPCtest_B.s_c;
          b_H[j] = raspberrypi_multicore_MPCtest_B.s_c;
        }

        j = raspberrypi_multicore_MP_xpotrf(L);
        guard2 = false;
        if (j == 0) {
          for (j = 0; j < 25; j++) {
            raspberrypi_multicore_MPCtest_B.varargin_1[j] = L[25 * j + j];
          }

          if (raspberrypi_multicore_M_minimum
              (raspberrypi_multicore_MPCtest_B.varargin_1) >
              1.4901161193847656E-7) {
            *BadH = 1.0;
            exitg1 = true;
          } else {
            guard2 = true;
          }
        } else {
          guard2 = true;
        }

        if (guard2) {
          *BadH = 3.0;
          Tries++;
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<S46>/FixedHorizonOptimizer' */
static void raspberrypi_multicore__trisolve(const real_T b_A[625], real_T b_B
  [625])
{
  real_T tmp_0;
  int32_T i;
  int32_T j;
  int32_T jBcol;
  int32_T k;
  int32_T kAcol;
  int32_T tmp;
  int32_T tmp_1;
  for (j = 0; j < 25; j++) {
    jBcol = 25 * j;
    for (k = 0; k < 25; k++) {
      kAcol = 25 * k;
      tmp = k + jBcol;
      tmp_0 = b_B[tmp];
      if (tmp_0 != 0.0) {
        b_B[tmp] = tmp_0 / b_A[k + kAcol];
        for (i = k + 2; i < 26; i++) {
          tmp_1 = (i + jBcol) - 1;
          b_B[tmp_1] -= b_A[(i + kAcol) - 1] * b_B[tmp];
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<S46>/FixedHorizonOptimizer' */
static void raspberrypi_multi_Unconstrained(const real_T b_Hinv[625], const
  real_T f[25], real_T x[25], int16_T n)
{
  real_T b_Hinv_0;
  int32_T i;
  int32_T i_0;
  for (i = 1; i - 1 < n; i++) {
    b_Hinv_0 = 0.0;
    for (i_0 = 0; i_0 < 25; i_0++) {
      b_Hinv_0 += -b_Hinv[(25 * i_0 + (int16_T)i) - 1] * f[i_0];
    }

    x[(int16_T)i - 1] = b_Hinv_0;
  }
}

/* Function for MATLAB Function: '<S46>/FixedHorizonOptimizer' */
static real_T raspberrypi_multicore_MPCt_norm(const real_T x[25])
{
  real_T absxk;
  real_T scale;
  real_T t;
  real_T y;
  int32_T k;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  for (k = 0; k < 25; k++) {
    absxk = fabs(x[k]);
    if (absxk > scale) {
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrt(y);
}

/* Function for MATLAB Function: '<S46>/FixedHorizonOptimizer' */
static void raspberrypi_multicore_MPCte_abs(const real_T x[25], real_T y[25])
{
  int32_T k;
  for (k = 0; k < 25; k++) {
    y[k] = fabs(x[k]);
  }
}

/* Function for MATLAB Function: '<S46>/FixedHorizonOptimizer' */
static real_T raspberrypi_multicore_M_maximum(const real_T x[25])
{
  real_T ex;
  int32_T idx;
  int32_T k;
  boolean_T exitg1;
  if (!rtIsNaN(x[0])) {
    idx = 1;
  } else {
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 26)) {
      if (!rtIsNaN(x[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    ex = x[0];
  } else {
    ex = x[idx - 1];
    while (idx + 1 <= 25) {
      if (ex < x[idx]) {
        ex = x[idx];
      }

      idx++;
    }
  }

  return ex;
}

/* Function for MATLAB Function: '<S46>/FixedHorizonOptimizer' */
static void raspberrypi_multicore_MPC_abs_d(const real_T x[252], real_T y[252])
{
  int32_T k;
  for (k = 0; k < 252; k++) {
    y[k] = fabs(x[k]);
  }
}

/* Function for MATLAB Function: '<S46>/FixedHorizonOptimizer' */
static void raspberrypi_multicore__maximum2(const real_T x[252], real_T y,
  real_T ex[252])
{
  real_T u0;
  int32_T k;
  for (k = 0; k < 252; k++) {
    u0 = x[k];
    if ((u0 > y) || rtIsNaN(y)) {
      ex[k] = u0;
    } else {
      ex[k] = y;
    }
  }
}

/* Function for MATLAB Function: '<S46>/FixedHorizonOptimizer' */
static real_T raspberrypi_multicore_MPC_xnrm2(int32_T n, const real_T x[625],
  int32_T ix0)
{
  real_T absxk;
  real_T scale;
  real_T t;
  real_T y;
  int32_T k;
  int32_T kend;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x[ix0 - 1]);
    } else {
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (k = ix0; k <= kend; k++) {
        absxk = fabs(x[k - 1]);
        if (absxk > scale) {
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * sqrt(y);
    }
  }

  return y;
}

real_T rt_hypotd_snf(real_T u0, real_T u1)
{
  real_T a;
  real_T y;
  a = fabs(u0);
  y = fabs(u1);
  if (a < y) {
    a /= y;
    y *= sqrt(a * a + 1.0);
  } else if (a > y) {
    y /= a;
    y = sqrt(y * y + 1.0) * a;
  } else if (!rtIsNaN(y)) {
    y = a * 1.4142135623730951;
  }

  return y;
}

/* Function for MATLAB Function: '<S46>/FixedHorizonOptimizer' */
static void raspberrypi_multicore_MPC_xgemv(int32_T b_m, int32_T n, const real_T
  b_A[625], int32_T ia0, const real_T x[625], int32_T ix0, real_T y[25])
{
  int32_T b;
  int32_T b_iy;
  int32_T d;
  int32_T ia;
  int32_T iac;
  int32_T ix;
  if ((b_m != 0) && (n != 0)) {
    for (b_iy = 0; b_iy < n; b_iy++) {
      y[b_iy] = 0.0;
    }

    b_iy = 0;
    b = (n - 1) * 25 + ia0;
    for (iac = ia0; iac <= b; iac += 25) {
      ix = ix0;
      raspberrypi_multicore_MPCtest_B.c = 0.0;
      d = (iac + b_m) - 1;
      for (ia = iac; ia <= d; ia++) {
        raspberrypi_multicore_MPCtest_B.c += b_A[ia - 1] * x[ix - 1];
        ix++;
      }

      y[b_iy] += raspberrypi_multicore_MPCtest_B.c;
      b_iy++;
    }
  }
}

/* Function for MATLAB Function: '<S46>/FixedHorizonOptimizer' */
static void raspberrypi_multicore_MPC_xgerc(int32_T b_m, int32_T n, real_T
  alpha1, int32_T ix0, const real_T y[25], real_T b_A[625], int32_T ia0)
{
  int32_T b;
  int32_T ijA;
  int32_T ix;
  int32_T j;
  int32_T jA;
  int32_T jy;
  if (!(alpha1 == 0.0)) {
    jA = ia0 - 1;
    jy = 0;
    for (j = 0; j < n; j++) {
      if (y[jy] != 0.0) {
        raspberrypi_multicore_MPCtest_B.temp_p = y[jy] * alpha1;
        ix = ix0;
        b = b_m + jA;
        for (ijA = jA; ijA < b; ijA++) {
          b_A[ijA] += b_A[ix - 1] * raspberrypi_multicore_MPCtest_B.temp_p;
          ix++;
        }
      }

      jy++;
      jA += 25;
    }
  }
}

/* Function for MATLAB Function: '<S46>/FixedHorizonOptimizer' */
static void raspberrypi_multicore_MP_xzlarf(int32_T b_m, int32_T n, int32_T iv0,
  real_T tau, real_T b_C[625], int32_T ic0, real_T work[25])
{
  int32_T coltop;
  int32_T exitg1;
  int32_T ia;
  int32_T lastc;
  int32_T lastv;
  boolean_T exitg2;
  if (tau != 0.0) {
    lastv = b_m;
    lastc = iv0 + b_m;
    while ((lastv > 0) && (b_C[lastc - 2] == 0.0)) {
      lastv--;
      lastc--;
    }

    lastc = n;
    exitg2 = false;
    while ((!exitg2) && (lastc > 0)) {
      coltop = (lastc - 1) * 25 + ic0;
      ia = coltop;
      do {
        exitg1 = 0;
        if (ia <= (coltop + lastv) - 1) {
          if (b_C[ia - 1] != 0.0) {
            exitg1 = 1;
          } else {
            ia++;
          }
        } else {
          lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    lastv = 0;
    lastc = 0;
  }

  if (lastv > 0) {
    raspberrypi_multicore_MPC_xgemv(lastv, lastc, b_C, ic0, b_C, iv0, work);
    raspberrypi_multicore_MPC_xgerc(lastv, lastc, -tau, iv0, work, b_C, ic0);
  }
}

/* Function for MATLAB Function: '<S46>/FixedHorizonOptimizer' */
static void raspberrypi_multicore_MPCte_qrf(real_T b_A[625], int32_T ia0,
  int32_T b_m, int32_T n, int32_T nfxd, real_T tau[25])
{
  memset(&raspberrypi_multicore_MPCtest_B.work[0], 0, 25U * sizeof(real_T));
  raspberrypi_multicore_MPCtest_B.i_j = 0;
  while (raspberrypi_multicore_MPCtest_B.i_j <= nfxd - 1) {
    raspberrypi_multicore_MPCtest_B.ii = ((raspberrypi_multicore_MPCtest_B.i_j *
      25 + ia0) + raspberrypi_multicore_MPCtest_B.i_j) - 1;
    raspberrypi_multicore_MPCtest_B.mmi_tmp = b_m -
      raspberrypi_multicore_MPCtest_B.i_j;
    if (raspberrypi_multicore_MPCtest_B.i_j + 1 < b_m) {
      raspberrypi_multicore_MPCtest_B.b_atmp =
        b_A[raspberrypi_multicore_MPCtest_B.ii];
      tau[raspberrypi_multicore_MPCtest_B.i_j] = 0.0;
      if (raspberrypi_multicore_MPCtest_B.mmi_tmp > 0) {
        raspberrypi_multicore_MPCtest_B.beta1 = raspberrypi_multicore_MPC_xnrm2
          (raspberrypi_multicore_MPCtest_B.mmi_tmp - 1, b_A,
           raspberrypi_multicore_MPCtest_B.ii + 2);
        if (raspberrypi_multicore_MPCtest_B.beta1 != 0.0) {
          raspberrypi_multicore_MPCtest_B.beta1 = rt_hypotd_snf
            (b_A[raspberrypi_multicore_MPCtest_B.ii],
             raspberrypi_multicore_MPCtest_B.beta1);
          if (b_A[raspberrypi_multicore_MPCtest_B.ii] >= 0.0) {
            raspberrypi_multicore_MPCtest_B.beta1 =
              -raspberrypi_multicore_MPCtest_B.beta1;
          }

          if (fabs(raspberrypi_multicore_MPCtest_B.beta1) <
              1.0020841800044864E-292) {
            raspberrypi_multicore_MPCtest_B.knt = -1;
            raspberrypi_multicore_MPCtest_B.c_k_k =
              raspberrypi_multicore_MPCtest_B.ii +
              raspberrypi_multicore_MPCtest_B.mmi_tmp;
            do {
              raspberrypi_multicore_MPCtest_B.knt++;
              raspberrypi_multicore_MPCtest_B.b_k_m =
                raspberrypi_multicore_MPCtest_B.ii + 1;
              while (raspberrypi_multicore_MPCtest_B.b_k_m + 1 <=
                     raspberrypi_multicore_MPCtest_B.c_k_k) {
                b_A[raspberrypi_multicore_MPCtest_B.b_k_m] *=
                  9.9792015476736E+291;
                raspberrypi_multicore_MPCtest_B.b_k_m++;
              }

              raspberrypi_multicore_MPCtest_B.beta1 *= 9.9792015476736E+291;
              raspberrypi_multicore_MPCtest_B.b_atmp *= 9.9792015476736E+291;
            } while (!(fabs(raspberrypi_multicore_MPCtest_B.beta1) >=
                       1.0020841800044864E-292));

            raspberrypi_multicore_MPCtest_B.beta1 = rt_hypotd_snf
              (raspberrypi_multicore_MPCtest_B.b_atmp,
               raspberrypi_multicore_MPC_xnrm2
               (raspberrypi_multicore_MPCtest_B.mmi_tmp - 1, b_A,
                raspberrypi_multicore_MPCtest_B.ii + 2));
            if (raspberrypi_multicore_MPCtest_B.b_atmp >= 0.0) {
              raspberrypi_multicore_MPCtest_B.beta1 =
                -raspberrypi_multicore_MPCtest_B.beta1;
            }

            tau[raspberrypi_multicore_MPCtest_B.i_j] =
              (raspberrypi_multicore_MPCtest_B.beta1 -
               raspberrypi_multicore_MPCtest_B.b_atmp) /
              raspberrypi_multicore_MPCtest_B.beta1;
            raspberrypi_multicore_MPCtest_B.b_atmp = 1.0 /
              (raspberrypi_multicore_MPCtest_B.b_atmp -
               raspberrypi_multicore_MPCtest_B.beta1);
            raspberrypi_multicore_MPCtest_B.b_k_m =
              raspberrypi_multicore_MPCtest_B.ii + 1;
            while (raspberrypi_multicore_MPCtest_B.b_k_m + 1 <=
                   raspberrypi_multicore_MPCtest_B.c_k_k) {
              b_A[raspberrypi_multicore_MPCtest_B.b_k_m] *=
                raspberrypi_multicore_MPCtest_B.b_atmp;
              raspberrypi_multicore_MPCtest_B.b_k_m++;
            }

            raspberrypi_multicore_MPCtest_B.c_k_k = 0;
            while (raspberrypi_multicore_MPCtest_B.c_k_k <=
                   raspberrypi_multicore_MPCtest_B.knt) {
              raspberrypi_multicore_MPCtest_B.beta1 *= 1.0020841800044864E-292;
              raspberrypi_multicore_MPCtest_B.c_k_k++;
            }

            raspberrypi_multicore_MPCtest_B.b_atmp =
              raspberrypi_multicore_MPCtest_B.beta1;
          } else {
            tau[raspberrypi_multicore_MPCtest_B.i_j] =
              (raspberrypi_multicore_MPCtest_B.beta1 -
               b_A[raspberrypi_multicore_MPCtest_B.ii]) /
              raspberrypi_multicore_MPCtest_B.beta1;
            raspberrypi_multicore_MPCtest_B.b_atmp = 1.0 /
              (b_A[raspberrypi_multicore_MPCtest_B.ii] -
               raspberrypi_multicore_MPCtest_B.beta1);
            raspberrypi_multicore_MPCtest_B.knt =
              raspberrypi_multicore_MPCtest_B.ii +
              raspberrypi_multicore_MPCtest_B.mmi_tmp;
            raspberrypi_multicore_MPCtest_B.c_k_k =
              raspberrypi_multicore_MPCtest_B.ii + 1;
            while (raspberrypi_multicore_MPCtest_B.c_k_k + 1 <=
                   raspberrypi_multicore_MPCtest_B.knt) {
              b_A[raspberrypi_multicore_MPCtest_B.c_k_k] *=
                raspberrypi_multicore_MPCtest_B.b_atmp;
              raspberrypi_multicore_MPCtest_B.c_k_k++;
            }

            raspberrypi_multicore_MPCtest_B.b_atmp =
              raspberrypi_multicore_MPCtest_B.beta1;
          }
        }
      }

      b_A[raspberrypi_multicore_MPCtest_B.ii] =
        raspberrypi_multicore_MPCtest_B.b_atmp;
    } else {
      tau[raspberrypi_multicore_MPCtest_B.i_j] = 0.0;
    }

    if (raspberrypi_multicore_MPCtest_B.i_j + 1 < n) {
      raspberrypi_multicore_MPCtest_B.b_atmp =
        b_A[raspberrypi_multicore_MPCtest_B.ii];
      b_A[raspberrypi_multicore_MPCtest_B.ii] = 1.0;
      raspberrypi_multicore_MP_xzlarf(raspberrypi_multicore_MPCtest_B.mmi_tmp,
        (n - raspberrypi_multicore_MPCtest_B.i_j) - 1,
        raspberrypi_multicore_MPCtest_B.ii + 1,
        tau[raspberrypi_multicore_MPCtest_B.i_j], b_A,
        raspberrypi_multicore_MPCtest_B.ii + 26,
        raspberrypi_multicore_MPCtest_B.work);
      b_A[raspberrypi_multicore_MPCtest_B.ii] =
        raspberrypi_multicore_MPCtest_B.b_atmp;
    }

    raspberrypi_multicore_MPCtest_B.i_j++;
  }
}

/* Function for MATLAB Function: '<S46>/FixedHorizonOptimizer' */
static void raspberrypi_multicore_MP_xgeqrf(real_T b_A[625], real_T tau[25])
{
  memset(&tau[0], 0, 25U * sizeof(real_T));
  raspberrypi_multicore_MPCte_qrf(b_A, 1, 25, 25, 25, tau);
}

/* Function for MATLAB Function: '<S46>/FixedHorizonOptimizer' */
static void raspberrypi_multicore_MP_xorgqr(int32_T b_m, int32_T n, int32_T k,
  real_T b_A[625], int32_T ia0, const real_T tau[25], int32_T itau0)
{
  if (n >= 1) {
    raspberrypi_multicore_MPCtest_B.i_m = k;
    while (raspberrypi_multicore_MPCtest_B.i_m <= n - 1) {
      raspberrypi_multicore_MPCtest_B.itau =
        (raspberrypi_multicore_MPCtest_B.i_m * 25 + ia0) - 1;
      raspberrypi_multicore_MPCtest_B.iaii = 0;
      while (raspberrypi_multicore_MPCtest_B.iaii <= b_m - 1) {
        b_A[raspberrypi_multicore_MPCtest_B.itau +
          raspberrypi_multicore_MPCtest_B.iaii] = 0.0;
        raspberrypi_multicore_MPCtest_B.iaii++;
      }

      b_A[raspberrypi_multicore_MPCtest_B.itau +
        raspberrypi_multicore_MPCtest_B.i_m] = 1.0;
      raspberrypi_multicore_MPCtest_B.i_m++;
    }

    raspberrypi_multicore_MPCtest_B.itau = (itau0 + k) - 2;
    memset(&raspberrypi_multicore_MPCtest_B.work_m[0], 0, 25U * sizeof(real_T));
    raspberrypi_multicore_MPCtest_B.i_m = k;
    while (raspberrypi_multicore_MPCtest_B.i_m >= 1) {
      raspberrypi_multicore_MPCtest_B.iaii =
        (((raspberrypi_multicore_MPCtest_B.i_m - 1) * 25 + ia0) +
         raspberrypi_multicore_MPCtest_B.i_m) - 1;
      if (raspberrypi_multicore_MPCtest_B.i_m < n) {
        b_A[raspberrypi_multicore_MPCtest_B.iaii - 1] = 1.0;
        raspberrypi_multicore_MP_xzlarf((b_m -
          raspberrypi_multicore_MPCtest_B.i_m) + 1, n -
          raspberrypi_multicore_MPCtest_B.i_m,
          raspberrypi_multicore_MPCtest_B.iaii,
          tau[raspberrypi_multicore_MPCtest_B.itau], b_A,
          raspberrypi_multicore_MPCtest_B.iaii + 25,
          raspberrypi_multicore_MPCtest_B.work_m);
      }

      if (raspberrypi_multicore_MPCtest_B.i_m < b_m) {
        raspberrypi_multicore_MPCtest_B.b_ich =
          (raspberrypi_multicore_MPCtest_B.iaii + b_m) -
          raspberrypi_multicore_MPCtest_B.i_m;
        raspberrypi_multicore_MPCtest_B.b_k_h5 =
          raspberrypi_multicore_MPCtest_B.iaii;
        while (raspberrypi_multicore_MPCtest_B.b_k_h5 + 1 <=
               raspberrypi_multicore_MPCtest_B.b_ich) {
          b_A[raspberrypi_multicore_MPCtest_B.b_k_h5] *=
            -tau[raspberrypi_multicore_MPCtest_B.itau];
          raspberrypi_multicore_MPCtest_B.b_k_h5++;
        }
      }

      b_A[raspberrypi_multicore_MPCtest_B.iaii - 1] = 1.0 -
        tau[raspberrypi_multicore_MPCtest_B.itau];
      raspberrypi_multicore_MPCtest_B.b_ich = 0;
      while (raspberrypi_multicore_MPCtest_B.b_ich <=
             raspberrypi_multicore_MPCtest_B.i_m - 2) {
        b_A[(raspberrypi_multicore_MPCtest_B.iaii -
             raspberrypi_multicore_MPCtest_B.b_ich) - 2] = 0.0;
        raspberrypi_multicore_MPCtest_B.b_ich++;
      }

      raspberrypi_multicore_MPCtest_B.itau--;
      raspberrypi_multicore_MPCtest_B.i_m--;
    }
  }
}

/* Function for MATLAB Function: '<S46>/FixedHorizonOptimizer' */
static void raspberrypi_multicore_MPCtes_qr(const real_T b_A[625], real_T Q[625],
  real_T R[625])
{
  memcpy(&raspberrypi_multicore_MPCtest_B.c_A[0], &b_A[0], 625U * sizeof(real_T));
  raspberrypi_multicore_MP_xgeqrf(raspberrypi_multicore_MPCtest_B.c_A,
    raspberrypi_multicore_MPCtest_B.tau);
  for (raspberrypi_multicore_MPCtest_B.j = 0; raspberrypi_multicore_MPCtest_B.j <
       25; raspberrypi_multicore_MPCtest_B.j++) {
    raspberrypi_multicore_MPCtest_B.i_nq = 0;
    while (raspberrypi_multicore_MPCtest_B.i_nq <=
           raspberrypi_multicore_MPCtest_B.j) {
      R[raspberrypi_multicore_MPCtest_B.i_nq + 25 *
        raspberrypi_multicore_MPCtest_B.j] =
        raspberrypi_multicore_MPCtest_B.c_A[25 *
        raspberrypi_multicore_MPCtest_B.j + raspberrypi_multicore_MPCtest_B.i_nq];
      raspberrypi_multicore_MPCtest_B.i_nq++;
    }

    raspberrypi_multicore_MPCtest_B.i_nq = raspberrypi_multicore_MPCtest_B.j + 1;
    while (raspberrypi_multicore_MPCtest_B.i_nq + 1 < 26) {
      R[raspberrypi_multicore_MPCtest_B.i_nq + 25 *
        raspberrypi_multicore_MPCtest_B.j] = 0.0;
      raspberrypi_multicore_MPCtest_B.i_nq++;
    }
  }

  raspberrypi_multicore_MP_xorgqr(25, 25, 25,
    raspberrypi_multicore_MPCtest_B.c_A, 1, raspberrypi_multicore_MPCtest_B.tau,
    1);
  for (raspberrypi_multicore_MPCtest_B.j = 0; raspberrypi_multicore_MPCtest_B.j <
       25; raspberrypi_multicore_MPCtest_B.j++) {
    memcpy(&Q[raspberrypi_multicore_MPCtest_B.j * 25],
           &raspberrypi_multicore_MPCtest_B.c_A[raspberrypi_multicore_MPCtest_B.j
           * 25], 25U * sizeof(real_T));
  }
}

/* Function for MATLAB Function: '<S46>/FixedHorizonOptimizer' */
static real_T raspberrypi_multicor_KWIKfactor(const real_T b_Ac[6300], const
  int16_T iC[252], int16_T nA, const real_T b_Linv[625], real_T RLinv[625],
  real_T b_D[625], real_T b_H[625], int16_T n)
{
  real_T Status;
  int32_T exitg1;
  int16_T b_j;
  int16_T c_k;
  Status = 1.0;
  memset(&RLinv[0], 0, 625U * sizeof(real_T));
  raspberrypi_multicore_MPCtest_B.i_h = 1;
  while (raspberrypi_multicore_MPCtest_B.i_h - 1 <= nA - 1) {
    raspberrypi_multicore_MPCtest_B.iC_o = iC[(int16_T)
      raspberrypi_multicore_MPCtest_B.i_h - 1];
    for (raspberrypi_multicore_MPCtest_B.b_i = 0;
         raspberrypi_multicore_MPCtest_B.b_i < 25;
         raspberrypi_multicore_MPCtest_B.b_i++) {
      raspberrypi_multicore_MPCtest_B.c_i = raspberrypi_multicore_MPCtest_B.b_i
        + 25 * ((int16_T)raspberrypi_multicore_MPCtest_B.i_h - 1);
      RLinv[raspberrypi_multicore_MPCtest_B.c_i] = 0.0;
      for (raspberrypi_multicore_MPCtest_B.e_i_b = 0;
           raspberrypi_multicore_MPCtest_B.e_i_b < 25;
           raspberrypi_multicore_MPCtest_B.e_i_b++) {
        RLinv[raspberrypi_multicore_MPCtest_B.c_i] += b_Ac[(252 *
          raspberrypi_multicore_MPCtest_B.e_i_b +
          raspberrypi_multicore_MPCtest_B.iC_o) - 1] * b_Linv[25 *
          raspberrypi_multicore_MPCtest_B.e_i_b +
          raspberrypi_multicore_MPCtest_B.b_i];
      }
    }

    raspberrypi_multicore_MPCtest_B.i_h++;
  }

  raspberrypi_multicore_MPCtes_qr(RLinv, raspberrypi_multicore_MPCtest_B.QQ,
    raspberrypi_multicore_MPCtest_B.RR);
  raspberrypi_multicore_MPCtest_B.b_i = 1;
  do {
    exitg1 = 0;
    if (raspberrypi_multicore_MPCtest_B.b_i - 1 <= nA - 1) {
      if (fabs(raspberrypi_multicore_MPCtest_B.RR[(((int16_T)
             raspberrypi_multicore_MPCtest_B.b_i - 1) * 25 + (int16_T)
            raspberrypi_multicore_MPCtest_B.b_i) - 1]) < 1.0E-12) {
        Status = -2.0;
        exitg1 = 1;
      } else {
        raspberrypi_multicore_MPCtest_B.b_i++;
      }
    } else {
      raspberrypi_multicore_MPCtest_B.c_i = 1;
      while (raspberrypi_multicore_MPCtest_B.c_i - 1 <= n - 1) {
        raspberrypi_multicore_MPCtest_B.e_i_b = 1;
        while (raspberrypi_multicore_MPCtest_B.e_i_b - 1 <= n - 1) {
          raspberrypi_multicore_MPCtest_B.b_Linv = 0.0;
          for (raspberrypi_multicore_MPCtest_B.b_i = 0;
               raspberrypi_multicore_MPCtest_B.b_i < 25;
               raspberrypi_multicore_MPCtest_B.b_i++) {
            raspberrypi_multicore_MPCtest_B.b_Linv += b_Linv[((int16_T)
              raspberrypi_multicore_MPCtest_B.c_i - 1) * 25 +
              raspberrypi_multicore_MPCtest_B.b_i] *
              raspberrypi_multicore_MPCtest_B.QQ[((int16_T)
              raspberrypi_multicore_MPCtest_B.e_i_b - 1) * 25 +
              raspberrypi_multicore_MPCtest_B.b_i];
          }

          raspberrypi_multicore_MPCtest_B.TL[((int16_T)
            raspberrypi_multicore_MPCtest_B.c_i + 25 * ((int16_T)
            raspberrypi_multicore_MPCtest_B.e_i_b - 1)) - 1] =
            raspberrypi_multicore_MPCtest_B.b_Linv;
          raspberrypi_multicore_MPCtest_B.e_i_b++;
        }

        raspberrypi_multicore_MPCtest_B.c_i++;
      }

      memset(&RLinv[0], 0, 625U * sizeof(real_T));
      for (b_j = nA; b_j > 0; b_j--) {
        raspberrypi_multicore_MPCtest_B.b_i = 25 * (b_j - 1);
        raspberrypi_multicore_MPCtest_B.c_i = (b_j +
          raspberrypi_multicore_MPCtest_B.b_i) - 1;
        RLinv[raspberrypi_multicore_MPCtest_B.c_i] = 1.0;
        for (c_k = b_j; c_k <= nA; c_k++) {
          raspberrypi_multicore_MPCtest_B.e_i_b = ((c_k - 1) * 25 + b_j) - 1;
          RLinv[raspberrypi_multicore_MPCtest_B.e_i_b] /=
            raspberrypi_multicore_MPCtest_B.RR[raspberrypi_multicore_MPCtest_B.c_i];
        }

        if (b_j > 1) {
          raspberrypi_multicore_MPCtest_B.i_h = 1;
          while (raspberrypi_multicore_MPCtest_B.i_h - 1 <= b_j - 2) {
            for (c_k = b_j; c_k <= nA; c_k++) {
              raspberrypi_multicore_MPCtest_B.c_i = (c_k - 1) * 25;
              raspberrypi_multicore_MPCtest_B.e_i_b =
                (raspberrypi_multicore_MPCtest_B.c_i + (int16_T)
                 raspberrypi_multicore_MPCtest_B.i_h) - 1;
              RLinv[raspberrypi_multicore_MPCtest_B.e_i_b] -=
                raspberrypi_multicore_MPCtest_B.RR
                [(raspberrypi_multicore_MPCtest_B.b_i + (int16_T)
                  raspberrypi_multicore_MPCtest_B.i_h) - 1] * RLinv
                [(raspberrypi_multicore_MPCtest_B.c_i + b_j) - 1];
            }

            raspberrypi_multicore_MPCtest_B.i_h++;
          }
        }
      }

      raspberrypi_multicore_MPCtest_B.e_i_b = 1;
      while (raspberrypi_multicore_MPCtest_B.e_i_b - 1 <= n - 1) {
        for (b_j = (int16_T)raspberrypi_multicore_MPCtest_B.e_i_b; b_j <= n; b_j
             ++) {
          raspberrypi_multicore_MPCtest_B.b_i = ((int16_T)
            raspberrypi_multicore_MPCtest_B.e_i_b + 25 * (b_j - 1)) - 1;
          b_H[raspberrypi_multicore_MPCtest_B.b_i] = 0.0;
          raspberrypi_multicore_MPCtest_B.c_i = nA + 1;
          if (nA + 1 > 32767) {
            raspberrypi_multicore_MPCtest_B.c_i = 32767;
          }

          for (c_k = (int16_T)raspberrypi_multicore_MPCtest_B.c_i; c_k <= n; c_k
               ++) {
            raspberrypi_multicore_MPCtest_B.c_i = (c_k - 1) * 25;
            b_H[raspberrypi_multicore_MPCtest_B.b_i] -=
              raspberrypi_multicore_MPCtest_B.TL
              [(raspberrypi_multicore_MPCtest_B.c_i + (int16_T)
                raspberrypi_multicore_MPCtest_B.e_i_b) - 1] *
              raspberrypi_multicore_MPCtest_B.TL
              [(raspberrypi_multicore_MPCtest_B.c_i + b_j) - 1];
          }

          b_H[(b_j + 25 * ((int16_T)raspberrypi_multicore_MPCtest_B.e_i_b - 1))
            - 1] = b_H[raspberrypi_multicore_MPCtest_B.b_i];
        }

        raspberrypi_multicore_MPCtest_B.e_i_b++;
      }

      raspberrypi_multicore_MPCtest_B.e_i_b = 1;
      while (raspberrypi_multicore_MPCtest_B.e_i_b - 1 <= nA - 1) {
        raspberrypi_multicore_MPCtest_B.i_h = 1;
        while (raspberrypi_multicore_MPCtest_B.i_h - 1 <= n - 1) {
          raspberrypi_multicore_MPCtest_B.b_i = ((int16_T)
            raspberrypi_multicore_MPCtest_B.i_h + 25 * ((int16_T)
            raspberrypi_multicore_MPCtest_B.e_i_b - 1)) - 1;
          b_D[raspberrypi_multicore_MPCtest_B.b_i] = 0.0;
          for (b_j = (int16_T)raspberrypi_multicore_MPCtest_B.e_i_b; b_j <= nA;
               b_j++) {
            raspberrypi_multicore_MPCtest_B.c_i = (b_j - 1) * 25;
            b_D[raspberrypi_multicore_MPCtest_B.b_i] +=
              raspberrypi_multicore_MPCtest_B.TL
              [(raspberrypi_multicore_MPCtest_B.c_i + (int16_T)
                raspberrypi_multicore_MPCtest_B.i_h) - 1] * RLinv
              [(raspberrypi_multicore_MPCtest_B.c_i + (int16_T)
                raspberrypi_multicore_MPCtest_B.e_i_b) - 1];
          }

          raspberrypi_multicore_MPCtest_B.i_h++;
        }

        raspberrypi_multicore_MPCtest_B.e_i_b++;
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return Status;
}

/* Function for MATLAB Function: '<S46>/FixedHorizonOptimizer' */
static real_T raspberrypi_multicore_MP_mtimes(const real_T b_A[25], const real_T
  b_B[25])
{
  real_T b_C;
  int32_T k;
  b_C = 0.0;
  for (k = 0; k < 25; k++) {
    b_C += b_A[k] * b_B[k];
  }

  return b_C;
}

/* Function for MATLAB Function: '<S46>/FixedHorizonOptimizer' */
static void raspberrypi_mult_DropConstraint(int16_T kDrop, int16_T iA[252],
  int16_T *nA, int16_T iC[252])
{
  int32_T tmp;
  int16_T i;
  iA[iC[kDrop - 1] - 1] = 0;
  if (kDrop < *nA) {
    tmp = *nA - 1;
    if (*nA - 1 < -32768) {
      tmp = -32768;
    }

    for (i = kDrop; i <= (int16_T)tmp; i++) {
      iC[i - 1] = iC[i];
    }
  }

  iC[*nA - 1] = 0;
  tmp = *nA - 1;
  if (*nA - 1 < -32768) {
    tmp = -32768;
  }

  *nA = (int16_T)tmp;
}

/* Function for MATLAB Function: '<S46>/FixedHorizonOptimizer' */
static void raspberrypi_multicore_MP_qpkwik(const real_T b_Linv[625], const
  real_T b_Hinv[625], const real_T f[25], const real_T b_Ac[6300], const real_T
  b[252], int16_T iA[252], int16_T maxiter, real_T FeasTol, real_T x[25], real_T
  lambda[252], real_T *status)
{
  int32_T exitg1;
  int32_T exitg3;
  int16_T kDrop;
  int16_T kNext;
  int16_T nA;
  int16_T tmp;
  int16_T tmp_0;
  uint16_T b_x;
  uint16_T q;
  boolean_T ColdReset;
  boolean_T DualFeasible;
  boolean_T cTolComputed;
  boolean_T exitg2;
  boolean_T exitg4;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  *status = 1.0;
  memset(&lambda[0], 0, 252U * sizeof(real_T));
  memset(&x[0], 0, 25U * sizeof(real_T));
  memset(&raspberrypi_multicore_MPCtest_B.r[0], 0, 25U * sizeof(real_T));
  raspberrypi_multicore_MPCtest_B.rMin = 0.0;
  cTolComputed = false;
  for (raspberrypi_multicore_MPCtest_B.i_k = 0;
       raspberrypi_multicore_MPCtest_B.i_k < 252;
       raspberrypi_multicore_MPCtest_B.i_k++) {
    raspberrypi_multicore_MPCtest_B.cTol[raspberrypi_multicore_MPCtest_B.i_k] =
      1.0;
    raspberrypi_multicore_MPCtest_B.iC[raspberrypi_multicore_MPCtest_B.i_k] = 0;
  }

  nA = 0;
  for (raspberrypi_multicore_MPCtest_B.i_k = 0;
       raspberrypi_multicore_MPCtest_B.i_k < 252;
       raspberrypi_multicore_MPCtest_B.i_k++) {
    if (iA[raspberrypi_multicore_MPCtest_B.i_k] == 1) {
      raspberrypi_multicore_MPCtest_B.ct = nA + 1;
      if (nA + 1 > 32767) {
        raspberrypi_multicore_MPCtest_B.ct = 32767;
      }

      nA = (int16_T)raspberrypi_multicore_MPCtest_B.ct;
      raspberrypi_multicore_MPCtest_B.iC[(int16_T)
        raspberrypi_multicore_MPCtest_B.ct - 1] = (int16_T)
        (raspberrypi_multicore_MPCtest_B.i_k + 1);
    }
  }

  guard1 = false;
  if (nA > 0) {
    memset(&raspberrypi_multicore_MPCtest_B.Opt[0], 0, 50U * sizeof(real_T));
    for (raspberrypi_multicore_MPCtest_B.i_k = 0;
         raspberrypi_multicore_MPCtest_B.i_k < 25;
         raspberrypi_multicore_MPCtest_B.i_k++) {
      raspberrypi_multicore_MPCtest_B.Rhs[raspberrypi_multicore_MPCtest_B.i_k] =
        f[raspberrypi_multicore_MPCtest_B.i_k];
      raspberrypi_multicore_MPCtest_B.Rhs[raspberrypi_multicore_MPCtest_B.i_k +
        25] = 0.0;
    }

    DualFeasible = false;
    raspberrypi_multicore_MPCtest_B.ct = 3 * nA;
    if (raspberrypi_multicore_MPCtest_B.ct > 32767) {
      raspberrypi_multicore_MPCtest_B.ct = 32767;
    }

    if ((int16_T)raspberrypi_multicore_MPCtest_B.ct > 50) {
      kNext = (int16_T)raspberrypi_multicore_MPCtest_B.ct;
    } else {
      kNext = 50;
    }

    q = (uint16_T)(kNext / 10U);
    b_x = (uint16_T)((uint32_T)kNext - q * 10);
    if ((b_x > 0) && (b_x >= 5)) {
      q++;
    }

    ColdReset = false;
    do {
      exitg3 = 0;
      if ((!DualFeasible) && (nA > 0) && ((int32_T)*status <= maxiter)) {
        raspberrypi_multicore_MPCtest_B.Xnorm0 = raspberrypi_multicor_KWIKfactor
          (b_Ac, raspberrypi_multicore_MPCtest_B.iC, nA, b_Linv,
           raspberrypi_multicore_MPCtest_B.RLinv,
           raspberrypi_multicore_MPCtest_B.b_D,
           raspberrypi_multicore_MPCtest_B.b_H, 25);
        if (raspberrypi_multicore_MPCtest_B.Xnorm0 < 0.0) {
          if (ColdReset) {
            *status = -2.0;
            exitg3 = 2;
          } else {
            nA = 0;
            memset(&iA[0], 0, 252U * sizeof(int16_T));
            memset(&raspberrypi_multicore_MPCtest_B.iC[0], 0, 252U * sizeof
                   (int16_T));
            ColdReset = true;
          }
        } else {
          raspberrypi_multicore_MPCtest_B.i_k = 1;
          while (raspberrypi_multicore_MPCtest_B.i_k - 1 <= nA - 1) {
            raspberrypi_multicore_MPCtest_B.ct = (int16_T)
              raspberrypi_multicore_MPCtest_B.i_k + 25;
            if ((int16_T)raspberrypi_multicore_MPCtest_B.i_k + 25 > 32767) {
              raspberrypi_multicore_MPCtest_B.ct = 32767;
            }

            raspberrypi_multicore_MPCtest_B.Rhs[raspberrypi_multicore_MPCtest_B.ct
              - 1] = b[raspberrypi_multicore_MPCtest_B.iC[(int16_T)
              raspberrypi_multicore_MPCtest_B.i_k - 1] - 1];
            for (kNext = (int16_T)raspberrypi_multicore_MPCtest_B.i_k; kNext <=
                 nA; kNext++) {
              raspberrypi_multicore_MPCtest_B.ct = (kNext + 25 * ((int16_T)
                raspberrypi_multicore_MPCtest_B.i_k - 1)) - 1;
              raspberrypi_multicore_MPCtest_B.U[raspberrypi_multicore_MPCtest_B.ct]
                = 0.0;
              raspberrypi_multicore_MPCtest_B.b_k_h = 1;
              while (raspberrypi_multicore_MPCtest_B.b_k_h - 1 <= nA - 1) {
                raspberrypi_multicore_MPCtest_B.c_k = ((int16_T)
                  raspberrypi_multicore_MPCtest_B.b_k_h - 1) * 25;
                raspberrypi_multicore_MPCtest_B.U[raspberrypi_multicore_MPCtest_B.ct]
                  += raspberrypi_multicore_MPCtest_B.RLinv
                  [(raspberrypi_multicore_MPCtest_B.c_k + kNext) - 1] *
                  raspberrypi_multicore_MPCtest_B.RLinv
                  [(raspberrypi_multicore_MPCtest_B.c_k + (int16_T)
                    raspberrypi_multicore_MPCtest_B.i_k) - 1];
                raspberrypi_multicore_MPCtest_B.b_k_h++;
              }

              raspberrypi_multicore_MPCtest_B.U[((int16_T)
                raspberrypi_multicore_MPCtest_B.i_k + 25 * (kNext - 1)) - 1] =
                raspberrypi_multicore_MPCtest_B.U[raspberrypi_multicore_MPCtest_B.ct];
            }

            raspberrypi_multicore_MPCtest_B.i_k++;
          }

          for (raspberrypi_multicore_MPCtest_B.i_k = 0;
               raspberrypi_multicore_MPCtest_B.i_k < 25;
               raspberrypi_multicore_MPCtest_B.i_k++) {
            raspberrypi_multicore_MPCtest_B.Xnorm0 = 0.0;
            for (raspberrypi_multicore_MPCtest_B.ct = 0;
                 raspberrypi_multicore_MPCtest_B.ct < 25;
                 raspberrypi_multicore_MPCtest_B.ct++) {
              raspberrypi_multicore_MPCtest_B.Xnorm0 +=
                raspberrypi_multicore_MPCtest_B.b_H[raspberrypi_multicore_MPCtest_B.i_k
                + 25 * raspberrypi_multicore_MPCtest_B.ct] *
                raspberrypi_multicore_MPCtest_B.Rhs[raspberrypi_multicore_MPCtest_B.ct];
            }

            raspberrypi_multicore_MPCtest_B.Opt[raspberrypi_multicore_MPCtest_B.i_k]
              = raspberrypi_multicore_MPCtest_B.Xnorm0;
            raspberrypi_multicore_MPCtest_B.b_k_h = 1;
            while (raspberrypi_multicore_MPCtest_B.b_k_h - 1 <= nA - 1) {
              raspberrypi_multicore_MPCtest_B.ct = (int16_T)
                raspberrypi_multicore_MPCtest_B.b_k_h + 25;
              if ((int16_T)raspberrypi_multicore_MPCtest_B.b_k_h + 25 > 32767) {
                raspberrypi_multicore_MPCtest_B.ct = 32767;
              }

              raspberrypi_multicore_MPCtest_B.Opt[raspberrypi_multicore_MPCtest_B.i_k]
                += raspberrypi_multicore_MPCtest_B.b_D[((int16_T)
                raspberrypi_multicore_MPCtest_B.b_k_h - 1) * 25 +
                raspberrypi_multicore_MPCtest_B.i_k] *
                raspberrypi_multicore_MPCtest_B.Rhs[raspberrypi_multicore_MPCtest_B.ct
                - 1];
              raspberrypi_multicore_MPCtest_B.b_k_h++;
            }
          }

          raspberrypi_multicore_MPCtest_B.b_k_h = 1;
          while (raspberrypi_multicore_MPCtest_B.b_k_h - 1 <= nA - 1) {
            raspberrypi_multicore_MPCtest_B.Xnorm0 = 0.0;
            for (raspberrypi_multicore_MPCtest_B.ct = 0;
                 raspberrypi_multicore_MPCtest_B.ct < 25;
                 raspberrypi_multicore_MPCtest_B.ct++) {
              raspberrypi_multicore_MPCtest_B.Xnorm0 +=
                raspberrypi_multicore_MPCtest_B.b_D[((int16_T)
                raspberrypi_multicore_MPCtest_B.b_k_h - 1) * 25 +
                raspberrypi_multicore_MPCtest_B.ct] *
                raspberrypi_multicore_MPCtest_B.Rhs[raspberrypi_multicore_MPCtest_B.ct];
            }

            raspberrypi_multicore_MPCtest_B.ct = (int16_T)
              raspberrypi_multicore_MPCtest_B.b_k_h + 25;
            if ((int16_T)raspberrypi_multicore_MPCtest_B.b_k_h + 25 > 32767) {
              raspberrypi_multicore_MPCtest_B.ct = 32767;
            }

            raspberrypi_multicore_MPCtest_B.Opt[raspberrypi_multicore_MPCtest_B.ct
              - 1] = raspberrypi_multicore_MPCtest_B.Xnorm0;
            raspberrypi_multicore_MPCtest_B.c_k = 1;
            while (raspberrypi_multicore_MPCtest_B.c_k - 1 <= nA - 1) {
              raspberrypi_multicore_MPCtest_B.ct = (int16_T)
                raspberrypi_multicore_MPCtest_B.b_k_h + 25;
              if ((int16_T)raspberrypi_multicore_MPCtest_B.b_k_h + 25 > 32767) {
                raspberrypi_multicore_MPCtest_B.ct = 32767;
              }

              raspberrypi_multicore_MPCtest_B.i_k = (int16_T)
                raspberrypi_multicore_MPCtest_B.b_k_h + 25;
              if ((int16_T)raspberrypi_multicore_MPCtest_B.b_k_h + 25 > 32767) {
                raspberrypi_multicore_MPCtest_B.i_k = 32767;
              }

              raspberrypi_multicore_MPCtest_B.i2 = (int16_T)
                raspberrypi_multicore_MPCtest_B.c_k + 25;
              if ((int16_T)raspberrypi_multicore_MPCtest_B.c_k + 25 > 32767) {
                raspberrypi_multicore_MPCtest_B.i2 = 32767;
              }

              raspberrypi_multicore_MPCtest_B.Opt[raspberrypi_multicore_MPCtest_B.ct
                - 1] = raspberrypi_multicore_MPCtest_B.U[(((int16_T)
                raspberrypi_multicore_MPCtest_B.c_k - 1) * 25 + (int16_T)
                raspberrypi_multicore_MPCtest_B.b_k_h) - 1] *
                raspberrypi_multicore_MPCtest_B.Rhs[raspberrypi_multicore_MPCtest_B.i2
                - 1] +
                raspberrypi_multicore_MPCtest_B.Opt[raspberrypi_multicore_MPCtest_B.i_k
                - 1];
              raspberrypi_multicore_MPCtest_B.c_k++;
            }

            raspberrypi_multicore_MPCtest_B.b_k_h++;
          }

          raspberrypi_multicore_MPCtest_B.Xnorm0 = -1.0E-12;
          kDrop = 0;
          raspberrypi_multicore_MPCtest_B.i_k = 1;
          while (raspberrypi_multicore_MPCtest_B.i_k - 1 <= nA - 1) {
            raspberrypi_multicore_MPCtest_B.ct = (int16_T)
              raspberrypi_multicore_MPCtest_B.i_k + 25;
            if ((int16_T)raspberrypi_multicore_MPCtest_B.i_k + 25 > 32767) {
              raspberrypi_multicore_MPCtest_B.ct = 32767;
            }

            lambda[raspberrypi_multicore_MPCtest_B.iC[(int16_T)
              raspberrypi_multicore_MPCtest_B.i_k - 1] - 1] =
              raspberrypi_multicore_MPCtest_B.Opt[raspberrypi_multicore_MPCtest_B.ct
              - 1];
            raspberrypi_multicore_MPCtest_B.ct = (int16_T)
              raspberrypi_multicore_MPCtest_B.i_k + 25;
            if ((int16_T)raspberrypi_multicore_MPCtest_B.i_k + 25 > 32767) {
              raspberrypi_multicore_MPCtest_B.ct = 32767;
            }

            if ((raspberrypi_multicore_MPCtest_B.Opt[raspberrypi_multicore_MPCtest_B.ct
                 - 1] < raspberrypi_multicore_MPCtest_B.Xnorm0) && ((int16_T)
                 raspberrypi_multicore_MPCtest_B.i_k <= nA)) {
              kDrop = (int16_T)raspberrypi_multicore_MPCtest_B.i_k;
              raspberrypi_multicore_MPCtest_B.ct = (int16_T)
                raspberrypi_multicore_MPCtest_B.i_k + 25;
              if ((int16_T)raspberrypi_multicore_MPCtest_B.i_k + 25 > 32767) {
                raspberrypi_multicore_MPCtest_B.ct = 32767;
              }

              raspberrypi_multicore_MPCtest_B.Xnorm0 =
                raspberrypi_multicore_MPCtest_B.Opt[raspberrypi_multicore_MPCtest_B.ct
                - 1];
            }

            raspberrypi_multicore_MPCtest_B.i_k++;
          }

          if (kDrop <= 0) {
            DualFeasible = true;
            memcpy(&x[0], &raspberrypi_multicore_MPCtest_B.Opt[0], 25U * sizeof
                   (real_T));
          } else {
            (*status)++;
            if ((int32_T)*status > q) {
              nA = 0;
              memset(&iA[0], 0, 252U * sizeof(int16_T));
              memset(&raspberrypi_multicore_MPCtest_B.iC[0], 0, 252U * sizeof
                     (int16_T));
              ColdReset = true;
            } else {
              lambda[raspberrypi_multicore_MPCtest_B.iC[kDrop - 1] - 1] = 0.0;
              raspberrypi_mult_DropConstraint(kDrop, iA, &nA,
                raspberrypi_multicore_MPCtest_B.iC);
            }
          }
        }
      } else {
        if (nA <= 0) {
          memset(&lambda[0], 0, 252U * sizeof(real_T));
          raspberrypi_multi_Unconstrained(b_Hinv, f, x, 25);
        }

        exitg3 = 1;
      }
    } while (exitg3 == 0);

    if (exitg3 == 1) {
      guard1 = true;
    }
  } else {
    raspberrypi_multi_Unconstrained(b_Hinv, f, x, 25);
    guard1 = true;
  }

  if (guard1) {
    raspberrypi_multicore_MPCtest_B.Xnorm0 = raspberrypi_multicore_MPCt_norm(x);
    exitg2 = false;
    while ((!exitg2) && ((int32_T)*status <= maxiter)) {
      raspberrypi_multicore_MPCtest_B.cMin = -FeasTol;
      kNext = 0;
      for (raspberrypi_multicore_MPCtest_B.i_k = 0;
           raspberrypi_multicore_MPCtest_B.i_k < 252;
           raspberrypi_multicore_MPCtest_B.i_k++) {
        raspberrypi_multicore_MPCtest_B.zTa =
          raspberrypi_multicore_MPCtest_B.cTol[raspberrypi_multicore_MPCtest_B.i_k];
        if (!cTolComputed) {
          for (raspberrypi_multicore_MPCtest_B.ct = 0;
               raspberrypi_multicore_MPCtest_B.ct < 25;
               raspberrypi_multicore_MPCtest_B.ct++) {
            raspberrypi_multicore_MPCtest_B.b_Ac[raspberrypi_multicore_MPCtest_B.ct]
              = b_Ac[raspberrypi_multicore_MPCtest_B.i_k + 252 *
              raspberrypi_multicore_MPCtest_B.ct] *
              x[raspberrypi_multicore_MPCtest_B.ct];
          }

          raspberrypi_multicore_MPCte_abs(raspberrypi_multicore_MPCtest_B.b_Ac,
            raspberrypi_multicore_MPCtest_B.z);
          raspberrypi_multicore_MPCtest_B.cVal = raspberrypi_multicore_M_maximum
            (raspberrypi_multicore_MPCtest_B.z);
          if ((!(raspberrypi_multicore_MPCtest_B.zTa >
                 raspberrypi_multicore_MPCtest_B.cVal)) && (!rtIsNaN
               (raspberrypi_multicore_MPCtest_B.cVal))) {
            raspberrypi_multicore_MPCtest_B.zTa =
              raspberrypi_multicore_MPCtest_B.cVal;
          }
        }

        if (iA[raspberrypi_multicore_MPCtest_B.i_k] == 0) {
          raspberrypi_multicore_MPCtest_B.cVal = 0.0;
          for (raspberrypi_multicore_MPCtest_B.ct = 0;
               raspberrypi_multicore_MPCtest_B.ct < 25;
               raspberrypi_multicore_MPCtest_B.ct++) {
            raspberrypi_multicore_MPCtest_B.cVal +=
              b_Ac[raspberrypi_multicore_MPCtest_B.i_k + 252 *
              raspberrypi_multicore_MPCtest_B.ct] *
              x[raspberrypi_multicore_MPCtest_B.ct];
          }

          raspberrypi_multicore_MPCtest_B.cVal =
            (raspberrypi_multicore_MPCtest_B.cVal -
             b[raspberrypi_multicore_MPCtest_B.i_k]) /
            raspberrypi_multicore_MPCtest_B.zTa;
          if (raspberrypi_multicore_MPCtest_B.cVal <
              raspberrypi_multicore_MPCtest_B.cMin) {
            raspberrypi_multicore_MPCtest_B.cMin =
              raspberrypi_multicore_MPCtest_B.cVal;
            kNext = (int16_T)(raspberrypi_multicore_MPCtest_B.i_k + 1);
          }
        }

        raspberrypi_multicore_MPCtest_B.cTol[raspberrypi_multicore_MPCtest_B.i_k]
          = raspberrypi_multicore_MPCtest_B.zTa;
      }

      cTolComputed = true;
      if (kNext <= 0) {
        exitg2 = true;
      } else if ((int32_T)*status == maxiter) {
        *status = 0.0;
        exitg2 = true;
      } else {
        do {
          exitg1 = 0;
          if ((kNext > 0) && ((int32_T)*status <= maxiter)) {
            guard2 = false;
            if (nA == 0) {
              for (raspberrypi_multicore_MPCtest_B.ct = 0;
                   raspberrypi_multicore_MPCtest_B.ct < 25;
                   raspberrypi_multicore_MPCtest_B.ct++) {
                raspberrypi_multicore_MPCtest_B.z[raspberrypi_multicore_MPCtest_B.ct]
                  = 0.0;
                for (raspberrypi_multicore_MPCtest_B.i_k = 0;
                     raspberrypi_multicore_MPCtest_B.i_k < 25;
                     raspberrypi_multicore_MPCtest_B.i_k++) {
                  raspberrypi_multicore_MPCtest_B.z[raspberrypi_multicore_MPCtest_B.ct]
                    += b_Ac[(252 * raspberrypi_multicore_MPCtest_B.i_k + kNext)
                    - 1] * b_Hinv[25 * raspberrypi_multicore_MPCtest_B.i_k +
                    raspberrypi_multicore_MPCtest_B.ct];
                }
              }

              guard2 = true;
            } else {
              raspberrypi_multicore_MPCtest_B.cMin =
                raspberrypi_multicor_KWIKfactor(b_Ac,
                raspberrypi_multicore_MPCtest_B.iC, nA, b_Linv,
                raspberrypi_multicore_MPCtest_B.RLinv,
                raspberrypi_multicore_MPCtest_B.b_D,
                raspberrypi_multicore_MPCtest_B.b_H, 25);
              if (raspberrypi_multicore_MPCtest_B.cMin <= 0.0) {
                *status = -2.0;
                exitg1 = 1;
              } else {
                for (raspberrypi_multicore_MPCtest_B.ct = 0;
                     raspberrypi_multicore_MPCtest_B.ct < 625;
                     raspberrypi_multicore_MPCtest_B.ct++) {
                  raspberrypi_multicore_MPCtest_B.U[raspberrypi_multicore_MPCtest_B.ct]
                    =
                    -raspberrypi_multicore_MPCtest_B.b_H[raspberrypi_multicore_MPCtest_B.ct];
                }

                for (raspberrypi_multicore_MPCtest_B.ct = 0;
                     raspberrypi_multicore_MPCtest_B.ct < 25;
                     raspberrypi_multicore_MPCtest_B.ct++) {
                  raspberrypi_multicore_MPCtest_B.z[raspberrypi_multicore_MPCtest_B.ct]
                    = 0.0;
                  for (raspberrypi_multicore_MPCtest_B.i_k = 0;
                       raspberrypi_multicore_MPCtest_B.i_k < 25;
                       raspberrypi_multicore_MPCtest_B.i_k++) {
                    raspberrypi_multicore_MPCtest_B.z[raspberrypi_multicore_MPCtest_B.ct]
                      += b_Ac[(252 * raspberrypi_multicore_MPCtest_B.i_k + kNext)
                      - 1] * raspberrypi_multicore_MPCtest_B.U[25 *
                      raspberrypi_multicore_MPCtest_B.i_k +
                      raspberrypi_multicore_MPCtest_B.ct];
                  }
                }

                raspberrypi_multicore_MPCtest_B.i_k = 1;
                while (raspberrypi_multicore_MPCtest_B.i_k - 1 <= nA - 1) {
                  raspberrypi_multicore_MPCtest_B.cVal = 0.0;
                  for (raspberrypi_multicore_MPCtest_B.ct = 0;
                       raspberrypi_multicore_MPCtest_B.ct < 25;
                       raspberrypi_multicore_MPCtest_B.ct++) {
                    raspberrypi_multicore_MPCtest_B.cVal += b_Ac[(252 *
                      raspberrypi_multicore_MPCtest_B.ct + kNext) - 1] *
                      raspberrypi_multicore_MPCtest_B.b_D[((int16_T)
                      raspberrypi_multicore_MPCtest_B.i_k - 1) * 25 +
                      raspberrypi_multicore_MPCtest_B.ct];
                  }

                  raspberrypi_multicore_MPCtest_B.r[(int16_T)
                    raspberrypi_multicore_MPCtest_B.i_k - 1] =
                    raspberrypi_multicore_MPCtest_B.cVal;
                  raspberrypi_multicore_MPCtest_B.i_k++;
                }

                guard2 = true;
              }
            }

            if (guard2) {
              kDrop = 0;
              raspberrypi_multicore_MPCtest_B.cMin = 0.0;
              DualFeasible = true;
              ColdReset = true;
              if (nA > 0) {
                raspberrypi_multicore_MPCtest_B.ct = 0;
                exitg4 = false;
                while ((!exitg4) && (raspberrypi_multicore_MPCtest_B.ct <= nA -
                                     1)) {
                  if (raspberrypi_multicore_MPCtest_B.r[raspberrypi_multicore_MPCtest_B.ct]
                      >= 1.0E-12) {
                    ColdReset = false;
                    exitg4 = true;
                  } else {
                    raspberrypi_multicore_MPCtest_B.ct++;
                  }
                }
              }

              if ((nA != 0) && (!ColdReset)) {
                raspberrypi_multicore_MPCtest_B.ct = 1;
                while (raspberrypi_multicore_MPCtest_B.ct - 1 <= nA - 1) {
                  raspberrypi_multicore_MPCtest_B.zTa =
                    raspberrypi_multicore_MPCtest_B.r[(int16_T)
                    raspberrypi_multicore_MPCtest_B.ct - 1];
                  if (raspberrypi_multicore_MPCtest_B.zTa > 1.0E-12) {
                    raspberrypi_multicore_MPCtest_B.zTa =
                      lambda[raspberrypi_multicore_MPCtest_B.iC[(int16_T)
                      raspberrypi_multicore_MPCtest_B.ct - 1] - 1] /
                      raspberrypi_multicore_MPCtest_B.zTa;
                    if ((kDrop == 0) || (raspberrypi_multicore_MPCtest_B.zTa <
                                         raspberrypi_multicore_MPCtest_B.rMin))
                    {
                      raspberrypi_multicore_MPCtest_B.rMin =
                        raspberrypi_multicore_MPCtest_B.zTa;
                      kDrop = (int16_T)raspberrypi_multicore_MPCtest_B.ct;
                    }
                  }

                  raspberrypi_multicore_MPCtest_B.ct++;
                }

                if (kDrop > 0) {
                  raspberrypi_multicore_MPCtest_B.cMin =
                    raspberrypi_multicore_MPCtest_B.rMin;
                  DualFeasible = false;
                }
              }

              for (raspberrypi_multicore_MPCtest_B.ct = 0;
                   raspberrypi_multicore_MPCtest_B.ct < 25;
                   raspberrypi_multicore_MPCtest_B.ct++) {
                raspberrypi_multicore_MPCtest_B.b_Ac[raspberrypi_multicore_MPCtest_B.ct]
                  = b_Ac[(252 * raspberrypi_multicore_MPCtest_B.ct + kNext) - 1];
              }

              raspberrypi_multicore_MPCtest_B.zTa =
                raspberrypi_multicore_MP_mtimes
                (raspberrypi_multicore_MPCtest_B.z,
                 raspberrypi_multicore_MPCtest_B.b_Ac);
              if (raspberrypi_multicore_MPCtest_B.zTa <= 0.0) {
                raspberrypi_multicore_MPCtest_B.zTa = 0.0;
                ColdReset = true;
              } else {
                raspberrypi_multicore_MPCtest_B.cVal = 0.0;
                for (raspberrypi_multicore_MPCtest_B.ct = 0;
                     raspberrypi_multicore_MPCtest_B.ct < 25;
                     raspberrypi_multicore_MPCtest_B.ct++) {
                  raspberrypi_multicore_MPCtest_B.cVal += b_Ac[(252 *
                    raspberrypi_multicore_MPCtest_B.ct + kNext) - 1] *
                    x[raspberrypi_multicore_MPCtest_B.ct];
                }

                raspberrypi_multicore_MPCtest_B.zTa = (b[kNext - 1] -
                  raspberrypi_multicore_MPCtest_B.cVal) /
                  raspberrypi_multicore_MPCtest_B.zTa;
                ColdReset = false;
              }

              if (DualFeasible && ColdReset) {
                *status = -1.0;
                exitg1 = 1;
              } else {
                if (ColdReset) {
                  raspberrypi_multicore_MPCtest_B.cVal =
                    raspberrypi_multicore_MPCtest_B.cMin;
                } else if (DualFeasible) {
                  raspberrypi_multicore_MPCtest_B.cVal =
                    raspberrypi_multicore_MPCtest_B.zTa;
                } else if ((raspberrypi_multicore_MPCtest_B.cMin <
                            raspberrypi_multicore_MPCtest_B.zTa) || rtIsNaN
                           (raspberrypi_multicore_MPCtest_B.zTa)) {
                  raspberrypi_multicore_MPCtest_B.cVal =
                    raspberrypi_multicore_MPCtest_B.cMin;
                } else {
                  raspberrypi_multicore_MPCtest_B.cVal =
                    raspberrypi_multicore_MPCtest_B.zTa;
                }

                raspberrypi_multicore_MPCtest_B.ct = 1;
                while (raspberrypi_multicore_MPCtest_B.ct - 1 <= nA - 1) {
                  raspberrypi_multicore_MPCtest_B.i_k =
                    raspberrypi_multicore_MPCtest_B.iC[(int16_T)
                    raspberrypi_multicore_MPCtest_B.ct - 1];
                  lambda[raspberrypi_multicore_MPCtest_B.i_k - 1] -=
                    raspberrypi_multicore_MPCtest_B.r[(int16_T)
                    raspberrypi_multicore_MPCtest_B.ct - 1] *
                    raspberrypi_multicore_MPCtest_B.cVal;
                  if ((raspberrypi_multicore_MPCtest_B.i_k <= 252) &&
                      (lambda[raspberrypi_multicore_MPCtest_B.i_k - 1] < 0.0)) {
                    lambda[raspberrypi_multicore_MPCtest_B.i_k - 1] = 0.0;
                  }

                  raspberrypi_multicore_MPCtest_B.ct++;
                }

                lambda[kNext - 1] += raspberrypi_multicore_MPCtest_B.cVal;
                if (raspberrypi_multicore_MPCtest_B.cVal ==
                    raspberrypi_multicore_MPCtest_B.cMin) {
                  raspberrypi_mult_DropConstraint(kDrop, iA, &nA,
                    raspberrypi_multicore_MPCtest_B.iC);
                }

                if (!ColdReset) {
                  for (raspberrypi_multicore_MPCtest_B.ct = 0;
                       raspberrypi_multicore_MPCtest_B.ct < 25;
                       raspberrypi_multicore_MPCtest_B.ct++) {
                    x[raspberrypi_multicore_MPCtest_B.ct] +=
                      raspberrypi_multicore_MPCtest_B.cVal *
                      raspberrypi_multicore_MPCtest_B.z[raspberrypi_multicore_MPCtest_B.ct];
                  }

                  if (raspberrypi_multicore_MPCtest_B.cVal ==
                      raspberrypi_multicore_MPCtest_B.zTa) {
                    if (nA == 25) {
                      *status = -1.0;
                      exitg1 = 1;
                    } else {
                      raspberrypi_multicore_MPCtest_B.ct = nA + 1;
                      if (nA + 1 > 32767) {
                        raspberrypi_multicore_MPCtest_B.ct = 32767;
                      }

                      nA = (int16_T)raspberrypi_multicore_MPCtest_B.ct;
                      raspberrypi_multicore_MPCtest_B.iC[(int16_T)
                        raspberrypi_multicore_MPCtest_B.ct - 1] = kNext;
                      kDrop = (int16_T)raspberrypi_multicore_MPCtest_B.ct;
                      exitg4 = false;
                      while ((!exitg4) && (kDrop > 1)) {
                        tmp = raspberrypi_multicore_MPCtest_B.iC[kDrop - 1];
                        tmp_0 = raspberrypi_multicore_MPCtest_B.iC[kDrop - 2];
                        if (tmp > tmp_0) {
                          exitg4 = true;
                        } else {
                          raspberrypi_multicore_MPCtest_B.iC[kDrop - 1] = tmp_0;
                          raspberrypi_multicore_MPCtest_B.iC[kDrop - 2] = tmp;
                          kDrop--;
                        }
                      }

                      iA[kNext - 1] = 1;
                      kNext = 0;
                      (*status)++;
                    }
                  } else {
                    (*status)++;
                  }
                } else {
                  (*status)++;
                }
              }
            }
          } else {
            raspberrypi_multicore_MPCtest_B.cMin =
              raspberrypi_multicore_MPCt_norm(x);
            if (fabs(raspberrypi_multicore_MPCtest_B.cMin -
                     raspberrypi_multicore_MPCtest_B.Xnorm0) > 0.001) {
              raspberrypi_multicore_MPCtest_B.Xnorm0 =
                raspberrypi_multicore_MPCtest_B.cMin;
              raspberrypi_multicore_MPC_abs_d(b,
                raspberrypi_multicore_MPCtest_B.dv7);
              raspberrypi_multicore__maximum2
                (raspberrypi_multicore_MPCtest_B.dv7, 1.0,
                 raspberrypi_multicore_MPCtest_B.cTol);
              cTolComputed = false;
            }

            exitg1 = 2;
          }
        } while (exitg1 == 0);

        if (exitg1 == 1) {
          exitg2 = true;
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<S46>/FixedHorizonOptimizer' */
static void raspberrypi_multico_mpc_solveQP(const real_T xQP[13], const real_T
  b_Kx[312], const real_T b_Kr[1872], const real_T rseq[78], const real_T b_Ku1
  [288], const real_T old_u[12], const real_T b_Kv[1176], const real_T vseq[49],
  const real_T b_Kut[1728], const real_T b_utarget[72], const real_T b_Linv[625],
  const real_T b_Hinv[625], const real_T b_Ac[6300], const real_T Bc[252],
  boolean_T iA[252], real_T zopt[25], real_T f[25], real_T *status)
{
  memset(&f[0], 0, 25U * sizeof(real_T));
  for (raspberrypi_multicore_MPCtest_B.i_n = 0;
       raspberrypi_multicore_MPCtest_B.i_n < 24;
       raspberrypi_multicore_MPCtest_B.i_n++) {
    raspberrypi_multicore_MPCtest_B.b_Kx = 0.0;
    for (raspberrypi_multicore_MPCtest_B.i1 = 0;
         raspberrypi_multicore_MPCtest_B.i1 < 13;
         raspberrypi_multicore_MPCtest_B.i1++) {
      raspberrypi_multicore_MPCtest_B.b_Kx += b_Kx[13 *
        raspberrypi_multicore_MPCtest_B.i_n + raspberrypi_multicore_MPCtest_B.i1]
        * xQP[raspberrypi_multicore_MPCtest_B.i1];
    }

    raspberrypi_multicore_MPCtest_B.b_Kr = 0.0;
    for (raspberrypi_multicore_MPCtest_B.i1 = 0;
         raspberrypi_multicore_MPCtest_B.i1 < 78;
         raspberrypi_multicore_MPCtest_B.i1++) {
      raspberrypi_multicore_MPCtest_B.b_Kr += b_Kr[78 *
        raspberrypi_multicore_MPCtest_B.i_n + raspberrypi_multicore_MPCtest_B.i1]
        * rseq[raspberrypi_multicore_MPCtest_B.i1];
    }

    raspberrypi_multicore_MPCtest_B.b_Ku1 = 0.0;
    for (raspberrypi_multicore_MPCtest_B.i1 = 0;
         raspberrypi_multicore_MPCtest_B.i1 < 12;
         raspberrypi_multicore_MPCtest_B.i1++) {
      raspberrypi_multicore_MPCtest_B.b_Ku1 += b_Ku1[12 *
        raspberrypi_multicore_MPCtest_B.i_n + raspberrypi_multicore_MPCtest_B.i1]
        * old_u[raspberrypi_multicore_MPCtest_B.i1];
    }

    raspberrypi_multicore_MPCtest_B.b_Kv = 0.0;
    for (raspberrypi_multicore_MPCtest_B.i1 = 0;
         raspberrypi_multicore_MPCtest_B.i1 < 49;
         raspberrypi_multicore_MPCtest_B.i1++) {
      raspberrypi_multicore_MPCtest_B.b_Kv += b_Kv[49 *
        raspberrypi_multicore_MPCtest_B.i_n + raspberrypi_multicore_MPCtest_B.i1]
        * vseq[raspberrypi_multicore_MPCtest_B.i1];
    }

    raspberrypi_multicore_MPCtest_B.b_Kut = 0.0;
    for (raspberrypi_multicore_MPCtest_B.i1 = 0;
         raspberrypi_multicore_MPCtest_B.i1 < 72;
         raspberrypi_multicore_MPCtest_B.i1++) {
      raspberrypi_multicore_MPCtest_B.b_Kut += b_Kut[72 *
        raspberrypi_multicore_MPCtest_B.i_n + raspberrypi_multicore_MPCtest_B.i1]
        * b_utarget[raspberrypi_multicore_MPCtest_B.i1];
    }

    f[raspberrypi_multicore_MPCtest_B.i_n] =
      (((raspberrypi_multicore_MPCtest_B.b_Kx +
         raspberrypi_multicore_MPCtest_B.b_Kr) +
        raspberrypi_multicore_MPCtest_B.b_Ku1) +
       raspberrypi_multicore_MPCtest_B.b_Kv) +
      raspberrypi_multicore_MPCtest_B.b_Kut;
  }

  for (raspberrypi_multicore_MPCtest_B.i_n = 0;
       raspberrypi_multicore_MPCtest_B.i_n < 252;
       raspberrypi_multicore_MPCtest_B.i_n++) {
    raspberrypi_multicore_MPCtest_B.iAnew[raspberrypi_multicore_MPCtest_B.i_n] =
      iA[raspberrypi_multicore_MPCtest_B.i_n];
  }

  raspberrypi_multicore_MP_qpkwik(b_Linv, b_Hinv, f, b_Ac, Bc,
    raspberrypi_multicore_MPCtest_B.iAnew, 30, 1.0E-6, zopt,
    raspberrypi_multicore_MPCtest_B.a__1, status);
  for (raspberrypi_multicore_MPCtest_B.i_n = 0;
       raspberrypi_multicore_MPCtest_B.i_n < 252;
       raspberrypi_multicore_MPCtest_B.i_n++) {
    iA[raspberrypi_multicore_MPCtest_B.i_n] =
      (raspberrypi_multicore_MPCtest_B.iAnew[raspberrypi_multicore_MPCtest_B.i_n]
       != 0);
  }

  if (*status < 0.0) {
    memset(&zopt[0], 0, 25U * sizeof(real_T));
  }
}

/* Function for MATLAB Function: '<S46>/FixedHorizonOptimizer' */
static void raspberrypi_multic_mpc_clipping(const real_T uk[12], const real_T
  uk1[12], const real_T b_uoff[12], const real_T b_Mlim[252], const real_T
  b_Mrows[28], real_T b_p, real_T b_ny, real_T b_degrees, real_T uk_clipped[12])
{
  real_T b_Mlim_0;
  real_T du;
  real_T dumax;
  real_T iu;
  real_T nylim;
  real_T pnu;
  real_T uk1_0;
  int32_T i;
  nylim = 2.0 * b_p * b_ny;
  pnu = b_p * 12.0;
  for (i = 0; i < 12; i++) {
    uk_clipped[i] = uk[i];
    raspberrypi_multicore_MPCtest_B.umax[i] = (rtInf);
    raspberrypi_multicore_MPCtest_B.dumin[i] = (rtMinusInf);
    raspberrypi_multicore_MPCtest_B.dumax[i] = (rtInf);
    raspberrypi_multicore_MPCtest_B.umin[i] = (rtMinusInf);
  }

  for (i = 0; i < 28; i++) {
    b_Mlim_0 = b_Mlim[i];
    iu = b_Mrows[i];
    if ((iu > nylim) && (iu <= nylim + 12.0)) {
      iu -= nylim;
      raspberrypi_multicore_MPCtest_B.umax[(int32_T)iu - 1] = b_uoff[(int32_T)iu
        - 1] + b_Mlim_0;
    } else {
      dumax = nylim + pnu;
      if ((iu > dumax) && (iu <= dumax + 12.0)) {
        iu -= dumax;
        raspberrypi_multicore_MPCtest_B.umin[(int32_T)iu - 1] = b_uoff[(int32_T)
          iu - 1] + -b_Mlim_0;
      } else {
        dumax = 2.0 * pnu + nylim;
        if ((iu > dumax) && (iu <= dumax + 12.0)) {
          raspberrypi_multicore_MPCtest_B.dumax[(int32_T)(iu - dumax) - 1] =
            b_Mlim_0;
        } else {
          dumax += b_degrees - 1.0;
          if ((iu > dumax) && (iu <= dumax + 12.0)) {
            raspberrypi_multicore_MPCtest_B.dumin[(int32_T)(iu - dumax) - 1] =
              -b_Mlim_0;
          }
        }
      }
    }
  }

  for (i = 0; i < 12; i++) {
    nylim = raspberrypi_multicore_MPCtest_B.umin[i];
    pnu = raspberrypi_multicore_MPCtest_B.umax[i];
    b_Mlim_0 = uk_clipped[i];
    iu = raspberrypi_multicore_MPCtest_B.dumin[i];
    dumax = raspberrypi_multicore_MPCtest_B.dumax[i];
    uk1_0 = uk1[i];
    du = uk[i] - uk1_0;
    if (du > dumax) {
      b_Mlim_0 = uk1_0 + dumax;
    } else if (du < iu) {
      b_Mlim_0 = uk1_0 + iu;
    }

    if (b_Mlim_0 > pnu) {
      b_Mlim_0 = pnu;
    } else if (b_Mlim_0 < nylim) {
      b_Mlim_0 = nylim;
    }

    uk_clipped[i] = b_Mlim_0;
  }
}

/* Function for MATLAB Function: '<S46>/FixedHorizonOptimizer' */
static void raspberrypi__mpcblock_optimizer(const real_T rseq[78], const real_T
  vseq[49], real_T switch_in, const real_T x[13], const real_T old_u[12], const
  boolean_T iA[252], real_T b_Mlim[252], real_T b_Mx[3276], real_T b_Mu1[3024],
  real_T b_Mv[12348], const real_T b_utarget[72], const real_T b_uoff[12], const
  real_T b_voff[6], const real_T b_yoff[13], real_T b_enable_value, real_T b_H
  [625], real_T b_Ac[6300], const real_T b_Wy[78], const real_T b_Wdu[12], const
  real_T b_Jm[1728], const real_T b_Wu[12], const real_T b_I1[864], const real_T
  b_A[169], const real_T Bu[1092], const real_T Bv[637], const real_T b_C[169],
  const real_T Dv[637], const real_T b_Mrows[28], const real_T b_Ecc[384], const
  real_T b_Fcc[416], const real_T b_Scc[192], const real_T b_Gcc[32], real_T u
  [12], real_T useq[84], real_T *status, boolean_T iAout[252])
{
  static const real_T c[36] = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  boolean_T exitg1;
  *status = 1.0;
  memset(&iAout[0], 0, 252U * sizeof(boolean_T));
  if (switch_in != b_enable_value) {
    for (raspberrypi_multicore_MPCtest_B.i_f = 0;
         raspberrypi_multicore_MPCtest_B.i_f < 12;
         raspberrypi_multicore_MPCtest_B.i_f++) {
      u[raspberrypi_multicore_MPCtest_B.i_f] =
        old_u[raspberrypi_multicore_MPCtest_B.i_f] +
        b_uoff[raspberrypi_multicore_MPCtest_B.i_f];
      for (raspberrypi_multicore_MPCtest_B.k = 0;
           raspberrypi_multicore_MPCtest_B.k < 7;
           raspberrypi_multicore_MPCtest_B.k++) {
        useq[raspberrypi_multicore_MPCtest_B.k + 7 *
          raspberrypi_multicore_MPCtest_B.i_f] =
          u[raspberrypi_multicore_MPCtest_B.i_f];
      }
    }
  } else {
    raspberrypi__mpc_constraintcoef(b_A, &Bu[0], &Bv[0], b_C, &Dv[0], b_Jm,
      raspberrypi_multicore_MPCtest_B.c_SuJm,
      raspberrypi_multicore_MPCtest_B.c_Sx,
      raspberrypi_multicore_MPCtest_B.c_Su1,
      raspberrypi_multicore_MPCtest_B.c_Hv);
    if (b_Mrows[0] > 0.0) {
      raspberrypi_multicore_MPCtest_B.i_f = 0;
      exitg1 = false;
      while ((!exitg1) && (raspberrypi_multicore_MPCtest_B.i_f < 28)) {
        if (b_Mrows[raspberrypi_multicore_MPCtest_B.i_f] <= 78.0) {
          raspberrypi_multicore_MPCtest_B.e_i = (int32_T)
            b_Mrows[raspberrypi_multicore_MPCtest_B.i_f];
          for (raspberrypi_multicore_MPCtest_B.k = 0;
               raspberrypi_multicore_MPCtest_B.k < 24;
               raspberrypi_multicore_MPCtest_B.k++) {
            b_Ac[raspberrypi_multicore_MPCtest_B.i_f + 252 *
              raspberrypi_multicore_MPCtest_B.k] =
              -raspberrypi_multicore_MPCtest_B.c_SuJm[(78 *
              raspberrypi_multicore_MPCtest_B.k +
              raspberrypi_multicore_MPCtest_B.e_i) - 1];
          }

          for (raspberrypi_multicore_MPCtest_B.k = 0;
               raspberrypi_multicore_MPCtest_B.k < 13;
               raspberrypi_multicore_MPCtest_B.k++) {
            b_Mx[raspberrypi_multicore_MPCtest_B.i_f + 252 *
              raspberrypi_multicore_MPCtest_B.k] =
              -raspberrypi_multicore_MPCtest_B.c_Sx[(78 *
              raspberrypi_multicore_MPCtest_B.k +
              raspberrypi_multicore_MPCtest_B.e_i) - 1];
          }

          for (raspberrypi_multicore_MPCtest_B.k = 0;
               raspberrypi_multicore_MPCtest_B.k < 12;
               raspberrypi_multicore_MPCtest_B.k++) {
            b_Mu1[raspberrypi_multicore_MPCtest_B.i_f + 252 *
              raspberrypi_multicore_MPCtest_B.k] =
              -raspberrypi_multicore_MPCtest_B.c_Su1[(78 *
              raspberrypi_multicore_MPCtest_B.k +
              raspberrypi_multicore_MPCtest_B.e_i) - 1];
          }

          for (raspberrypi_multicore_MPCtest_B.k = 0;
               raspberrypi_multicore_MPCtest_B.k < 49;
               raspberrypi_multicore_MPCtest_B.k++) {
            b_Mv[raspberrypi_multicore_MPCtest_B.i_f + 252 *
              raspberrypi_multicore_MPCtest_B.k] =
              -raspberrypi_multicore_MPCtest_B.c_Hv[(78 *
              raspberrypi_multicore_MPCtest_B.k +
              raspberrypi_multicore_MPCtest_B.e_i) - 1];
          }

          raspberrypi_multicore_MPCtest_B.i_f++;
        } else if (b_Mrows[raspberrypi_multicore_MPCtest_B.i_f] <= 156.0) {
          raspberrypi_multicore_MPCtest_B.e_i = (int32_T)
            (b_Mrows[raspberrypi_multicore_MPCtest_B.i_f] - 78.0);
          for (raspberrypi_multicore_MPCtest_B.k = 0;
               raspberrypi_multicore_MPCtest_B.k < 24;
               raspberrypi_multicore_MPCtest_B.k++) {
            b_Ac[raspberrypi_multicore_MPCtest_B.i_f + 252 *
              raspberrypi_multicore_MPCtest_B.k] =
              raspberrypi_multicore_MPCtest_B.c_SuJm[(78 *
              raspberrypi_multicore_MPCtest_B.k +
              raspberrypi_multicore_MPCtest_B.e_i) - 1];
          }

          for (raspberrypi_multicore_MPCtest_B.k = 0;
               raspberrypi_multicore_MPCtest_B.k < 13;
               raspberrypi_multicore_MPCtest_B.k++) {
            b_Mx[raspberrypi_multicore_MPCtest_B.i_f + 252 *
              raspberrypi_multicore_MPCtest_B.k] =
              raspberrypi_multicore_MPCtest_B.c_Sx[(78 *
              raspberrypi_multicore_MPCtest_B.k +
              raspberrypi_multicore_MPCtest_B.e_i) - 1];
          }

          for (raspberrypi_multicore_MPCtest_B.k = 0;
               raspberrypi_multicore_MPCtest_B.k < 12;
               raspberrypi_multicore_MPCtest_B.k++) {
            b_Mu1[raspberrypi_multicore_MPCtest_B.i_f + 252 *
              raspberrypi_multicore_MPCtest_B.k] =
              raspberrypi_multicore_MPCtest_B.c_Su1[(78 *
              raspberrypi_multicore_MPCtest_B.k +
              raspberrypi_multicore_MPCtest_B.e_i) - 1];
          }

          for (raspberrypi_multicore_MPCtest_B.k = 0;
               raspberrypi_multicore_MPCtest_B.k < 49;
               raspberrypi_multicore_MPCtest_B.k++) {
            b_Mv[raspberrypi_multicore_MPCtest_B.i_f + 252 *
              raspberrypi_multicore_MPCtest_B.k] =
              raspberrypi_multicore_MPCtest_B.c_Hv[(78 *
              raspberrypi_multicore_MPCtest_B.k +
              raspberrypi_multicore_MPCtest_B.e_i) - 1];
          }

          raspberrypi_multicore_MPCtest_B.i_f++;
        } else {
          exitg1 = true;
        }
      }
    }

    raspbe_mpc_customconstraintcoef(raspberrypi_multicore_MPCtest_B.c_SuJm,
      raspberrypi_multicore_MPCtest_B.c_Sx,
      raspberrypi_multicore_MPCtest_B.c_Su1,
      raspberrypi_multicore_MPCtest_B.c_Hv, b_C, Dv, b_Jm, b_Ecc, b_Fcc, b_Scc,
      b_Gcc, b_uoff, b_voff, b_yoff, raspberrypi_multicore_MPCtest_B.MuCC,
      raspberrypi_multicore_MPCtest_B.MvCC,
      raspberrypi_multicore_MPCtest_B.Mu1CC,
      raspberrypi_multicore_MPCtest_B.MxCC, &b_Mlim[28]);
    for (raspberrypi_multicore_MPCtest_B.k = 0;
         raspberrypi_multicore_MPCtest_B.k < 24;
         raspberrypi_multicore_MPCtest_B.k++) {
      for (raspberrypi_multicore_MPCtest_B.e_i = 0;
           raspberrypi_multicore_MPCtest_B.e_i < 224;
           raspberrypi_multicore_MPCtest_B.e_i++) {
        b_Ac[(raspberrypi_multicore_MPCtest_B.e_i + 252 *
              raspberrypi_multicore_MPCtest_B.k) + 28] =
          -raspberrypi_multicore_MPCtest_B.MuCC[224 *
          raspberrypi_multicore_MPCtest_B.k +
          raspberrypi_multicore_MPCtest_B.e_i];
      }
    }

    for (raspberrypi_multicore_MPCtest_B.k = 0;
         raspberrypi_multicore_MPCtest_B.k < 13;
         raspberrypi_multicore_MPCtest_B.k++) {
      memcpy(&b_Mx[raspberrypi_multicore_MPCtest_B.k * 252 + 28],
             &raspberrypi_multicore_MPCtest_B.MxCC[raspberrypi_multicore_MPCtest_B.k
             * 224], 224U * sizeof(real_T));
    }

    for (raspberrypi_multicore_MPCtest_B.k = 0;
         raspberrypi_multicore_MPCtest_B.k < 12;
         raspberrypi_multicore_MPCtest_B.k++) {
      memcpy(&b_Mu1[raspberrypi_multicore_MPCtest_B.k * 252 + 28],
             &raspberrypi_multicore_MPCtest_B.Mu1CC[raspberrypi_multicore_MPCtest_B.k
             * 224], 224U * sizeof(real_T));
    }

    for (raspberrypi_multicore_MPCtest_B.k = 0;
         raspberrypi_multicore_MPCtest_B.k < 49;
         raspberrypi_multicore_MPCtest_B.k++) {
      memcpy(&b_Mv[raspberrypi_multicore_MPCtest_B.k * 252 + 28],
             &raspberrypi_multicore_MPCtest_B.MvCC[raspberrypi_multicore_MPCtest_B.k
             * 224], 224U * sizeof(real_T));
    }

    memset(&raspberrypi_multicore_MPCtest_B.b_I[0], 0, 144U * sizeof(real_T));
    for (raspberrypi_multicore_MPCtest_B.k = 0;
         raspberrypi_multicore_MPCtest_B.k < 12;
         raspberrypi_multicore_MPCtest_B.k++) {
      raspberrypi_multicore_MPCtest_B.b_I[raspberrypi_multicore_MPCtest_B.k + 12
        * raspberrypi_multicore_MPCtest_B.k] = 1.0;
    }

    raspberrypi_multicore_MPCt_kron(c, raspberrypi_multicore_MPCtest_B.b_I,
      raspberrypi_multicore_MPCtest_B.dv);
    for (raspberrypi_multicore_MPCtest_B.k = 0;
         raspberrypi_multicore_MPCtest_B.k < 24;
         raspberrypi_multicore_MPCtest_B.k++) {
      memset
        (&raspberrypi_multicore_MPCtest_B.dv2[raspberrypi_multicore_MPCtest_B.k *
         72], 0, 72U * sizeof(real_T));
      for (raspberrypi_multicore_MPCtest_B.i_c = 0;
           raspberrypi_multicore_MPCtest_B.i_c < 72;
           raspberrypi_multicore_MPCtest_B.i_c++) {
        for (raspberrypi_multicore_MPCtest_B.e_i = 0;
             raspberrypi_multicore_MPCtest_B.e_i < 72;
             raspberrypi_multicore_MPCtest_B.e_i++) {
          raspberrypi_multicore_MPCtest_B.i_f = 72 *
            raspberrypi_multicore_MPCtest_B.k +
            raspberrypi_multicore_MPCtest_B.e_i;
          raspberrypi_multicore_MPCtest_B.dv2[raspberrypi_multicore_MPCtest_B.i_f]
            += raspberrypi_multicore_MPCtest_B.dv[72 *
            raspberrypi_multicore_MPCtest_B.i_c +
            raspberrypi_multicore_MPCtest_B.e_i] * b_Jm[72 *
            raspberrypi_multicore_MPCtest_B.k +
            raspberrypi_multicore_MPCtest_B.i_c];
        }
      }
    }

    raspberryp_mpc_calculatehessian(b_Wy, b_Wu, b_Wdu,
      raspberrypi_multicore_MPCtest_B.c_SuJm,
      raspberrypi_multicore_MPCtest_B.dv2, b_Jm, b_I1,
      raspberrypi_multicore_MPCtest_B.c_Su1,
      raspberrypi_multicore_MPCtest_B.c_Sx, raspberrypi_multicore_MPCtest_B.c_Hv,
      raspberrypi_multicore_MPCtes_nu, raspberrypi_multicore_MPCtes_ny,
      raspberrypi_multicore_MPCtest_B.b, raspberrypi_multicore_MPCtest_B.c_Ku1,
      raspberrypi_multicore_MPCtest_B.c_Kut,
      raspberrypi_multicore_MPCtest_B.c_Kx, raspberrypi_multicore_MPCtest_B.c_Kv,
      raspberrypi_multicore_MPCtest_B.c_Kr);
    for (raspberrypi_multicore_MPCtest_B.k = 0;
         raspberrypi_multicore_MPCtest_B.k < 24;
         raspberrypi_multicore_MPCtest_B.k++) {
      memcpy(&b_H[raspberrypi_multicore_MPCtest_B.k * 25],
             &raspberrypi_multicore_MPCtest_B.b[raspberrypi_multicore_MPCtest_B.k
             * 24], 24U * sizeof(real_T));
    }

    memcpy(&raspberrypi_multicore_MPCtest_B.c_Linv[0], &b_H[0], 625U * sizeof
           (real_T));
    raspberrypi_mu_mpc_checkhessian(raspberrypi_multicore_MPCtest_B.c_Linv,
      raspberrypi_multicore_MPCtest_B.d_Linv,
      &raspberrypi_multicore_MPCtest_B.BadH);
    if (raspberrypi_multicore_MPCtest_B.BadH > 1.0) {
      for (raspberrypi_multicore_MPCtest_B.i_f = 0;
           raspberrypi_multicore_MPCtest_B.i_f < 12;
           raspberrypi_multicore_MPCtest_B.i_f++) {
        u[raspberrypi_multicore_MPCtest_B.i_f] =
          old_u[raspberrypi_multicore_MPCtest_B.i_f] +
          b_uoff[raspberrypi_multicore_MPCtest_B.i_f];
        for (raspberrypi_multicore_MPCtest_B.k = 0;
             raspberrypi_multicore_MPCtest_B.k < 7;
             raspberrypi_multicore_MPCtest_B.k++) {
          useq[raspberrypi_multicore_MPCtest_B.k + 7 *
            raspberrypi_multicore_MPCtest_B.i_f] =
            u[raspberrypi_multicore_MPCtest_B.i_f];
        }
      }

      *status = -2.0;
    } else {
      memset(&raspberrypi_multicore_MPCtest_B.c_I[0], 0, 625U * sizeof(int8_T));
      for (raspberrypi_multicore_MPCtest_B.k = 0;
           raspberrypi_multicore_MPCtest_B.k < 25;
           raspberrypi_multicore_MPCtest_B.k++) {
        raspberrypi_multicore_MPCtest_B.c_I[raspberrypi_multicore_MPCtest_B.k +
          25 * raspberrypi_multicore_MPCtest_B.k] = 1;
      }

      for (raspberrypi_multicore_MPCtest_B.k = 0;
           raspberrypi_multicore_MPCtest_B.k < 25;
           raspberrypi_multicore_MPCtest_B.k++) {
        for (raspberrypi_multicore_MPCtest_B.e_i = 0;
             raspberrypi_multicore_MPCtest_B.e_i < 25;
             raspberrypi_multicore_MPCtest_B.e_i++) {
          raspberrypi_multicore_MPCtest_B.i_f = 25 *
            raspberrypi_multicore_MPCtest_B.k +
            raspberrypi_multicore_MPCtest_B.e_i;
          raspberrypi_multicore_MPCtest_B.c_Linv[raspberrypi_multicore_MPCtest_B.i_f]
            =
            raspberrypi_multicore_MPCtest_B.c_I[raspberrypi_multicore_MPCtest_B.i_f];
        }
      }

      raspberrypi_multicore__trisolve(raspberrypi_multicore_MPCtest_B.d_Linv,
        raspberrypi_multicore_MPCtest_B.c_Linv);
      for (raspberrypi_multicore_MPCtest_B.i_f = 0;
           raspberrypi_multicore_MPCtest_B.i_f < 252;
           raspberrypi_multicore_MPCtest_B.i_f++) {
        iAout[raspberrypi_multicore_MPCtest_B.i_f] =
          iA[raspberrypi_multicore_MPCtest_B.i_f];
        raspberrypi_multicore_MPCtest_B.BadH = 0.0;
        for (raspberrypi_multicore_MPCtest_B.k = 0;
             raspberrypi_multicore_MPCtest_B.k < 13;
             raspberrypi_multicore_MPCtest_B.k++) {
          raspberrypi_multicore_MPCtest_B.BadH += b_Mx[252 *
            raspberrypi_multicore_MPCtest_B.k +
            raspberrypi_multicore_MPCtest_B.i_f] *
            x[raspberrypi_multicore_MPCtest_B.k];
        }

        raspberrypi_multicore_MPCtest_B.b_Mlim_p[raspberrypi_multicore_MPCtest_B.i_f]
          = b_Mlim[raspberrypi_multicore_MPCtest_B.i_f] +
          raspberrypi_multicore_MPCtest_B.BadH;
        raspberrypi_multicore_MPCtest_B.b_Mu1[raspberrypi_multicore_MPCtest_B.i_f]
          = 0.0;
      }

      for (raspberrypi_multicore_MPCtest_B.k = 0;
           raspberrypi_multicore_MPCtest_B.k < 12;
           raspberrypi_multicore_MPCtest_B.k++) {
        for (raspberrypi_multicore_MPCtest_B.e_i = 0;
             raspberrypi_multicore_MPCtest_B.e_i < 252;
             raspberrypi_multicore_MPCtest_B.e_i++) {
          raspberrypi_multicore_MPCtest_B.b_Mu1[raspberrypi_multicore_MPCtest_B.e_i]
            += b_Mu1[252 * raspberrypi_multicore_MPCtest_B.k +
            raspberrypi_multicore_MPCtest_B.e_i] *
            old_u[raspberrypi_multicore_MPCtest_B.k];
        }
      }

      for (raspberrypi_multicore_MPCtest_B.k = 0;
           raspberrypi_multicore_MPCtest_B.k < 25;
           raspberrypi_multicore_MPCtest_B.k++) {
        for (raspberrypi_multicore_MPCtest_B.e_i = 0;
             raspberrypi_multicore_MPCtest_B.e_i < 25;
             raspberrypi_multicore_MPCtest_B.e_i++) {
          raspberrypi_multicore_MPCtest_B.i_f =
            raspberrypi_multicore_MPCtest_B.e_i + 25 *
            raspberrypi_multicore_MPCtest_B.k;
          raspberrypi_multicore_MPCtest_B.d_Linv[raspberrypi_multicore_MPCtest_B.i_f]
            = 0.0;
          for (raspberrypi_multicore_MPCtest_B.i_c = 0;
               raspberrypi_multicore_MPCtest_B.i_c < 25;
               raspberrypi_multicore_MPCtest_B.i_c++) {
            raspberrypi_multicore_MPCtest_B.d_Linv[raspberrypi_multicore_MPCtest_B.i_f]
              += raspberrypi_multicore_MPCtest_B.c_Linv[25 *
              raspberrypi_multicore_MPCtest_B.e_i +
              raspberrypi_multicore_MPCtest_B.i_c] *
              raspberrypi_multicore_MPCtest_B.c_Linv[25 *
              raspberrypi_multicore_MPCtest_B.k +
              raspberrypi_multicore_MPCtest_B.i_c];
          }
        }
      }

      for (raspberrypi_multicore_MPCtest_B.k = 0;
           raspberrypi_multicore_MPCtest_B.k < 252;
           raspberrypi_multicore_MPCtest_B.k++) {
        raspberrypi_multicore_MPCtest_B.BadH = 0.0;
        for (raspberrypi_multicore_MPCtest_B.e_i = 0;
             raspberrypi_multicore_MPCtest_B.e_i < 49;
             raspberrypi_multicore_MPCtest_B.e_i++) {
          raspberrypi_multicore_MPCtest_B.BadH += b_Mv[252 *
            raspberrypi_multicore_MPCtest_B.e_i +
            raspberrypi_multicore_MPCtest_B.k] *
            vseq[raspberrypi_multicore_MPCtest_B.e_i];
        }

        raspberrypi_multicore_MPCtest_B.b_Mlim_l[raspberrypi_multicore_MPCtest_B.k]
          =
          -((raspberrypi_multicore_MPCtest_B.b_Mlim_p[raspberrypi_multicore_MPCtest_B.k]
             + raspberrypi_multicore_MPCtest_B.b_Mu1[raspberrypi_multicore_MPCtest_B.k])
            + raspberrypi_multicore_MPCtest_B.BadH);
      }

      raspberrypi_multico_mpc_solveQP(x, raspberrypi_multicore_MPCtest_B.c_Kx,
        raspberrypi_multicore_MPCtest_B.c_Kr, rseq,
        raspberrypi_multicore_MPCtest_B.c_Ku1, old_u,
        raspberrypi_multicore_MPCtest_B.c_Kv, vseq,
        raspberrypi_multicore_MPCtest_B.c_Kut, b_utarget,
        raspberrypi_multicore_MPCtest_B.c_Linv,
        raspberrypi_multicore_MPCtest_B.d_Linv, b_Ac,
        raspberrypi_multicore_MPCtest_B.b_Mlim_l, iAout,
        raspberrypi_multicore_MPCtest_B.zopt, raspberrypi_multicore_MPCtest_B.f,
        status);
      for (raspberrypi_multicore_MPCtest_B.k = 0;
           raspberrypi_multicore_MPCtest_B.k < 12;
           raspberrypi_multicore_MPCtest_B.k++) {
        u[raspberrypi_multicore_MPCtest_B.k] =
          (old_u[raspberrypi_multicore_MPCtest_B.k] +
           raspberrypi_multicore_MPCtest_B.zopt[raspberrypi_multicore_MPCtest_B.k])
          + b_uoff[raspberrypi_multicore_MPCtest_B.k];
      }

      if (*status == 0.0) {
        for (raspberrypi_multicore_MPCtest_B.k = 0;
             raspberrypi_multicore_MPCtest_B.k < 12;
             raspberrypi_multicore_MPCtest_B.k++) {
          raspberrypi_multicore_MPCtest_B.old_u[raspberrypi_multicore_MPCtest_B.k]
            = old_u[raspberrypi_multicore_MPCtest_B.k] +
            b_uoff[raspberrypi_multicore_MPCtest_B.k];
        }

        memcpy(&raspberrypi_multicore_MPCtest_B.u[0], &u[0], 12U * sizeof(real_T));
        raspberrypi_multic_mpc_clipping(raspberrypi_multicore_MPCtest_B.u,
          raspberrypi_multicore_MPCtest_B.old_u, b_uoff, b_Mlim, b_Mrows,
          raspberrypi_multicore_MPCtest_p, raspberrypi_multicore_MPCtes_ny,
          raspberrypi_multicore_M_degrees, u);
      }

      memset(&raspberrypi_multicore_MPCtest_B.b_I[0], 0, 144U * sizeof(real_T));
      for (raspberrypi_multicore_MPCtest_B.k = 0;
           raspberrypi_multicore_MPCtest_B.k < 12;
           raspberrypi_multicore_MPCtest_B.k++) {
        raspberrypi_multicore_MPCtest_B.b_I[raspberrypi_multicore_MPCtest_B.k +
          12 * raspberrypi_multicore_MPCtest_B.k] = 1.0;
      }

      raspberrypi_multicore_MPCt_kron(c, raspberrypi_multicore_MPCtest_B.b_I,
        raspberrypi_multicore_MPCtest_B.dv);
      for (raspberrypi_multicore_MPCtest_B.k = 0;
           raspberrypi_multicore_MPCtest_B.k < 24;
           raspberrypi_multicore_MPCtest_B.k++) {
        memset
          (&raspberrypi_multicore_MPCtest_B.dv2[raspberrypi_multicore_MPCtest_B.k
           * 72], 0, 72U * sizeof(real_T));
        for (raspberrypi_multicore_MPCtest_B.i_c = 0;
             raspberrypi_multicore_MPCtest_B.i_c < 72;
             raspberrypi_multicore_MPCtest_B.i_c++) {
          for (raspberrypi_multicore_MPCtest_B.e_i = 0;
               raspberrypi_multicore_MPCtest_B.e_i < 72;
               raspberrypi_multicore_MPCtest_B.e_i++) {
            raspberrypi_multicore_MPCtest_B.i_f = 72 *
              raspberrypi_multicore_MPCtest_B.k +
              raspberrypi_multicore_MPCtest_B.e_i;
            raspberrypi_multicore_MPCtest_B.dv2[raspberrypi_multicore_MPCtest_B.i_f]
              += raspberrypi_multicore_MPCtest_B.dv[72 *
              raspberrypi_multicore_MPCtest_B.i_c +
              raspberrypi_multicore_MPCtest_B.e_i] * b_Jm[72 *
              raspberrypi_multicore_MPCtest_B.k +
              raspberrypi_multicore_MPCtest_B.i_c];
          }
        }
      }

      for (raspberrypi_multicore_MPCtest_B.k = 0;
           raspberrypi_multicore_MPCtest_B.k < 12;
           raspberrypi_multicore_MPCtest_B.k++) {
        raspberrypi_multicore_MPCtest_B.old_u[raspberrypi_multicore_MPCtest_B.k]
          = old_u[raspberrypi_multicore_MPCtest_B.k] +
          b_uoff[raspberrypi_multicore_MPCtest_B.k];
      }

      memset(&raspberrypi_multicore_MPCtest_B.dv13[0], 0, 72U * sizeof(real_T));
      for (raspberrypi_multicore_MPCtest_B.e_i = 0;
           raspberrypi_multicore_MPCtest_B.e_i < 24;
           raspberrypi_multicore_MPCtest_B.e_i++) {
        for (raspberrypi_multicore_MPCtest_B.k = 0;
             raspberrypi_multicore_MPCtest_B.k < 72;
             raspberrypi_multicore_MPCtest_B.k++) {
          raspberrypi_multicore_MPCtest_B.dv13[raspberrypi_multicore_MPCtest_B.k]
            += raspberrypi_multicore_MPCtest_B.dv2[72 *
            raspberrypi_multicore_MPCtest_B.e_i +
            raspberrypi_multicore_MPCtest_B.k] *
            raspberrypi_multicore_MPCtest_B.zopt[raspberrypi_multicore_MPCtest_B.e_i];
        }
      }

      memset(&raspberrypi_multicore_MPCtest_B.b_I1_j[0], 0, 72U * sizeof(real_T));
      for (raspberrypi_multicore_MPCtest_B.e_i = 0;
           raspberrypi_multicore_MPCtest_B.e_i < 12;
           raspberrypi_multicore_MPCtest_B.e_i++) {
        for (raspberrypi_multicore_MPCtest_B.k = 0;
             raspberrypi_multicore_MPCtest_B.k < 72;
             raspberrypi_multicore_MPCtest_B.k++) {
          raspberrypi_multicore_MPCtest_B.b_I1_j[raspberrypi_multicore_MPCtest_B.k]
            += b_I1[72 * raspberrypi_multicore_MPCtest_B.e_i +
            raspberrypi_multicore_MPCtest_B.k] *
            raspberrypi_multicore_MPCtest_B.old_u[raspberrypi_multicore_MPCtest_B.e_i];
        }
      }

      for (raspberrypi_multicore_MPCtest_B.k = 0;
           raspberrypi_multicore_MPCtest_B.k < 72;
           raspberrypi_multicore_MPCtest_B.k++) {
        raspberrypi_multicore_MPCtest_B.dv14[raspberrypi_multicore_MPCtest_B.k] =
          raspberrypi_multicore_MPCtest_B.dv13[raspberrypi_multicore_MPCtest_B.k]
          + raspberrypi_multicore_MPCtest_B.b_I1_j[raspberrypi_multicore_MPCtest_B.k];
      }

      for (raspberrypi_multicore_MPCtest_B.k = 0;
           raspberrypi_multicore_MPCtest_B.k < 6;
           raspberrypi_multicore_MPCtest_B.k++) {
        for (raspberrypi_multicore_MPCtest_B.e_i = 0;
             raspberrypi_multicore_MPCtest_B.e_i < 12;
             raspberrypi_multicore_MPCtest_B.e_i++) {
          useq[raspberrypi_multicore_MPCtest_B.k + 7 *
            raspberrypi_multicore_MPCtest_B.e_i] =
            raspberrypi_multicore_MPCtest_B.dv14[12 *
            raspberrypi_multicore_MPCtest_B.k +
            raspberrypi_multicore_MPCtest_B.e_i];
        }
      }

      for (raspberrypi_multicore_MPCtest_B.e_i = 0;
           raspberrypi_multicore_MPCtest_B.e_i < 12;
           raspberrypi_multicore_MPCtest_B.e_i++) {
        useq[7 * raspberrypi_multicore_MPCtest_B.e_i + 6] = useq[7 *
          raspberrypi_multicore_MPCtest_B.e_i + 5];
      }
    }
  }
}

/* Function for MATLAB Function: '<S46>/FixedHorizonOptimizer' */
static void raspberrypi_multicore_MP_repmat(const real_T a[13], real_T b[91])
{
  int32_T jtilecol;
  for (jtilecol = 0; jtilecol < 7; jtilecol++) {
    memcpy(&b[jtilecol * 13], &a[0], 13U * sizeof(real_T));
  }
}

/* Function for MATLAB Function: '<S46>/FixedHorizonOptimizer' */
static void raspberrypi_mpc_computeSequence(const real_T x[13], const real_T
  uopt[84], const real_T v[49], const real_T mvoff[12], const real_T b_yoff[13],
  const real_T b_xoff[13], const real_T b_A[169], const real_T Bu[1092], const
  real_T Bv[637], const real_T b_C[169], const real_T Dv[637], real_T yopt[91],
  real_T xopt[91])
{
  real_T tmp;
  int32_T b_k;
  int32_T i;
  int32_T ibmat;
  memcpy(&raspberrypi_multicore_MPCtest_B.xQP[0], &x[0], 13U * sizeof(real_T));
  for (i = 0; i < 6; i++) {
    ibmat = i * 12;
    for (b_k = 0; b_k < 12; b_k++) {
      raspberrypi_multicore_MPCtest_B.b_j[ibmat + b_k] = mvoff[b_k];
      raspberrypi_multicore_MPCtest_B.uopt_dim[i + 6 * b_k] = uopt[7 * b_k + i]
        - raspberrypi_multicore_MPCtest_B.b_j[12 * i + b_k];
    }
  }

  memset(&yopt[0], 0, 91U * sizeof(real_T));
  memset(&xopt[0], 0, 91U * sizeof(real_T));
  raspberrypi_multicore_MP_repmat(b_yoff, raspberrypi_multicore_MPCtest_B.dv12);
  for (ibmat = 0; ibmat < 7; ibmat++) {
    for (i = 0; i < 13; i++) {
      xopt[ibmat + 7 * i] = raspberrypi_multicore_MPCtest_B.xQP[i];
    }

    for (i = 0; i < 7; i++) {
      raspberrypi_multicore_MPCtest_B.vk[i] = v[ibmat * 7 + i];
    }

    for (i = 0; i < 13; i++) {
      raspberrypi_multicore_MPCtest_B.b_C_p[i] = 0.0;
      for (b_k = 0; b_k < 13; b_k++) {
        raspberrypi_multicore_MPCtest_B.b_C_p[i] += b_C[13 * b_k + i] *
          raspberrypi_multicore_MPCtest_B.xQP[b_k];
      }

      raspberrypi_multicore_MPCtest_B.Dv_a[i] = 0.0;
      for (b_k = 0; b_k < 7; b_k++) {
        raspberrypi_multicore_MPCtest_B.Dv_a[i] += Dv[13 * b_k + i] *
          raspberrypi_multicore_MPCtest_B.vk[b_k];
      }

      yopt[ibmat + 7 * i] = raspberrypi_multicore_MPCtest_B.b_C_p[i] +
        raspberrypi_multicore_MPCtest_B.Dv_a[i];
    }

    if (ibmat < 6) {
      for (i = 0; i < 13; i++) {
        raspberrypi_multicore_MPCtest_B.b_C_p[i] = 0.0;
        for (b_k = 0; b_k < 13; b_k++) {
          raspberrypi_multicore_MPCtest_B.b_C_p[i] += b_A[13 * b_k + i] *
            raspberrypi_multicore_MPCtest_B.xQP[b_k];
        }

        raspberrypi_multicore_MPCtest_B.Dv_a[i] = 0.0;
        for (b_k = 0; b_k < 12; b_k++) {
          raspberrypi_multicore_MPCtest_B.Dv_a[i] += Bu[13 * b_k + i] *
            raspberrypi_multicore_MPCtest_B.uopt_dim[6 * b_k + ibmat];
        }
      }

      for (i = 0; i < 13; i++) {
        tmp = 0.0;
        for (b_k = 0; b_k < 7; b_k++) {
          tmp += Bv[13 * b_k + i] * raspberrypi_multicore_MPCtest_B.vk[b_k];
        }

        raspberrypi_multicore_MPCtest_B.xQP[i] =
          (raspberrypi_multicore_MPCtest_B.b_C_p[i] +
           raspberrypi_multicore_MPCtest_B.Dv_a[i]) + tmp;
      }
    }

    for (i = 0; i < 13; i++) {
      b_k = 7 * i + ibmat;
      yopt[b_k] += raspberrypi_multicore_MPCtest_B.dv12[13 * ibmat + i];
    }
  }

  raspberrypi_multicore_MP_repmat(b_xoff, raspberrypi_multicore_MPCtest_B.dv12);
  for (i = 0; i < 7; i++) {
    for (b_k = 0; b_k < 13; b_k++) {
      ibmat = 7 * b_k + i;
      xopt[ibmat] += raspberrypi_multicore_MPCtest_B.dv12[13 * i + b_k];
    }
  }
}

/* Model step function for TID0 */
void raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis_step0(void) /* Sample time: [0.005s, 0.0s] */
{
  int_T tid = 0;
  int16_T temp;
  int8_T wrBufIdx;
  uint8_T resAngle[2];
  uint8_T status;
  boolean_T p;
  boolean_T rEQ0;
  boolean_T rtb_DigitalRead_0;
  boolean_T rtb_DigitalRead_k_0;
  boolean_T rtb_DigitalRead_p_0;
  boolean_T rtb_DigitalRead_pz_0;
  static const int8_T d[3] = { 1, 0, 0 };

  static const real_T tmp[12] = { 0.0, 0.0, 9.8, 0.0, 0.0, 9.8, 0.0, 0.0, 9.8,
    0.0, 0.0, 9.8 };

  static const int8_T tmp_0[3] = { 0, 0, 1 };

  static const real_T angleOff[12] = { -8.996, -0.7468, -7.452, -7.073, -1.056,
    4.301, 3.073, 5.056, 6.301, 1.877, -5.857, 2.0 };

  static const real_T tmp_1[81] = { 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.001 };

  static const real_T tmp_2[36] = { 2.14E-7, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.37E-7, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.57E-8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.002071, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0005364, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0003997 };

  real_T *rtb_MATLABSystem10_o5_0;
  boolean_T exitg1;

  {                                    /* Sample time: [0.005s, 0.0s] */
    rate_scheduler();
  }

  /* MATLABSystem: '<Root>/MATLAB System' */
  if (raspberrypi_multicore_MPCtes_DW.obj_e1.SampleTime !=
      raspberrypi_multicore_MPCtest_P.Ts_DynSim) {
    raspberrypi_multicore_MPCtes_DW.obj_e1.SampleTime =
      raspberrypi_multicore_MPCtest_P.Ts_DynSim;
  }

  /*         %% Define output properties */
  /*  Call C-function implementing device output */
  /* y = coder.ceval('source_output'); */
  eventRead();
  temp = returnJSInfo(&raspberrypi_multicore_MPCtest_B.b_varargout_2_c[0],
                      &raspberrypi_multicore_MPCtest_B.b_varargout_3_c[0]);
  returnButtonState();
  returnAxisState();

  /* MATLABSystem: '<Root>/MATLAB System3' incorporates:
   *  MATLABSystem: '<Root>/MATLAB System'
   */
  raspberrypi_multicore_MPCtest_B.MATLABSystem3_o1 =
    raspberrypi_multicore_MPCtes_DW.obj_a.XOld;
  raspberrypi_multicore_MPCtest_B.MATLABSystem3_o2_f =
    raspberrypi_multicore_MPCtes_DW.obj_a.YOld;
  raspberrypi_multicore_MPCtest_B.absx =
    raspberrypi_multicore_MPCtes_DW.obj_a.AOld;
  raspberrypi_multicore_MPCtest_B.b_varargout_4_a =
    raspberrypi_multicore_MPCtes_DW.obj_a.BOld;
  raspberrypi_multicore_MPCtest_B.Switch1 =
    raspberrypi_multicore_MPCtes_DW.obj_a.leftOld;
  raspberrypi_multicore_MPCtest_B.q =
    raspberrypi_multicore_MPCtes_DW.obj_a.rightOld;
  raspberrypi_multicore_MPCtest_B.b_varargout_5_p =
    raspberrypi_multicore_MPCtes_DW.obj_a.upOld;
  raspberrypi_multicore_MPCtest_B.v2_idx_1 =
    raspberrypi_multicore_MPCtes_DW.obj_a.downOld;
  raspberrypi_multicore_MPCtest_B.b_varargout_9_idx_0 =
    raspberrypi_multicore_MPCtes_DW.obj_a.LeftAxis[0];
  raspberrypi_multicore_MPCtest_B.b_varargout_10_idx_0 =
    raspberrypi_multicore_MPCtes_DW.obj_a.RightAxis[0];
  raspberrypi_multicore_MPCtest_B.b_varargout_9_idx_1 =
    raspberrypi_multicore_MPCtes_DW.obj_a.LeftAxis[1];
  raspberrypi_multicore_MPCtest_B.b_varargout_10_idx_1 =
    raspberrypi_multicore_MPCtes_DW.obj_a.RightAxis[1];
  if ((raspberrypi_multicore_MPCtes_DW.obj_a.count >= 10.0) && (temp > 0)) {
    raspberrypi_multicore_MPCtest_B.ar = temp + 1;
    if (temp + 1 > 32767) {
      raspberrypi_multicore_MPCtest_B.ar = 32767;
    }

    raspberrypi_multicore_MPCtest_B.c_h = raspberrypi_multicore_MPCtest_B.ar - 1;
    for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
         raspberrypi_multicore_MPCtest_B.i_f2 <=
         raspberrypi_multicore_MPCtest_B.c_h;
         raspberrypi_multicore_MPCtest_B.i_f2++) {
      switch (raspberrypi_multicore_MPCtest_B.b_varargout_3_c[(int16_T)
              (raspberrypi_multicore_MPCtest_B.i_f2 + 1) - 1]) {
       case 0:
        raspberrypi_multicore_MPCtest_B.ar =
          raspberrypi_multicore_MPCtest_B.b_varargout_2_c[(int16_T)
          (raspberrypi_multicore_MPCtest_B.i_f2 + 1) - 1];
        if (raspberrypi_multicore_MPCtest_B.ar == 1) {
          raspberrypi_multicore_MPCtest_B.absx = 1.0;
        } else if (raspberrypi_multicore_MPCtest_B.ar == 0) {
          raspberrypi_multicore_MPCtest_B.absx = 0.0;
        }

        raspberrypi_multicore_MPCtest_B.b_varargout_9_idx_1 = (real_T)
          raspberrypi_multicore_MPCtest_B.b_varargout_2_c[(int16_T)
          (raspberrypi_multicore_MPCtest_B.i_f2 + 1) - 1] / 32767.0;
        break;

       case 1:
        raspberrypi_multicore_MPCtest_B.ar =
          raspberrypi_multicore_MPCtest_B.b_varargout_2_c[(int16_T)
          (raspberrypi_multicore_MPCtest_B.i_f2 + 1) - 1];
        if (raspberrypi_multicore_MPCtest_B.ar == 1) {
          raspberrypi_multicore_MPCtest_B.b_varargout_4_a = 1.0;
        } else if (raspberrypi_multicore_MPCtest_B.ar == 0) {
          raspberrypi_multicore_MPCtest_B.b_varargout_4_a = 0.0;
        }

        raspberrypi_multicore_MPCtest_B.b_varargout_9_idx_0 = -(real_T)
          raspberrypi_multicore_MPCtest_B.b_varargout_2_c[(int16_T)
          (raspberrypi_multicore_MPCtest_B.i_f2 + 1) - 1] / 32767.0;
        break;

       case 2:
        raspberrypi_multicore_MPCtest_B.b_varargout_10_idx_1 = (real_T)
          raspberrypi_multicore_MPCtest_B.b_varargout_2_c[(int16_T)
          (raspberrypi_multicore_MPCtest_B.i_f2 + 1) - 1] / 32767.0;
        break;

       case 3:
        raspberrypi_multicore_MPCtest_B.ar =
          raspberrypi_multicore_MPCtest_B.b_varargout_2_c[(int16_T)
          (raspberrypi_multicore_MPCtest_B.i_f2 + 1) - 1];
        if (raspberrypi_multicore_MPCtest_B.ar == 1) {
          raspberrypi_multicore_MPCtest_B.MATLABSystem3_o1 = 1.0;
        } else if (raspberrypi_multicore_MPCtest_B.ar == 0) {
          raspberrypi_multicore_MPCtest_B.MATLABSystem3_o1 = 0.0;
        }

        raspberrypi_multicore_MPCtest_B.b_varargout_10_idx_0 = -(real_T)
          raspberrypi_multicore_MPCtest_B.b_varargout_2_c[(int16_T)
          (raspberrypi_multicore_MPCtest_B.i_f2 + 1) - 1] / 32767.0;
        break;

       case 4:
        raspberrypi_multicore_MPCtest_B.ar =
          raspberrypi_multicore_MPCtest_B.b_varargout_2_c[(int16_T)
          (raspberrypi_multicore_MPCtest_B.i_f2 + 1) - 1];
        if (raspberrypi_multicore_MPCtest_B.ar == 1) {
          raspberrypi_multicore_MPCtest_B.MATLABSystem3_o2_f = 1.0;
        } else if (raspberrypi_multicore_MPCtest_B.ar == 0) {
          raspberrypi_multicore_MPCtest_B.MATLABSystem3_o2_f = 0.0;
          raspberrypi_multicore_MPCtest_B.Switch1 = 0.0;
          raspberrypi_multicore_MPCtest_B.q = 0.0;
        } else if (raspberrypi_multicore_MPCtest_B.ar == -32767) {
          raspberrypi_multicore_MPCtest_B.Switch1 = 1.0;
        } else if (raspberrypi_multicore_MPCtest_B.ar == 32767) {
          raspberrypi_multicore_MPCtest_B.q = 1.0;
        }
        break;

       case 5:
        raspberrypi_multicore_MPCtest_B.ar =
          raspberrypi_multicore_MPCtest_B.b_varargout_2_c[(int16_T)
          (raspberrypi_multicore_MPCtest_B.i_f2 + 1) - 1];
        if (raspberrypi_multicore_MPCtest_B.ar == 32767) {
          raspberrypi_multicore_MPCtest_B.v2_idx_1 = 1.0;
        } else if (raspberrypi_multicore_MPCtest_B.ar == -32767) {
          raspberrypi_multicore_MPCtest_B.b_varargout_5_p = 1.0;
        } else if (raspberrypi_multicore_MPCtest_B.ar == 0) {
          raspberrypi_multicore_MPCtest_B.v2_idx_1 = 0.0;
          raspberrypi_multicore_MPCtest_B.b_varargout_5_p = 0.0;
        }
        break;
      }
    }
  }

  raspberrypi_multicore_MPCtes_DW.obj_a.XOld =
    raspberrypi_multicore_MPCtest_B.MATLABSystem3_o1;
  raspberrypi_multicore_MPCtes_DW.obj_a.YOld =
    raspberrypi_multicore_MPCtest_B.MATLABSystem3_o2_f;
  raspberrypi_multicore_MPCtes_DW.obj_a.AOld =
    raspberrypi_multicore_MPCtest_B.absx;
  raspberrypi_multicore_MPCtes_DW.obj_a.BOld =
    raspberrypi_multicore_MPCtest_B.b_varargout_4_a;
  raspberrypi_multicore_MPCtes_DW.obj_a.upOld =
    raspberrypi_multicore_MPCtest_B.b_varargout_5_p;
  raspberrypi_multicore_MPCtes_DW.obj_a.downOld =
    raspberrypi_multicore_MPCtest_B.v2_idx_1;
  raspberrypi_multicore_MPCtes_DW.obj_a.leftOld =
    raspberrypi_multicore_MPCtest_B.Switch1;
  raspberrypi_multicore_MPCtes_DW.obj_a.rightOld =
    raspberrypi_multicore_MPCtest_B.q;
  raspberrypi_multicore_MPCtes_DW.obj_a.LeftAxis[0] =
    raspberrypi_multicore_MPCtest_B.b_varargout_9_idx_0;
  raspberrypi_multicore_MPCtes_DW.obj_a.RightAxis[0] =
    raspberrypi_multicore_MPCtest_B.b_varargout_10_idx_0;
  raspberrypi_multicore_MPCtes_DW.obj_a.LeftAxis[1] =
    raspberrypi_multicore_MPCtest_B.b_varargout_9_idx_1;
  raspberrypi_multicore_MPCtes_DW.obj_a.RightAxis[1] =
    raspberrypi_multicore_MPCtest_B.b_varargout_10_idx_1;
  raspberrypi_multicore_MPCtes_DW.obj_a.count++;
  if (raspberrypi_multicore_MPCtes_DW.obj_a.count >= 60.0) {
    raspberrypi_multicore_MPCtes_DW.obj_a.count = 60.0;
  }

  raspberrypi_m_MATLABSystem4(raspberrypi_multicore_MPCtest_B.MATLABSystem3_o1,
    &raspberrypi_multicore_MPCtest_B.MATLABSystem4,
    &raspberrypi_multicore_MPCtes_DW.MATLABSystem4);
  raspberrypi_m_MATLABSystem4(raspberrypi_multicore_MPCtest_B.MATLABSystem3_o2_f,
    &raspberrypi_multicore_MPCtest_B.MATLABSystem9,
    &raspberrypi_multicore_MPCtes_DW.MATLABSystem9);

  /* Chart: '<Root>/Chart' */
  if (raspberrypi_multicore_MPCtes_DW.temporalCounter_i1 < 255U) {
    raspberrypi_multicore_MPCtes_DW.temporalCounter_i1++;
  }

  raspberrypi_multicore_MPCtes_DW.sfEvent = -1;
  if (raspberrypi_multicore_MPCtes_DW.is_active_c12_raspberrypi_multi == 0U) {
    raspberrypi_multicore_MPCtes_DW.is_active_c12_raspberrypi_multi = 1U;
    raspberrypi_multicore_MPCtes_DW.is_c12_raspberrypi_multicore_MP =
      raspberrypi_multicor_IN_StandBy;
    raspberrypi_multicore_MPCtest_B.estEN = 0;
    raspberrypi_multicore_MPCtest_B.oscEN = 0.0;
    raspberrypi_multicore_MPCtest_B.mpcSTOP = 1.0;
  } else {
    switch (raspberrypi_multicore_MPCtes_DW.is_c12_raspberrypi_multicore_MP) {
     case raspberrypi_mu_IN_MPC_enable_p1:
      raspberrypi_multicore_MPCtest_B.estEN = 1;
      raspberrypi_multicore_MPCtest_B.oscEN = 0.0;
      raspberrypi_multicore_MPCtest_B.mpcSTOP = 1.0;
      if (raspberrypi_multicore_MPCtes_DW.temporalCounter_i1 >= 200U) {
        raspberrypi_multicore_MPCtes_DW.is_c12_raspberrypi_multicore_MP =
          raspberrypi_mu_IN_MPC_enable_p2;
        raspberrypi_multicore_MPCtest_B.estEN = 0;
        raspberrypi_multicore_MPCtest_B.oscEN = 0.0;
        raspberrypi_multicore_MPCtest_B.mpcSTOP = 0.0;
      }
      break;

     case raspberrypi_mu_IN_MPC_enable_p2:
      raspberrypi_multicore_MPCtest_B.estEN = 0;
      raspberrypi_multicore_MPCtest_B.oscEN = 0.0;
      raspberrypi_multicore_MPCtest_B.mpcSTOP = 0.0;
      if (raspberrypi_multicore_MPCtest_B.MATLABSystem4.MATLABSystem4 == 1.0) {
        raspberrypi_multicore_MPCtes_DW.is_c12_raspberrypi_multicore_MP =
          raspberrypi_mult_IN_step_enable;
        raspberrypi_multicore_MPCtest_B.oscEN = 1.0;
        raspberrypi_multicore_MPCtest_B.mpcSTOP = 0.0;
      }
      break;

     case raspberrypi_multicor_IN_StandBy:
      raspberrypi_multicore_MPCtest_B.estEN = 0;
      raspberrypi_multicore_MPCtest_B.oscEN = 0.0;
      raspberrypi_multicore_MPCtest_B.mpcSTOP = 1.0;
      if (raspberrypi_multicore_MPCtest_B.MATLABSystem9.MATLABSystem4 == 1.0) {
        raspberrypi_multicore_MPCtes_DW.is_c12_raspberrypi_multicore_MP =
          raspberrypi_mu_IN_MPC_enable_p1;
        raspberrypi_multicore_MPCtes_DW.temporalCounter_i1 = 0U;
        raspberrypi_multicore_MPCtest_B.estEN = 1;
        raspberrypi_multicore_MPCtest_B.oscEN = 0.0;
        raspberrypi_multicore_MPCtest_B.mpcSTOP = 1.0;
      }
      break;

     default:
      /* case IN_step_enable: */
      raspberrypi_multicore_MPCtest_B.estEN = 0;
      raspberrypi_multicore_MPCtest_B.oscEN = 1.0;
      raspberrypi_multicore_MPCtest_B.mpcSTOP = 0.0;
      if (raspberrypi_multicore_MPCtest_B.MATLABSystem4.MATLABSystem4 == 1.0) {
        raspberrypi_multicore_MPCtes_DW.is_c12_raspberrypi_multicore_MP =
          raspberrypi_multicor_IN_StandBy;
        raspberrypi_multicore_MPCtest_B.oscEN = 0.0;
        raspberrypi_multicore_MPCtest_B.mpcSTOP = 1.0;
      }
      break;
    }
  }

  /* End of Chart: '<Root>/Chart' */

  /* RateTransition: '<Root>/Rate Transition' */
  rtw_pthread_mutex_lock
    (raspberrypi_multicore_MPCtes_DW.RateTransition_d0_SEMAPHORE);
  raspberrypi_multicore_MPCtes_DW.RateTransition_RDBuf =
    raspberrypi_multicore_MPCtes_DW.RateTransition_LstBufWR;
  rtw_pthread_mutex_unlock
    (raspberrypi_multicore_MPCtes_DW.RateTransition_d0_SEMAPHORE);
  raspberrypi_multicore_MPCtest_B.ar =
    raspberrypi_multicore_MPCtes_DW.RateTransition_RDBuf * 67;
  for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
       raspberrypi_multicore_MPCtest_B.i_f2 < 67;
       raspberrypi_multicore_MPCtest_B.i_f2++) {
    raspberrypi_multicore_MPCtest_B.RateTransition[raspberrypi_multicore_MPCtest_B.i_f2]
      =
      raspberrypi_multicore_MPCtes_DW.RateTransition_Buf[raspberrypi_multicore_MPCtest_B.i_f2
      + raspberrypi_multicore_MPCtest_B.ar];
  }

  /* End of RateTransition: '<Root>/Rate Transition' */

  /* MATLABSystem: '<Root>/MATLAB System10' incorporates:
   *  RateTransition: '<Root>/Rate Transition'
   */
  rEQ0 = false;
  p = true;
  raspberrypi_multicore_MPCtest_B.ar = 0;
  exitg1 = false;
  while ((!exitg1) && (raspberrypi_multicore_MPCtest_B.ar < 9)) {
    if (!(raspberrypi_multicore_MPCtes_DW.obj_cy.Inorm[raspberrypi_multicore_MPCtest_B.ar]
          ==
          raspberrypi_multicore_MPCtest_P.MATLABSystem10_Inorm[raspberrypi_multicore_MPCtest_B.ar]))
    {
      p = false;
      exitg1 = true;
    } else {
      raspberrypi_multicore_MPCtest_B.ar++;
    }
  }

  if (p) {
    rEQ0 = true;
  }

  if (!rEQ0) {
    memcpy(&raspberrypi_multicore_MPCtes_DW.obj_cy.Inorm[0],
           &raspberrypi_multicore_MPCtest_P.MATLABSystem10_Inorm[0], 9U * sizeof
           (real_T));
  }

  if (raspberrypi_multicore_MPCtes_DW.obj_cy.m !=
      raspberrypi_multicore_MPCtest_P.MATLABSystem10_m) {
    raspberrypi_multicore_MPCtes_DW.obj_cy.m =
      raspberrypi_multicore_MPCtest_P.MATLABSystem10_m;
  }

  if (raspberrypi_multicore_MPCtes_DW.obj_cy.hIni !=
      raspberrypi_multicore_MPCtest_P.MATLABSystem10_hIni) {
    raspberrypi_multicore_MPCtes_DW.obj_cy.hIni =
      raspberrypi_multicore_MPCtest_P.MATLABSystem10_hIni;
  }

  if (raspberrypi_multicore_MPCtes_DW.obj_cy.lateral_width !=
      raspberrypi_multicore_MPCtest_P.MATLABSystem10_lateral_width) {
    raspberrypi_multicore_MPCtes_DW.obj_cy.lateral_width =
      raspberrypi_multicore_MPCtest_P.MATLABSystem10_lateral_width;
  }

  if (raspberrypi_multicore_MPCtes_DW.obj_cy.sagetial_width !=
      raspberrypi_multicore_MPCtest_P.MATLABSystem10_sagetial_width) {
    raspberrypi_multicore_MPCtes_DW.obj_cy.sagetial_width =
      raspberrypi_multicore_MPCtest_P.MATLABSystem10_sagetial_width;
  }

  if (raspberrypi_multicore_MPCtes_DW.obj_cy.roll_Off !=
      raspberrypi_multicore_MPCtest_P.MATLABSystem10_roll_Off) {
    raspberrypi_multicore_MPCtes_DW.obj_cy.roll_Off =
      raspberrypi_multicore_MPCtest_P.MATLABSystem10_roll_Off;
  }

  if (fabs(raspberrypi_multicore_MPCtest_B.RateTransition[0] - 66.0) < 0.1) {
    for (raspberrypi_multicore_MPCtest_B.ar = 0;
         raspberrypi_multicore_MPCtest_B.ar < 12;
         raspberrypi_multicore_MPCtest_B.ar++) {
      raspberrypi_multicore_MPCtes_DW.obj_cy.U_Old[raspberrypi_multicore_MPCtest_B.ar]
        = raspberrypi_multicore_MPCtest_B.RateTransition
        [(raspberrypi_multicore_MPCtest_B.ar + 2) - 1];
    }

    for (raspberrypi_multicore_MPCtest_B.ar = 0;
         raspberrypi_multicore_MPCtest_B.ar < 13;
         raspberrypi_multicore_MPCtest_B.ar++) {
      raspberrypi_multicore_MPCtes_DW.obj_cy.X_mpc_Old[raspberrypi_multicore_MPCtest_B.ar]
        = raspberrypi_multicore_MPCtest_B.RateTransition
        [(raspberrypi_multicore_MPCtest_B.ar + 14) - 1];
    }

    memcpy(&raspberrypi_multicore_MPCtes_DW.obj_cy.Inow_Old[0],
           &raspberrypi_multicore_MPCtest_B.RateTransition[26], 9U * sizeof
           (real_T));
    for (raspberrypi_multicore_MPCtest_B.ar = 0;
         raspberrypi_multicore_MPCtest_B.ar < 13;
         raspberrypi_multicore_MPCtest_B.ar++) {
      raspberrypi_multicore_MPCtest_B.b_varargout_4[raspberrypi_multicore_MPCtest_B.ar]
        = raspberrypi_multicore_MPCtest_B.RateTransition
        [(raspberrypi_multicore_MPCtest_B.ar + 36) - 1];
    }

    for (raspberrypi_multicore_MPCtest_B.ar = 0;
         raspberrypi_multicore_MPCtest_B.ar < 12;
         raspberrypi_multicore_MPCtest_B.ar++) {
      raspberrypi_multicore_MPCtes_DW.obj_cy.SP_MPC_Old[raspberrypi_multicore_MPCtest_B.ar]
        = raspberrypi_multicore_MPCtest_B.RateTransition
        [(raspberrypi_multicore_MPCtest_B.ar + 49) - 1];
    }

    raspberrypi_multicore_MPCtest_B.Switch1 =
      raspberrypi_multicore_MPCtest_B.RateTransition[61];
    raspberrypi_multicore_MPCtest_B.pyNew[0] =
      raspberrypi_multicore_MPCtest_B.RateTransition[62];
    raspberrypi_multicore_MPCtest_B.pyNew[1] =
      raspberrypi_multicore_MPCtest_B.RateTransition[63];
    raspberrypi_multicore_MPCtest_B.pyNew[2] =
      raspberrypi_multicore_MPCtest_B.RateTransition[64];
    raspberrypi_multicore_MPCtest_B.pyNew[3] =
      raspberrypi_multicore_MPCtest_B.RateTransition[65];
    raspberrypi_multicore_MPCtest_B.absx =
      raspberrypi_multicore_MPCtest_B.RateTransition[66];
  } else {
    memcpy(&raspberrypi_multicore_MPCtest_B.b_varargout_4[0],
           &raspberrypi_multicore_MPCtes_DW.obj_cy.refP_Old[0], 13U * sizeof
           (real_T));
    raspberrypi_multicore_MPCtest_B.Switch1 =
      raspberrypi_multicore_MPCtes_DW.obj_cy.MPC_Count_Old;
    raspberrypi_multicore_MPCtest_B.pyNew[0] =
      raspberrypi_multicore_MPCtes_DW.obj_cy.LegStateMPC_Old[0];
    raspberrypi_multicore_MPCtest_B.pyNew[1] =
      raspberrypi_multicore_MPCtes_DW.obj_cy.LegStateMPC_Old[1];
    raspberrypi_multicore_MPCtest_B.pyNew[2] =
      raspberrypi_multicore_MPCtes_DW.obj_cy.LegStateMPC_Old[2];
    raspberrypi_multicore_MPCtest_B.pyNew[3] =
      raspberrypi_multicore_MPCtes_DW.obj_cy.LegStateMPC_Old[3];
    raspberrypi_multicore_MPCtest_B.absx =
      raspberrypi_multicore_MPCtes_DW.obj_cy.phiSlow_Old;
  }

  memcpy(&raspberrypi_multicore_MPCtes_DW.obj_cy.refP_Old[0],
         &raspberrypi_multicore_MPCtest_B.b_varargout_4[0], 13U * sizeof(real_T));
  raspberrypi_multicore_MPCtes_DW.obj_cy.MPC_Count_Old =
    raspberrypi_multicore_MPCtest_B.Switch1;
  raspberrypi_multicore_MPCtes_DW.obj_cy.LegStateMPC_Old[0] =
    raspberrypi_multicore_MPCtest_B.pyNew[0];
  raspberrypi_multicore_MPCtes_DW.obj_cy.LegStateMPC_Old[1] =
    raspberrypi_multicore_MPCtest_B.pyNew[1];
  raspberrypi_multicore_MPCtes_DW.obj_cy.LegStateMPC_Old[2] =
    raspberrypi_multicore_MPCtest_B.pyNew[2];
  raspberrypi_multicore_MPCtes_DW.obj_cy.LegStateMPC_Old[3] =
    raspberrypi_multicore_MPCtest_B.pyNew[3];
  raspberrypi_multicore_MPCtes_DW.obj_cy.phiSlow_Old =
    raspberrypi_multicore_MPCtest_B.absx;
  memcpy(&raspberrypi_multicore_MPCtest_B.MATLABSystem10_o1[0],
         &raspberrypi_multicore_MPCtes_DW.obj_cy.U_Old[0], 12U * sizeof(real_T));
  rtb_MATLABSystem10_o5_0 = &raspberrypi_multicore_MPCtes_DW.obj_cy.SP_MPC_Old[0];

  /* MATLABSystem: '<Root>/MATLAB System8' incorporates:
   *  MATLABSystem: '<Root>/MATLAB System3'
   */
  if ((raspberrypi_multicore_MPCtest_B.b_varargout_4_a > 0.5) ||
      (raspberrypi_multicore_MPCtes_DW.obj_d.ESOld > 0.5)) {
    raspberrypi_multicore_MPCtest_B.ar = 1;
  } else {
    raspberrypi_multicore_MPCtest_B.ar = 0;
  }

  raspberrypi_multicore_MPCtes_DW.obj_d.ESOld =
    raspberrypi_multicore_MPCtest_B.ar;

  /* MATLABSystem: '<Root>/MATLAB System8' */
  raspberrypi_multicore_MPCtest_B.MATLABSystem8 =
    raspberrypi_multicore_MPCtest_B.ar;

  /* MATLABSystem: '<Root>/MATLAB System11' incorporates:
   *  Constant: '<Root>/Constant22'
   *  Constant: '<Root>/Constant23'
   *  Product: '<Root>/Divide2'
   */
  raspberrypi_multicore_MPCtest_B.b_varargout_4_a =
    raspberrypi_multicore_MPCtest_P.Constant22_Value /
    raspberrypi_multicore_MPCtest_P.T_gait * 0.005 *
    raspberrypi_multicore_MPCtes_DW.obj_iw.tCount;
  raspberrypi_multicore_MPCtest_B.absx =
    raspberrypi_multicore_MPCtest_B.b_varargout_4_a;
  if (rtIsNaN(raspberrypi_multicore_MPCtest_B.b_varargout_4_a) || rtIsInf
      (raspberrypi_multicore_MPCtest_B.b_varargout_4_a)) {
    raspberrypi_multicore_MPCtest_B.b_varargout_4_a = (rtNaN);
  } else if (raspberrypi_multicore_MPCtest_B.b_varargout_4_a == 0.0) {
    raspberrypi_multicore_MPCtest_B.b_varargout_4_a = 0.0;
  } else {
    raspberrypi_multicore_MPCtest_B.b_varargout_4_a = fmod
      (raspberrypi_multicore_MPCtest_B.b_varargout_4_a, 6.2831853071795862);
    rEQ0 = (raspberrypi_multicore_MPCtest_B.b_varargout_4_a == 0.0);
    if (!rEQ0) {
      raspberrypi_multicore_MPCtest_B.q = fabs
        (raspberrypi_multicore_MPCtest_B.absx / 6.2831853071795862);
      rEQ0 = !(fabs(raspberrypi_multicore_MPCtest_B.q - floor
                    (raspberrypi_multicore_MPCtest_B.q + 0.5)) >
               2.2204460492503131E-16 * raspberrypi_multicore_MPCtest_B.q);
    }

    if (rEQ0) {
      raspberrypi_multicore_MPCtest_B.b_varargout_4_a = 0.0;
    } else if (raspberrypi_multicore_MPCtest_B.absx < 0.0) {
      raspberrypi_multicore_MPCtest_B.b_varargout_4_a += 6.2831853071795862;
    }
  }

  /* MATLABSystem: '<Root>/MATLAB System11' */
  raspberrypi_multicore_MPCtest_B.MATLABSystem11_o2 = 0.0;

  /* MATLABSystem: '<Root>/MATLAB System11' */
  if ((raspberrypi_multicore_MPCtest_B.oscEN > 0.5) ||
      ((raspberrypi_multicore_MPCtest_B.oscEN < 0.5) &&
       (raspberrypi_multicore_MPCtest_B.b_varargout_4_a > 0.08))) {
    raspberrypi_multicore_MPCtes_DW.obj_iw.tCount++;
  }

  if ((raspberrypi_multicore_MPCtest_B.oscEN < 0.5) &&
      (raspberrypi_multicore_MPCtest_B.b_varargout_4_a < 0.08)) {
    /* MATLABSystem: '<Root>/MATLAB System11' */
    raspberrypi_multicore_MPCtest_B.MATLABSystem11_o2 = 1.0;
  }

  /* MATLAB Function: '<Root>/MATLAB Function2' incorporates:
   *  Constant: '<Root>/Constant8'
   *  MATLABSystem: '<Root>/MATLAB System17'
   *  UnitDelay: '<Root>/Unit Delay5'
   *  UnitDelay: '<Root>/Unit Delay6'
   */
  for (raspberrypi_multicore_MPCtest_B.ar = 0;
       raspberrypi_multicore_MPCtest_B.ar < 4;
       raspberrypi_multicore_MPCtest_B.ar++) {
    raspberrypi_multicore_MPCtest_B.W[raspberrypi_multicore_MPCtest_B.ar] = 1.0;
    raspberrypi_multicore_MPCtest_B.W[raspberrypi_multicore_MPCtest_B.ar + 4] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay5_DSTATE[3 *
      raspberrypi_multicore_MPCtest_B.ar];
    raspberrypi_multicore_MPCtest_B.i_f2 = 3 *
      raspberrypi_multicore_MPCtest_B.ar + 1;
    raspberrypi_multicore_MPCtest_B.W[raspberrypi_multicore_MPCtest_B.ar + 8] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay5_DSTATE[raspberrypi_multicore_MPCtest_B.i_f2];
    raspberrypi_multicore_MPCtest_B.pArray_L_Adm[3 *
      raspberrypi_multicore_MPCtest_B.ar] =
      raspberrypi_multicore_MPCtest_B.W[raspberrypi_multicore_MPCtest_B.ar];
    raspberrypi_multicore_MPCtest_B.pArray_L_Adm[raspberrypi_multicore_MPCtest_B.i_f2]
      = raspberrypi_multicore_MPCtest_B.W[raspberrypi_multicore_MPCtest_B.ar + 4];
    raspberrypi_multicore_MPCtest_B.pArray_L_Adm[3 *
      raspberrypi_multicore_MPCtest_B.ar + 2] =
      raspberrypi_multicore_MPCtest_B.W[raspberrypi_multicore_MPCtest_B.ar + 8];
  }

  for (raspberrypi_multicore_MPCtest_B.ar = 0;
       raspberrypi_multicore_MPCtest_B.ar < 3;
       raspberrypi_multicore_MPCtest_B.ar++) {
    for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
         raspberrypi_multicore_MPCtest_B.i_f2 < 3;
         raspberrypi_multicore_MPCtest_B.i_f2++) {
      raspberrypi_multicore_MPCtest_B.c_h = raspberrypi_multicore_MPCtest_B.i_f2
        + 3 * raspberrypi_multicore_MPCtest_B.ar;
      raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.c_h] =
        0.0;
      raspberrypi_multicore_MPCtest_B.ia = raspberrypi_multicore_MPCtest_B.ar <<
        2;
      raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.c_h] +=
        raspberrypi_multicore_MPCtest_B.W[raspberrypi_multicore_MPCtest_B.ia] *
        raspberrypi_multicore_MPCtest_B.pArray_L_Adm[raspberrypi_multicore_MPCtest_B.i_f2];
      raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.c_h] +=
        raspberrypi_multicore_MPCtest_B.W[raspberrypi_multicore_MPCtest_B.ia + 1]
        * raspberrypi_multicore_MPCtest_B.pArray_L_Adm[raspberrypi_multicore_MPCtest_B.i_f2
        + 3];
      raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.c_h] +=
        raspberrypi_multicore_MPCtest_B.W[raspberrypi_multicore_MPCtest_B.ia + 2]
        * raspberrypi_multicore_MPCtest_B.pArray_L_Adm[raspberrypi_multicore_MPCtest_B.i_f2
        + 6];
      raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.c_h] +=
        raspberrypi_multicore_MPCtest_B.W[raspberrypi_multicore_MPCtest_B.ia + 3]
        * raspberrypi_multicore_MPCtest_B.pArray_L_Adm[raspberrypi_multicore_MPCtest_B.i_f2
        + 9];
    }
  }

  rEQ0 = true;
  for (raspberrypi_multicore_MPCtest_B.ar = 0;
       raspberrypi_multicore_MPCtest_B.ar < 9;
       raspberrypi_multicore_MPCtest_B.ar++) {
    raspberrypi_multicore_MPCtest_B.absx =
      raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.ar];
    raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar] =
      0.0;
    if (rEQ0 && ((!rtIsInf(raspberrypi_multicore_MPCtest_B.absx)) && (!rtIsNaN
          (raspberrypi_multicore_MPCtest_B.absx)))) {
    } else {
      rEQ0 = false;
    }
  }

  if (!rEQ0) {
    for (raspberrypi_multicore_MPCtest_B.ar = 0;
         raspberrypi_multicore_MPCtest_B.ar < 9;
         raspberrypi_multicore_MPCtest_B.ar++) {
      raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar] =
        (rtNaN);
    }
  } else {
    raspberrypi_multicore_MPCte_svd(raspberrypi_multicore_MPCtest_B.R,
      raspberrypi_multicore_MPCtest_B.U_a, raspberrypi_multicore_MPCtest_B.a,
      raspberrypi_multicore_MPCtest_B.V);
    raspberrypi_multicore_MPCtest_B.absx = fabs
      (raspberrypi_multicore_MPCtest_B.a[0]);
    if ((!rtIsInf(raspberrypi_multicore_MPCtest_B.absx)) && (!rtIsNaN
         (raspberrypi_multicore_MPCtest_B.absx))) {
      if (raspberrypi_multicore_MPCtest_B.absx <= 2.2250738585072014E-308) {
        raspberrypi_multicore_MPCtest_B.absx = 4.94065645841247E-324;
      } else {
        frexp(raspberrypi_multicore_MPCtest_B.absx,
              &raspberrypi_multicore_MPCtest_B.vcol);
        raspberrypi_multicore_MPCtest_B.absx = ldexp(1.0,
          raspberrypi_multicore_MPCtest_B.vcol - 53);
      }
    } else {
      raspberrypi_multicore_MPCtest_B.absx = (rtNaN);
    }

    raspberrypi_multicore_MPCtest_B.absx *= 3.0;
    raspberrypi_multicore_MPCtest_B.i_f2 = -1;
    raspberrypi_multicore_MPCtest_B.ar = 0;
    while ((raspberrypi_multicore_MPCtest_B.ar < 3) &&
           (raspberrypi_multicore_MPCtest_B.a[raspberrypi_multicore_MPCtest_B.ar]
            > raspberrypi_multicore_MPCtest_B.absx)) {
      raspberrypi_multicore_MPCtest_B.i_f2++;
      raspberrypi_multicore_MPCtest_B.ar++;
    }

    if (raspberrypi_multicore_MPCtest_B.i_f2 + 1 > 0) {
      raspberrypi_multicore_MPCtest_B.vcol = 0;
      raspberrypi_multicore_MPCtest_B.c_h = 0;
      while (raspberrypi_multicore_MPCtest_B.c_h <=
             raspberrypi_multicore_MPCtest_B.i_f2) {
        raspberrypi_multicore_MPCtest_B.absx = 1.0 /
          raspberrypi_multicore_MPCtest_B.a[raspberrypi_multicore_MPCtest_B.c_h];
        raspberrypi_multicore_MPCtest_B.ar =
          raspberrypi_multicore_MPCtest_B.vcol;
        while (raspberrypi_multicore_MPCtest_B.ar + 1 <=
               raspberrypi_multicore_MPCtest_B.vcol + 3) {
          raspberrypi_multicore_MPCtest_B.V[raspberrypi_multicore_MPCtest_B.ar] *=
            raspberrypi_multicore_MPCtest_B.absx;
          raspberrypi_multicore_MPCtest_B.ar++;
        }

        raspberrypi_multicore_MPCtest_B.vcol += 3;
        raspberrypi_multicore_MPCtest_B.c_h++;
      }

      raspberrypi_multicore_MPCtest_B.ar = 0;
      while (raspberrypi_multicore_MPCtest_B.ar + 1 <= 3) {
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar] =
          0.0;
        raspberrypi_multicore_MPCtest_B.ar++;
      }

      raspberrypi_multicore_MPCtest_B.ar = 3;
      while (raspberrypi_multicore_MPCtest_B.ar + 1 <= 6) {
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar] =
          0.0;
        raspberrypi_multicore_MPCtest_B.ar++;
      }

      raspberrypi_multicore_MPCtest_B.ar = 6;
      while (raspberrypi_multicore_MPCtest_B.ar + 1 <= 9) {
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar] =
          0.0;
        raspberrypi_multicore_MPCtest_B.ar++;
      }

      raspberrypi_multicore_MPCtest_B.ar = -1;
      raspberrypi_multicore_MPCtest_B.c_h = 3 *
        raspberrypi_multicore_MPCtest_B.i_f2 + 1;
      raspberrypi_multicore_MPCtest_B.vcol = 1;
      while (raspberrypi_multicore_MPCtest_B.vcol <=
             raspberrypi_multicore_MPCtest_B.c_h) {
        raspberrypi_multicore_MPCtest_B.ia = raspberrypi_multicore_MPCtest_B.ar;
        raspberrypi_multicore_MPCtest_B.b_ic = 0;
        while (raspberrypi_multicore_MPCtest_B.b_ic + 1 <= 3) {
          raspberrypi_multicore_MPCtest_B.ia++;
          raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.b_ic]
            +=
            raspberrypi_multicore_MPCtest_B.U_a[raspberrypi_multicore_MPCtest_B.vcol
            - 1] *
            raspberrypi_multicore_MPCtest_B.V[raspberrypi_multicore_MPCtest_B.ia];
          raspberrypi_multicore_MPCtest_B.b_ic++;
        }

        raspberrypi_multicore_MPCtest_B.ar += 3;
        raspberrypi_multicore_MPCtest_B.vcol += 3;
      }

      raspberrypi_multicore_MPCtest_B.ar = -1;
      raspberrypi_multicore_MPCtest_B.c_h = 3 *
        raspberrypi_multicore_MPCtest_B.i_f2 + 2;
      raspberrypi_multicore_MPCtest_B.vcol = 2;
      while (raspberrypi_multicore_MPCtest_B.vcol <=
             raspberrypi_multicore_MPCtest_B.c_h) {
        raspberrypi_multicore_MPCtest_B.ia = raspberrypi_multicore_MPCtest_B.ar;
        raspberrypi_multicore_MPCtest_B.b_ic = 3;
        while (raspberrypi_multicore_MPCtest_B.b_ic + 1 <= 6) {
          raspberrypi_multicore_MPCtest_B.ia++;
          raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.b_ic]
            +=
            raspberrypi_multicore_MPCtest_B.U_a[raspberrypi_multicore_MPCtest_B.vcol
            - 1] *
            raspberrypi_multicore_MPCtest_B.V[raspberrypi_multicore_MPCtest_B.ia];
          raspberrypi_multicore_MPCtest_B.b_ic++;
        }

        raspberrypi_multicore_MPCtest_B.ar += 3;
        raspberrypi_multicore_MPCtest_B.vcol += 3;
      }

      raspberrypi_multicore_MPCtest_B.ar = -1;
      raspberrypi_multicore_MPCtest_B.c_h = 3 *
        raspberrypi_multicore_MPCtest_B.i_f2 + 3;
      raspberrypi_multicore_MPCtest_B.vcol = 3;
      while (raspberrypi_multicore_MPCtest_B.vcol <=
             raspberrypi_multicore_MPCtest_B.c_h) {
        raspberrypi_multicore_MPCtest_B.ia = raspberrypi_multicore_MPCtest_B.ar;
        raspberrypi_multicore_MPCtest_B.b_ic = 6;
        while (raspberrypi_multicore_MPCtest_B.b_ic + 1 <= 9) {
          raspberrypi_multicore_MPCtest_B.ia++;
          raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.b_ic]
            +=
            raspberrypi_multicore_MPCtest_B.U_a[raspberrypi_multicore_MPCtest_B.vcol
            - 1] *
            raspberrypi_multicore_MPCtest_B.V[raspberrypi_multicore_MPCtest_B.ia];
          raspberrypi_multicore_MPCtest_B.b_ic++;
        }

        raspberrypi_multicore_MPCtest_B.ar += 3;
        raspberrypi_multicore_MPCtest_B.vcol += 3;
      }
    }
  }

  for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
       raspberrypi_multicore_MPCtest_B.i_f2 < 3;
       raspberrypi_multicore_MPCtest_B.i_f2++) {
    raspberrypi_multicore_MPCtest_B.a[raspberrypi_multicore_MPCtest_B.i_f2] =
      0.0;
    for (raspberrypi_multicore_MPCtest_B.ar = 0;
         raspberrypi_multicore_MPCtest_B.ar < 4;
         raspberrypi_multicore_MPCtest_B.ar++) {
      raspberrypi_multicore_MPCtest_B.vcol =
        raspberrypi_multicore_MPCtest_B.i_f2 + 3 *
        raspberrypi_multicore_MPCtest_B.ar;
      raspberrypi_multicore_MPCtest_B.W[raspberrypi_multicore_MPCtest_B.vcol] =
        0.0;
      raspberrypi_multicore_MPCtest_B.W[raspberrypi_multicore_MPCtest_B.vcol] +=
        raspberrypi_multicore_MPCtest_B.pArray_L_Adm[3 *
        raspberrypi_multicore_MPCtest_B.ar] *
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.i_f2];
      raspberrypi_multicore_MPCtest_B.W[raspberrypi_multicore_MPCtest_B.vcol] +=
        raspberrypi_multicore_MPCtest_B.pArray_L_Adm[3 *
        raspberrypi_multicore_MPCtest_B.ar + 1] *
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.i_f2
        + 3];
      raspberrypi_multicore_MPCtest_B.c_h = 3 *
        raspberrypi_multicore_MPCtest_B.ar + 2;
      raspberrypi_multicore_MPCtest_B.W[raspberrypi_multicore_MPCtest_B.vcol] +=
        raspberrypi_multicore_MPCtest_B.pArray_L_Adm[raspberrypi_multicore_MPCtest_B.c_h]
        * raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.i_f2
        + 6];
      raspberrypi_multicore_MPCtest_B.a[raspberrypi_multicore_MPCtest_B.i_f2] +=
        raspberrypi_multicore_MPCtest_B.W[raspberrypi_multicore_MPCtest_B.vcol] *
        raspberrypi_multicore_MPCtes_DW.UnitDelay5_DSTATE[raspberrypi_multicore_MPCtest_B.c_h];
    }

    raspberrypi_multicore_MPCtest_B.surP[raspberrypi_multicore_MPCtest_B.i_f2] =
      raspberrypi_multicore_MPCtest_B.a[raspberrypi_multicore_MPCtest_B.i_f2];
  }

  if (raspberrypi_multicore_MPCtest_P.Constant8_Value > 0.5) {
    raspberrypi_multicore_MPCtest_B.surP[0] = 0.0;
    raspberrypi_multicore_MPCtest_B.surP[1] = 0.0;
    raspberrypi_multicore_MPCtest_B.a[1] = 0.0;
    raspberrypi_multicore_MPCtest_B.surP[2] = 0.0;
    raspberrypi_multicore_MPCtest_B.a[2] = 0.0;
  }

  raspberrypi_multicore_MPCtest_B.v1[0] = 1.0;
  raspberrypi_multicore_MPCtest_B.v1[1] = 0.0;
  raspberrypi_multicore_MPCtest_B.v1[2] = raspberrypi_multicore_MPCtest_B.a[1];
  raspberrypi_multicore_MPCtest_B.a[0] = 0.0 *
    raspberrypi_multicore_MPCtest_B.a[2] - raspberrypi_multicore_MPCtest_B.a[1];
  raspberrypi_multicore_MPCtest_B.a[1] = 0.0 *
    raspberrypi_multicore_MPCtest_B.a[1] - raspberrypi_multicore_MPCtest_B.a[2];
  raspberrypi_multicore_MPCtest_B.a[2] = 1.0;
  raspberrypi_multicore_MPCtest_B.Angle3new_c = raspberrypi_multicore_MP_norm_k
    (raspberrypi_multicore_MPCtest_B.a);
  raspberrypi_multicore_MPCtest_B.t10 = raspberrypi_multicore_MP_norm_k
    (raspberrypi_multicore_MPCtest_B.v1);
  raspberrypi_multicore_MPCtest_B.absx = raspberrypi_multicore_MPCtest_B.a[0] /
    raspberrypi_multicore_MPCtest_B.Angle3new_c;
  raspberrypi_multicore_MPCtest_B.q = 1.0 / raspberrypi_multicore_MPCtest_B.t10;
  raspberrypi_multicore_MPCtest_B.b_varargout_5_p =
    raspberrypi_multicore_MPCtest_B.a[1] /
    raspberrypi_multicore_MPCtest_B.Angle3new_c;
  raspberrypi_multicore_MPCtest_B.v2_idx_1 = 0.0 /
    raspberrypi_multicore_MPCtest_B.t10;
  raspberrypi_multicore_MPCtest_B.b_varargout_10_idx_0 = 1.0 /
    raspberrypi_multicore_MPCtest_B.Angle3new_c;
  raspberrypi_multicore_MPCtest_B.v2_idx_2 = raspberrypi_multicore_MPCtest_B.v1
    [2] / raspberrypi_multicore_MPCtest_B.t10;
  raspberrypi_multicore_MPCtest_B.rtb_headG_tmp_tmp_f = sin
    (raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[5]);
  raspberrypi_multicore_MPCtest_B.rtb_headG_tmp_tmp = cos
    (raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[5]);
  raspberrypi_multicore_MPCtest_B.rtb_headG_tmp = sin
    (raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[4]);
  raspberrypi_multicore_MPCtest_B.rtb_headG_tmp_i = cos
    (raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[4]);
  raspberrypi_multicore_MPCtest_B.rtb_headG_tmp_o = sin
    (raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[3]);
  raspberrypi_multicore_MPCtest_B.rtb_headG_tmp_m = cos
    (raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[3]);

  /* MATLABSystem: '<Root>/MATLAB System2' incorporates:
   *  MATLABSystem: '<Root>/MATLAB System10'
   */
  if (raspberrypi_multicore_MPCtes_DW.obj_f.lateral_width !=
      raspberrypi_multicore_MPCtest_P.MATLABSystem2_lateral_width) {
    raspberrypi_multicore_MPCtes_DW.obj_f.lateral_width =
      raspberrypi_multicore_MPCtest_P.MATLABSystem2_lateral_width;
  }

  if (raspberrypi_multicore_MPCtes_DW.obj_f.sagetial_width !=
      raspberrypi_multicore_MPCtest_P.MATLABSystem2_sagetial_width) {
    raspberrypi_multicore_MPCtes_DW.obj_f.sagetial_width =
      raspberrypi_multicore_MPCtest_P.MATLABSystem2_sagetial_width;
  }

  if (raspberrypi_multicore_MPCtes_DW.obj_f.roll_Off !=
      raspberrypi_multicore_MPCtest_P.MATLABSystem2_roll_Off) {
    raspberrypi_multicore_MPCtes_DW.obj_f.roll_Off =
      raspberrypi_multicore_MPCtest_P.MATLABSystem2_roll_Off;
  }

  if (raspberrypi_multicore_MPCtes_DW.obj_f.hIni !=
      raspberrypi_multicore_MPCtest_P.MATLABSystem2_hIni) {
    raspberrypi_multicore_MPCtes_DW.obj_f.hIni =
      raspberrypi_multicore_MPCtest_P.MATLABSystem2_hIni;
  }

  if (raspberrypi_multicore_MPCtes_DW.obj_f.m !=
      raspberrypi_multicore_MPCtest_P.MATLABSystem2_m) {
    raspberrypi_multicore_MPCtes_DW.obj_f.m =
      raspberrypi_multicore_MPCtest_P.MATLABSystem2_m;
  }

  if (raspberrypi_multicore_MPCtes_DW.obj_f.ks1 !=
      raspberrypi_multicore_MPCtest_P.MATLABSystem2_ks1) {
    raspberrypi_multicore_MPCtes_DW.obj_f.ks1 =
      raspberrypi_multicore_MPCtest_P.MATLABSystem2_ks1;
  }

  if (raspberrypi_multicore_MPCtes_DW.obj_f.ks2 !=
      raspberrypi_multicore_MPCtest_P.MATLABSystem2_ks2) {
    raspberrypi_multicore_MPCtes_DW.obj_f.ks2 =
      raspberrypi_multicore_MPCtest_P.MATLABSystem2_ks2;
  }

  if (raspberrypi_multicore_MPCtes_DW.obj_f.ks3 !=
      raspberrypi_multicore_MPCtest_P.MATLABSystem2_ks3) {
    raspberrypi_multicore_MPCtes_DW.obj_f.ks3 =
      raspberrypi_multicore_MPCtest_P.MATLABSystem2_ks3;
  }

  memcpy(&raspberrypi_multicore_MPCtest_B.pArray_L_Adm[0],
         &raspberrypi_multicore_MPCtest_B.MATLABSystem10_o1[0], 12U * sizeof
         (real_T));

  /*  Implement algorithm of admittance ctr, refer to md file for more info. */
  /*  PArray:=[Px_i,Py_i,Pz_i], 12*1, foot-end position in leg coordinate */
  /*  AngleArray:=[Mi1,Mi2,Mi3] */
  /*  CoM position */
  /*  euler angles */
  /*  3D rotation matrix, vb=M*v: */
  /*  rotate a vector in one frame, */
  /*  or change the vector 'v' in rotated frame to 'vb' in world frame */
  /*  3D rotation matrix, vb=M*v: */
  /*  rotate a vector in one frame, */
  /*  or change the vector 'v' in rotated frame to 'vb' in world frame */
  /*  3D rotation matrix, vb=M*v: */
  /*  rotate a vector in one frame, */
  /*  or change the vector 'v' in rotated frame to 'vb' in world frame */
  raspberrypi_multicore_MPCtest_B.s = sin
    (raspberrypi_multicore_MPCtes_DW.obj_cy.X_mpc_Old[5]);
  raspberrypi_multicore_MPCtest_B.s_k = cos
    (raspberrypi_multicore_MPCtes_DW.obj_cy.X_mpc_Old[5]);
  raspberrypi_multicore_MPCtest_B.t6 = sin
    (raspberrypi_multicore_MPCtes_DW.obj_cy.X_mpc_Old[4]);
  raspberrypi_multicore_MPCtest_B.t7 = cos
    (raspberrypi_multicore_MPCtes_DW.obj_cy.X_mpc_Old[4]);
  raspberrypi_multicore_MPCtest_B.t8 = sin
    (raspberrypi_multicore_MPCtes_DW.obj_cy.X_mpc_Old[3]);
  raspberrypi_multicore_MPCtest_B.t9 = cos
    (raspberrypi_multicore_MPCtes_DW.obj_cy.X_mpc_Old[3]);

  /*  foot position in the world coordinate */
  raspberrypi_multicore_MPCtest_B.R[0] = raspberrypi_multicore_MPCtest_B.s_k;
  raspberrypi_multicore_MPCtest_B.R[3] = -raspberrypi_multicore_MPCtest_B.s;
  raspberrypi_multicore_MPCtest_B.R[6] = 0.0;
  raspberrypi_multicore_MPCtest_B.R[1] = raspberrypi_multicore_MPCtest_B.s;
  raspberrypi_multicore_MPCtest_B.R[4] = raspberrypi_multicore_MPCtest_B.s_k;
  raspberrypi_multicore_MPCtest_B.R[7] = 0.0;
  raspberrypi_multicore_MPCtest_B.V[0] = raspberrypi_multicore_MPCtest_B.t7;
  raspberrypi_multicore_MPCtest_B.V[3] = 0.0;
  raspberrypi_multicore_MPCtest_B.V[6] = raspberrypi_multicore_MPCtest_B.t6;
  raspberrypi_multicore_MPCtest_B.R[2] = 0.0;
  raspberrypi_multicore_MPCtest_B.V[1] = 0.0;
  raspberrypi_multicore_MPCtest_B.R[5] = 0.0;
  raspberrypi_multicore_MPCtest_B.V[4] = 1.0;
  raspberrypi_multicore_MPCtest_B.R[8] = 1.0;
  raspberrypi_multicore_MPCtest_B.V[7] = 0.0;
  raspberrypi_multicore_MPCtest_B.V[2] = -raspberrypi_multicore_MPCtest_B.t6;
  raspberrypi_multicore_MPCtest_B.V[5] = 0.0;
  raspberrypi_multicore_MPCtest_B.V[8] = raspberrypi_multicore_MPCtest_B.t7;
  for (raspberrypi_multicore_MPCtest_B.ar = 0;
       raspberrypi_multicore_MPCtest_B.ar < 3;
       raspberrypi_multicore_MPCtest_B.ar++) {
    for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
         raspberrypi_multicore_MPCtest_B.i_f2 < 3;
         raspberrypi_multicore_MPCtest_B.i_f2++) {
      raspberrypi_multicore_MPCtest_B.c_h = raspberrypi_multicore_MPCtest_B.ar +
        3 * raspberrypi_multicore_MPCtest_B.i_f2;
      raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.c_h] =
        0.0;
      raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.c_h] +=
        raspberrypi_multicore_MPCtest_B.V[3 *
        raspberrypi_multicore_MPCtest_B.i_f2] *
        raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.ar];
      raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.c_h] +=
        raspberrypi_multicore_MPCtest_B.V[3 *
        raspberrypi_multicore_MPCtest_B.i_f2 + 1] *
        raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.ar + 3];
      raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.c_h] +=
        raspberrypi_multicore_MPCtest_B.V[3 *
        raspberrypi_multicore_MPCtest_B.i_f2 + 2] *
        raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.ar + 6];
    }

    raspberrypi_multicore_MPCtest_B.U_a[3 * raspberrypi_multicore_MPCtest_B.ar] =
      d[raspberrypi_multicore_MPCtest_B.ar];
  }

  raspberrypi_multicore_MPCtest_B.U_a[1] = 0.0;
  raspberrypi_multicore_MPCtest_B.U_a[4] = raspberrypi_multicore_MPCtest_B.t9;
  raspberrypi_multicore_MPCtest_B.U_a[7] = -raspberrypi_multicore_MPCtest_B.t8;
  raspberrypi_multicore_MPCtest_B.U_a[2] = 0.0;
  raspberrypi_multicore_MPCtest_B.U_a[5] = raspberrypi_multicore_MPCtest_B.t8;
  raspberrypi_multicore_MPCtest_B.U_a[8] = raspberrypi_multicore_MPCtest_B.t9;
  for (raspberrypi_multicore_MPCtest_B.ar = 0;
       raspberrypi_multicore_MPCtest_B.ar < 3;
       raspberrypi_multicore_MPCtest_B.ar++) {
    for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
         raspberrypi_multicore_MPCtest_B.i_f2 < 3;
         raspberrypi_multicore_MPCtest_B.i_f2++) {
      raspberrypi_multicore_MPCtest_B.c_h = raspberrypi_multicore_MPCtest_B.i_f2
        + 3 * raspberrypi_multicore_MPCtest_B.ar;
      raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.c_h] =
        0.0;
      raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.c_h] +=
        raspberrypi_multicore_MPCtest_B.U_a[3 *
        raspberrypi_multicore_MPCtest_B.i_f2] *
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar];
      raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.c_h] +=
        raspberrypi_multicore_MPCtest_B.U_a[3 *
        raspberrypi_multicore_MPCtest_B.i_f2 + 1] *
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar +
        3];
      raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.c_h] +=
        raspberrypi_multicore_MPCtest_B.U_a[3 *
        raspberrypi_multicore_MPCtest_B.i_f2 + 2] *
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar +
        6];
    }
  }

  for (raspberrypi_multicore_MPCtest_B.ar = 0;
       raspberrypi_multicore_MPCtest_B.ar < 4;
       raspberrypi_multicore_MPCtest_B.ar++) {
    raspberrypi_multicore_MPCtest_B.W[3 * raspberrypi_multicore_MPCtest_B.ar] =
      rtb_MATLABSystem10_o5_0[3 * raspberrypi_multicore_MPCtest_B.ar] -
      raspberrypi_multicore_MPCtes_DW.obj_cy.X_mpc_Old[0];
    raspberrypi_multicore_MPCtest_B.i_f2 = 3 *
      raspberrypi_multicore_MPCtest_B.ar + 1;
    raspberrypi_multicore_MPCtest_B.W[raspberrypi_multicore_MPCtest_B.i_f2] =
      rtb_MATLABSystem10_o5_0[raspberrypi_multicore_MPCtest_B.i_f2] -
      raspberrypi_multicore_MPCtes_DW.obj_cy.X_mpc_Old[1];
    raspberrypi_multicore_MPCtest_B.i_f2 = 3 *
      raspberrypi_multicore_MPCtest_B.ar + 2;
    raspberrypi_multicore_MPCtest_B.W[raspberrypi_multicore_MPCtest_B.i_f2] =
      rtb_MATLABSystem10_o5_0[raspberrypi_multicore_MPCtest_B.i_f2] -
      raspberrypi_multicore_MPCtes_DW.obj_cy.X_mpc_Old[2];
  }

  for (raspberrypi_multicore_MPCtest_B.ar = 0;
       raspberrypi_multicore_MPCtest_B.ar < 3;
       raspberrypi_multicore_MPCtest_B.ar++) {
    for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
         raspberrypi_multicore_MPCtest_B.i_f2 < 4;
         raspberrypi_multicore_MPCtest_B.i_f2++) {
      raspberrypi_multicore_MPCtest_B.c_h = 3 *
        raspberrypi_multicore_MPCtest_B.i_f2 +
        raspberrypi_multicore_MPCtest_B.ar;
      raspberrypi_multicore_MPCtest_B.vBM[raspberrypi_multicore_MPCtest_B.c_h] =
        ((raspberrypi_multicore_MPCtest_B.W[3 *
          raspberrypi_multicore_MPCtest_B.i_f2 + 1] *
          raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.ar +
          3] + raspberrypi_multicore_MPCtest_B.W[3 *
          raspberrypi_multicore_MPCtest_B.i_f2] *
          raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.ar])
         + raspberrypi_multicore_MPCtest_B.W[3 *
         raspberrypi_multicore_MPCtest_B.i_f2 + 2] *
         raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.ar +
         6]) -
        raspberrypi_multicore_MPCtes_DW.obj_f.PendAllnorm[raspberrypi_multicore_MPCtest_B.c_h];
    }
  }

  for (raspberrypi_multicore_MPCtest_B.ar = 0;
       raspberrypi_multicore_MPCtest_B.ar < 12;
       raspberrypi_multicore_MPCtest_B.ar++) {
    raspberrypi_multicore_MPCtest_B.W[raspberrypi_multicore_MPCtest_B.ar] =
      raspberrypi_multicore_MPCtest_B.vBM[raspberrypi_multicore_MPCtest_B.ar] *
      1000.0;
  }

  /*  foot position in the leg coordinate */
  if (raspberrypi_multicore_MPCtest_B.mpcSTOP > 0.5) {
    raspberrypi_multicore_MPCtest_B.W[0] = 0.0;
    raspberrypi_multicore_MPCtest_B.W[3] = 0.0;
    raspberrypi_multicore_MPCtest_B.W[6] = 0.0;
    raspberrypi_multicore_MPCtest_B.W[9] = 0.0;
    raspberrypi_multicore_MPCtest_B.s =
      raspberrypi_multicore_MPCtes_DW.obj_f.roll_Off * 1000.0;
    raspberrypi_multicore_MPCtest_B.W[1] = raspberrypi_multicore_MPCtest_B.s;
    raspberrypi_multicore_MPCtest_B.s_k =
      -raspberrypi_multicore_MPCtes_DW.obj_f.roll_Off * 1000.0;
    raspberrypi_multicore_MPCtest_B.W[4] = raspberrypi_multicore_MPCtest_B.s_k;
    raspberrypi_multicore_MPCtest_B.W[7] = raspberrypi_multicore_MPCtest_B.s;
    raspberrypi_multicore_MPCtest_B.W[10] = raspberrypi_multicore_MPCtest_B.s_k;
    raspberrypi_multicore_MPCtest_B.s =
      -raspberrypi_multicore_MPCtes_DW.obj_f.hIni * 1000.0;
    raspberrypi_multicore_MPCtest_B.W[2] = raspberrypi_multicore_MPCtest_B.s;
    raspberrypi_multicore_MPCtest_B.W[5] = raspberrypi_multicore_MPCtest_B.s;
    raspberrypi_multicore_MPCtest_B.W[8] = raspberrypi_multicore_MPCtest_B.s;
    raspberrypi_multicore_MPCtest_B.W[11] = raspberrypi_multicore_MPCtest_B.s;
    for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
         raspberrypi_multicore_MPCtest_B.i_f2 < 12;
         raspberrypi_multicore_MPCtest_B.i_f2++) {
      raspberrypi_multicore_MPCtest_B.pArray_L_Adm[raspberrypi_multicore_MPCtest_B.i_f2]
        = tmp[raspberrypi_multicore_MPCtest_B.i_f2] *
        raspberrypi_multicore_MPCtes_DW.obj_f.m / 4.0;
    }
  }

  raspberryp_AdmittanceCtr_IK_one(&raspberrypi_multicore_MPCtes_DW.obj_f,
    &raspberrypi_multicore_MPCtest_B.W[0], 1.0,
    raspberrypi_multicore_MPCtest_B.a, &raspberrypi_multicore_MPCtest_B.s_k);
  raspberryp_AdmittanceCtr_IK_one(&raspberrypi_multicore_MPCtes_DW.obj_f,
    &raspberrypi_multicore_MPCtest_B.W[3], 2.0,
    raspberrypi_multicore_MPCtest_B.Angle2new,
    &raspberrypi_multicore_MPCtest_B.s_k);
  raspberryp_AdmittanceCtr_IK_one(&raspberrypi_multicore_MPCtes_DW.obj_f,
    &raspberrypi_multicore_MPCtest_B.W[6], 3.0,
    raspberrypi_multicore_MPCtest_B.Angle3new,
    &raspberrypi_multicore_MPCtest_B.s_k);
  raspberryp_AdmittanceCtr_IK_one(&raspberrypi_multicore_MPCtes_DW.obj_f,
    &raspberrypi_multicore_MPCtest_B.W[9], 4.0,
    raspberrypi_multicore_MPCtest_B.Angle4new,
    &raspberrypi_multicore_MPCtest_B.s_k);

  /* AUTOGEN_JACOBI_1 */
  /*     JACOBI = AUTOGEN_JACOBI_1(LOA,LAB,LAD,LDP,CDP,THETA1,THETA2,THETA3) */
  /*     This function was generated by the Symbolic Math Toolbox version 8.7. */
  /*     12-Aug-2021 16:19:08 */
  raspberrypi_multicore_MPCtest_B.s = cos
    (raspberrypi_multicore_MPCtes_DW.obj_f.CDP);
  raspberrypi_multicore_MPCtest_B.s_k = sin
    (raspberrypi_multicore_MPCtes_DW.obj_f.CDP);
  raspberrypi_multicore_MPCtest_B.t4 = cos(raspberrypi_multicore_MPCtest_B.a[0]);
  raspberrypi_multicore_MPCtest_B.t6 = cos(raspberrypi_multicore_MPCtest_B.a[2]);
  raspberrypi_multicore_MPCtest_B.t7 = sin(raspberrypi_multicore_MPCtest_B.a[0]);
  raspberrypi_multicore_MPCtest_B.t8 = sin(raspberrypi_multicore_MPCtest_B.a[1]);
  raspberrypi_multicore_MPCtest_B.t9 = sin(raspberrypi_multicore_MPCtest_B.a[2]);
  raspberrypi_multicore_MPCtest_B.t10 =
    raspberrypi_multicore_MPCtes_DW.obj_f.CDP +
    raspberrypi_multicore_MPCtest_B.a[1];
  raspberrypi_multicore_MPCtest_B.t11 = cos(raspberrypi_multicore_MPCtest_B.t10);

  /* AUTOGEN_JACOBI_2 */
  /*     JACOBI = AUTOGEN_JACOBI_2(LOA,LAB,LAD,LDP,CDP,THETA1,THETA2,THETA3) */
  /*     This function was generated by the Symbolic Math Toolbox version 8.7. */
  /*     12-Aug-2021 16:19:09 */
  raspberrypi_multicore_MPCtest_B.t4_j = cos
    (raspberrypi_multicore_MPCtest_B.Angle2new[0]);
  raspberrypi_multicore_MPCtest_B.t6_e = cos
    (raspberrypi_multicore_MPCtest_B.Angle2new[2]);
  raspberrypi_multicore_MPCtest_B.t7_o = sin
    (raspberrypi_multicore_MPCtest_B.Angle2new[0]);
  raspberrypi_multicore_MPCtest_B.t8_b = sin
    (raspberrypi_multicore_MPCtest_B.Angle2new[1]);
  raspberrypi_multicore_MPCtest_B.t9_a = sin
    (raspberrypi_multicore_MPCtest_B.Angle2new[2]);
  raspberrypi_multicore_MPCtest_B.t11_g =
    raspberrypi_multicore_MPCtes_DW.obj_f.CDP +
    -raspberrypi_multicore_MPCtest_B.Angle2new[1];
  raspberrypi_multicore_MPCtest_B.t12 = cos
    (raspberrypi_multicore_MPCtest_B.t11_g);

  /* AUTOGEN_JACOBI_3 */
  /*     JACOBI = AUTOGEN_JACOBI_3(LOA,LAB,LAD,LDP,CDP,THETA1,THETA2,THETA3) */
  /*     This function was generated by the Symbolic Math Toolbox version 8.7. */
  /*     12-Aug-2021 16:19:08 */
  raspberrypi_multicore_MPCtest_B.t6_ex = cos
    (raspberrypi_multicore_MPCtest_B.Angle3new[2]);
  raspberrypi_multicore_MPCtest_B.t7_f = sin
    (raspberrypi_multicore_MPCtest_B.Angle3new[0]);
  raspberrypi_multicore_MPCtest_B.t8_h = sin
    (raspberrypi_multicore_MPCtest_B.Angle3new[1]);
  raspberrypi_multicore_MPCtest_B.t9_e = sin
    (raspberrypi_multicore_MPCtest_B.Angle3new[2]);
  raspberrypi_multicore_MPCtest_B.t10_c =
    raspberrypi_multicore_MPCtes_DW.obj_f.CDP +
    raspberrypi_multicore_MPCtest_B.Angle3new[1];
  raspberrypi_multicore_MPCtest_B.t11_a = cos
    (raspberrypi_multicore_MPCtest_B.t10_c);

  /* AUTOGEN_JACOBI_4 */
  /*     JACOBI = AUTOGEN_JACOBI_4(LOA,LAB,LAD,LDP,CDP,THETA1,THETA2,THETA3) */
  /*     This function was generated by the Symbolic Math Toolbox version 8.7. */
  /*     12-Aug-2021 16:19:09 */
  raspberrypi_multicore_MPCtest_B.t6_d = cos
    (raspberrypi_multicore_MPCtest_B.Angle4new[2]);
  raspberrypi_multicore_MPCtest_B.t7_a = sin
    (raspberrypi_multicore_MPCtest_B.Angle4new[0]);
  raspberrypi_multicore_MPCtest_B.t8_p = sin
    (raspberrypi_multicore_MPCtest_B.Angle4new[1]);
  raspberrypi_multicore_MPCtest_B.t9_m = sin
    (raspberrypi_multicore_MPCtest_B.Angle4new[2]);
  raspberrypi_multicore_MPCtest_B.t11_o =
    raspberrypi_multicore_MPCtes_DW.obj_f.CDP +
    -raspberrypi_multicore_MPCtest_B.Angle4new[1];
  raspberrypi_multicore_MPCtest_B.t12_n = cos
    (raspberrypi_multicore_MPCtest_B.t11_o);
  if ((fabs(raspberrypi_multicore_MPCtes_DW.obj_f.ks1) < 1.0E-9) || (fabs
       (raspberrypi_multicore_MPCtes_DW.obj_f.ks2) < 1.0E-9) || (fabs
       (raspberrypi_multicore_MPCtes_DW.obj_f.ks3) < 1.0E-9)) {
    raspberrypi_multicore_MPCtest_B.v1[0] = 0.0;
    raspberrypi_multicore_MPCtest_B.v1[1] = 0.0;
    raspberrypi_multicore_MPCtest_B.v1[2] = 0.0;
    raspberrypi_multicore_MPCtest_B.deltaP2[0] = 0.0;
    raspberrypi_multicore_MPCtest_B.deltaP3[0] = 0.0;
    raspberrypi_multicore_MPCtest_B.deltaP4[0] = 0.0;
    raspberrypi_multicore_MPCtest_B.deltaP2[1] = 0.0;
    raspberrypi_multicore_MPCtest_B.deltaP3[1] = 0.0;
    raspberrypi_multicore_MPCtest_B.deltaP4[1] = 0.0;
    raspberrypi_multicore_MPCtest_B.deltaP2[2] = 0.0;
    raspberrypi_multicore_MPCtest_B.deltaP3[2] = 0.0;
    raspberrypi_multicore_MPCtest_B.deltaP4[2] = 0.0;
  } else {
    raspberrypi_multicore_MPCtest_B.deltaP4[0] = 1.0 /
      raspberrypi_multicore_MPCtes_DW.obj_f.ks1;
    raspberrypi_multicore_MPCtest_B.deltaP4[1] = 1.0 /
      raspberrypi_multicore_MPCtes_DW.obj_f.ks2;
    raspberrypi_multicore_MPCtest_B.deltaP4[2] = 1.0 /
      raspberrypi_multicore_MPCtes_DW.obj_f.ks3;
    memset(&raspberrypi_multicore_MPCtest_B.MRz[0], 0, 9U * sizeof(real_T));
    raspberrypi_multicore_MPCtest_B.MRz[0] =
      raspberrypi_multicore_MPCtest_B.deltaP4[0];
    raspberrypi_multicore_MPCtest_B.MRz[4] =
      raspberrypi_multicore_MPCtest_B.deltaP4[1];
    raspberrypi_multicore_MPCtest_B.MRz[8] =
      raspberrypi_multicore_MPCtest_B.deltaP4[2];
    for (raspberrypi_multicore_MPCtest_B.ar = 0;
         raspberrypi_multicore_MPCtest_B.ar < 9;
         raspberrypi_multicore_MPCtest_B.ar++) {
      raspberrypi_multicore_MPCtest_B.V[raspberrypi_multicore_MPCtest_B.ar] =
        -raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar]
        / 1000.0;
    }

    raspberrypi_multicore_MPCtest_B.Angle3new_c =
      raspberrypi_multicore_MPCtes_DW.obj_f.BC *
      raspberrypi_multicore_MPCtest_B.t7;
    raspberrypi_multicore_MPCtest_B.U_a[0] =
      raspberrypi_multicore_MPCtest_B.Angle3new_c;
    raspberrypi_multicore_MPCtest_B.U_a[3] =
      -raspberrypi_multicore_MPCtes_DW.obj_f.BC *
      raspberrypi_multicore_MPCtest_B.t4 * raspberrypi_multicore_MPCtest_B.t9;
    raspberrypi_multicore_MPCtest_B.U_a[6] =
      raspberrypi_multicore_MPCtes_DW.obj_f.BC *
      raspberrypi_multicore_MPCtest_B.t4 * raspberrypi_multicore_MPCtest_B.t6;
    raspberrypi_multicore_MPCtest_B.U_a[1] =
      raspberrypi_multicore_MPCtes_DW.obj_f.DP * sin
      (raspberrypi_multicore_MPCtest_B.t10);
    raspberrypi_multicore_MPCtest_B.U_a[4] =
      raspberrypi_multicore_MPCtes_DW.obj_f.DP *
      raspberrypi_multicore_MPCtest_B.t9 * raspberrypi_multicore_MPCtest_B.t11;
    raspberrypi_multicore_MPCtest_B.U_a[7] =
      -raspberrypi_multicore_MPCtes_DW.obj_f.DP *
      raspberrypi_multicore_MPCtest_B.t6 * raspberrypi_multicore_MPCtest_B.t11;
    raspberrypi_multicore_MPCtest_B.U_a[2] = 0.0;
    raspberrypi_multicore_MPCtest_B.t10 =
      raspberrypi_multicore_MPCtes_DW.obj_f.DP *
      raspberrypi_multicore_MPCtest_B.s_k;
    raspberrypi_multicore_MPCtest_B.t4 = raspberrypi_multicore_MPCtest_B.t10 *
      cos(raspberrypi_multicore_MPCtest_B.a[1]);
    raspberrypi_multicore_MPCtest_B.t11 =
      raspberrypi_multicore_MPCtes_DW.obj_f.DP *
      raspberrypi_multicore_MPCtest_B.s;
    raspberrypi_multicore_MPCtest_B.U_a[5] =
      ((-raspberrypi_multicore_MPCtes_DW.obj_f.OR *
        raspberrypi_multicore_MPCtest_B.t9 -
        raspberrypi_multicore_MPCtes_DW.obj_f.BC *
        raspberrypi_multicore_MPCtest_B.t6 * raspberrypi_multicore_MPCtest_B.t7)
       + raspberrypi_multicore_MPCtest_B.t4 * raspberrypi_multicore_MPCtest_B.t6)
      + raspberrypi_multicore_MPCtest_B.t11 * raspberrypi_multicore_MPCtest_B.t6
      * raspberrypi_multicore_MPCtest_B.t8;
    raspberrypi_multicore_MPCtest_B.U_a[8] =
      ((raspberrypi_multicore_MPCtes_DW.obj_f.OR *
        raspberrypi_multicore_MPCtest_B.t6 -
        raspberrypi_multicore_MPCtest_B.Angle3new_c *
        raspberrypi_multicore_MPCtest_B.t9) + raspberrypi_multicore_MPCtest_B.t4
       * raspberrypi_multicore_MPCtest_B.t9) +
      raspberrypi_multicore_MPCtest_B.t11 * raspberrypi_multicore_MPCtest_B.t8 *
      raspberrypi_multicore_MPCtest_B.t9;
    raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_m[0] =
      raspberrypi_multicore_MPCtest_B.pArray_L_Adm[0];
    raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_m[1] =
      raspberrypi_multicore_MPCtest_B.pArray_L_Adm[1];
    raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_m[2] =
      raspberrypi_multicore_MPCtest_B.pArray_L_Adm[2];
    for (raspberrypi_multicore_MPCtest_B.ar = 0;
         raspberrypi_multicore_MPCtest_B.ar < 3;
         raspberrypi_multicore_MPCtest_B.ar++) {
      for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
           raspberrypi_multicore_MPCtest_B.i_f2 < 3;
           raspberrypi_multicore_MPCtest_B.i_f2++) {
        raspberrypi_multicore_MPCtest_B.vcol =
          raspberrypi_multicore_MPCtest_B.ar + 3 *
          raspberrypi_multicore_MPCtest_B.i_f2;
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.vcol]
          = 0.0;
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.vcol]
          += raspberrypi_multicore_MPCtest_B.U_a[3 *
          raspberrypi_multicore_MPCtest_B.i_f2] *
          raspberrypi_multicore_MPCtest_B.V[raspberrypi_multicore_MPCtest_B.ar];
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.vcol]
          += raspberrypi_multicore_MPCtest_B.U_a[3 *
          raspberrypi_multicore_MPCtest_B.i_f2 + 1] *
          raspberrypi_multicore_MPCtest_B.V[raspberrypi_multicore_MPCtest_B.ar +
          3];
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.vcol]
          += raspberrypi_multicore_MPCtest_B.U_a[3 *
          raspberrypi_multicore_MPCtest_B.i_f2 + 2] *
          raspberrypi_multicore_MPCtest_B.V[raspberrypi_multicore_MPCtest_B.ar +
          6];
      }

      raspberrypi_multicore_MPCtest_B.v1[raspberrypi_multicore_MPCtest_B.ar] =
        0.0;
      for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
           raspberrypi_multicore_MPCtest_B.i_f2 < 3;
           raspberrypi_multicore_MPCtest_B.i_f2++) {
        raspberrypi_multicore_MPCtest_B.vcol =
          raspberrypi_multicore_MPCtest_B.ar + 3 *
          raspberrypi_multicore_MPCtest_B.i_f2;
        raspberrypi_multicore_MPCtest_B.MRz_i[raspberrypi_multicore_MPCtest_B.vcol]
          = 0.0;
        raspberrypi_multicore_MPCtest_B.MRz_i[raspberrypi_multicore_MPCtest_B.vcol]
          += raspberrypi_multicore_MPCtest_B.R[3 *
          raspberrypi_multicore_MPCtest_B.i_f2] *
          raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar];
        raspberrypi_multicore_MPCtest_B.MRz_i[raspberrypi_multicore_MPCtest_B.vcol]
          += raspberrypi_multicore_MPCtest_B.R[3 *
          raspberrypi_multicore_MPCtest_B.i_f2 + 1] *
          raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar
          + 3];
        raspberrypi_multicore_MPCtest_B.MRz_i[raspberrypi_multicore_MPCtest_B.vcol]
          += raspberrypi_multicore_MPCtest_B.R[3 *
          raspberrypi_multicore_MPCtest_B.i_f2 + 2] *
          raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar
          + 6];
        raspberrypi_multicore_MPCtest_B.v1[raspberrypi_multicore_MPCtest_B.ar] +=
          raspberrypi_multicore_MPCtest_B.MRz_i[raspberrypi_multicore_MPCtest_B.vcol]
          * raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_m[raspberrypi_multicore_MPCtest_B.i_f2];
      }
    }

    memset(&raspberrypi_multicore_MPCtest_B.MRz[0], 0, 9U * sizeof(real_T));
    raspberrypi_multicore_MPCtest_B.MRz[0] =
      raspberrypi_multicore_MPCtest_B.deltaP4[0];
    raspberrypi_multicore_MPCtest_B.MRz[4] =
      raspberrypi_multicore_MPCtest_B.deltaP4[1];
    raspberrypi_multicore_MPCtest_B.MRz[8] =
      raspberrypi_multicore_MPCtest_B.deltaP4[2];
    for (raspberrypi_multicore_MPCtest_B.ar = 0;
         raspberrypi_multicore_MPCtest_B.ar < 9;
         raspberrypi_multicore_MPCtest_B.ar++) {
      raspberrypi_multicore_MPCtest_B.V[raspberrypi_multicore_MPCtest_B.ar] =
        -raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar]
        / 1000.0;
    }

    raspberrypi_multicore_MPCtest_B.Angle3new_c =
      raspberrypi_multicore_MPCtes_DW.obj_f.BC *
      raspberrypi_multicore_MPCtest_B.t7_o;
    raspberrypi_multicore_MPCtest_B.U_a[0] =
      raspberrypi_multicore_MPCtest_B.Angle3new_c;
    raspberrypi_multicore_MPCtest_B.U_a[3] =
      raspberrypi_multicore_MPCtes_DW.obj_f.BC *
      raspberrypi_multicore_MPCtest_B.t4_j *
      raspberrypi_multicore_MPCtest_B.t9_a;
    raspberrypi_multicore_MPCtest_B.U_a[6] =
      -raspberrypi_multicore_MPCtes_DW.obj_f.BC *
      raspberrypi_multicore_MPCtest_B.t4_j *
      raspberrypi_multicore_MPCtest_B.t6_e;
    raspberrypi_multicore_MPCtest_B.U_a[1] =
      -raspberrypi_multicore_MPCtes_DW.obj_f.DP * sin
      (raspberrypi_multicore_MPCtest_B.t11_g);
    raspberrypi_multicore_MPCtest_B.U_a[4] =
      -raspberrypi_multicore_MPCtes_DW.obj_f.DP *
      raspberrypi_multicore_MPCtest_B.t9_a * raspberrypi_multicore_MPCtest_B.t12;
    raspberrypi_multicore_MPCtest_B.U_a[7] =
      raspberrypi_multicore_MPCtes_DW.obj_f.DP *
      raspberrypi_multicore_MPCtest_B.t6_e * raspberrypi_multicore_MPCtest_B.t12;
    raspberrypi_multicore_MPCtest_B.U_a[2] = 0.0;
    raspberrypi_multicore_MPCtest_B.t4 = raspberrypi_multicore_MPCtest_B.t10 *
      cos(raspberrypi_multicore_MPCtest_B.Angle2new[1]);
    raspberrypi_multicore_MPCtest_B.U_a[5] =
      ((raspberrypi_multicore_MPCtes_DW.obj_f.BC *
        raspberrypi_multicore_MPCtest_B.t6_e *
        raspberrypi_multicore_MPCtest_B.t7_o +
        raspberrypi_multicore_MPCtes_DW.obj_f.OR *
        raspberrypi_multicore_MPCtest_B.t9_a) +
       raspberrypi_multicore_MPCtest_B.t4 * raspberrypi_multicore_MPCtest_B.t6_e)
      - raspberrypi_multicore_MPCtest_B.t11 *
      raspberrypi_multicore_MPCtest_B.t6_e *
      raspberrypi_multicore_MPCtest_B.t8_b;
    raspberrypi_multicore_MPCtest_B.U_a[8] =
      ((raspberrypi_multicore_MPCtest_B.Angle3new_c *
        raspberrypi_multicore_MPCtest_B.t9_a +
        -raspberrypi_multicore_MPCtes_DW.obj_f.OR *
        raspberrypi_multicore_MPCtest_B.t6_e) +
       raspberrypi_multicore_MPCtest_B.t4 * raspberrypi_multicore_MPCtest_B.t9_a)
      - raspberrypi_multicore_MPCtest_B.t11 *
      raspberrypi_multicore_MPCtest_B.t8_b *
      raspberrypi_multicore_MPCtest_B.t9_a;
    raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_m[0] =
      raspberrypi_multicore_MPCtest_B.pArray_L_Adm[3];
    raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_m[1] =
      raspberrypi_multicore_MPCtest_B.pArray_L_Adm[4];
    raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_m[2] =
      raspberrypi_multicore_MPCtest_B.pArray_L_Adm[5];
    for (raspberrypi_multicore_MPCtest_B.ar = 0;
         raspberrypi_multicore_MPCtest_B.ar < 3;
         raspberrypi_multicore_MPCtest_B.ar++) {
      for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
           raspberrypi_multicore_MPCtest_B.i_f2 < 3;
           raspberrypi_multicore_MPCtest_B.i_f2++) {
        raspberrypi_multicore_MPCtest_B.vcol =
          raspberrypi_multicore_MPCtest_B.ar + 3 *
          raspberrypi_multicore_MPCtest_B.i_f2;
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.vcol]
          = 0.0;
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.vcol]
          += raspberrypi_multicore_MPCtest_B.U_a[3 *
          raspberrypi_multicore_MPCtest_B.i_f2] *
          raspberrypi_multicore_MPCtest_B.V[raspberrypi_multicore_MPCtest_B.ar];
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.vcol]
          += raspberrypi_multicore_MPCtest_B.U_a[3 *
          raspberrypi_multicore_MPCtest_B.i_f2 + 1] *
          raspberrypi_multicore_MPCtest_B.V[raspberrypi_multicore_MPCtest_B.ar +
          3];
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.vcol]
          += raspberrypi_multicore_MPCtest_B.U_a[3 *
          raspberrypi_multicore_MPCtest_B.i_f2 + 2] *
          raspberrypi_multicore_MPCtest_B.V[raspberrypi_multicore_MPCtest_B.ar +
          6];
      }

      raspberrypi_multicore_MPCtest_B.deltaP2[raspberrypi_multicore_MPCtest_B.ar]
        = 0.0;
      for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
           raspberrypi_multicore_MPCtest_B.i_f2 < 3;
           raspberrypi_multicore_MPCtest_B.i_f2++) {
        raspberrypi_multicore_MPCtest_B.vcol =
          raspberrypi_multicore_MPCtest_B.ar + 3 *
          raspberrypi_multicore_MPCtest_B.i_f2;
        raspberrypi_multicore_MPCtest_B.MRz_i[raspberrypi_multicore_MPCtest_B.vcol]
          = 0.0;
        raspberrypi_multicore_MPCtest_B.MRz_i[raspberrypi_multicore_MPCtest_B.vcol]
          += raspberrypi_multicore_MPCtest_B.R[3 *
          raspberrypi_multicore_MPCtest_B.i_f2] *
          raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar];
        raspberrypi_multicore_MPCtest_B.MRz_i[raspberrypi_multicore_MPCtest_B.vcol]
          += raspberrypi_multicore_MPCtest_B.R[3 *
          raspberrypi_multicore_MPCtest_B.i_f2 + 1] *
          raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar
          + 3];
        raspberrypi_multicore_MPCtest_B.MRz_i[raspberrypi_multicore_MPCtest_B.vcol]
          += raspberrypi_multicore_MPCtest_B.R[3 *
          raspberrypi_multicore_MPCtest_B.i_f2 + 2] *
          raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar
          + 6];
        raspberrypi_multicore_MPCtest_B.deltaP2[raspberrypi_multicore_MPCtest_B.ar]
          +=
          raspberrypi_multicore_MPCtest_B.MRz_i[raspberrypi_multicore_MPCtest_B.vcol]
          * raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_m[raspberrypi_multicore_MPCtest_B.i_f2];
      }
    }

    memset(&raspberrypi_multicore_MPCtest_B.MRz[0], 0, 9U * sizeof(real_T));
    raspberrypi_multicore_MPCtest_B.MRz[0] =
      raspberrypi_multicore_MPCtest_B.deltaP4[0];
    raspberrypi_multicore_MPCtest_B.MRz[4] =
      raspberrypi_multicore_MPCtest_B.deltaP4[1];
    raspberrypi_multicore_MPCtest_B.MRz[8] =
      raspberrypi_multicore_MPCtest_B.deltaP4[2];
    for (raspberrypi_multicore_MPCtest_B.ar = 0;
         raspberrypi_multicore_MPCtest_B.ar < 9;
         raspberrypi_multicore_MPCtest_B.ar++) {
      raspberrypi_multicore_MPCtest_B.V[raspberrypi_multicore_MPCtest_B.ar] =
        -raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar]
        / 1000.0;
    }

    raspberrypi_multicore_MPCtest_B.Angle3new_c =
      raspberrypi_multicore_MPCtes_DW.obj_f.BC *
      raspberrypi_multicore_MPCtest_B.t7_f;
    raspberrypi_multicore_MPCtest_B.U_a[0] =
      raspberrypi_multicore_MPCtest_B.Angle3new_c;
    raspberrypi_multicore_MPCtest_B.t4 =
      raspberrypi_multicore_MPCtes_DW.obj_f.BC * cos
      (raspberrypi_multicore_MPCtest_B.Angle3new[0]);
    raspberrypi_multicore_MPCtest_B.U_a[3] = raspberrypi_multicore_MPCtest_B.t4 *
      raspberrypi_multicore_MPCtest_B.t9_e;
    raspberrypi_multicore_MPCtest_B.U_a[6] = raspberrypi_multicore_MPCtest_B.t4 *
      raspberrypi_multicore_MPCtest_B.t6_ex;
    raspberrypi_multicore_MPCtest_B.U_a[1] =
      raspberrypi_multicore_MPCtes_DW.obj_f.DP * sin
      (raspberrypi_multicore_MPCtest_B.t10_c);
    raspberrypi_multicore_MPCtest_B.U_a[4] =
      -raspberrypi_multicore_MPCtes_DW.obj_f.DP *
      raspberrypi_multicore_MPCtest_B.t9_e *
      raspberrypi_multicore_MPCtest_B.t11_a;
    raspberrypi_multicore_MPCtest_B.U_a[7] =
      -raspberrypi_multicore_MPCtes_DW.obj_f.DP *
      raspberrypi_multicore_MPCtest_B.t6_ex *
      raspberrypi_multicore_MPCtest_B.t11_a;
    raspberrypi_multicore_MPCtest_B.U_a[2] = 0.0;
    raspberrypi_multicore_MPCtest_B.t4 = raspberrypi_multicore_MPCtest_B.t10 *
      cos(raspberrypi_multicore_MPCtest_B.Angle3new[1]);
    raspberrypi_multicore_MPCtest_B.U_a[5] =
      ((raspberrypi_multicore_MPCtes_DW.obj_f.BC *
        raspberrypi_multicore_MPCtest_B.t6_ex *
        raspberrypi_multicore_MPCtest_B.t7_f +
        -raspberrypi_multicore_MPCtes_DW.obj_f.OR *
        raspberrypi_multicore_MPCtest_B.t9_e) -
       raspberrypi_multicore_MPCtest_B.t4 *
       raspberrypi_multicore_MPCtest_B.t6_ex) -
      raspberrypi_multicore_MPCtest_B.t11 *
      raspberrypi_multicore_MPCtest_B.t6_ex *
      raspberrypi_multicore_MPCtest_B.t8_h;
    raspberrypi_multicore_MPCtest_B.U_a[8] =
      ((-raspberrypi_multicore_MPCtes_DW.obj_f.OR *
        raspberrypi_multicore_MPCtest_B.t6_ex -
        raspberrypi_multicore_MPCtest_B.Angle3new_c *
        raspberrypi_multicore_MPCtest_B.t9_e) +
       raspberrypi_multicore_MPCtest_B.t4 * raspberrypi_multicore_MPCtest_B.t9_e)
      + raspberrypi_multicore_MPCtest_B.t11 *
      raspberrypi_multicore_MPCtest_B.t8_h *
      raspberrypi_multicore_MPCtest_B.t9_e;
    raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_m[0] =
      raspberrypi_multicore_MPCtest_B.pArray_L_Adm[6];
    raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_m[1] =
      raspberrypi_multicore_MPCtest_B.pArray_L_Adm[7];
    raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_m[2] =
      raspberrypi_multicore_MPCtest_B.pArray_L_Adm[8];
    for (raspberrypi_multicore_MPCtest_B.ar = 0;
         raspberrypi_multicore_MPCtest_B.ar < 3;
         raspberrypi_multicore_MPCtest_B.ar++) {
      for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
           raspberrypi_multicore_MPCtest_B.i_f2 < 3;
           raspberrypi_multicore_MPCtest_B.i_f2++) {
        raspberrypi_multicore_MPCtest_B.vcol =
          raspberrypi_multicore_MPCtest_B.ar + 3 *
          raspberrypi_multicore_MPCtest_B.i_f2;
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.vcol]
          = 0.0;
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.vcol]
          += raspberrypi_multicore_MPCtest_B.U_a[3 *
          raspberrypi_multicore_MPCtest_B.i_f2] *
          raspberrypi_multicore_MPCtest_B.V[raspberrypi_multicore_MPCtest_B.ar];
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.vcol]
          += raspberrypi_multicore_MPCtest_B.U_a[3 *
          raspberrypi_multicore_MPCtest_B.i_f2 + 1] *
          raspberrypi_multicore_MPCtest_B.V[raspberrypi_multicore_MPCtest_B.ar +
          3];
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.vcol]
          += raspberrypi_multicore_MPCtest_B.U_a[3 *
          raspberrypi_multicore_MPCtest_B.i_f2 + 2] *
          raspberrypi_multicore_MPCtest_B.V[raspberrypi_multicore_MPCtest_B.ar +
          6];
      }

      raspberrypi_multicore_MPCtest_B.deltaP3[raspberrypi_multicore_MPCtest_B.ar]
        = 0.0;
      for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
           raspberrypi_multicore_MPCtest_B.i_f2 < 3;
           raspberrypi_multicore_MPCtest_B.i_f2++) {
        raspberrypi_multicore_MPCtest_B.vcol =
          raspberrypi_multicore_MPCtest_B.ar + 3 *
          raspberrypi_multicore_MPCtest_B.i_f2;
        raspberrypi_multicore_MPCtest_B.MRz_i[raspberrypi_multicore_MPCtest_B.vcol]
          = 0.0;
        raspberrypi_multicore_MPCtest_B.MRz_i[raspberrypi_multicore_MPCtest_B.vcol]
          += raspberrypi_multicore_MPCtest_B.R[3 *
          raspberrypi_multicore_MPCtest_B.i_f2] *
          raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar];
        raspberrypi_multicore_MPCtest_B.MRz_i[raspberrypi_multicore_MPCtest_B.vcol]
          += raspberrypi_multicore_MPCtest_B.R[3 *
          raspberrypi_multicore_MPCtest_B.i_f2 + 1] *
          raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar
          + 3];
        raspberrypi_multicore_MPCtest_B.MRz_i[raspberrypi_multicore_MPCtest_B.vcol]
          += raspberrypi_multicore_MPCtest_B.R[3 *
          raspberrypi_multicore_MPCtest_B.i_f2 + 2] *
          raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar
          + 6];
        raspberrypi_multicore_MPCtest_B.deltaP3[raspberrypi_multicore_MPCtest_B.ar]
          +=
          raspberrypi_multicore_MPCtest_B.MRz_i[raspberrypi_multicore_MPCtest_B.vcol]
          * raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_m[raspberrypi_multicore_MPCtest_B.i_f2];
      }
    }

    memset(&raspberrypi_multicore_MPCtest_B.MRz[0], 0, 9U * sizeof(real_T));
    raspberrypi_multicore_MPCtest_B.MRz[0] =
      raspberrypi_multicore_MPCtest_B.deltaP4[0];
    raspberrypi_multicore_MPCtest_B.MRz[4] =
      raspberrypi_multicore_MPCtest_B.deltaP4[1];
    raspberrypi_multicore_MPCtest_B.MRz[8] =
      raspberrypi_multicore_MPCtest_B.deltaP4[2];
    for (raspberrypi_multicore_MPCtest_B.ar = 0;
         raspberrypi_multicore_MPCtest_B.ar < 9;
         raspberrypi_multicore_MPCtest_B.ar++) {
      raspberrypi_multicore_MPCtest_B.V[raspberrypi_multicore_MPCtest_B.ar] =
        -raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar]
        / 1000.0;
    }

    raspberrypi_multicore_MPCtest_B.Angle3new_c =
      raspberrypi_multicore_MPCtes_DW.obj_f.BC *
      raspberrypi_multicore_MPCtest_B.t7_a;
    raspberrypi_multicore_MPCtest_B.U_a[0] =
      raspberrypi_multicore_MPCtest_B.Angle3new_c;
    raspberrypi_multicore_MPCtest_B.t4 =
      -raspberrypi_multicore_MPCtes_DW.obj_f.BC * cos
      (raspberrypi_multicore_MPCtest_B.Angle4new[0]);
    raspberrypi_multicore_MPCtest_B.U_a[3] = raspberrypi_multicore_MPCtest_B.t4 *
      raspberrypi_multicore_MPCtest_B.t9_m;
    raspberrypi_multicore_MPCtest_B.U_a[6] = raspberrypi_multicore_MPCtest_B.t4 *
      raspberrypi_multicore_MPCtest_B.t6_d;
    raspberrypi_multicore_MPCtest_B.U_a[1] =
      -raspberrypi_multicore_MPCtes_DW.obj_f.DP * sin
      (raspberrypi_multicore_MPCtest_B.t11_o);
    raspberrypi_multicore_MPCtest_B.U_a[4] =
      raspberrypi_multicore_MPCtes_DW.obj_f.DP *
      raspberrypi_multicore_MPCtest_B.t9_m *
      raspberrypi_multicore_MPCtest_B.t12_n;
    raspberrypi_multicore_MPCtest_B.U_a[7] =
      raspberrypi_multicore_MPCtes_DW.obj_f.DP *
      raspberrypi_multicore_MPCtest_B.t6_d *
      raspberrypi_multicore_MPCtest_B.t12_n;
    raspberrypi_multicore_MPCtest_B.U_a[2] = 0.0;
    raspberrypi_multicore_MPCtest_B.t10 *= cos
      (raspberrypi_multicore_MPCtest_B.Angle4new[1]);
    raspberrypi_multicore_MPCtest_B.U_a[5] =
      ((raspberrypi_multicore_MPCtes_DW.obj_f.OR *
        raspberrypi_multicore_MPCtest_B.t9_m -
        raspberrypi_multicore_MPCtes_DW.obj_f.BC *
        raspberrypi_multicore_MPCtest_B.t6_d *
        raspberrypi_multicore_MPCtest_B.t7_a) -
       raspberrypi_multicore_MPCtest_B.t10 *
       raspberrypi_multicore_MPCtest_B.t6_d) +
      raspberrypi_multicore_MPCtest_B.t11 * raspberrypi_multicore_MPCtest_B.t6_d
      * raspberrypi_multicore_MPCtest_B.t8_p;
    raspberrypi_multicore_MPCtest_B.U_a[8] =
      ((raspberrypi_multicore_MPCtest_B.Angle3new_c *
        raspberrypi_multicore_MPCtest_B.t9_m +
        raspberrypi_multicore_MPCtes_DW.obj_f.OR *
        raspberrypi_multicore_MPCtest_B.t6_d) +
       raspberrypi_multicore_MPCtest_B.t10 *
       raspberrypi_multicore_MPCtest_B.t9_m) -
      raspberrypi_multicore_MPCtest_B.t11 * raspberrypi_multicore_MPCtest_B.t8_p
      * raspberrypi_multicore_MPCtest_B.t9_m;
    raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_m[0] =
      raspberrypi_multicore_MPCtest_B.pArray_L_Adm[9];
    raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_m[1] =
      raspberrypi_multicore_MPCtest_B.pArray_L_Adm[10];
    raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_m[2] =
      raspberrypi_multicore_MPCtest_B.pArray_L_Adm[11];
    for (raspberrypi_multicore_MPCtest_B.ar = 0;
         raspberrypi_multicore_MPCtest_B.ar < 3;
         raspberrypi_multicore_MPCtest_B.ar++) {
      for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
           raspberrypi_multicore_MPCtest_B.i_f2 < 3;
           raspberrypi_multicore_MPCtest_B.i_f2++) {
        raspberrypi_multicore_MPCtest_B.vcol =
          raspberrypi_multicore_MPCtest_B.ar + 3 *
          raspberrypi_multicore_MPCtest_B.i_f2;
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.vcol]
          = 0.0;
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.vcol]
          += raspberrypi_multicore_MPCtest_B.U_a[3 *
          raspberrypi_multicore_MPCtest_B.i_f2] *
          raspberrypi_multicore_MPCtest_B.V[raspberrypi_multicore_MPCtest_B.ar];
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.vcol]
          += raspberrypi_multicore_MPCtest_B.U_a[3 *
          raspberrypi_multicore_MPCtest_B.i_f2 + 1] *
          raspberrypi_multicore_MPCtest_B.V[raspberrypi_multicore_MPCtest_B.ar +
          3];
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.vcol]
          += raspberrypi_multicore_MPCtest_B.U_a[3 *
          raspberrypi_multicore_MPCtest_B.i_f2 + 2] *
          raspberrypi_multicore_MPCtest_B.V[raspberrypi_multicore_MPCtest_B.ar +
          6];
      }

      raspberrypi_multicore_MPCtest_B.deltaP4[raspberrypi_multicore_MPCtest_B.ar]
        = 0.0;
      for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
           raspberrypi_multicore_MPCtest_B.i_f2 < 3;
           raspberrypi_multicore_MPCtest_B.i_f2++) {
        raspberrypi_multicore_MPCtest_B.vcol =
          raspberrypi_multicore_MPCtest_B.ar + 3 *
          raspberrypi_multicore_MPCtest_B.i_f2;
        raspberrypi_multicore_MPCtest_B.MRz_i[raspberrypi_multicore_MPCtest_B.vcol]
          = 0.0;
        raspberrypi_multicore_MPCtest_B.MRz_i[raspberrypi_multicore_MPCtest_B.vcol]
          += raspberrypi_multicore_MPCtest_B.R[3 *
          raspberrypi_multicore_MPCtest_B.i_f2] *
          raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar];
        raspberrypi_multicore_MPCtest_B.MRz_i[raspberrypi_multicore_MPCtest_B.vcol]
          += raspberrypi_multicore_MPCtest_B.R[3 *
          raspberrypi_multicore_MPCtest_B.i_f2 + 1] *
          raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar
          + 3];
        raspberrypi_multicore_MPCtest_B.MRz_i[raspberrypi_multicore_MPCtest_B.vcol]
          += raspberrypi_multicore_MPCtest_B.R[3 *
          raspberrypi_multicore_MPCtest_B.i_f2 + 2] *
          raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar
          + 6];
        raspberrypi_multicore_MPCtest_B.deltaP4[raspberrypi_multicore_MPCtest_B.ar]
          +=
          raspberrypi_multicore_MPCtest_B.MRz_i[raspberrypi_multicore_MPCtest_B.vcol]
          * raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_m[raspberrypi_multicore_MPCtest_B.i_f2];
      }
    }
  }

  raspberrypi_multicore_MPCtest_B.t11_g = raspberrypi_multicore_MPCtest_B.a[0] +
    raspberrypi_multicore_MPCtest_B.v1[0];
  raspberrypi_multicore_MPCtest_B.t12 =
    raspberrypi_multicore_MPCtest_B.Angle2new[0] +
    raspberrypi_multicore_MPCtest_B.deltaP2[0];
  raspberrypi_multicore_MPCtest_B.Angle3new_c =
    raspberrypi_multicore_MPCtest_B.Angle3new[0] +
    raspberrypi_multicore_MPCtest_B.deltaP3[0];
  raspberrypi_multicore_MPCtest_B.t4_j =
    raspberrypi_multicore_MPCtest_B.Angle4new[0] +
    raspberrypi_multicore_MPCtest_B.deltaP4[0];
  raspberrypi_multicore_MPCtest_B.t6_e = raspberrypi_multicore_MPCtest_B.a[1] +
    raspberrypi_multicore_MPCtest_B.v1[1];
  raspberrypi_multicore_MPCtest_B.t6_ex =
    raspberrypi_multicore_MPCtest_B.Angle2new[1] +
    raspberrypi_multicore_MPCtest_B.deltaP2[1];
  raspberrypi_multicore_MPCtest_B.t6_d =
    raspberrypi_multicore_MPCtest_B.Angle3new[1] +
    raspberrypi_multicore_MPCtest_B.deltaP3[1];
  raspberrypi_multicore_MPCtest_B.t11_o =
    raspberrypi_multicore_MPCtest_B.Angle4new[1] +
    raspberrypi_multicore_MPCtest_B.deltaP4[1];
  raspberrypi_multicore_MPCtest_B.t8 = raspberrypi_multicore_MPCtest_B.a[2] +
    raspberrypi_multicore_MPCtest_B.v1[2];
  raspberrypi_multicore_MPCtest_B.t8_b =
    raspberrypi_multicore_MPCtest_B.Angle2new[2] +
    raspberrypi_multicore_MPCtest_B.deltaP2[2];
  raspberrypi_multicore_MPCtest_B.t8_h =
    raspberrypi_multicore_MPCtest_B.Angle3new[2] +
    raspberrypi_multicore_MPCtest_B.deltaP3[2];
  raspberrypi_multicore_MPCtest_B.t8_p =
    raspberrypi_multicore_MPCtest_B.Angle4new[2] +
    raspberrypi_multicore_MPCtest_B.deltaP4[2];

  /* AUTOGEN_FK_1 */
  /*     PP = AUTOGEN_FK_1(LOA,LAB,LAD,LDP,CDP,THETA1,THETA2,THETA3) */
  /*     This function was generated by the Symbolic Math Toolbox version 8.7. */
  /*     12-Aug-2021 16:19:08 */
  raspberrypi_multicore_MPCtest_B.t11_a = cos(raspberrypi_multicore_MPCtest_B.t8);
  raspberrypi_multicore_MPCtest_B.t6 = sin(raspberrypi_multicore_MPCtest_B.t11_g);
  raspberrypi_multicore_MPCtest_B.t7 = sin(raspberrypi_multicore_MPCtest_B.t6_e);
  raspberrypi_multicore_MPCtest_B.t8 = sin(raspberrypi_multicore_MPCtest_B.t8);
  raspberrypi_multicore_MPCtest_B.t9 = raspberrypi_multicore_MPCtest_B.t11_a *
    raspberrypi_multicore_MPCtest_B.t11_a;
  raspberrypi_multicore_MPCtest_B.t10 = raspberrypi_multicore_MPCtest_B.t8 *
    raspberrypi_multicore_MPCtest_B.t8;
  raspberrypi_multicore_MPCtest_B.t12_n = raspberrypi_multicore_MPCtest_B.t9 +
    raspberrypi_multicore_MPCtest_B.t10;
  raspberrypi_multicore_MPCtest_B.t11 = (cos
    (raspberrypi_multicore_MPCtest_B.t6_e) - 1.0) *
    raspberrypi_multicore_MPCtest_B.t12_n + 1.0;

  /* AUTOGEN_FK_2 */
  /*     PP = AUTOGEN_FK_2(LOA,LAB,LAD,LDP,CDP,THETA1,THETA2,THETA3) */
  /*     This function was generated by the Symbolic Math Toolbox version 8.7. */
  /*     12-Aug-2021 16:19:09 */
  raspberrypi_multicore_MPCtest_B.t4 = cos(raspberrypi_multicore_MPCtest_B.t8_b);
  raspberrypi_multicore_MPCtest_B.t6_e = sin(raspberrypi_multicore_MPCtest_B.t12);
  raspberrypi_multicore_MPCtest_B.t7_o = sin
    (raspberrypi_multicore_MPCtest_B.t6_ex);
  raspberrypi_multicore_MPCtest_B.t8_b = sin
    (raspberrypi_multicore_MPCtest_B.t8_b);
  raspberrypi_multicore_MPCtest_B.t9_a = raspberrypi_multicore_MPCtest_B.t4 *
    raspberrypi_multicore_MPCtest_B.t4;
  raspberrypi_multicore_MPCtest_B.t10_c = raspberrypi_multicore_MPCtest_B.t8_b *
    raspberrypi_multicore_MPCtest_B.t8_b;
  raspberrypi_multicore_MPCtest_B.t14 = raspberrypi_multicore_MPCtest_B.t9_a +
    raspberrypi_multicore_MPCtest_B.t10_c;
  raspberrypi_multicore_MPCtest_B.t16 = (cos
    (raspberrypi_multicore_MPCtest_B.t6_ex) - 1.0) *
    raspberrypi_multicore_MPCtest_B.t14 + 1.0;

  /* AUTOGEN_FK_3 */
  /*     PP = AUTOGEN_FK_3(LOA,LAB,LAD,LDP,CDP,THETA1,THETA2,THETA3) */
  /*     This function was generated by the Symbolic Math Toolbox version 8.7. */
  /*     12-Aug-2021 16:19:09 */
  raspberrypi_multicore_MPCtest_B.t5 = cos(raspberrypi_multicore_MPCtest_B.t8_h);
  raspberrypi_multicore_MPCtest_B.t6_ex = sin
    (raspberrypi_multicore_MPCtest_B.Angle3new_c);
  raspberrypi_multicore_MPCtest_B.t7_f = sin
    (raspberrypi_multicore_MPCtest_B.t6_d);
  raspberrypi_multicore_MPCtest_B.t8_h = sin
    (raspberrypi_multicore_MPCtest_B.t8_h);
  raspberrypi_multicore_MPCtest_B.t9_e = raspberrypi_multicore_MPCtest_B.t5 *
    raspberrypi_multicore_MPCtest_B.t5;
  raspberrypi_multicore_MPCtest_B.t10_l = raspberrypi_multicore_MPCtest_B.t8_h *
    raspberrypi_multicore_MPCtest_B.t8_h;
  raspberrypi_multicore_MPCtest_B.t14_p = raspberrypi_multicore_MPCtest_B.t9_e +
    raspberrypi_multicore_MPCtest_B.t10_l;
  raspberrypi_multicore_MPCtest_B.t16_p = (cos
    (raspberrypi_multicore_MPCtest_B.t6_d) - 1.0) *
    raspberrypi_multicore_MPCtest_B.t14_p + 1.0;

  /* AUTOGEN_FK_4 */
  /*     PP = AUTOGEN_FK_4(LOA,LAB,LAD,LDP,CDP,THETA1,THETA2,THETA3) */
  /*     This function was generated by the Symbolic Math Toolbox version 8.7. */
  /*     12-Aug-2021 16:19:09 */
  raspberrypi_multicore_MPCtest_B.t5_f = cos
    (raspberrypi_multicore_MPCtest_B.t8_p);
  raspberrypi_multicore_MPCtest_B.t6_d = sin
    (raspberrypi_multicore_MPCtest_B.t4_j);
  raspberrypi_multicore_MPCtest_B.t7_a = sin
    (raspberrypi_multicore_MPCtest_B.t11_o);
  raspberrypi_multicore_MPCtest_B.t8_p = sin
    (raspberrypi_multicore_MPCtest_B.t8_p);
  raspberrypi_multicore_MPCtest_B.t9_m = raspberrypi_multicore_MPCtest_B.t5_f *
    raspberrypi_multicore_MPCtest_B.t5_f;
  raspberrypi_multicore_MPCtest_B.t10_i = raspberrypi_multicore_MPCtest_B.t8_p *
    raspberrypi_multicore_MPCtest_B.t8_p;
  raspberrypi_multicore_MPCtest_B.t14_o = raspberrypi_multicore_MPCtest_B.t9_m +
    raspberrypi_multicore_MPCtest_B.t10_i;
  raspberrypi_multicore_MPCtest_B.t11_o = (cos
    (raspberrypi_multicore_MPCtest_B.t11_o) - 1.0) *
    raspberrypi_multicore_MPCtest_B.t14_o + 1.0;
  raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp_tmp =
    raspberrypi_multicore_MPCtes_DW.obj_f.AB *
    raspberrypi_multicore_MPCtest_B.s_k;
  raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp =
    raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp_tmp *
    raspberrypi_multicore_MPCtest_B.t7;
  raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp_tmp_h =
    raspberrypi_multicore_MPCtes_DW.obj_f.DP * (1.0 /
    raspberrypi_multicore_MPCtes_DW.obj_f.AB);
  raspberrypi_multicore_MPCtest_B.pArray_L_Adm[0] =
    ((((raspberrypi_multicore_MPCtest_B.s - 1.0) *
       raspberrypi_multicore_MPCtest_B.t12_n + 1.0) *
      (-raspberrypi_multicore_MPCtes_DW.obj_f.AB *
       raspberrypi_multicore_MPCtest_B.t11) +
      raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp *
      raspberrypi_multicore_MPCtest_B.t9) +
     raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp *
     raspberrypi_multicore_MPCtest_B.t10) *
    raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp_tmp_h + ((cos
    (raspberrypi_multicore_MPCtest_B.t11_g) - 1.0) *
    raspberrypi_multicore_MPCtest_B.t12_n + 1.0) *
    -raspberrypi_multicore_MPCtes_DW.obj_f.BC;
  raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp =
    raspberrypi_multicore_MPCtes_DW.obj_f.AB *
    raspberrypi_multicore_MPCtest_B.t7 * raspberrypi_multicore_MPCtest_B.t8;
  raspberrypi_multicore_MPCtest_B.pArray_L_Adm[1] =
    ((((raspberrypi_multicore_MPCtest_B.s - 1.0) *
       raspberrypi_multicore_MPCtest_B.t10 + 1.0) *
      raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp +
      raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp_tmp *
      raspberrypi_multicore_MPCtest_B.t8 * raspberrypi_multicore_MPCtest_B.t11)
     + raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp *
     raspberrypi_multicore_MPCtest_B.t9 * (raspberrypi_multicore_MPCtest_B.s -
      1.0)) * raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp_tmp_h +
    (raspberrypi_multicore_MPCtes_DW.obj_f.OR *
     raspberrypi_multicore_MPCtest_B.t11_a -
     raspberrypi_multicore_MPCtes_DW.obj_f.BC *
     raspberrypi_multicore_MPCtest_B.t6 * raspberrypi_multicore_MPCtest_B.t8);
  raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp =
    raspberrypi_multicore_MPCtes_DW.obj_f.AB *
    raspberrypi_multicore_MPCtest_B.t11_a * raspberrypi_multicore_MPCtest_B.t7;
  raspberrypi_multicore_MPCtest_B.pArray_L_Adm[2] =
    (raspberrypi_multicore_MPCtes_DW.obj_f.BC *
     raspberrypi_multicore_MPCtest_B.t11_a * raspberrypi_multicore_MPCtest_B.t6
     + raspberrypi_multicore_MPCtes_DW.obj_f.OR *
     raspberrypi_multicore_MPCtest_B.t8) - ((((raspberrypi_multicore_MPCtest_B.s
    - 1.0) * raspberrypi_multicore_MPCtest_B.t9 + 1.0) *
    raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp +
    raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp_tmp *
    raspberrypi_multicore_MPCtest_B.t11_a * raspberrypi_multicore_MPCtest_B.t11)
    + raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp *
    raspberrypi_multicore_MPCtest_B.t10 * (raspberrypi_multicore_MPCtest_B.s -
    1.0)) * raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp_tmp_h;
  raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp =
    raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp_tmp *
    raspberrypi_multicore_MPCtest_B.t7_o;
  raspberrypi_multicore_MPCtest_B.pArray_L_Adm[3] = ((cos
    (raspberrypi_multicore_MPCtest_B.t12) - 1.0) *
    raspberrypi_multicore_MPCtest_B.t14 + 1.0) *
    -raspberrypi_multicore_MPCtes_DW.obj_f.BC -
    ((((raspberrypi_multicore_MPCtest_B.s - 1.0) *
       raspberrypi_multicore_MPCtest_B.t14 + 1.0) *
      (raspberrypi_multicore_MPCtes_DW.obj_f.AB *
       raspberrypi_multicore_MPCtest_B.t16) +
      raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp *
      raspberrypi_multicore_MPCtest_B.t9_a) +
     raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp *
     raspberrypi_multicore_MPCtest_B.t10_c) *
    raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp_tmp_h;
  raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp =
    raspberrypi_multicore_MPCtes_DW.obj_f.AB *
    raspberrypi_multicore_MPCtest_B.t7_o * raspberrypi_multicore_MPCtest_B.t8_b;
  raspberrypi_multicore_MPCtest_B.s_k *=
    -raspberrypi_multicore_MPCtes_DW.obj_f.AB;
  raspberrypi_multicore_MPCtest_B.pArray_L_Adm[4] =
    (raspberrypi_multicore_MPCtes_DW.obj_f.BC *
     raspberrypi_multicore_MPCtest_B.t6_e * raspberrypi_multicore_MPCtest_B.t8_b
     + -raspberrypi_multicore_MPCtes_DW.obj_f.OR *
     raspberrypi_multicore_MPCtest_B.t4) - ((raspberrypi_multicore_MPCtest_B.s_k
    * raspberrypi_multicore_MPCtest_B.t8_b * raspberrypi_multicore_MPCtest_B.t16
    + raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp *
    (raspberrypi_multicore_MPCtest_B.t10_c * (raspberrypi_multicore_MPCtest_B.s
    - 1.0) + 1.0)) + raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp *
    raspberrypi_multicore_MPCtest_B.t9_a * (raspberrypi_multicore_MPCtest_B.s -
    1.0)) * raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp_tmp_h;
  raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp =
    raspberrypi_multicore_MPCtes_DW.obj_f.AB *
    raspberrypi_multicore_MPCtest_B.t4 * raspberrypi_multicore_MPCtest_B.t7_o;
  raspberrypi_multicore_MPCtest_B.pArray_L_Adm[5] =
    ((raspberrypi_multicore_MPCtest_B.s_k * raspberrypi_multicore_MPCtest_B.t4 *
      raspberrypi_multicore_MPCtest_B.t16 +
      raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp *
      (raspberrypi_multicore_MPCtest_B.t9_a * (raspberrypi_multicore_MPCtest_B.s
        - 1.0) + 1.0)) + raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp *
     raspberrypi_multicore_MPCtest_B.t10_c * (raspberrypi_multicore_MPCtest_B.s
      - 1.0)) * raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp_tmp_h +
    (-raspberrypi_multicore_MPCtes_DW.obj_f.OR *
     raspberrypi_multicore_MPCtest_B.t8_b -
     raspberrypi_multicore_MPCtes_DW.obj_f.BC *
     raspberrypi_multicore_MPCtest_B.t4 * raspberrypi_multicore_MPCtest_B.t6_e);
  raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp =
    raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp_tmp *
    raspberrypi_multicore_MPCtest_B.t7_f;
  raspberrypi_multicore_MPCtest_B.pArray_L_Adm[6] =
    ((((raspberrypi_multicore_MPCtest_B.s - 1.0) *
       raspberrypi_multicore_MPCtest_B.t14_p + 1.0) *
      (-raspberrypi_multicore_MPCtes_DW.obj_f.AB *
       raspberrypi_multicore_MPCtest_B.t16_p) +
      raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp *
      raspberrypi_multicore_MPCtest_B.t9_e) +
     raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp *
     raspberrypi_multicore_MPCtest_B.t10_l) *
    raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp_tmp_h + ((cos
    (raspberrypi_multicore_MPCtest_B.Angle3new_c) - 1.0) *
    raspberrypi_multicore_MPCtest_B.t14_p + 1.0) *
    -raspberrypi_multicore_MPCtes_DW.obj_f.BC;
  raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp =
    raspberrypi_multicore_MPCtes_DW.obj_f.AB *
    raspberrypi_multicore_MPCtest_B.t7_f * raspberrypi_multicore_MPCtest_B.t8_h;
  raspberrypi_multicore_MPCtest_B.pArray_L_Adm[7] =
    (raspberrypi_multicore_MPCtes_DW.obj_f.BC *
     raspberrypi_multicore_MPCtest_B.t6_ex *
     raspberrypi_multicore_MPCtest_B.t8_h +
     raspberrypi_multicore_MPCtes_DW.obj_f.OR *
     raspberrypi_multicore_MPCtest_B.t5) -
    ((raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp_tmp *
      raspberrypi_multicore_MPCtest_B.t8_h *
      raspberrypi_multicore_MPCtest_B.t16_p +
      raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp *
      (raspberrypi_multicore_MPCtest_B.t10_l *
       (raspberrypi_multicore_MPCtest_B.s - 1.0) + 1.0)) +
     raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp *
     raspberrypi_multicore_MPCtest_B.t9_e * (raspberrypi_multicore_MPCtest_B.s -
      1.0)) * raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp_tmp_h;
  raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp =
    raspberrypi_multicore_MPCtes_DW.obj_f.AB *
    raspberrypi_multicore_MPCtest_B.t5 * raspberrypi_multicore_MPCtest_B.t7_f;
  raspberrypi_multicore_MPCtest_B.pArray_L_Adm[8] =
    (raspberrypi_multicore_MPCtes_DW.obj_f.BC *
     raspberrypi_multicore_MPCtest_B.t5 * raspberrypi_multicore_MPCtest_B.t6_ex
     + -raspberrypi_multicore_MPCtes_DW.obj_f.OR *
     raspberrypi_multicore_MPCtest_B.t8_h) -
    ((raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp_tmp *
      raspberrypi_multicore_MPCtest_B.t5 * raspberrypi_multicore_MPCtest_B.t16_p
      + raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp *
      (raspberrypi_multicore_MPCtest_B.t9_e * (raspberrypi_multicore_MPCtest_B.s
        - 1.0) + 1.0)) + raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp *
     raspberrypi_multicore_MPCtest_B.t10_l * (raspberrypi_multicore_MPCtest_B.s
      - 1.0)) * raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp_tmp_h;
  raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp =
    raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp_tmp *
    raspberrypi_multicore_MPCtest_B.t7_a;
  raspberrypi_multicore_MPCtest_B.pArray_L_Adm[9] = ((cos
    (raspberrypi_multicore_MPCtest_B.t4_j) - 1.0) *
    raspberrypi_multicore_MPCtest_B.t14_o + 1.0) *
    -raspberrypi_multicore_MPCtes_DW.obj_f.BC -
    ((((raspberrypi_multicore_MPCtest_B.s - 1.0) *
       raspberrypi_multicore_MPCtest_B.t14_o + 1.0) *
      (raspberrypi_multicore_MPCtes_DW.obj_f.AB *
       raspberrypi_multicore_MPCtest_B.t11_o) +
      raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp *
      raspberrypi_multicore_MPCtest_B.t9_m) +
     raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp *
     raspberrypi_multicore_MPCtest_B.t10_i) *
    raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp_tmp_h;
  raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp =
    raspberrypi_multicore_MPCtes_DW.obj_f.AB *
    raspberrypi_multicore_MPCtest_B.t7_a * raspberrypi_multicore_MPCtest_B.t8_p;
  raspberrypi_multicore_MPCtest_B.pArray_L_Adm[10] =
    ((raspberrypi_multicore_MPCtest_B.s_k * raspberrypi_multicore_MPCtest_B.t8_p
      * raspberrypi_multicore_MPCtest_B.t11_o +
      raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp *
      (raspberrypi_multicore_MPCtest_B.t10_i *
       (raspberrypi_multicore_MPCtest_B.s - 1.0) + 1.0)) +
     raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp *
     raspberrypi_multicore_MPCtest_B.t9_m * (raspberrypi_multicore_MPCtest_B.s -
      1.0)) * raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp_tmp_h +
    (-raspberrypi_multicore_MPCtes_DW.obj_f.OR *
     raspberrypi_multicore_MPCtest_B.t5_f -
     raspberrypi_multicore_MPCtes_DW.obj_f.BC *
     raspberrypi_multicore_MPCtest_B.t6_d * raspberrypi_multicore_MPCtest_B.t8_p);
  raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp =
    raspberrypi_multicore_MPCtes_DW.obj_f.AB *
    raspberrypi_multicore_MPCtest_B.t5_f * raspberrypi_multicore_MPCtest_B.t7_a;
  raspberrypi_multicore_MPCtest_B.pArray_L_Adm[11] =
    ((raspberrypi_multicore_MPCtest_B.s_k * raspberrypi_multicore_MPCtest_B.t5_f
      * raspberrypi_multicore_MPCtest_B.t11_o +
      raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp *
      (raspberrypi_multicore_MPCtest_B.t9_m * (raspberrypi_multicore_MPCtest_B.s
        - 1.0) + 1.0)) + raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp *
     raspberrypi_multicore_MPCtest_B.t10_i * (raspberrypi_multicore_MPCtest_B.s
      - 1.0)) * raspberrypi_multicore_MPCtest_B.rtb_pArray_L_Adm_tmp_tmp_h +
    (raspberrypi_multicore_MPCtes_DW.obj_f.OR *
     raspberrypi_multicore_MPCtest_B.t8_p -
     raspberrypi_multicore_MPCtes_DW.obj_f.BC *
     raspberrypi_multicore_MPCtest_B.t5_f * raspberrypi_multicore_MPCtest_B.t6_d);

  /* End of MATLABSystem: '<Root>/MATLAB System2' */

  /* MATLABSystem: '<Root>/MATLAB System17' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function4'
   *  MATLABSystem: '<Root>/MATLAB System10'
   *  MATLABSystem: '<Root>/MATLAB System11'
   *  MATLABSystem: '<Root>/MATLAB System2'
   *  UnitDelay: '<Root>/Unit Delay6'
   */
  if (!raspberrypi_multicore_M_isequal(raspberrypi_multicore_MPCtes_DW.obj_l.r0,
       raspberrypi_multicore_MPCtest_P.MATLABSystem17_r0)) {
    raspberrypi_multicore_MPCtes_DW.obj_l.r0[0] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_r0[0];
    raspberrypi_multicore_MPCtes_DW.obj_l.r0[1] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_r0[1];
    raspberrypi_multicore_MPCtes_DW.obj_l.r0[2] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_r0[2];
  }

  if (!raspberrypi_multicore_M_isequal
      (raspberrypi_multicore_MPCtes_DW.obj_l.theta0,
       raspberrypi_multicore_MPCtest_P.MATLABSystem17_theta0)) {
    raspberrypi_multicore_MPCtes_DW.obj_l.theta0[0] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_theta0[0];
    raspberrypi_multicore_MPCtes_DW.obj_l.theta0[1] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_theta0[1];
    raspberrypi_multicore_MPCtes_DW.obj_l.theta0[2] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_theta0[2];
  }

  if (!raspberrypi_multicore_M_isequal(raspberrypi_multicore_MPCtes_DW.obj_l.dr0,
       raspberrypi_multicore_MPCtest_P.MATLABSystem17_dr0)) {
    raspberrypi_multicore_MPCtes_DW.obj_l.dr0[0] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_dr0[0];
    raspberrypi_multicore_MPCtes_DW.obj_l.dr0[1] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_dr0[1];
    raspberrypi_multicore_MPCtes_DW.obj_l.dr0[2] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_dr0[2];
  }

  if (!raspberrypi_multicore_M_isequal
      (raspberrypi_multicore_MPCtes_DW.obj_l.omega0,
       raspberrypi_multicore_MPCtest_P.MATLABSystem17_omega0)) {
    raspberrypi_multicore_MPCtes_DW.obj_l.omega0[0] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_omega0[0];
    raspberrypi_multicore_MPCtes_DW.obj_l.omega0[1] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_omega0[1];
    raspberrypi_multicore_MPCtes_DW.obj_l.omega0[2] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_omega0[2];
  }

  if (raspberrypi_multicore_MPCtes_DW.obj_l.lateral_width !=
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_lateral_width) {
    raspberrypi_multicore_MPCtes_DW.obj_l.lateral_width =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_lateral_width;
  }

  if (raspberrypi_multicore_MPCtes_DW.obj_l.sagetial_width !=
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_sagetial_width) {
    raspberrypi_multicore_MPCtes_DW.obj_l.sagetial_width =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_sagetial_width;
  }

  if (raspberrypi_multicore_MPCtes_DW.obj_l.roll_Off !=
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_roll_Off) {
    raspberrypi_multicore_MPCtes_DW.obj_l.roll_Off =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_roll_Off;
  }

  if (raspberrypi_multicore_MPCtes_DW.obj_l.m !=
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_m) {
    raspberrypi_multicore_MPCtes_DW.obj_l.m =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_m;
  }

  if (raspberrypi_multicore_MPCtes_DW.obj_l.kx !=
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_kx) {
    raspberrypi_multicore_MPCtes_DW.obj_l.kx =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_kx;
  }

  if (raspberrypi_multicore_MPCtes_DW.obj_l.ky !=
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_ky) {
    raspberrypi_multicore_MPCtes_DW.obj_l.ky =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_ky;
  }

  if (raspberrypi_multicore_MPCtes_DW.obj_l.kRz !=
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_kRz) {
    raspberrypi_multicore_MPCtes_DW.obj_l.kRz =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_kRz;
  }

  if (raspberrypi_multicore_MPCtes_DW.obj_l.T !=
      raspberrypi_multicore_MPCtest_P.T_gait) {
    raspberrypi_multicore_MPCtes_DW.obj_l.T =
      raspberrypi_multicore_MPCtest_P.T_gait;
  }

  if (raspberrypi_multicore_MPCtes_DW.obj_l.StepH !=
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_StepH) {
    raspberrypi_multicore_MPCtes_DW.obj_l.StepH =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_StepH;
  }

  if (raspberrypi_multicore_MPCtes_DW.obj_l.SampleTime !=
      raspberrypi_multicore_MPCtest_P.Ts_DynSim) {
    raspberrypi_multicore_MPCtes_DW.obj_l.SampleTime =
      raspberrypi_multicore_MPCtest_P.Ts_DynSim;
  }

  if (raspberrypi_multicore_MPCtes_DW.obj_l.OffsetTime !=
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_OffsetTime) {
    raspberrypi_multicore_MPCtes_DW.obj_l.OffsetTime =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_OffsetTime;
  }

  if (raspberrypi_multicore_MPCtes_DW.obj_l.TickTime !=
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_TickTime) {
    raspberrypi_multicore_MPCtes_DW.obj_l.TickTime =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_TickTime;
  }

  if (raspberrypi_multicore_MPCtes_DW.obj_l.startPhase !=
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_startPhase) {
    raspberrypi_multicore_MPCtes_DW.obj_l.startPhase =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_startPhase;
  }

  /*  X_FB: system states from the estimator */
  /*  X_mpc: predicted next step's systems states from the MPC controller */
  /*  touchInd: indicator of wether a swing leg touches the ground */
  /*  T is the moving period */
  raspberrypi_multicore_MPCtest_B.pxNew[0] = 1.0;
  raspberrypi_multicore_MPCtest_B.pxNew[1] = 0.0;
  raspberrypi_multicore_MPCtest_B.pxNew[2] = 0.0;
  raspberrypi_multicore_MPCtest_B.pxNew[3] = 1.0;
  raspberrypi_multicore_MPCtest_B.s =
    raspberrypi_multicore_MPCtest_B.b_varargout_4_a / 3.1415926535897931;
  raspberrypi_multicore_MPCtest_B.s_k = raspberrypi_multicore_MPCtest_B.s;
  if (raspberrypi_multicore_MPCtest_B.b_varargout_4_a > 3.1415926535897931) {
    raspberrypi_multicore_MPCtest_B.pxNew[0] = 0.0;
    raspberrypi_multicore_MPCtest_B.pxNew[1] = 1.0;
    raspberrypi_multicore_MPCtest_B.pxNew[2] = 1.0;
    raspberrypi_multicore_MPCtest_B.pxNew[3] = 0.0;
    raspberrypi_multicore_MPCtest_B.s_k = raspberrypi_multicore_MPCtest_B.s -
      1.0;
  }

  if (raspberrypi_multicore_MPCtest_B.MATLABSystem11_o2 > 0.5) {
    raspberrypi_multicore_MPCtest_B.pxNew[0] = 1.0;
    raspberrypi_multicore_MPCtest_B.pxNew[1] = 1.0;
    raspberrypi_multicore_MPCtest_B.pxNew[2] = 1.0;
    raspberrypi_multicore_MPCtest_B.pxNew[3] = 1.0;
  }

  /*  3D rotation matrix, vb=M*v: */
  /*  rotate a vector in one frame, */
  /*  or change the vector 'v' in rotated frame to 'vb' in world frame */
  raspberrypi_multicore_MPCtest_B.t6 = sin
    (raspberrypi_multicore_MPCtest_B.b_varargout_4[5]);
  raspberrypi_multicore_MPCtest_B.t7 = cos
    (raspberrypi_multicore_MPCtest_B.b_varargout_4[5]);
  raspberrypi_multicore_MPCtest_B.R[0] = raspberrypi_multicore_MPCtest_B.t7;
  raspberrypi_multicore_MPCtest_B.R[1] = -raspberrypi_multicore_MPCtest_B.t6;
  raspberrypi_multicore_MPCtest_B.R[2] = 0.0;
  raspberrypi_multicore_MPCtest_B.R[3] = raspberrypi_multicore_MPCtest_B.t6;
  raspberrypi_multicore_MPCtest_B.R[4] = raspberrypi_multicore_MPCtest_B.t7;
  raspberrypi_multicore_MPCtest_B.R[5] = 0.0;
  raspberrypi_multicore_MPCtest_B.t6 =
    raspberrypi_multicore_MPCtest_B.b_varargout_4[6];
  raspberrypi_multicore_MPCtest_B.t7 =
    raspberrypi_multicore_MPCtest_B.b_varargout_4[7];
  for (raspberrypi_multicore_MPCtest_B.ar = 0;
       raspberrypi_multicore_MPCtest_B.ar < 3;
       raspberrypi_multicore_MPCtest_B.ar++) {
    raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.ar + 6] =
      tmp_0[raspberrypi_multicore_MPCtest_B.ar];
    raspberrypi_multicore_MPCtest_B.v1[raspberrypi_multicore_MPCtest_B.ar] =
      raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.ar + 3] *
      raspberrypi_multicore_MPCtest_B.t7 +
      raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.ar] *
      raspberrypi_multicore_MPCtest_B.t6;
  }

  /*  Yet to ADD Rx and Ry !!!!!!!!!!!!!!!!! */
  /*  3D rotation matrix, vb=M*v: */
  /*  rotate a vector in one frame, */
  /*  or change the vector 'v' in rotated frame to 'vb' in world frame */
  raspberrypi_multicore_MPCtest_B.U_a[0] =
    raspberrypi_multicore_MPCtest_B.rtb_headG_tmp_tmp;
  raspberrypi_multicore_MPCtest_B.U_a[1] =
    -raspberrypi_multicore_MPCtest_B.rtb_headG_tmp_tmp_f;
  raspberrypi_multicore_MPCtest_B.U_a[2] = 0.0;
  raspberrypi_multicore_MPCtest_B.U_a[3] =
    raspberrypi_multicore_MPCtest_B.rtb_headG_tmp_tmp_f;
  raspberrypi_multicore_MPCtest_B.U_a[4] =
    raspberrypi_multicore_MPCtest_B.rtb_headG_tmp_tmp;
  raspberrypi_multicore_MPCtest_B.U_a[5] = 0.0;
  raspberrypi_multicore_MPCtest_B.U_a[6] = 0.0;
  raspberrypi_multicore_MPCtest_B.U_a[7] = 0.0;
  raspberrypi_multicore_MPCtest_B.U_a[8] = 1.0;
  raspberrypi_multicore_MPCtest_B.t8 =
    raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[6];
  raspberrypi_multicore_MPCtest_B.t9 =
    raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[7];

  /*  Yet to ADD Rx and Ry !!!!!!!!!!!!!!!!! */
  /* %% next step foot-placement in the leg coordinate */
  /*  dead zone for vNowL */
  for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
       raspberrypi_multicore_MPCtest_B.i_f2 < 3;
       raspberrypi_multicore_MPCtest_B.i_f2++) {
    raspberrypi_multicore_MPCtest_B.t6 =
      raspberrypi_multicore_MPCtest_B.U_a[raspberrypi_multicore_MPCtest_B.i_f2 +
      3] * raspberrypi_multicore_MPCtest_B.t9 +
      raspberrypi_multicore_MPCtest_B.U_a[raspberrypi_multicore_MPCtest_B.i_f2] *
      raspberrypi_multicore_MPCtest_B.t8;
    if (fabs(raspberrypi_multicore_MPCtest_B.t6) >= 0.02) {
      if (raspberrypi_multicore_MPCtest_B.t6 < 0.0) {
        raspberrypi_multicore_MPCtest_B.t7 = -1.0;
      } else if (raspberrypi_multicore_MPCtest_B.t6 > 0.0) {
        raspberrypi_multicore_MPCtest_B.t7 = 1.0;
      } else if (raspberrypi_multicore_MPCtest_B.t6 == 0.0) {
        raspberrypi_multicore_MPCtest_B.t7 = 0.0;
      } else {
        raspberrypi_multicore_MPCtest_B.t7 = (rtNaN);
      }

      raspberrypi_multicore_MPCtest_B.t6 -= raspberrypi_multicore_MPCtest_B.t7 *
        0.02;
    } else {
      raspberrypi_multicore_MPCtest_B.t6 = 0.0;
    }

    raspberrypi_multicore_MPCtest_B.deltaP2[raspberrypi_multicore_MPCtest_B.i_f2]
      = raspberrypi_multicore_MPCtest_B.t6;
  }

  /* v=vNow+[-obj.kx*(vDesL(1)-vNowL(1)); ... */
  /*     -obj.ky*(vDesL(2)-vNowL(2));0]; */
  for (raspberrypi_multicore_MPCtest_B.ar = 0;
       raspberrypi_multicore_MPCtest_B.ar < 19;
       raspberrypi_multicore_MPCtest_B.ar++) {
    raspberrypi_multicore_MPCtest_B.i_f2 = ((raspberrypi_multicore_MPCtest_B.ar
      + 2) - 1) * 3;
    raspberrypi_multicore_MPCtes_DW.obj_l.vNowN[3 *
      raspberrypi_multicore_MPCtest_B.ar] =
      raspberrypi_multicore_MPCtes_DW.obj_l.vNowN[raspberrypi_multicore_MPCtest_B.i_f2];
    raspberrypi_multicore_MPCtes_DW.obj_l.vNowN[3 *
      raspberrypi_multicore_MPCtest_B.ar + 1] =
      raspberrypi_multicore_MPCtes_DW.obj_l.vNowN[raspberrypi_multicore_MPCtest_B.i_f2
      + 1];
    raspberrypi_multicore_MPCtes_DW.obj_l.vNowN[3 *
      raspberrypi_multicore_MPCtest_B.ar + 2] =
      raspberrypi_multicore_MPCtes_DW.obj_l.vNowN[raspberrypi_multicore_MPCtest_B.i_f2
      + 2];
  }

  raspberrypi_multicore_MPCtes_DW.obj_l.vNowN[57] =
    raspberrypi_multicore_MPCtest_B.deltaP2[0];
  raspberrypi_multicore_MPCtes_DW.obj_l.vNowN[58] =
    raspberrypi_multicore_MPCtest_B.deltaP2[1];
  raspberrypi_multicore_MPCtes_DW.obj_l.vNowN[59] =
    raspberrypi_multicore_MPCtest_B.deltaP2[2];
  for (raspberrypi_multicore_MPCtest_B.ar = 0;
       raspberrypi_multicore_MPCtest_B.ar < 20;
       raspberrypi_multicore_MPCtest_B.ar++) {
    raspberrypi_multicore_MPCtest_B.i_f2 = raspberrypi_multicore_MPCtest_B.ar *
      3;
    raspberrypi_multicore_MPCtest_B.vNowFilt[raspberrypi_multicore_MPCtest_B.ar]
      =
      (raspberrypi_multicore_MPCtes_DW.obj_l.vNowN[raspberrypi_multicore_MPCtest_B.i_f2
       + 2] +
       (raspberrypi_multicore_MPCtes_DW.obj_l.vNowN[raspberrypi_multicore_MPCtest_B.i_f2
        + 1] +
        raspberrypi_multicore_MPCtes_DW.obj_l.vNowN[raspberrypi_multicore_MPCtest_B.i_f2]))
      / 20.0;
  }

  raspberrypi_multicore_MPCtest_B.t6 = ((raspberrypi_multicore_MPCtest_B.v1[0] -
    raspberrypi_multicore_MPCtest_B.vNowFilt[0]) *
    -raspberrypi_multicore_MPCtes_DW.obj_l.kx +
    raspberrypi_multicore_MPCtest_B.v1[0]) *
    raspberrypi_multicore_MPCtes_DW.obj_l.T / 4.0;
  raspberrypi_multicore_MPCtest_B.t7 = ((raspberrypi_multicore_MPCtest_B.v1[1] -
    raspberrypi_multicore_MPCtest_B.vNowFilt[1]) *
    -raspberrypi_multicore_MPCtes_DW.obj_l.ky +
    raspberrypi_multicore_MPCtest_B.v1[1]) *
    raspberrypi_multicore_MPCtes_DW.obj_l.T / 4.0;
  raspberrypi_multicore_MPCtest_B.t8 = raspberrypi_multicore_MPCtest_B.v1[2] *
    raspberrypi_multicore_MPCtes_DW.obj_l.T / 4.0;
  for (raspberrypi_multicore_MPCtest_B.ar = 0;
       raspberrypi_multicore_MPCtest_B.ar < 4;
       raspberrypi_multicore_MPCtest_B.ar++) {
    raspberrypi_multicore_MPCtest_B.desAllL[3 *
      raspberrypi_multicore_MPCtest_B.ar] =
      raspberrypi_multicore_MPCtes_DW.obj_l.pLnorm[3 *
      raspberrypi_multicore_MPCtest_B.ar] + raspberrypi_multicore_MPCtest_B.t6;
    raspberrypi_multicore_MPCtest_B.vcol = 3 *
      raspberrypi_multicore_MPCtest_B.ar + 1;
    raspberrypi_multicore_MPCtest_B.desAllL[raspberrypi_multicore_MPCtest_B.vcol]
      =
      raspberrypi_multicore_MPCtes_DW.obj_l.pLnorm[raspberrypi_multicore_MPCtest_B.vcol]
      + raspberrypi_multicore_MPCtest_B.t7;
    raspberrypi_multicore_MPCtest_B.vcol = 3 *
      raspberrypi_multicore_MPCtest_B.ar + 2;
    raspberrypi_multicore_MPCtest_B.desAllL[raspberrypi_multicore_MPCtest_B.vcol]
      =
      raspberrypi_multicore_MPCtes_DW.obj_l.pLnorm[raspberrypi_multicore_MPCtest_B.vcol]
      + raspberrypi_multicore_MPCtest_B.t8;
  }

  /*              desAll=Rz(desYaw)*Ry(desPit)*Rx(desRoll)*desAllL+[X_FB(1);X_FB(2);0]*[1,1,1,1]; */
  /*              for i=1:1:4 */
  /*                  desAll(3,i)=[1,desAll(1,i),desAll(2,i)]*[surP(1);surP(2);surP(3)]; */
  /*              end */
  /* %% next step foot-end position planning in the leg coordinate */
  memset(&raspberrypi_multicore_MPCtest_B.pL_sw[0], 0, 12U * sizeof(real_T));
  for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
       raspberrypi_multicore_MPCtest_B.i_f2 < 4;
       raspberrypi_multicore_MPCtest_B.i_f2++) {
    if (raspberrypi_multicore_MPCtest_B.pxNew[raspberrypi_multicore_MPCtest_B.i_f2]
        < 0.5) {
      raspberrypi_multicore_MPCtest_B.s_k *= 1.2;

      /*  to accelerate the swing trajectory tracing */
      if (raspberrypi_multicore_MPCtest_B.s_k >= 1.0) {
        raspberrypi_multicore_MPCtest_B.s_k = 1.0;
      }

      /*  a is the control point vector, s is the time vector */
      if (raspberrypi_multicore_MPCtest_B.s_k <= 0.5) {
        raspberrypi_multicore_MPCtest_B.t6 = 2.0 *
          raspberrypi_multicore_MPCtest_B.s_k;

        /*  a is the control point vector, s is the time vector */
        raspberrypi_multicore_MPCtest_B.ar = 3 *
          raspberrypi_multicore_MPCtest_B.i_f2 + 2;
        raspberrypi_multicore_MPCtest_B.t7 =
          raspberrypi_multicore_MPCtes_DW.obj_l.pL_LS[raspberrypi_multicore_MPCtest_B.ar];
        raspberrypi_multicore_MPCtest_B.t8 =
          (raspberrypi_multicore_MPCtes_DW.obj_l.pL_LS[raspberrypi_multicore_MPCtest_B.ar]
           + -0.196) / 2.0 + raspberrypi_multicore_MPCtes_DW.obj_l.StepH;
        raspberrypi_multicore_MPCtest_B.Angle3new_c =
          raspberrypi_multicore_factorial(3.0);
        raspberrypi_multicore_MPCtest_B.t9 = raspberrypi_multicore_factorial(2.0);
        raspberrypi_multicore_MPCtest_B.t6_e = raspberrypi_multicore_factorial
          (1.0);
        raspberrypi_multicore_MPCtest_B.t7_o = raspberrypi_multicore_factorial
          (0.0);
        raspberrypi_multicore_MPCtest_B.t6 =
          ((raspberrypi_multicore_MPCtest_B.Angle3new_c /
            raspberrypi_multicore_MPCtest_B.t7_o /
            raspberrypi_multicore_MPCtest_B.Angle3new_c * rt_powd_snf
            (raspberrypi_multicore_MPCtest_B.t6, 0.0) * rt_powd_snf(1.0 -
             raspberrypi_multicore_MPCtest_B.t6, 3.0) *
            raspberrypi_multicore_MPCtest_B.t7 +
            raspberrypi_multicore_MPCtest_B.Angle3new_c /
            raspberrypi_multicore_MPCtest_B.t6_e /
            raspberrypi_multicore_MPCtest_B.t9 * rt_powd_snf
            (raspberrypi_multicore_MPCtest_B.t6, 1.0) * rt_powd_snf(1.0 -
             raspberrypi_multicore_MPCtest_B.t6, 2.0) *
            raspberrypi_multicore_MPCtest_B.t7) +
           raspberrypi_multicore_MPCtest_B.t8 *
           (raspberrypi_multicore_MPCtest_B.Angle3new_c /
            raspberrypi_multicore_MPCtest_B.t9 /
            raspberrypi_multicore_MPCtest_B.t6_e * rt_powd_snf
            (raspberrypi_multicore_MPCtest_B.t6, 2.0) * rt_powd_snf(1.0 -
             raspberrypi_multicore_MPCtest_B.t6, 1.0))) +
          raspberrypi_multicore_MPCtest_B.t8 *
          (raspberrypi_multicore_MPCtest_B.Angle3new_c /
           raspberrypi_multicore_MPCtest_B.Angle3new_c /
           raspberrypi_multicore_MPCtest_B.t7_o * rt_powd_snf
           (raspberrypi_multicore_MPCtest_B.t6, 3.0));
      } else {
        raspberrypi_multicore_MPCtest_B.t6 =
          (raspberrypi_multicore_MPCtest_B.s_k - 0.5) * 2.0;

        /*  a is the control point vector, s is the time vector */
        raspberrypi_multicore_MPCtest_B.t7 = raspberrypi_multicore_factorial(3.0);
        raspberrypi_multicore_MPCtest_B.t8 =
          (raspberrypi_multicore_MPCtes_DW.obj_l.pL_LS[3 *
           raspberrypi_multicore_MPCtest_B.i_f2 + 2] + -0.196) / 2.0 +
          raspberrypi_multicore_MPCtes_DW.obj_l.StepH;
        raspberrypi_multicore_MPCtest_B.t9 = raspberrypi_multicore_factorial(2.0);
        raspberrypi_multicore_MPCtest_B.t6_e = raspberrypi_multicore_factorial
          (1.0);
        raspberrypi_multicore_MPCtest_B.t7_o = raspberrypi_multicore_factorial
          (0.0);
        raspberrypi_multicore_MPCtest_B.t6 =
          ((raspberrypi_multicore_MPCtest_B.t8 *
            (raspberrypi_multicore_MPCtest_B.t7 /
             raspberrypi_multicore_MPCtest_B.t7_o /
             raspberrypi_multicore_MPCtest_B.t7 * rt_powd_snf
             (raspberrypi_multicore_MPCtest_B.t6, 0.0) * rt_powd_snf(1.0 -
              raspberrypi_multicore_MPCtest_B.t6, 3.0)) +
            raspberrypi_multicore_MPCtest_B.t8 *
            (raspberrypi_multicore_MPCtest_B.t7 /
             raspberrypi_multicore_MPCtest_B.t6_e /
             raspberrypi_multicore_MPCtest_B.t9 * rt_powd_snf
             (raspberrypi_multicore_MPCtest_B.t6, 1.0) * rt_powd_snf(1.0 -
              raspberrypi_multicore_MPCtest_B.t6, 2.0))) +
           raspberrypi_multicore_MPCtest_B.t7 /
           raspberrypi_multicore_MPCtest_B.t9 /
           raspberrypi_multicore_MPCtest_B.t6_e * rt_powd_snf
           (raspberrypi_multicore_MPCtest_B.t6, 2.0) * rt_powd_snf(1.0 -
            raspberrypi_multicore_MPCtest_B.t6, 1.0) * -0.196) +
          raspberrypi_multicore_MPCtest_B.t7 /
          raspberrypi_multicore_MPCtest_B.t7 /
          raspberrypi_multicore_MPCtest_B.t7_o * rt_powd_snf
          (raspberrypi_multicore_MPCtest_B.t6, 3.0) * rt_powd_snf(1.0 -
          raspberrypi_multicore_MPCtest_B.t6, 0.0) * -0.196;
      }

      raspberrypi_multicore_MPCtest_B.t7 =
        raspberrypi_multicore_MPCtes_DW.obj_l.pL_LS[3 *
        raspberrypi_multicore_MPCtest_B.i_f2];
      raspberrypi_multicore_MPCtest_B.t8 =
        raspberrypi_multicore_MPCtest_B.desAllL[3 *
        raspberrypi_multicore_MPCtest_B.i_f2];
      raspberrypi_multicore_MPCtest_B.t9 = raspberrypi_multicore_factorial(3.0) /
        raspberrypi_multicore_factorial(0.0) / raspberrypi_multicore_factorial
        (3.0) * rt_powd_snf(raspberrypi_multicore_MPCtest_B.s_k, 0.0) *
        rt_powd_snf(1.0 - raspberrypi_multicore_MPCtest_B.s_k, 3.0);
      raspberrypi_multicore_MPCtest_B.t6_e = raspberrypi_multicore_factorial(3.0)
        / raspberrypi_multicore_factorial(1.0) / raspberrypi_multicore_factorial
        (2.0) * rt_powd_snf(raspberrypi_multicore_MPCtest_B.s_k, 1.0) *
        rt_powd_snf(1.0 - raspberrypi_multicore_MPCtest_B.s_k, 2.0);
      raspberrypi_multicore_MPCtest_B.t7_o = raspberrypi_multicore_factorial(3.0)
        / raspberrypi_multicore_factorial(2.0) / raspberrypi_multicore_factorial
        (1.0) * rt_powd_snf(raspberrypi_multicore_MPCtest_B.s_k, 2.0) *
        rt_powd_snf(1.0 - raspberrypi_multicore_MPCtest_B.s_k, 1.0);
      raspberrypi_multicore_MPCtest_B.t8_b = raspberrypi_multicore_factorial(3.0)
        / raspberrypi_multicore_factorial(3.0) / raspberrypi_multicore_factorial
        (0.0) * rt_powd_snf(raspberrypi_multicore_MPCtest_B.s_k, 3.0) *
        rt_powd_snf(1.0 - raspberrypi_multicore_MPCtest_B.s_k, 0.0);
      raspberrypi_multicore_MPCtest_B.pL_sw[3 *
        raspberrypi_multicore_MPCtest_B.i_f2] =
        ((raspberrypi_multicore_MPCtest_B.t9 *
          raspberrypi_multicore_MPCtest_B.t7 +
          raspberrypi_multicore_MPCtest_B.t6_e *
          raspberrypi_multicore_MPCtest_B.t7) +
         raspberrypi_multicore_MPCtest_B.t7_o *
         raspberrypi_multicore_MPCtest_B.t8) +
        raspberrypi_multicore_MPCtest_B.t8_b *
        raspberrypi_multicore_MPCtest_B.t8;
      raspberrypi_multicore_MPCtest_B.ar = 3 *
        raspberrypi_multicore_MPCtest_B.i_f2 + 1;
      raspberrypi_multicore_MPCtest_B.t7 =
        raspberrypi_multicore_MPCtes_DW.obj_l.pL_LS[raspberrypi_multicore_MPCtest_B.ar];
      raspberrypi_multicore_MPCtest_B.t8 =
        raspberrypi_multicore_MPCtest_B.desAllL[raspberrypi_multicore_MPCtest_B.ar];
      raspberrypi_multicore_MPCtest_B.pL_sw[raspberrypi_multicore_MPCtest_B.ar] =
        ((raspberrypi_multicore_MPCtest_B.t9 *
          raspberrypi_multicore_MPCtest_B.t7 +
          raspberrypi_multicore_MPCtest_B.t6_e *
          raspberrypi_multicore_MPCtest_B.t7) +
         raspberrypi_multicore_MPCtest_B.t7_o *
         raspberrypi_multicore_MPCtest_B.t8) +
        raspberrypi_multicore_MPCtest_B.t8_b *
        raspberrypi_multicore_MPCtest_B.t8;
      raspberrypi_multicore_MPCtest_B.pL_sw[3 *
        raspberrypi_multicore_MPCtest_B.i_f2 + 2] =
        raspberrypi_multicore_MPCtest_B.t6;

      /*                      if touchInd(i)>0.5 */
      /*                          PendAll(:,i)=obj.PendAllOld(:,i); */
      /*                          PendAll(3,i)=zSur(i); */
      /*                      end */
    }
  }

  /* %% generate the foot-end position cmd via interploating the MPC's predicted states */
  /*  As foot-end positions are planned as above, this part only */
  /*  interploate the pCoM to derive the foot-end positions in leg */
  /*  coordinates */
  if (fabs(raspberrypi_multicore_MPCtes_DW.obj_l.MPC_Count_Old -
           raspberrypi_multicore_MPCtest_B.Switch1) < 0.1) {
    if (raspberrypi_multicore_MPCtes_DW.obj_l.interpol_Count >= 3.0) {
      raspberrypi_multicore_MPCtes_DW.obj_l.interpol_Count = 3.0;
    }

    raspberrypi_multicore_MPCtest_B.s_k = (3.0 -
      raspberrypi_multicore_MPCtes_DW.obj_l.interpol_Count) / 3.0;
    raspberrypi_multicore_MPCtest_B.t6 =
      raspberrypi_multicore_MPCtes_DW.obj_l.interpol_Count / 3.0;
    for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
         raspberrypi_multicore_MPCtest_B.i_f2 < 12;
         raspberrypi_multicore_MPCtest_B.i_f2++) {
      raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[raspberrypi_multicore_MPCtest_B.i_f2]
        = raspberrypi_multicore_MPCtest_B.s_k *
        raspberrypi_multicore_MPCtes_DW.obj_l.pArray_L_Adm_Old[raspberrypi_multicore_MPCtest_B.i_f2]
        + raspberrypi_multicore_MPCtest_B.t6 *
        raspberrypi_multicore_MPCtes_DW.obj_l.pArray_L_Adm_Now[raspberrypi_multicore_MPCtest_B.i_f2];
    }

    raspberrypi_multicore_MPCtes_DW.obj_l.interpol_Count++;
  } else {
    memcpy(&raspberrypi_multicore_MPCtes_DW.obj_l.pArray_L_Adm_Old[0],
           &raspberrypi_multicore_MPCtes_DW.obj_l.pArray_L_Adm_Now[0], 12U *
           sizeof(real_T));
    for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
         raspberrypi_multicore_MPCtest_B.i_f2 < 4;
         raspberrypi_multicore_MPCtest_B.i_f2++) {
      if ((raspberrypi_multicore_MPCtes_DW.obj_l.MPC_legStateOld[raspberrypi_multicore_MPCtest_B.i_f2]
           < 0.5) &&
          (raspberrypi_multicore_MPCtest_B.pyNew[raspberrypi_multicore_MPCtest_B.i_f2]
           > 0.5)) {
        raspberrypi_multicore_MPCtest_B.ar =
          (raspberrypi_multicore_MPCtest_B.i_f2 + 1) * 3;
        raspberrypi_multicore_MPCtes_DW.obj_l.pArray_L_Adm_Old[raspberrypi_multicore_MPCtest_B.ar
          - 3] = raspberrypi_multicore_MPCtes_DW.obj_l.PendAllLocalOld[3 *
          raspberrypi_multicore_MPCtest_B.i_f2];
        raspberrypi_multicore_MPCtes_DW.obj_l.pArray_L_Adm_Old[raspberrypi_multicore_MPCtest_B.ar
          - 2] = raspberrypi_multicore_MPCtes_DW.obj_l.PendAllLocalOld[3 *
          raspberrypi_multicore_MPCtest_B.i_f2 + 1];
        raspberrypi_multicore_MPCtes_DW.obj_l.pArray_L_Adm_Old[raspberrypi_multicore_MPCtest_B.ar
          - 1] = raspberrypi_multicore_MPCtes_DW.obj_l.PendAllLocalOld[3 *
          raspberrypi_multicore_MPCtest_B.i_f2 + 2];
      }
    }

    for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
         raspberrypi_multicore_MPCtest_B.i_f2 < 12;
         raspberrypi_multicore_MPCtest_B.i_f2++) {
      raspberrypi_multicore_MPCtes_DW.obj_l.pArray_L_Adm_Now[raspberrypi_multicore_MPCtest_B.i_f2]
        =
        raspberrypi_multicore_MPCtest_B.pArray_L_Adm[raspberrypi_multicore_MPCtest_B.i_f2]
        / 1000.0;
      raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[raspberrypi_multicore_MPCtest_B.i_f2]
        =
        raspberrypi_multicore_MPCtes_DW.obj_l.pArray_L_Adm_Old[raspberrypi_multicore_MPCtest_B.i_f2];
    }

    raspberrypi_multicore_MPCtes_DW.obj_l.interpol_Count = 1.0;
  }

  /*  update of pL_LS */
  /* %% data update */
  for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
       raspberrypi_multicore_MPCtest_B.i_f2 < 4;
       raspberrypi_multicore_MPCtest_B.i_f2++) {
    raspberrypi_multicore_MPCtest_B.s_k =
      raspberrypi_multicore_MPCtest_B.pyNew[raspberrypi_multicore_MPCtest_B.i_f2];
    raspberrypi_multicore_MPCtest_B.ar = (int32_T)
      raspberrypi_multicore_MPCtest_B.pxNew[raspberrypi_multicore_MPCtest_B.i_f2];
    if (raspberrypi_multicore_MPCtest_B.ar < 0.5) {
      raspberrypi_multicore_MPCtest_B.desAllL[3 *
        raspberrypi_multicore_MPCtest_B.i_f2] =
        raspberrypi_multicore_MPCtest_B.pL_sw[3 *
        raspberrypi_multicore_MPCtest_B.i_f2];
      raspberrypi_multicore_MPCtest_B.vcol = 3 *
        raspberrypi_multicore_MPCtest_B.i_f2 + 1;
      raspberrypi_multicore_MPCtest_B.desAllL[raspberrypi_multicore_MPCtest_B.vcol]
        =
        raspberrypi_multicore_MPCtest_B.pL_sw[raspberrypi_multicore_MPCtest_B.vcol];
      raspberrypi_multicore_MPCtest_B.vcol = 3 *
        raspberrypi_multicore_MPCtest_B.i_f2 + 2;
      raspberrypi_multicore_MPCtest_B.desAllL[raspberrypi_multicore_MPCtest_B.vcol]
        =
        raspberrypi_multicore_MPCtest_B.pL_sw[raspberrypi_multicore_MPCtest_B.vcol];
    } else if (raspberrypi_multicore_MPCtest_B.s_k > 0.5) {
      raspberrypi_multicore_MPCtest_B.desAllL[3 *
        raspberrypi_multicore_MPCtest_B.i_f2] =
        raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[3 *
        raspberrypi_multicore_MPCtest_B.i_f2];
      raspberrypi_multicore_MPCtest_B.vcol = 3 *
        raspberrypi_multicore_MPCtest_B.i_f2 + 1;
      raspberrypi_multicore_MPCtest_B.desAllL[raspberrypi_multicore_MPCtest_B.vcol]
        =
        raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[raspberrypi_multicore_MPCtest_B.vcol];
      raspberrypi_multicore_MPCtest_B.vcol = 3 *
        raspberrypi_multicore_MPCtest_B.i_f2 + 2;
      raspberrypi_multicore_MPCtest_B.desAllL[raspberrypi_multicore_MPCtest_B.vcol]
        =
        raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[raspberrypi_multicore_MPCtest_B.vcol];
    } else {
      raspberrypi_multicore_MPCtest_B.desAllL[3 *
        raspberrypi_multicore_MPCtest_B.i_f2] =
        raspberrypi_multicore_MPCtes_DW.obj_l.PendAllLocalOld[3 *
        raspberrypi_multicore_MPCtest_B.i_f2];
      raspberrypi_multicore_MPCtest_B.vcol = 3 *
        raspberrypi_multicore_MPCtest_B.i_f2 + 1;
      raspberrypi_multicore_MPCtest_B.desAllL[raspberrypi_multicore_MPCtest_B.vcol]
        =
        raspberrypi_multicore_MPCtes_DW.obj_l.PendAllLocalOld[raspberrypi_multicore_MPCtest_B.vcol];
      raspberrypi_multicore_MPCtest_B.vcol = 3 *
        raspberrypi_multicore_MPCtest_B.i_f2 + 2;
      raspberrypi_multicore_MPCtest_B.desAllL[raspberrypi_multicore_MPCtest_B.vcol]
        =
        raspberrypi_multicore_MPCtes_DW.obj_l.PendAllLocalOld[raspberrypi_multicore_MPCtest_B.vcol];
    }

    if (raspberrypi_multicore_MPCtest_B.ar > 0.5) {
      raspberrypi_multicore_MPCtes_DW.obj_l.pL_LS[3 *
        raspberrypi_multicore_MPCtest_B.i_f2] =
        raspberrypi_multicore_MPCtest_B.desAllL[3 *
        raspberrypi_multicore_MPCtest_B.i_f2];
      raspberrypi_multicore_MPCtest_B.ar = 3 *
        raspberrypi_multicore_MPCtest_B.i_f2 + 1;
      raspberrypi_multicore_MPCtes_DW.obj_l.pL_LS[raspberrypi_multicore_MPCtest_B.ar]
        =
        raspberrypi_multicore_MPCtest_B.desAllL[raspberrypi_multicore_MPCtest_B.ar];
      raspberrypi_multicore_MPCtest_B.ar = 3 *
        raspberrypi_multicore_MPCtest_B.i_f2 + 2;
      raspberrypi_multicore_MPCtes_DW.obj_l.pL_LS[raspberrypi_multicore_MPCtest_B.ar]
        =
        raspberrypi_multicore_MPCtest_B.desAllL[raspberrypi_multicore_MPCtest_B.ar];
    }

    raspberrypi_multicore_MPCtes_DW.obj_l.MPC_legStateOld[raspberrypi_multicore_MPCtest_B.i_f2]
      = raspberrypi_multicore_MPCtest_B.s_k;
  }

  raspberrypi_multicore_MPCtes_DW.obj_l.MPC_Count_Old =
    raspberrypi_multicore_MPCtest_B.Switch1;
  for (raspberrypi_multicore_MPCtest_B.ar = 0;
       raspberrypi_multicore_MPCtest_B.ar < 12;
       raspberrypi_multicore_MPCtest_B.ar++) {
    raspberrypi_multicore_MPCtest_B.Switch1 =
      raspberrypi_multicore_MPCtest_B.desAllL[raspberrypi_multicore_MPCtest_B.ar];
    raspberrypi_multicore_MPCtes_DW.obj_l.PendAllLocalOld[raspberrypi_multicore_MPCtest_B.ar]
      = raspberrypi_multicore_MPCtest_B.Switch1;

    /* MATLABSystem: '<Root>/MATLAB System12' */
    raspberrypi_multicore_MPCtest_B.pL_sw[raspberrypi_multicore_MPCtest_B.ar] =
      raspberrypi_multicore_MPCtest_B.Switch1 * 1000.0;
  }

  /* MATLABSystem: '<Root>/MATLAB System12' */
  /* +[0;37;0;0;-37;0;0;37;0;0;-37;0]; */
  if (raspberrypi_multicore_MPCtest_B.MATLABSystem8 > 0.5) {
    memcpy(&raspberrypi_multicore_MPCtest_B.pL_sw[0],
           &raspberrypi_multicore_MPCtes_DW.obj_b0.pArray_L_Old[0], 12U * sizeof
           (real_T));
  }

  memcpy(&raspberrypi_multicore_MPCtes_DW.obj_b0.pArray_L_Old[0],
         &raspberrypi_multicore_MPCtest_B.pL_sw[0], 12U * sizeof(real_T));

  /* MATLABSystem: '<Root>/MATLAB System14' incorporates:
   *  MATLAB Function: '<Root>/Norminal_to_ServoAngle'
   *  MATLABSystem: '<Root>/MATLAB System12'
   */
  rEQ0 = false;
  p = true;
  raspberrypi_multicore_MPCtest_B.ar = 0;
  exitg1 = false;
  while ((!exitg1) && (raspberrypi_multicore_MPCtest_B.ar < 3)) {
    if (!(raspberrypi_multicore_MPCtes_DW.obj_k.LF_Off[raspberrypi_multicore_MPCtest_B.ar]
          ==
          raspberrypi_multicore_MPCtest_P.MATLABSystem14_LF_Off[raspberrypi_multicore_MPCtest_B.ar]))
    {
      p = false;
      exitg1 = true;
    } else {
      raspberrypi_multicore_MPCtest_B.ar++;
    }
  }

  if (p) {
    rEQ0 = true;
  }

  if (!rEQ0) {
    raspberrypi_multicore_MPCtes_DW.obj_k.LF_Off[0] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem14_LF_Off[0];
    raspberrypi_multicore_MPCtes_DW.obj_k.LF_Off[1] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem14_LF_Off[1];
    raspberrypi_multicore_MPCtes_DW.obj_k.LF_Off[2] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem14_LF_Off[2];
  }

  rEQ0 = false;
  p = true;
  raspberrypi_multicore_MPCtest_B.ar = 0;
  exitg1 = false;
  while ((!exitg1) && (raspberrypi_multicore_MPCtest_B.ar < 3)) {
    if (!(raspberrypi_multicore_MPCtes_DW.obj_k.RF_Off[raspberrypi_multicore_MPCtest_B.ar]
          ==
          raspberrypi_multicore_MPCtest_P.MATLABSystem14_RF_Off[raspberrypi_multicore_MPCtest_B.ar]))
    {
      p = false;
      exitg1 = true;
    } else {
      raspberrypi_multicore_MPCtest_B.ar++;
    }
  }

  if (p) {
    rEQ0 = true;
  }

  if (!rEQ0) {
    raspberrypi_multicore_MPCtes_DW.obj_k.RF_Off[0] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem14_RF_Off[0];
    raspberrypi_multicore_MPCtes_DW.obj_k.RF_Off[1] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem14_RF_Off[1];
    raspberrypi_multicore_MPCtes_DW.obj_k.RF_Off[2] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem14_RF_Off[2];
  }

  rEQ0 = false;
  p = true;
  raspberrypi_multicore_MPCtest_B.ar = 0;
  exitg1 = false;
  while ((!exitg1) && (raspberrypi_multicore_MPCtest_B.ar < 3)) {
    if (!(raspberrypi_multicore_MPCtes_DW.obj_k.LH_Off[raspberrypi_multicore_MPCtest_B.ar]
          ==
          raspberrypi_multicore_MPCtest_P.MATLABSystem14_LH_Off[raspberrypi_multicore_MPCtest_B.ar]))
    {
      p = false;
      exitg1 = true;
    } else {
      raspberrypi_multicore_MPCtest_B.ar++;
    }
  }

  if (p) {
    rEQ0 = true;
  }

  if (!rEQ0) {
    raspberrypi_multicore_MPCtes_DW.obj_k.LH_Off[0] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem14_LH_Off[0];
    raspberrypi_multicore_MPCtes_DW.obj_k.LH_Off[1] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem14_LH_Off[1];
    raspberrypi_multicore_MPCtes_DW.obj_k.LH_Off[2] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem14_LH_Off[2];
  }

  rEQ0 = false;
  p = true;
  raspberrypi_multicore_MPCtest_B.ar = 0;
  exitg1 = false;
  while ((!exitg1) && (raspberrypi_multicore_MPCtest_B.ar < 3)) {
    if (!(raspberrypi_multicore_MPCtes_DW.obj_k.RH_Off[raspberrypi_multicore_MPCtest_B.ar]
          ==
          raspberrypi_multicore_MPCtest_P.MATLABSystem14_RH_Off[raspberrypi_multicore_MPCtest_B.ar]))
    {
      p = false;
      exitg1 = true;
    } else {
      raspberrypi_multicore_MPCtest_B.ar++;
    }
  }

  if (p) {
    rEQ0 = true;
  }

  if (!rEQ0) {
    raspberrypi_multicore_MPCtes_DW.obj_k.RH_Off[0] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem14_RH_Off[0];
    raspberrypi_multicore_MPCtes_DW.obj_k.RH_Off[1] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem14_RH_Off[1];
    raspberrypi_multicore_MPCtes_DW.obj_k.RH_Off[2] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem14_RH_Off[2];
  }

  /* function flag = supportsMultipleInstanceImpl(obj) */
  /*  Support System object in Simulink For Each subsystem */
  /*  Do not enable For Each support if your System object allocates exclusive resources that may */
  /*  conflict with other System objects, such as allocating file handles, memory by address, or hardware resources. */
  /*     flag = true; */
  /* end */
  /*  Implement algorithm. */
  /*  PArray:=[Px_i,Py_i,Pz_i], 12*1, foot-end position in leg coordinate */
  /*  AngleArray:=[Mi1,Mi2,Mi3] */
  raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[0] =
    raspberrypi_multicore_MPCtes_DW.obj_k.LF_Off[0];
  raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[3] =
    raspberrypi_multicore_MPCtes_DW.obj_k.RF_Off[0];
  raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[6] =
    raspberrypi_multicore_MPCtes_DW.obj_k.LH_Off[0];
  raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[9] =
    raspberrypi_multicore_MPCtes_DW.obj_k.RH_Off[0];
  raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[1] =
    raspberrypi_multicore_MPCtes_DW.obj_k.LF_Off[1];
  raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[4] =
    raspberrypi_multicore_MPCtes_DW.obj_k.RF_Off[1];
  raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[7] =
    raspberrypi_multicore_MPCtes_DW.obj_k.LH_Off[1];
  raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[10] =
    raspberrypi_multicore_MPCtes_DW.obj_k.RH_Off[1];
  raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[2] =
    raspberrypi_multicore_MPCtes_DW.obj_k.LF_Off[2];
  raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[5] =
    raspberrypi_multicore_MPCtes_DW.obj_k.RF_Off[2];
  raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[8] =
    raspberrypi_multicore_MPCtes_DW.obj_k.LH_Off[2];
  raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[11] =
    raspberrypi_multicore_MPCtes_DW.obj_k.RH_Off[2];
  for (raspberrypi_multicore_MPCtest_B.ar = 0;
       raspberrypi_multicore_MPCtest_B.ar < 12;
       raspberrypi_multicore_MPCtest_B.ar++) {
    raspberrypi_multicore_MPCtest_B.Switch1 =
      raspberrypi_multicore_MPCtest_B.pL_sw[raspberrypi_multicore_MPCtest_B.ar]
      + raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[raspberrypi_multicore_MPCtest_B.ar];
    raspberrypi_multicore_MPCtest_B.x[raspberrypi_multicore_MPCtest_B.ar] =
      rtIsNaN(raspberrypi_multicore_MPCtest_B.Switch1);
    raspberrypi_multicore_MPCtest_B.pL_sw[raspberrypi_multicore_MPCtest_B.ar] =
      raspberrypi_multicore_MPCtest_B.Switch1;
  }

  raspberrypi_multicore_MPCtest_B.i_f2 = 0;
  raspberrypi_multicore_MPCtest_B.ar = 1;
  exitg1 = false;
  while ((!exitg1) && (raspberrypi_multicore_MPCtest_B.ar - 1 < 12)) {
    if (raspberrypi_multicore_MPCtest_B.x[raspberrypi_multicore_MPCtest_B.ar - 1])
    {
      raspberrypi_multicore_MPCtest_B.i_f2++;
      raspberrypi_multicore_MPCtest_B.ii_data[raspberrypi_multicore_MPCtest_B.i_f2
        - 1] = raspberrypi_multicore_MPCtest_B.ar;
      if (raspberrypi_multicore_MPCtest_B.i_f2 >= 12) {
        exitg1 = true;
      } else {
        raspberrypi_multicore_MPCtest_B.ar++;
      }
    } else {
      raspberrypi_multicore_MPCtest_B.ar++;
    }
  }

  if (1 > raspberrypi_multicore_MPCtest_B.i_f2) {
    raspberrypi_multicore_MPCtest_B.i_f2 = 0;
  }

  if (0 <= raspberrypi_multicore_MPCtest_B.i_f2 - 1) {
    memcpy(&raspberrypi_multicore_MPCtest_B.b_index_data[0],
           &raspberrypi_multicore_MPCtest_B.ii_data[0],
           raspberrypi_multicore_MPCtest_B.i_f2 * sizeof(int32_T));
  }

  if (raspberrypi_multicore_MPCtest_B.i_f2 != 0) {
    for (raspberrypi_multicore_MPCtest_B.ar = 0;
         raspberrypi_multicore_MPCtest_B.ar <
         raspberrypi_multicore_MPCtest_B.i_f2;
         raspberrypi_multicore_MPCtest_B.ar++) {
      raspberrypi_multicore_MPCtest_B.vcol =
        raspberrypi_multicore_MPCtest_B.b_index_data[raspberrypi_multicore_MPCtest_B.ar];
      raspberrypi_multicore_MPCtest_B.pL_sw[raspberrypi_multicore_MPCtest_B.vcol
        - 1] =
        raspberrypi_multicore_MPCtes_DW.obj_k.LastInput[raspberrypi_multicore_MPCtest_B.vcol
        - 1];
    }
  }

  /*  pArray must be row vector */
  for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
       raspberrypi_multicore_MPCtest_B.i_f2 < 4;
       raspberrypi_multicore_MPCtest_B.i_f2++) {
    if ((raspberrypi_multicore_MPCtest_B.i_f2 + 1 == 2) ||
        (raspberrypi_multicore_MPCtest_B.i_f2 + 1 == 4)) {
      raspberrypi_multicore_MPCtest_B.pL_sw_c[1] =
        -raspberrypi_multicore_MPCtes_DW.obj_k.PyLim[0];
      raspberrypi_multicore_MPCtest_B.pL_sw_c[2] =
        -raspberrypi_multicore_MPCtes_DW.obj_k.PyLim[1];
    } else {
      raspberrypi_multicore_MPCtest_B.pL_sw_c[1] =
        raspberrypi_multicore_MPCtes_DW.obj_k.PyLim[0];
      raspberrypi_multicore_MPCtest_B.pL_sw_c[2] =
        raspberrypi_multicore_MPCtes_DW.obj_k.PyLim[1];
    }

    raspberrypi_multicore_MPCtest_B.v1[0] =
      raspberrypi_multicore_MPCtest_B.pL_sw[3 *
      raspberrypi_multicore_MPCtest_B.i_f2];
    raspberrypi_multicore_MPCtest_B.v1[1] =
      raspberrypi_multicore_MPCtes_DW.obj_k.PxLim[0];
    raspberrypi_multicore_MPCtest_B.v1[2] =
      raspberrypi_multicore_MPCtes_DW.obj_k.PxLim[1];
    raspberrypi_multicore_MPCtest_B.ar = 3 *
      raspberrypi_multicore_MPCtest_B.i_f2 + 1;
    raspberrypi_multicore_MPCtest_B.pL_sw_c[0] =
      raspberrypi_multicore_MPCtest_B.pL_sw[raspberrypi_multicore_MPCtest_B.ar];
    raspberrypi_multicore_MPCtest_B.vcol = 3 *
      raspberrypi_multicore_MPCtest_B.i_f2 + 2;
    raspberrypi_multicore_MPCtest_B.deltaP2[0] =
      raspberrypi_multicore_MPCtest_B.pL_sw[raspberrypi_multicore_MPCtest_B.vcol];
    raspberrypi_multicore_MPCtest_B.deltaP2[1] =
      raspberrypi_multicore_MPCtes_DW.obj_k.PzLim[0];
    raspberrypi_multicore_MPCtest_B.deltaP2[2] =
      raspberrypi_multicore_MPCtes_DW.obj_k.PzLim[1];
    raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[3 *
      raspberrypi_multicore_MPCtest_B.i_f2] = raspberrypi_multicore_MP_median
      (raspberrypi_multicore_MPCtest_B.v1);
    raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[raspberrypi_multicore_MPCtest_B.ar]
      = raspberrypi_multicore_MP_median(raspberrypi_multicore_MPCtest_B.pL_sw_c);
    raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[raspberrypi_multicore_MPCtest_B.vcol]
      = raspberrypi_multicore_MP_median(raspberrypi_multicore_MPCtest_B.deltaP2);
  }

  raspberrypi_multicore_IK_IK_one(&raspberrypi_multicore_MPCtes_DW.obj_k,
    &raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[0], 1.0,
    raspberrypi_multicore_MPCtest_B.v1, &raspberrypi_multicore_MPCtest_B.s_k);
  raspberrypi_multicore_IK_IK_one(&raspberrypi_multicore_MPCtes_DW.obj_k,
    &raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[3], 2.0,
    raspberrypi_multicore_MPCtest_B.deltaP2,
    &raspberrypi_multicore_MPCtest_B.s_k);
  raspberrypi_multicore_IK_IK_one(&raspberrypi_multicore_MPCtes_DW.obj_k,
    &raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[6], 3.0,
    raspberrypi_multicore_MPCtest_B.deltaP3,
    &raspberrypi_multicore_MPCtest_B.s_k);
  raspberrypi_multicore_IK_IK_one(&raspberrypi_multicore_MPCtes_DW.obj_k,
    &raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[9], 4.0,
    raspberrypi_multicore_MPCtest_B.a, &raspberrypi_multicore_MPCtest_B.s_k);
  raspberrypi_multicore_MPCtest_B.pL_sw[0] = raspberrypi_multicore_MPCtest_B.v1
    [0] + raspberrypi_multicore_MPCtes_DW.obj_k.AngleOff[0];
  raspberrypi_multicore_MPCtest_B.pL_sw[3] =
    raspberrypi_multicore_MPCtest_B.deltaP2[0] +
    raspberrypi_multicore_MPCtes_DW.obj_k.AngleOff[3];
  raspberrypi_multicore_MPCtest_B.pL_sw[6] =
    raspberrypi_multicore_MPCtest_B.deltaP3[0] +
    raspberrypi_multicore_MPCtes_DW.obj_k.AngleOff[6];
  raspberrypi_multicore_MPCtest_B.pL_sw[9] = raspberrypi_multicore_MPCtest_B.a[0]
    + raspberrypi_multicore_MPCtes_DW.obj_k.AngleOff[9];
  raspberrypi_multicore_MPCtest_B.pL_sw[1] = raspberrypi_multicore_MPCtest_B.v1
    [1] + raspberrypi_multicore_MPCtes_DW.obj_k.AngleOff[1];
  raspberrypi_multicore_MPCtest_B.pL_sw[4] =
    raspberrypi_multicore_MPCtest_B.deltaP2[1] +
    raspberrypi_multicore_MPCtes_DW.obj_k.AngleOff[4];
  raspberrypi_multicore_MPCtest_B.pL_sw[7] =
    raspberrypi_multicore_MPCtest_B.deltaP3[1] +
    raspberrypi_multicore_MPCtes_DW.obj_k.AngleOff[7];
  raspberrypi_multicore_MPCtest_B.pL_sw[10] = raspberrypi_multicore_MPCtest_B.a
    [1] + raspberrypi_multicore_MPCtes_DW.obj_k.AngleOff[10];
  raspberrypi_multicore_MPCtest_B.pL_sw[2] = raspberrypi_multicore_MPCtest_B.v1
    [2] + raspberrypi_multicore_MPCtes_DW.obj_k.AngleOff[2];
  raspberrypi_multicore_MPCtest_B.pL_sw[5] =
    raspberrypi_multicore_MPCtest_B.deltaP2[2] +
    raspberrypi_multicore_MPCtes_DW.obj_k.AngleOff[5];
  raspberrypi_multicore_MPCtest_B.pL_sw[8] =
    raspberrypi_multicore_MPCtest_B.deltaP3[2] +
    raspberrypi_multicore_MPCtes_DW.obj_k.AngleOff[8];
  raspberrypi_multicore_MPCtest_B.pL_sw[11] = raspberrypi_multicore_MPCtest_B.a
    [2] + raspberrypi_multicore_MPCtes_DW.obj_k.AngleOff[11];
  for (raspberrypi_multicore_MPCtest_B.ar = 0;
       raspberrypi_multicore_MPCtest_B.ar < 12;
       raspberrypi_multicore_MPCtest_B.ar++) {
    raspberrypi_multicore_MPCtes_DW.obj_k.LastInput[raspberrypi_multicore_MPCtest_B.ar]
      =
      raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[raspberrypi_multicore_MPCtest_B.ar];
    raspberrypi_multicore_MPCtest_B.pL_sw[raspberrypi_multicore_MPCtest_B.ar] =
      raspberrypi_multicore_MPCtest_B.pL_sw[raspberrypi_multicore_MPCtest_B.ar] /
      3.1415926535897931 * 180.0;
  }

  /* End of MATLABSystem: '<Root>/MATLAB System14' */

  /* MATLAB Function: '<Root>/Norminal_to_ServoAngle' */
  raspberrypi_multicore_MPCtest_B.pL_sw[8] =
    -raspberrypi_multicore_MPCtest_B.pL_sw[8];
  raspberrypi_multicore_MPCtest_B.pL_sw[11] =
    -raspberrypi_multicore_MPCtest_B.pL_sw[11];
  for (raspberrypi_multicore_MPCtest_B.ar = 0;
       raspberrypi_multicore_MPCtest_B.ar < 12;
       raspberrypi_multicore_MPCtest_B.ar++) {
    raspberrypi_multicore_MPCtest_B.pL_sw[raspberrypi_multicore_MPCtest_B.ar] =
      -raspberrypi_multicore_MPCtest_B.pL_sw[raspberrypi_multicore_MPCtest_B.ar]
      + angleOff[raspberrypi_multicore_MPCtest_B.ar];
  }

  /* MATLABSystem: '<Root>/MATLAB System16' incorporates:
   *  UnitDelay: '<Root>/Unit Delay'
   */
  if (raspberrypi_multicore_MPCtes_DW.obj_p.cons_Vel !=
      raspberrypi_multicore_MPCtest_P.MATLABSystem16_cons_Vel) {
    raspberrypi_multicore_MPCtes_DW.obj_p.cons_Vel =
      raspberrypi_multicore_MPCtest_P.MATLABSystem16_cons_Vel;
  }

  if (raspberrypi_multicore_MPCtes_DW.obj_p.dt !=
      raspberrypi_multicore_MPCtest_P.Ts_DynSim) {
    raspberrypi_multicore_MPCtes_DW.obj_p.dt =
      raspberrypi_multicore_MPCtest_P.Ts_DynSim;
  }

  /*          %% Define output properties */
  raspberrypi_multicore_MPCtest_B.vcol = 1;
  if (raspberrypi_multicore_MPCtes_DW.obj_p.Ini_count <= 3.0) {
    memcpy(&raspberrypi_multicore_MPCtes_DW.obj_p.PosDesOld[0],
           &raspberrypi_multicore_MPCtes_DW.UnitDelay_DSTATE[0], 12U * sizeof
           (real_T));
    raspberrypi_multicore_MPCtes_DW.obj_p.Ini_count++;
    raspberrypi_multicore_MPCtest_B.vcol = 0;
  }

  for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
       raspberrypi_multicore_MPCtest_B.i_f2 < 12;
       raspberrypi_multicore_MPCtest_B.i_f2++) {
    raspberrypi_multicore_MPCtest_B.t9 =
      raspberrypi_multicore_MPCtest_B.pL_sw[raspberrypi_multicore_MPCtest_B.i_f2]
      - raspberrypi_multicore_MPCtes_DW.obj_p.PosDesOld[raspberrypi_multicore_MPCtest_B.i_f2];
    if (fabs(raspberrypi_multicore_MPCtest_B.t9 /
             raspberrypi_multicore_MPCtes_DW.obj_p.dt) >= fabs
        (raspberrypi_multicore_MPCtes_DW.obj_p.cons_Vel)) {
      if (raspberrypi_multicore_MPCtest_B.t9 < 0.0) {
        raspberrypi_multicore_MPCtest_B.t9 = -1.0;
      } else if (raspberrypi_multicore_MPCtest_B.t9 > 0.0) {
        raspberrypi_multicore_MPCtest_B.t9 = 1.0;
      } else if (raspberrypi_multicore_MPCtest_B.t9 == 0.0) {
        raspberrypi_multicore_MPCtest_B.t9 = 0.0;
      } else {
        raspberrypi_multicore_MPCtest_B.t9 = (rtNaN);
      }

      raspberrypi_multicore_MPCtest_B.t9 = raspberrypi_multicore_MPCtest_B.t9 *
        raspberrypi_multicore_MPCtes_DW.obj_p.cons_Vel *
        raspberrypi_multicore_MPCtes_DW.obj_p.dt;
    }

    raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[raspberrypi_multicore_MPCtest_B.i_f2]
      = raspberrypi_multicore_MPCtest_B.t9;
  }

  /* Switch: '<Root>/Switch' incorporates:
   *  Constant: '<Root>/Constant6'
   *  Constant: '<Root>/Constant7'
   *  DigitalClock: '<Root>/Digital Clock'
   */
  if (raspberrypi_multicore_MPCtes_M->Timing.taskTime0 >
      raspberrypi_multicore_MPCtest_P.Switch_Threshold) {
    raspberrypi_multicore_MPCtest_B.Angle3new_c =
      raspberrypi_multicore_MPCtest_P.Constant6_Value;
  } else {
    raspberrypi_multicore_MPCtest_B.Angle3new_c =
      raspberrypi_multicore_MPCtest_P.Constant7_Value;
  }

  /* End of Switch: '<Root>/Switch' */

  /* MATLABSystem: '<Root>/MATLAB System16' */
  if (raspberrypi_multicore_MPCtest_B.Angle3new_c < 0.5) {
    for (raspberrypi_multicore_MPCtest_B.ar = 0;
         raspberrypi_multicore_MPCtest_B.ar < 12;
         raspberrypi_multicore_MPCtest_B.ar++) {
      raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[raspberrypi_multicore_MPCtest_B.ar]
        =
        raspberrypi_multicore_MPCtest_B.pL_sw[raspberrypi_multicore_MPCtest_B.ar]
        - raspberrypi_multicore_MPCtes_DW.obj_p.PosDesOld[raspberrypi_multicore_MPCtest_B.ar];
    }
  }

  for (raspberrypi_multicore_MPCtest_B.ar = 0;
       raspberrypi_multicore_MPCtest_B.ar < 12;
       raspberrypi_multicore_MPCtest_B.ar++) {
    raspberrypi_multicore_MPCtes_DW.obj_p.PosDesOld[raspberrypi_multicore_MPCtest_B.ar]
      +=
      raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[raspberrypi_multicore_MPCtest_B.ar];
  }

  /* Switch: '<S14>/Switch1' incorporates:
   *  Constant: '<S14>/Constant12'
   *  MATLABSystem: '<Root>/MATLAB System16'
   *  Memory: '<S14>/Memory1'
   */
  if (raspberrypi_multicore_MPCtes_DW.Memory1_PreviousInput >
      raspberrypi_multicore_MPCtest_P.Switch1_Threshold) {
    raspberrypi_multicore_MPCtest_B.Switch1 =
      raspberrypi_multicore_MPCtest_P.Constant12_Value;
  } else {
    raspberrypi_multicore_MPCtest_B.Switch1 =
      raspberrypi_multicore_MPCtest_B.vcol;
  }

  /* End of Switch: '<S14>/Switch1' */

  /* SignalConversion generated from: '<S48>/ SFunction ' incorporates:
   *  Constant: '<S14>/Constant'
   *  Constant: '<S14>/Constant13'
   *  Constant: '<S14>/Constant14'
   *  Constant: '<S14>/Constant15'
   *  MATLAB Function: '<S14>/MATLAB Function'
   */
  raspberrypi_multicore_MPCtest_B.pL_sw[0] =
    raspberrypi_multicore_MPCtest_P.Constant_Value[0];
  raspberrypi_multicore_MPCtest_B.pL_sw[3] =
    raspberrypi_multicore_MPCtest_P.Constant13_Value[0];
  raspberrypi_multicore_MPCtest_B.pL_sw[6] =
    raspberrypi_multicore_MPCtest_P.Constant14_Value[0];
  raspberrypi_multicore_MPCtest_B.pL_sw[9] =
    raspberrypi_multicore_MPCtest_P.Constant15_Value[0];
  raspberrypi_multicore_MPCtest_B.pL_sw[1] =
    raspberrypi_multicore_MPCtest_P.Constant_Value[1];
  raspberrypi_multicore_MPCtest_B.pL_sw[4] =
    raspberrypi_multicore_MPCtest_P.Constant13_Value[1];
  raspberrypi_multicore_MPCtest_B.pL_sw[7] =
    raspberrypi_multicore_MPCtest_P.Constant14_Value[1];
  raspberrypi_multicore_MPCtest_B.pL_sw[10] =
    raspberrypi_multicore_MPCtest_P.Constant15_Value[1];
  raspberrypi_multicore_MPCtest_B.pL_sw[2] =
    raspberrypi_multicore_MPCtest_P.Constant_Value[2];
  raspberrypi_multicore_MPCtest_B.pL_sw[5] =
    raspberrypi_multicore_MPCtest_P.Constant13_Value[2];
  raspberrypi_multicore_MPCtest_B.pL_sw[8] =
    raspberrypi_multicore_MPCtest_P.Constant14_Value[2];
  raspberrypi_multicore_MPCtest_B.pL_sw[11] =
    raspberrypi_multicore_MPCtest_P.Constant15_Value[2];

  /* MATLAB Function: '<S14>/MATLAB Function' incorporates:
   *  Constant: '<Root>/Constant4'
   *  Constant: '<S14>/Constant10'
   *  Constant: '<S14>/Constant11'
   *  MATLABSystem: '<Root>/MATLAB System16'
   */
  for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
       raspberrypi_multicore_MPCtest_B.i_f2 < 60;
       raspberrypi_multicore_MPCtest_B.i_f2++) {
    raspberrypi_multicore_MPCtest_B.cmd[raspberrypi_multicore_MPCtest_B.i_f2] =
      0U;
  }

  raspberrypi_multicore_MPCtest_B.cmd[0] = 102U;
  raspberrypi_multicore_MPCtest_B.Angle3new_c = rt_roundd_snf
    (raspberrypi_multicore_MPCtest_P.Constant10_Value);
  if (raspberrypi_multicore_MPCtest_B.Angle3new_c < 256.0) {
    if (raspberrypi_multicore_MPCtest_B.Angle3new_c >= 0.0) {
      raspberrypi_multicore_MPCtest_B.cmd[1] = (uint8_T)
        raspberrypi_multicore_MPCtest_B.Angle3new_c;
    } else {
      raspberrypi_multicore_MPCtest_B.cmd[1] = 0U;
    }
  } else {
    raspberrypi_multicore_MPCtest_B.cmd[1] = MAX_uint8_T;
  }

  if ((raspberrypi_multicore_MPCtest_P.Constant4_Value > 0.5) &&
      (raspberrypi_multicore_MPCtest_B.Switch1 > 0.5)) {
    raspberrypi_multicore_MPCtest_B.cmd[2] = 2U;
  } else if ((raspberrypi_multicore_MPCtest_P.Constant4_Value > 0.5) &&
             (raspberrypi_multicore_MPCtest_B.Switch1 < 0.5)) {
    raspberrypi_multicore_MPCtest_B.cmd[2] = 0U;
  } else if ((raspberrypi_multicore_MPCtest_P.Constant4_Value < 0.5) &&
             (raspberrypi_multicore_MPCtest_B.Switch1 > 0.5)) {
    raspberrypi_multicore_MPCtest_B.cmd[2] = 1U;
  } else {
    raspberrypi_multicore_MPCtest_B.cmd[2] = 3U;
  }

  raspberrypi_multicore_MPCtest_B.Angle3new_c = rt_roundd_snf
    (raspberrypi_multicore_MPCtest_P.Constant11_Value);
  if (raspberrypi_multicore_MPCtest_B.Angle3new_c < 256.0) {
    if (raspberrypi_multicore_MPCtest_B.Angle3new_c >= 0.0) {
      status = (uint8_T)raspberrypi_multicore_MPCtest_B.Angle3new_c;
    } else {
      status = 0U;
    }
  } else {
    status = MAX_uint8_T;
  }

  raspberrypi_multicore_MPCtest_B.ar = (int32_T)(status * 10U);
  if ((uint32_T)raspberrypi_multicore_MPCtest_B.ar > 255U) {
    raspberrypi_multicore_MPCtest_B.ar = 255;
  }

  raspberrypi_multicore_MPCtest_B.ar += (uint32_T)
    raspberrypi_multicore_MPCtest_B.cmd[2];
  if ((uint32_T)raspberrypi_multicore_MPCtest_B.ar > 255U) {
    raspberrypi_multicore_MPCtest_B.ar = 255;
  }

  raspberrypi_multicore_MPCtest_B.cmd[2] = (uint8_T)
    raspberrypi_multicore_MPCtest_B.ar;
  for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
       raspberrypi_multicore_MPCtest_B.i_f2 < 12;
       raspberrypi_multicore_MPCtest_B.i_f2++) {
    raspberrypi_multicore_MPCtest_B.Angle3new_c = rt_roundd_snf
      (raspberrypi_multicore_MPCtest_B.pL_sw[raspberrypi_multicore_MPCtest_B.i_f2]);
    if (raspberrypi_multicore_MPCtest_B.Angle3new_c < 256.0) {
      if (raspberrypi_multicore_MPCtest_B.Angle3new_c >= 0.0) {
        raspberrypi_multicore_MPCtest_B.cmd[raspberrypi_multicore_MPCtest_B.i_f2
          + 3] = (uint8_T)raspberrypi_multicore_MPCtest_B.Angle3new_c;
      } else {
        raspberrypi_multicore_MPCtest_B.cmd[raspberrypi_multicore_MPCtest_B.i_f2
          + 3] = 0U;
      }
    } else {
      raspberrypi_multicore_MPCtest_B.cmd[raspberrypi_multicore_MPCtest_B.i_f2 +
        3] = MAX_uint8_T;
    }

    raspberrypi_multicore_MPCtest_B.Angle3new_c = rt_roundd_snf
      (raspberrypi_multicore_MPCtes_DW.obj_p.PosDesOld[raspberrypi_multicore_MPCtest_B.i_f2]
       * 10.0);
    if (raspberrypi_multicore_MPCtest_B.Angle3new_c < 32768.0) {
      if (raspberrypi_multicore_MPCtest_B.Angle3new_c >= -32768.0) {
        temp = (int16_T)raspberrypi_multicore_MPCtest_B.Angle3new_c;
      } else {
        temp = MIN_int16_T;
      }
    } else {
      temp = MAX_int16_T;
    }

    memcpy((void *)&resAngle[0], (void *)&temp, (uint32_T)((size_t)2 * sizeof
            (uint8_T)));
    raspberrypi_multicore_MPCtest_B.ar = (raspberrypi_multicore_MPCtest_B.i_f2 +
      1) << 1;
    raspberrypi_multicore_MPCtest_B.cmd[raspberrypi_multicore_MPCtest_B.ar + 13]
      = resAngle[0];
    raspberrypi_multicore_MPCtest_B.cmd[raspberrypi_multicore_MPCtest_B.ar + 14]
      = resAngle[1];
  }

  raspberrypi_multicore_MPCtest_B.Switch1 = raspberrypi_multicore_MPCtest_B.cmd
    [0];
  for (raspberrypi_multicore_MPCtest_B.ar = 0;
       raspberrypi_multicore_MPCtest_B.ar < 58;
       raspberrypi_multicore_MPCtest_B.ar++) {
    raspberrypi_multicore_MPCtest_B.Switch1 += (real_T)
      raspberrypi_multicore_MPCtest_B.cmd[raspberrypi_multicore_MPCtest_B.ar + 1];
  }

  if (rtIsInf(raspberrypi_multicore_MPCtest_B.Switch1)) {
    raspberrypi_multicore_MPCtest_B.cmd[59] = 0U;
  } else if (raspberrypi_multicore_MPCtest_B.Switch1 == 0.0) {
    raspberrypi_multicore_MPCtest_B.cmd[59] = 0U;
  } else {
    raspberrypi_multicore_MPCtest_B.ar = (int32_T)fmod
      (raspberrypi_multicore_MPCtest_B.Switch1, 256.0);
    if (raspberrypi_multicore_MPCtest_B.ar < 256) {
      raspberrypi_multicore_MPCtest_B.cmd[59] = (uint8_T)
        raspberrypi_multicore_MPCtest_B.ar;
    } else {
      raspberrypi_multicore_MPCtest_B.cmd[59] = MAX_uint8_T;
    }
  }

  /* MATLABSystem: '<S14>/SPI Master Transfer' */
  raspberrypi_multicore_MPCtest_B.PinNameLoc = SPI0_CE0;
  MW_SPI_SetSlaveSelect(raspberrypi_multicore_MPCtes_DW.obj_h.MW_SPI_HANDLE,
                        raspberrypi_multicore_MPCtest_B.PinNameLoc, true);
  raspberrypi_multicore_MPCtest_B.ClockModeValue = MW_SPI_MODE_0;
  raspberrypi_multicore_MPCtest_B.MsbFirstTransferLoc =
    MW_SPI_MOST_SIGNIFICANT_BIT_FIRST;
  status = MW_SPI_SetFormat(raspberrypi_multicore_MPCtes_DW.obj_h.MW_SPI_HANDLE,
    8, raspberrypi_multicore_MPCtest_B.ClockModeValue,
    raspberrypi_multicore_MPCtest_B.MsbFirstTransferLoc);
  if (status == 0) {
    MW_SPI_MasterWriteRead_8bits
      (raspberrypi_multicore_MPCtes_DW.obj_h.MW_SPI_HANDLE,
       &raspberrypi_multicore_MPCtest_B.cmd[0],
       &raspberrypi_multicore_MPCtest_B.rdDataRaw[0], 60U);
  }

  memcpy((void *)&raspberrypi_multicore_MPCtest_B.cmd[0], (void *)
         &raspberrypi_multicore_MPCtest_B.rdDataRaw[0], (uint32_T)((size_t)60 *
          sizeof(uint8_T)));

  /* MATLAB Function: '<S14>/MATLAB Function1' incorporates:
   *  MATLABSystem: '<S14>/SPI Master Transfer'
   */
  raspberrypi_multicore_MPCtest_B.vcol = 0;
  if (raspberrypi_multicore_MPCtest_B.cmd[0] != 102) {
    raspberrypi_multicore_MPCtest_B.vcol = 1;
  }

  raspberrypi_multicore_MPCtest_B.Switch1 = raspberrypi_multicore_MPCtest_B.cmd
    [0];
  for (raspberrypi_multicore_MPCtest_B.ar = 0;
       raspberrypi_multicore_MPCtest_B.ar < 58;
       raspberrypi_multicore_MPCtest_B.ar++) {
    raspberrypi_multicore_MPCtest_B.Switch1 += (real_T)
      raspberrypi_multicore_MPCtest_B.cmd[raspberrypi_multicore_MPCtest_B.ar + 1];
  }

  if (rtIsInf(raspberrypi_multicore_MPCtest_B.Switch1)) {
    raspberrypi_multicore_MPCtest_B.Angle3new_c = (rtNaN);
  } else if (raspberrypi_multicore_MPCtest_B.Switch1 == 0.0) {
    raspberrypi_multicore_MPCtest_B.Angle3new_c = 0.0;
  } else {
    raspberrypi_multicore_MPCtest_B.Angle3new_c = fmod
      (raspberrypi_multicore_MPCtest_B.Switch1, 256.0);
  }

  if (raspberrypi_multicore_MPCtest_B.Angle3new_c < 256.0) {
    status = (uint8_T)raspberrypi_multicore_MPCtest_B.Angle3new_c;
  } else {
    status = MAX_uint8_T;
  }

  if (status != raspberrypi_multicore_MPCtest_B.cmd[59]) {
    raspberrypi_multicore_MPCtest_B.vcol = 1;
  }

  for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
       raspberrypi_multicore_MPCtest_B.i_f2 < 12;
       raspberrypi_multicore_MPCtest_B.i_f2++) {
    memcpy((void *)&temp, (void *)&raspberrypi_multicore_MPCtest_B.cmd
           [((raspberrypi_multicore_MPCtest_B.i_f2 + 1) << 1) + 12], (uint32_T)
           ((size_t)1 * sizeof(int16_T)));
    raspberrypi_multicore_MPCtes_DW.UnitDelay_DSTATE[raspberrypi_multicore_MPCtest_B.i_f2]
      = (real_T)temp / 10.0;
  }

  memcpy((void *)&temp, (void *)&raspberrypi_multicore_MPCtest_B.cmd[38],
         (uint32_T)((size_t)1 * sizeof(int16_T)));
  raspberrypi_multicore_MPCtest_B.v1[0] = (real_T)temp / 32768.0 * 16.0 * 9.8;
  memcpy((void *)&temp, (void *)&raspberrypi_multicore_MPCtest_B.cmd[40],
         (uint32_T)((size_t)1 * sizeof(int16_T)));
  raspberrypi_multicore_MPCtest_B.v1[1] = (real_T)temp / 32768.0 * 16.0 * 9.8;
  memcpy((void *)&temp, (void *)&raspberrypi_multicore_MPCtest_B.cmd[42],
         (uint32_T)((size_t)1 * sizeof(int16_T)));
  raspberrypi_multicore_MPCtest_B.v1[2] = (real_T)temp / 32768.0 * 16.0 * 9.8;
  memcpy((void *)&temp, (void *)&raspberrypi_multicore_MPCtest_B.cmd[44],
         (uint32_T)((size_t)1 * sizeof(int16_T)));
  raspberrypi_multicore_MPCtest_B.a[0] = (real_T)temp / 32768.0 * 2000.0 / 180.0
    * 3.1415926535897931;
  memcpy((void *)&temp, (void *)&raspberrypi_multicore_MPCtest_B.cmd[46],
         (uint32_T)((size_t)1 * sizeof(int16_T)));
  raspberrypi_multicore_MPCtest_B.a[1] = (real_T)temp / 32768.0 * 2000.0 / 180.0
    * 3.1415926535897931;
  memcpy((void *)&temp, (void *)&raspberrypi_multicore_MPCtest_B.cmd[48],
         (uint32_T)((size_t)1 * sizeof(int16_T)));
  raspberrypi_multicore_MPCtest_B.a[2] = (real_T)temp / 32768.0 * 2000.0 / 180.0
    * 3.1415926535897931;
  memcpy((void *)&temp, (void *)&raspberrypi_multicore_MPCtest_B.cmd[50],
         (uint32_T)((size_t)1 * sizeof(int16_T)));
  raspberrypi_multicore_MPCtest_B.deltaP2[0] = (real_T)temp / 32768.0 * 180.0 /
    180.0 * 3.1415926535897931;
  memcpy((void *)&temp, (void *)&raspberrypi_multicore_MPCtest_B.cmd[52],
         (uint32_T)((size_t)1 * sizeof(int16_T)));
  raspberrypi_multicore_MPCtest_B.deltaP2[1] = (real_T)temp / 32768.0 * 180.0 /
    180.0 * 3.1415926535897931;
  memcpy((void *)&temp, (void *)&raspberrypi_multicore_MPCtest_B.cmd[54],
         (uint32_T)((size_t)1 * sizeof(int16_T)));
  raspberrypi_multicore_MPCtest_B.deltaP2[2] = (real_T)temp / 32768.0 * 180.0 /
    180.0 * 3.1415926535897931;
  raspberrypi_multicore_MPCtest_B.err = raspberrypi_multicore_MPCtest_B.vcol;

  /* End of MATLAB Function: '<S14>/MATLAB Function1' */

  /* MATLABSystem: '<Root>/MATLAB System1' */
  /*          %% Define output properties */
  /*  the output accL_W is the decoupled linear acc */
  /*  acc and omega is in the world coordinate */
  /*  omega is in radian */
  /*  3D rotation matrix, from world to body */
  raspberrypi_multicore_MPCtest_B.Switch1 = sin
    (raspberrypi_multicore_MPCtest_B.deltaP2[0]);
  raspberrypi_multicore_MPCtest_B.s_k = cos
    (raspberrypi_multicore_MPCtest_B.deltaP2[0]);

  /*  3D rotation matrix, from world to body */
  raspberrypi_multicore_MPCtest_B.t6 = sin
    (raspberrypi_multicore_MPCtest_B.deltaP2[1]);
  raspberrypi_multicore_MPCtest_B.t7 = cos
    (raspberrypi_multicore_MPCtest_B.deltaP2[1]);

  /*  3D rotation matrix, vb=M*v: */
  /*  rotate a vector in one frame, */
  /*  or change the vector 'v' in rotated frame to 'vb' in world frame */
  raspberrypi_multicore_MPCtest_B.t8 = sin
    (raspberrypi_multicore_MPCtest_B.deltaP2[2]);
  raspberrypi_multicore_MPCtest_B.t9 = cos
    (raspberrypi_multicore_MPCtest_B.deltaP2[2]);
  raspberrypi_multicore_MPCtest_B.MRz[0] = raspberrypi_multicore_MPCtest_B.t9;
  raspberrypi_multicore_MPCtest_B.MRz[3] = -raspberrypi_multicore_MPCtest_B.t8;
  raspberrypi_multicore_MPCtest_B.MRz[6] = 0.0;
  raspberrypi_multicore_MPCtest_B.MRz[1] = raspberrypi_multicore_MPCtest_B.t8;
  raspberrypi_multicore_MPCtest_B.MRz[4] = raspberrypi_multicore_MPCtest_B.t9;
  raspberrypi_multicore_MPCtest_B.MRz[7] = 0.0;
  raspberrypi_multicore_MPCtest_B.R[0] = raspberrypi_multicore_MPCtest_B.t7;
  raspberrypi_multicore_MPCtest_B.R[3] = 0.0;
  raspberrypi_multicore_MPCtest_B.R[6] = raspberrypi_multicore_MPCtest_B.t6;
  raspberrypi_multicore_MPCtest_B.MRz[2] = 0.0;
  raspberrypi_multicore_MPCtest_B.R[1] = 0.0;
  raspberrypi_multicore_MPCtest_B.MRz[5] = 0.0;
  raspberrypi_multicore_MPCtest_B.R[4] = 1.0;
  raspberrypi_multicore_MPCtest_B.MRz[8] = 1.0;
  raspberrypi_multicore_MPCtest_B.R[7] = 0.0;
  raspberrypi_multicore_MPCtest_B.R[2] = -raspberrypi_multicore_MPCtest_B.t6;
  raspberrypi_multicore_MPCtest_B.R[5] = 0.0;
  raspberrypi_multicore_MPCtest_B.R[8] = raspberrypi_multicore_MPCtest_B.t7;
  for (raspberrypi_multicore_MPCtest_B.ar = 0;
       raspberrypi_multicore_MPCtest_B.ar < 3;
       raspberrypi_multicore_MPCtest_B.ar++) {
    for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
         raspberrypi_multicore_MPCtest_B.i_f2 < 3;
         raspberrypi_multicore_MPCtest_B.i_f2++) {
      raspberrypi_multicore_MPCtest_B.vcol = raspberrypi_multicore_MPCtest_B.ar
        + 3 * raspberrypi_multicore_MPCtest_B.i_f2;
      raspberrypi_multicore_MPCtest_B.V[raspberrypi_multicore_MPCtest_B.vcol] =
        0.0;
      raspberrypi_multicore_MPCtest_B.V[raspberrypi_multicore_MPCtest_B.vcol] +=
        raspberrypi_multicore_MPCtest_B.R[3 *
        raspberrypi_multicore_MPCtest_B.i_f2] *
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar];
      raspberrypi_multicore_MPCtest_B.V[raspberrypi_multicore_MPCtest_B.vcol] +=
        raspberrypi_multicore_MPCtest_B.R[3 *
        raspberrypi_multicore_MPCtest_B.i_f2 + 1] *
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar +
        3];
      raspberrypi_multicore_MPCtest_B.V[raspberrypi_multicore_MPCtest_B.vcol] +=
        raspberrypi_multicore_MPCtest_B.R[3 *
        raspberrypi_multicore_MPCtest_B.i_f2 + 2] *
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar +
        6];
    }

    raspberrypi_multicore_MPCtest_B.U_a[3 * raspberrypi_multicore_MPCtest_B.ar] =
      d[raspberrypi_multicore_MPCtest_B.ar];
  }

  raspberrypi_multicore_MPCtest_B.U_a[1] = 0.0;
  raspberrypi_multicore_MPCtest_B.U_a[4] = raspberrypi_multicore_MPCtest_B.s_k;
  raspberrypi_multicore_MPCtest_B.U_a[7] =
    -raspberrypi_multicore_MPCtest_B.Switch1;
  raspberrypi_multicore_MPCtest_B.U_a[2] = 0.0;
  raspberrypi_multicore_MPCtest_B.U_a[5] =
    raspberrypi_multicore_MPCtest_B.Switch1;
  raspberrypi_multicore_MPCtest_B.U_a[8] = raspberrypi_multicore_MPCtest_B.s_k;
  for (raspberrypi_multicore_MPCtest_B.ar = 0;
       raspberrypi_multicore_MPCtest_B.ar < 3;
       raspberrypi_multicore_MPCtest_B.ar++) {
    for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
         raspberrypi_multicore_MPCtest_B.i_f2 < 3;
         raspberrypi_multicore_MPCtest_B.i_f2++) {
      raspberrypi_multicore_MPCtest_B.c_h = raspberrypi_multicore_MPCtest_B.i_f2
        + 3 * raspberrypi_multicore_MPCtest_B.ar;
      raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.c_h] =
        0.0;
      raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.c_h] +=
        raspberrypi_multicore_MPCtest_B.U_a[3 *
        raspberrypi_multicore_MPCtest_B.ar] *
        raspberrypi_multicore_MPCtest_B.V[raspberrypi_multicore_MPCtest_B.i_f2];
      raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.c_h] +=
        raspberrypi_multicore_MPCtest_B.U_a[3 *
        raspberrypi_multicore_MPCtest_B.ar + 1] *
        raspberrypi_multicore_MPCtest_B.V[raspberrypi_multicore_MPCtest_B.i_f2 +
        3];
      raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.c_h] +=
        raspberrypi_multicore_MPCtest_B.U_a[3 *
        raspberrypi_multicore_MPCtest_B.ar + 2] *
        raspberrypi_multicore_MPCtest_B.V[raspberrypi_multicore_MPCtest_B.i_f2 +
        6];
    }
  }

  for (raspberrypi_multicore_MPCtest_B.ar = 0;
       raspberrypi_multicore_MPCtest_B.ar < 3;
       raspberrypi_multicore_MPCtest_B.ar++) {
    raspberrypi_multicore_MPCtest_B.deltaP3[raspberrypi_multicore_MPCtest_B.ar] =
      ((raspberrypi_multicore_MPCtest_B.R[3 * raspberrypi_multicore_MPCtest_B.ar
        + 1] * 0.0 + raspberrypi_multicore_MPCtest_B.R[3 *
        raspberrypi_multicore_MPCtest_B.ar] * 0.0) +
       raspberrypi_multicore_MPCtest_B.R[3 * raspberrypi_multicore_MPCtest_B.ar
       + 2] * -9.8) +
      raspberrypi_multicore_MPCtest_B.v1[raspberrypi_multicore_MPCtest_B.ar];
  }

  if (raspberrypi_multicore_MPCtest_B.estEN > 0.5) {
    for (raspberrypi_multicore_MPCtest_B.ar = 0;
         raspberrypi_multicore_MPCtest_B.ar < 3;
         raspberrypi_multicore_MPCtest_B.ar++) {
      raspberrypi_multicore_MPCtes_DW.obj.Roff[3 *
        raspberrypi_multicore_MPCtest_B.ar] =
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar];
      raspberrypi_multicore_MPCtes_DW.obj.Roff[3 *
        raspberrypi_multicore_MPCtest_B.ar + 1] =
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar +
        3];
      raspberrypi_multicore_MPCtes_DW.obj.Roff[3 *
        raspberrypi_multicore_MPCtest_B.ar + 2] =
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar +
        6];
    }

    raspberrypi_multicore_MPCtest_B.Switch1 =
      raspberrypi_multicore_MPCtes_DW.obj.accStore[0];
    raspberrypi_multicore_MPCtest_B.s_k =
      raspberrypi_multicore_MPCtes_DW.obj.accStore[1];
    raspberrypi_multicore_MPCtest_B.t6 =
      raspberrypi_multicore_MPCtes_DW.obj.accStore[2];
    for (raspberrypi_multicore_MPCtest_B.ar = 0;
         raspberrypi_multicore_MPCtest_B.ar < 399;
         raspberrypi_multicore_MPCtest_B.ar++) {
      raspberrypi_multicore_MPCtest_B.i_f2 = (raspberrypi_multicore_MPCtest_B.ar
        + 1) * 3;
      raspberrypi_multicore_MPCtest_B.Switch1 +=
        raspberrypi_multicore_MPCtes_DW.obj.accStore[raspberrypi_multicore_MPCtest_B.i_f2];
      raspberrypi_multicore_MPCtest_B.s_k +=
        raspberrypi_multicore_MPCtes_DW.obj.accStore[raspberrypi_multicore_MPCtest_B.i_f2
        + 1];
      raspberrypi_multicore_MPCtest_B.t6 +=
        raspberrypi_multicore_MPCtes_DW.obj.accStore[raspberrypi_multicore_MPCtest_B.i_f2
        + 2];
    }

    raspberrypi_multicore_MPCtes_DW.obj.accOff[0] =
      -raspberrypi_multicore_MPCtest_B.Switch1 / 400.0;
    raspberrypi_multicore_MPCtes_DW.obj.accOff[1] =
      -raspberrypi_multicore_MPCtest_B.s_k / 400.0;
    raspberrypi_multicore_MPCtes_DW.obj.accOff[2] =
      -raspberrypi_multicore_MPCtest_B.t6 / 400.0;
  }

  /*  Rotation matrix to euler angles, ZYX intrinsic order */
  for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
       raspberrypi_multicore_MPCtest_B.i_f2 < 3;
       raspberrypi_multicore_MPCtest_B.i_f2++) {
    for (raspberrypi_multicore_MPCtest_B.ar = 0;
         raspberrypi_multicore_MPCtest_B.ar < 3;
         raspberrypi_multicore_MPCtest_B.ar++) {
      raspberrypi_multicore_MPCtest_B.vcol =
        raspberrypi_multicore_MPCtest_B.i_f2 + 3 *
        raspberrypi_multicore_MPCtest_B.ar;
      raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.vcol] =
        0.0;
      raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.vcol] +=
        raspberrypi_multicore_MPCtest_B.R[3 * raspberrypi_multicore_MPCtest_B.ar]
        * raspberrypi_multicore_MPCtes_DW.obj.Roff[raspberrypi_multicore_MPCtest_B.i_f2];
      raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.vcol] +=
        raspberrypi_multicore_MPCtest_B.R[3 * raspberrypi_multicore_MPCtest_B.ar
        + 1] *
        raspberrypi_multicore_MPCtes_DW.obj.Roff[raspberrypi_multicore_MPCtest_B.i_f2
        + 3];
      raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.vcol] +=
        raspberrypi_multicore_MPCtest_B.R[3 * raspberrypi_multicore_MPCtest_B.ar
        + 2] *
        raspberrypi_multicore_MPCtes_DW.obj.Roff[raspberrypi_multicore_MPCtest_B.i_f2
        + 6];
    }

    raspberrypi_multicore_MPCtest_B.v1[raspberrypi_multicore_MPCtest_B.i_f2] =
      0.0;
  }

  raspberrypi_multicore_MPCtest_B.Switch1 = sqrt
    (raspberrypi_multicore_MPCtest_B.MRz[0] *
     raspberrypi_multicore_MPCtest_B.MRz[0] +
     raspberrypi_multicore_MPCtest_B.MRz[1] *
     raspberrypi_multicore_MPCtest_B.MRz[1]);
  if (raspberrypi_multicore_MPCtest_B.Switch1 < 1.0E-6) {
    raspberrypi_multicore_MPCtest_B.v1[0] = rt_atan2d_snf
      (-raspberrypi_multicore_MPCtest_B.MRz[7],
       raspberrypi_multicore_MPCtest_B.MRz[4]);
    raspberrypi_multicore_MPCtest_B.v1[1] = rt_atan2d_snf
      (-raspberrypi_multicore_MPCtest_B.MRz[2],
       raspberrypi_multicore_MPCtest_B.Switch1);
  } else {
    raspberrypi_multicore_MPCtest_B.v1[0] = rt_atan2d_snf
      (raspberrypi_multicore_MPCtest_B.MRz[5],
       raspberrypi_multicore_MPCtest_B.MRz[8]);
    raspberrypi_multicore_MPCtest_B.v1[1] = rt_atan2d_snf
      (-raspberrypi_multicore_MPCtest_B.MRz[2],
       raspberrypi_multicore_MPCtest_B.Switch1);
    raspberrypi_multicore_MPCtest_B.v1[2] = rt_atan2d_snf
      (raspberrypi_multicore_MPCtest_B.MRz[1],
       raspberrypi_multicore_MPCtest_B.MRz[0]);
  }

  /* %% expand the yaw angle into endless radian angle */
  if ((fabs((raspberrypi_multicore_MPCtest_B.v1[2] -
             raspberrypi_multicore_MPCtes_DW.obj.sitaOld[2]) +
            raspberrypi_multicore_MPCtes_DW.obj.yawOff) > 4.71238898038469) &&
      (raspberrypi_multicore_MPCtest_B.estEN < 0.5)) {
    raspberrypi_multicore_MPCtest_B.t7 =
      (raspberrypi_multicore_MPCtes_DW.obj.sitaOld[2] -
       raspberrypi_multicore_MPCtest_B.v1[2]) -
      raspberrypi_multicore_MPCtes_DW.obj.yawOff;
    if (raspberrypi_multicore_MPCtest_B.t7 < 0.0) {
      raspberrypi_multicore_MPCtest_B.t7 = -1.0;
    } else if (raspberrypi_multicore_MPCtest_B.t7 > 0.0) {
      raspberrypi_multicore_MPCtest_B.t7 = 1.0;
    } else if (raspberrypi_multicore_MPCtest_B.t7 == 0.0) {
      raspberrypi_multicore_MPCtest_B.t7 = 0.0;
    } else {
      raspberrypi_multicore_MPCtest_B.t7 = (rtNaN);
    }

    raspberrypi_multicore_MPCtes_DW.obj.yawOff += 6.2831853071795862 *
      raspberrypi_multicore_MPCtest_B.t7;
  } else if (raspberrypi_multicore_MPCtest_B.estEN > 0.5) {
    raspberrypi_multicore_MPCtes_DW.obj.yawOff = 0.0;
  }

  raspberrypi_multicore_MPCtest_B.v1[2] +=
    raspberrypi_multicore_MPCtes_DW.obj.yawOff;
  raspberrypi_multicore_MPCtes_DW.obj.sitaOld[0] =
    raspberrypi_multicore_MPCtest_B.v1[0];
  raspberrypi_multicore_MPCtes_DW.obj.sitaOld[1] =
    raspberrypi_multicore_MPCtest_B.v1[1];
  raspberrypi_multicore_MPCtes_DW.obj.sitaOld[2] =
    raspberrypi_multicore_MPCtest_B.v1[2];
  for (raspberrypi_multicore_MPCtest_B.ar = 0;
       raspberrypi_multicore_MPCtest_B.ar < 399;
       raspberrypi_multicore_MPCtest_B.ar++) {
    raspberrypi_multicore_MPCtest_B.dv4[3 * raspberrypi_multicore_MPCtest_B.ar] =
      raspberrypi_multicore_MPCtes_DW.obj.accStore[3 *
      raspberrypi_multicore_MPCtest_B.ar];
    raspberrypi_multicore_MPCtest_B.i_f2 = 3 *
      raspberrypi_multicore_MPCtest_B.ar + 1;
    raspberrypi_multicore_MPCtest_B.dv4[raspberrypi_multicore_MPCtest_B.i_f2] =
      raspberrypi_multicore_MPCtes_DW.obj.accStore[raspberrypi_multicore_MPCtest_B.i_f2];
    raspberrypi_multicore_MPCtest_B.i_f2 = 3 *
      raspberrypi_multicore_MPCtest_B.ar + 2;
    raspberrypi_multicore_MPCtest_B.dv4[raspberrypi_multicore_MPCtest_B.i_f2] =
      raspberrypi_multicore_MPCtes_DW.obj.accStore[raspberrypi_multicore_MPCtest_B.i_f2];
  }

  for (raspberrypi_multicore_MPCtest_B.ar = 0;
       raspberrypi_multicore_MPCtest_B.ar < 399;
       raspberrypi_multicore_MPCtest_B.ar++) {
    raspberrypi_multicore_MPCtest_B.i_f2 = 3 *
      ((raspberrypi_multicore_MPCtest_B.ar + 2) - 1);
    raspberrypi_multicore_MPCtes_DW.obj.accStore[raspberrypi_multicore_MPCtest_B.i_f2]
      = raspberrypi_multicore_MPCtest_B.dv4[3 *
      raspberrypi_multicore_MPCtest_B.ar];
    raspberrypi_multicore_MPCtes_DW.obj.accStore[raspberrypi_multicore_MPCtest_B.i_f2
      + 1] = raspberrypi_multicore_MPCtest_B.dv4[3 *
      raspberrypi_multicore_MPCtest_B.ar + 1];
    raspberrypi_multicore_MPCtes_DW.obj.accStore[raspberrypi_multicore_MPCtest_B.i_f2
      + 2] = raspberrypi_multicore_MPCtest_B.dv4[3 *
      raspberrypi_multicore_MPCtest_B.ar + 2];
  }

  raspberrypi_multicore_MPCtes_DW.obj.accStore[0] =
    raspberrypi_multicore_MPCtest_B.deltaP3[0];
  raspberrypi_multicore_MPCtest_B.Switch1 =
    raspberrypi_multicore_MPCtest_B.deltaP3[0] +
    raspberrypi_multicore_MPCtes_DW.obj.accOff[0];
  raspberrypi_multicore_MPCtes_DW.obj.accStore[1] =
    raspberrypi_multicore_MPCtest_B.deltaP3[1];
  raspberrypi_multicore_MPCtest_B.s_k = raspberrypi_multicore_MPCtest_B.deltaP3
    [1] + raspberrypi_multicore_MPCtes_DW.obj.accOff[1];
  raspberrypi_multicore_MPCtes_DW.obj.accStore[2] =
    raspberrypi_multicore_MPCtest_B.deltaP3[2];
  raspberrypi_multicore_MPCtest_B.t7 = raspberrypi_multicore_MPCtest_B.deltaP3[2]
    + raspberrypi_multicore_MPCtes_DW.obj.accOff[2];
  for (raspberrypi_multicore_MPCtest_B.ar = 0;
       raspberrypi_multicore_MPCtest_B.ar < 3;
       raspberrypi_multicore_MPCtest_B.ar++) {
    raspberrypi_multicore_MPCtest_B.t8 =
      raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar];
    raspberrypi_multicore_MPCtest_B.t6 = raspberrypi_multicore_MPCtest_B.t8 *
      raspberrypi_multicore_MPCtest_B.Switch1;
    raspberrypi_multicore_MPCtest_B.t9 = raspberrypi_multicore_MPCtest_B.t8 *
      raspberrypi_multicore_MPCtest_B.a[0];
    raspberrypi_multicore_MPCtest_B.t8 =
      raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar + 3];
    raspberrypi_multicore_MPCtest_B.t6 += raspberrypi_multicore_MPCtest_B.t8 *
      raspberrypi_multicore_MPCtest_B.s_k;
    raspberrypi_multicore_MPCtest_B.t9 += raspberrypi_multicore_MPCtest_B.t8 *
      raspberrypi_multicore_MPCtest_B.a[1];
    raspberrypi_multicore_MPCtest_B.t8 =
      raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar + 6];
    raspberrypi_multicore_MPCtest_B.deltaP2[raspberrypi_multicore_MPCtest_B.ar] =
      raspberrypi_multicore_MPCtest_B.t8 * raspberrypi_multicore_MPCtest_B.t7 +
      raspberrypi_multicore_MPCtest_B.t6;
    raspberrypi_multicore_MPCtest_B.deltaP3[raspberrypi_multicore_MPCtest_B.ar] =
      raspberrypi_multicore_MPCtest_B.t8 * raspberrypi_multicore_MPCtest_B.a[2]
      + raspberrypi_multicore_MPCtest_B.t9;
  }

  /* MATLAB Function: '<Root>/ServoAngle_to_Norminal' */
  for (raspberrypi_multicore_MPCtest_B.ar = 0;
       raspberrypi_multicore_MPCtest_B.ar < 12;
       raspberrypi_multicore_MPCtest_B.ar++) {
    raspberrypi_multicore_MPCtest_B.pL_sw[raspberrypi_multicore_MPCtest_B.ar] =
      (raspberrypi_multicore_MPCtes_DW.UnitDelay_DSTATE[raspberrypi_multicore_MPCtest_B.ar]
       - angleOff[raspberrypi_multicore_MPCtest_B.ar]) / 180.0 *
      3.1415926535897931;
  }

  raspberrypi_multicore_MPCtest_B.pL_sw[8] =
    -raspberrypi_multicore_MPCtest_B.pL_sw[8];
  raspberrypi_multicore_MPCtest_B.pL_sw[11] =
    -raspberrypi_multicore_MPCtest_B.pL_sw[11];
  for (raspberrypi_multicore_MPCtest_B.ar = 0;
       raspberrypi_multicore_MPCtest_B.ar < 12;
       raspberrypi_multicore_MPCtest_B.ar++) {
    raspberrypi_multicore_MPCtest_B.pL_sw[raspberrypi_multicore_MPCtest_B.ar] =
      -raspberrypi_multicore_MPCtest_B.pL_sw[raspberrypi_multicore_MPCtest_B.ar];
  }

  /* End of MATLAB Function: '<Root>/ServoAngle_to_Norminal' */

  /* MATLABSystem: '<Root>/MATLAB System13' */
  /*  pArray_L is the foot-end position in leg coordinate */
  raspberrypi_multicore_FK_FK_one(&raspberrypi_multicore_MPCtes_DW.obj_m,
    &raspberrypi_multicore_MPCtest_B.pL_sw[0], 1.0,
    raspberrypi_multicore_MPCtest_B.a, &raspberrypi_multicore_MPCtest_B.s_k);
  raspberrypi_multicore_FK_FK_one(&raspberrypi_multicore_MPCtes_DW.obj_m,
    &raspberrypi_multicore_MPCtest_B.pL_sw[3], 2.0,
    raspberrypi_multicore_MPCtest_B.Angle2new,
    &raspberrypi_multicore_MPCtest_B.s_k);
  raspberrypi_multicore_FK_FK_one(&raspberrypi_multicore_MPCtes_DW.obj_m,
    &raspberrypi_multicore_MPCtest_B.pL_sw[6], 3.0,
    raspberrypi_multicore_MPCtest_B.Angle3new,
    &raspberrypi_multicore_MPCtest_B.s_k);
  raspberrypi_multicore_FK_FK_one(&raspberrypi_multicore_MPCtes_DW.obj_m,
    &raspberrypi_multicore_MPCtest_B.pL_sw[9], 4.0,
    raspberrypi_multicore_MPCtest_B.Angle4new,
    &raspberrypi_multicore_MPCtest_B.s_k);
  raspberrypi_multicore_MPCtest_B.pL_sw[0] = raspberrypi_multicore_MPCtest_B.a[0];
  raspberrypi_multicore_MPCtest_B.pL_sw[3] =
    raspberrypi_multicore_MPCtest_B.Angle2new[0];
  raspberrypi_multicore_MPCtest_B.pL_sw[6] =
    raspberrypi_multicore_MPCtest_B.Angle3new[0];
  raspberrypi_multicore_MPCtest_B.pL_sw[9] =
    raspberrypi_multicore_MPCtest_B.Angle4new[0];
  raspberrypi_multicore_MPCtest_B.pL_sw[1] = raspberrypi_multicore_MPCtest_B.a[1];
  raspberrypi_multicore_MPCtest_B.pL_sw[4] =
    raspberrypi_multicore_MPCtest_B.Angle2new[1];
  raspberrypi_multicore_MPCtest_B.pL_sw[7] =
    raspberrypi_multicore_MPCtest_B.Angle3new[1];
  raspberrypi_multicore_MPCtest_B.pL_sw[10] =
    raspberrypi_multicore_MPCtest_B.Angle4new[1];
  raspberrypi_multicore_MPCtest_B.pL_sw[2] = raspberrypi_multicore_MPCtest_B.a[2];
  raspberrypi_multicore_MPCtest_B.pL_sw[5] =
    raspberrypi_multicore_MPCtest_B.Angle2new[2];
  raspberrypi_multicore_MPCtest_B.pL_sw[8] =
    raspberrypi_multicore_MPCtest_B.Angle3new[2];
  raspberrypi_multicore_MPCtest_B.pL_sw[11] =
    raspberrypi_multicore_MPCtest_B.Angle4new[2];

  /* MATLAB Function: '<Root>/Leg Cor 2 Body Cor' incorporates:
   *  MATLABSystem: '<Root>/MATLAB System13'
   */
  raspberrypi_multicore_MPCtest_B.pArray_B[0] = 105.4;
  raspberrypi_multicore_MPCtest_B.pArray_B[3] = 105.4;
  raspberrypi_multicore_MPCtest_B.pArray_B[6] = -105.4;
  raspberrypi_multicore_MPCtest_B.pArray_B[9] = -105.4;
  raspberrypi_multicore_MPCtest_B.pArray_B[1] = 48.5;
  raspberrypi_multicore_MPCtest_B.pArray_B[4] = -48.5;
  raspberrypi_multicore_MPCtest_B.pArray_B[7] = 48.5;
  raspberrypi_multicore_MPCtest_B.pArray_B[10] = -48.5;
  raspberrypi_multicore_MPCtest_B.pArray_B[2] = 0.0;
  raspberrypi_multicore_MPCtest_B.pArray_B[5] = 0.0;
  raspberrypi_multicore_MPCtest_B.pArray_B[8] = 0.0;
  raspberrypi_multicore_MPCtest_B.pArray_B[11] = 0.0;
  for (raspberrypi_multicore_MPCtest_B.ar = 0;
       raspberrypi_multicore_MPCtest_B.ar < 12;
       raspberrypi_multicore_MPCtest_B.ar++) {
    raspberrypi_multicore_MPCtest_B.pArray_B[raspberrypi_multicore_MPCtest_B.ar]
      +=
      raspberrypi_multicore_MPCtest_B.pL_sw[raspberrypi_multicore_MPCtest_B.ar];
  }

  /* End of MATLAB Function: '<Root>/Leg Cor 2 Body Cor' */

  /* MATLABSystem: '<S52>/Digital Read' */
  if (raspberrypi_multicore_MPCtes_DW.obj_ls.SampleTime !=
      raspberrypi_multicore_MPCtest_P.DigitalRead_SampleTime) {
    raspberrypi_multicore_MPCtes_DW.obj_ls.SampleTime =
      raspberrypi_multicore_MPCtest_P.DigitalRead_SampleTime;
  }

  EXT_GPIO_read(24U, &rtb_DigitalRead_pz_0);

  /* MATLABSystem: '<S53>/Digital Read' */
  if (raspberrypi_multicore_MPCtes_DW.obj_cu.SampleTime !=
      raspberrypi_multicore_MPCtest_P.DigitalRead_SampleTime_f) {
    raspberrypi_multicore_MPCtes_DW.obj_cu.SampleTime =
      raspberrypi_multicore_MPCtest_P.DigitalRead_SampleTime_f;
  }

  EXT_GPIO_read(23U, &rtb_DigitalRead_p_0);

  /* MATLABSystem: '<S54>/Digital Read' */
  if (raspberrypi_multicore_MPCtes_DW.obj_i.SampleTime !=
      raspberrypi_multicore_MPCtest_P.DigitalRead_SampleTime_n) {
    raspberrypi_multicore_MPCtes_DW.obj_i.SampleTime =
      raspberrypi_multicore_MPCtest_P.DigitalRead_SampleTime_n;
  }

  EXT_GPIO_read(18U, &rtb_DigitalRead_k_0);

  /* MATLABSystem: '<S55>/Digital Read' */
  if (raspberrypi_multicore_MPCtes_DW.obj_fv.SampleTime !=
      raspberrypi_multicore_MPCtest_P.DigitalRead_SampleTime_p) {
    raspberrypi_multicore_MPCtes_DW.obj_fv.SampleTime =
      raspberrypi_multicore_MPCtest_P.DigitalRead_SampleTime_p;
  }

  EXT_GPIO_read(4U, &rtb_DigitalRead_0);

  /* DataTypeConversion: '<S16>/Cast To Double' incorporates:
   *  MATLABSystem: '<S52>/Digital Read'
   *  MATLABSystem: '<S53>/Digital Read'
   *  MATLABSystem: '<S54>/Digital Read'
   *  MATLABSystem: '<S55>/Digital Read'
   */
  raspberrypi_multicore_MPCtest_B.pyNew[0] = rtb_DigitalRead_pz_0;
  raspberrypi_multicore_MPCtest_B.pyNew[1] = rtb_DigitalRead_p_0;
  raspberrypi_multicore_MPCtest_B.pyNew[2] = rtb_DigitalRead_k_0;
  raspberrypi_multicore_MPCtest_B.pyNew[3] = rtb_DigitalRead_0;

  /* MATLAB Function: '<S15>/MATLAB Function1' incorporates:
   *  MATLAB Function: '<S15>/MATLAB Function'
   *  MATLABSystem: '<Root>/MATLAB System1'
   *  MATLABSystem: '<S15>/MATLAB System'
   */
  raspberrypi_multicore_MPCtest_B.Switch1 = sin
    (raspberrypi_multicore_MPCtest_B.v1[1]);
  raspberrypi_multicore_MPCtest_B.s_k = cos(raspberrypi_multicore_MPCtest_B.v1[1]);
  raspberrypi_multicore_MPCtest_B.t6 = sin(raspberrypi_multicore_MPCtest_B.v1[0]);
  raspberrypi_multicore_MPCtest_B.t7 = cos(raspberrypi_multicore_MPCtest_B.v1[0]);
  for (raspberrypi_multicore_MPCtest_B.ar = 0;
       raspberrypi_multicore_MPCtest_B.ar < 12;
       raspberrypi_multicore_MPCtest_B.ar++) {
    raspberrypi_multicore_MPCtest_B.pArray_float_tmp[raspberrypi_multicore_MPCtest_B.ar]
      =
      raspberrypi_multicore_MPCtest_B.pArray_B[raspberrypi_multicore_MPCtest_B.ar]
      / 1000.0;
  }

  raspberrypi_multicore_MPCtest_B.R[0] = raspberrypi_multicore_MPCtest_B.s_k;
  raspberrypi_multicore_MPCtest_B.R[3] = 0.0;
  raspberrypi_multicore_MPCtest_B.R[6] = raspberrypi_multicore_MPCtest_B.Switch1;
  raspberrypi_multicore_MPCtest_B.R[2] =
    -raspberrypi_multicore_MPCtest_B.Switch1;
  raspberrypi_multicore_MPCtest_B.R[5] = 0.0;
  raspberrypi_multicore_MPCtest_B.R[8] = raspberrypi_multicore_MPCtest_B.s_k;
  raspberrypi_multicore_MPCtest_B.R[1] = 0.0;
  raspberrypi_multicore_MPCtest_B.U_a[0] = 1.0;
  raspberrypi_multicore_MPCtest_B.R[4] = 1.0;
  raspberrypi_multicore_MPCtest_B.U_a[3] = 0.0;
  raspberrypi_multicore_MPCtest_B.R[7] = 0.0;
  raspberrypi_multicore_MPCtest_B.U_a[6] = 0.0;
  raspberrypi_multicore_MPCtest_B.U_a[1] = 0.0;
  raspberrypi_multicore_MPCtest_B.U_a[4] = raspberrypi_multicore_MPCtest_B.t7;
  raspberrypi_multicore_MPCtest_B.U_a[7] = -raspberrypi_multicore_MPCtest_B.t6;
  raspberrypi_multicore_MPCtest_B.U_a[2] = 0.0;
  raspberrypi_multicore_MPCtest_B.U_a[5] = raspberrypi_multicore_MPCtest_B.t6;
  raspberrypi_multicore_MPCtest_B.U_a[8] = raspberrypi_multicore_MPCtest_B.t7;
  for (raspberrypi_multicore_MPCtest_B.ar = 0;
       raspberrypi_multicore_MPCtest_B.ar < 3;
       raspberrypi_multicore_MPCtest_B.ar++) {
    for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
         raspberrypi_multicore_MPCtest_B.i_f2 < 3;
         raspberrypi_multicore_MPCtest_B.i_f2++) {
      raspberrypi_multicore_MPCtest_B.vcol = raspberrypi_multicore_MPCtest_B.ar
        + 3 * raspberrypi_multicore_MPCtest_B.i_f2;
      raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.vcol] =
        0.0;
      raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.vcol] +=
        raspberrypi_multicore_MPCtest_B.U_a[3 *
        raspberrypi_multicore_MPCtest_B.i_f2] *
        raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.ar];
      raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.vcol] +=
        raspberrypi_multicore_MPCtest_B.U_a[3 *
        raspberrypi_multicore_MPCtest_B.i_f2 + 1] *
        raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.ar + 3];
      raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.vcol] +=
        raspberrypi_multicore_MPCtest_B.U_a[3 *
        raspberrypi_multicore_MPCtest_B.i_f2 + 2] *
        raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.ar + 6];
    }

    for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
         raspberrypi_multicore_MPCtest_B.i_f2 < 4;
         raspberrypi_multicore_MPCtest_B.i_f2++) {
      raspberrypi_multicore_MPCtest_B.vcol = raspberrypi_multicore_MPCtest_B.ar
        + 3 * raspberrypi_multicore_MPCtest_B.i_f2;
      raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[raspberrypi_multicore_MPCtest_B.vcol]
        = 0.0;
      raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[raspberrypi_multicore_MPCtest_B.vcol]
        += raspberrypi_multicore_MPCtest_B.pArray_float_tmp[3 *
        raspberrypi_multicore_MPCtest_B.i_f2] *
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar];
      raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[raspberrypi_multicore_MPCtest_B.vcol]
        += raspberrypi_multicore_MPCtest_B.pArray_float_tmp[3 *
        raspberrypi_multicore_MPCtest_B.i_f2 + 1] *
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar +
        3];
      raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[raspberrypi_multicore_MPCtest_B.vcol]
        += raspberrypi_multicore_MPCtest_B.pArray_float_tmp[3 *
        raspberrypi_multicore_MPCtest_B.i_f2 + 2] *
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar +
        6];
    }
  }

  if (!rtIsNaN(raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[2])) {
    raspberrypi_multicore_MPCtest_B.i_f2 = 1;
  } else {
    raspberrypi_multicore_MPCtest_B.i_f2 = 0;
    raspberrypi_multicore_MPCtest_B.ar = 2;
    exitg1 = false;
    while ((!exitg1) && (raspberrypi_multicore_MPCtest_B.ar < 5)) {
      if (!rtIsNaN(raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp
                   [(raspberrypi_multicore_MPCtest_B.ar - 1) * 3 + 2])) {
        raspberrypi_multicore_MPCtest_B.i_f2 =
          raspberrypi_multicore_MPCtest_B.ar;
        exitg1 = true;
      } else {
        raspberrypi_multicore_MPCtest_B.ar++;
      }
    }
  }

  if (raspberrypi_multicore_MPCtest_B.i_f2 == 0) {
    raspberrypi_multicore_MPCtest_B.t8 =
      raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[2];
  } else {
    raspberrypi_multicore_MPCtest_B.t8 =
      raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp
      [(raspberrypi_multicore_MPCtest_B.i_f2 - 1) * 3 + 2];
    while (raspberrypi_multicore_MPCtest_B.i_f2 + 1 < 5) {
      raspberrypi_multicore_MPCtest_B.Angle3new_c =
        raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[3 *
        raspberrypi_multicore_MPCtest_B.i_f2 + 2];
      if (raspberrypi_multicore_MPCtest_B.t8 >
          raspberrypi_multicore_MPCtest_B.Angle3new_c) {
        raspberrypi_multicore_MPCtest_B.t8 =
          raspberrypi_multicore_MPCtest_B.Angle3new_c;
      }

      raspberrypi_multicore_MPCtest_B.i_f2++;
    }
  }

  for (raspberrypi_multicore_MPCtest_B.ar = 0;
       raspberrypi_multicore_MPCtest_B.ar < 4;
       raspberrypi_multicore_MPCtest_B.ar++) {
    raspberrypi_multicore_MPCtest_B.temp[3 * raspberrypi_multicore_MPCtest_B.ar]
      = raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[3 *
      raspberrypi_multicore_MPCtest_B.ar];
    raspberrypi_multicore_MPCtest_B.i_f2 = 3 *
      raspberrypi_multicore_MPCtest_B.ar + 1;
    raspberrypi_multicore_MPCtest_B.temp[raspberrypi_multicore_MPCtest_B.i_f2] =
      raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[raspberrypi_multicore_MPCtest_B.i_f2];
    raspberrypi_multicore_MPCtest_B.i_f2 = 3 *
      raspberrypi_multicore_MPCtest_B.ar + 2;
    raspberrypi_multicore_MPCtest_B.temp[raspberrypi_multicore_MPCtest_B.i_f2] =
      raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[raspberrypi_multicore_MPCtest_B.i_f2]
      + -raspberrypi_multicore_MPCtest_B.t8;
  }

  /* MATLABSystem: '<S15>/MATLAB System6' incorporates:
   *  DataTypeConversion: '<S16>/Cast To Double'
   *  MATLAB Function: '<S15>/MATLAB Function1'
   *  UnitDelay: '<S15>/Unit Delay'
   */
  /*  SW: [4,1], indicates the touch state of four legs */
  /*  pArray_W:[3,4], the foot-end position in the world coordinate */
  /*  SP: [3,4], the foot-end position in the world coordinate */
  /*  SPLeg: [4,1], on or off gournd indicator */
  if ((raspberrypi_multicore_MPCtes_DW.obj_lj.count < 0.5) ||
      (raspberrypi_multicore_MPCtest_B.estEN > 0.5)) {
    raspberrypi_multicore_MPCtes_DW.obj_lj.SPOld[0] =
      raspberrypi_multicore_MPCtest_B.temp[0];
    raspberrypi_multicore_MPCtes_DW.obj_lj.SPOld[3] =
      raspberrypi_multicore_MPCtest_B.temp[3];
    raspberrypi_multicore_MPCtes_DW.obj_lj.SPOld[6] =
      raspberrypi_multicore_MPCtest_B.temp[6];
    raspberrypi_multicore_MPCtes_DW.obj_lj.SPOld[9] =
      raspberrypi_multicore_MPCtest_B.temp[9];
    raspberrypi_multicore_MPCtes_DW.obj_lj.SPOld[1] =
      raspberrypi_multicore_MPCtest_B.temp[1];
    raspberrypi_multicore_MPCtes_DW.obj_lj.SPOld[4] =
      raspberrypi_multicore_MPCtest_B.temp[4];
    raspberrypi_multicore_MPCtes_DW.obj_lj.SPOld[7] =
      raspberrypi_multicore_MPCtest_B.temp[7];
    raspberrypi_multicore_MPCtes_DW.obj_lj.SPOld[10] =
      raspberrypi_multicore_MPCtest_B.temp[10];
    raspberrypi_multicore_MPCtes_DW.obj_lj.SPOld[2] =
      raspberrypi_multicore_MPCtest_B.temp[2];
    raspberrypi_multicore_MPCtes_DW.obj_lj.SPOld[5] =
      raspberrypi_multicore_MPCtest_B.temp[5];
    raspberrypi_multicore_MPCtes_DW.obj_lj.SPOld[8] =
      raspberrypi_multicore_MPCtest_B.temp[8];
    raspberrypi_multicore_MPCtes_DW.obj_lj.SPOld[11] =
      raspberrypi_multicore_MPCtest_B.temp[11];
    raspberrypi_multicore_MPCtes_DW.obj_lj.count++;
  }

  for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
       raspberrypi_multicore_MPCtest_B.i_f2 < 4;
       raspberrypi_multicore_MPCtest_B.i_f2++) {
    if (raspberrypi_multicore_MPCtest_B.pyNew[raspberrypi_multicore_MPCtest_B.i_f2]
        > 0.5) {
      raspberrypi_multicore_MPCtest_B.pxNew[raspberrypi_multicore_MPCtest_B.i_f2]
        = 1.0;
      if (raspberrypi_multicore_MPCtes_DW.obj_lj.SWOld[raspberrypi_multicore_MPCtest_B.i_f2]
          > 0.5) {
        raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[3 *
          raspberrypi_multicore_MPCtest_B.i_f2] =
          raspberrypi_multicore_MPCtes_DW.obj_lj.SPOld[3 *
          raspberrypi_multicore_MPCtest_B.i_f2];
        raspberrypi_multicore_MPCtest_B.vcol = 3 *
          raspberrypi_multicore_MPCtest_B.i_f2 + 1;
        raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[raspberrypi_multicore_MPCtest_B.vcol]
          =
          raspberrypi_multicore_MPCtes_DW.obj_lj.SPOld[raspberrypi_multicore_MPCtest_B.vcol];
        raspberrypi_multicore_MPCtest_B.vcol = 3 *
          raspberrypi_multicore_MPCtest_B.i_f2 + 2;
        raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[raspberrypi_multicore_MPCtest_B.vcol]
          =
          raspberrypi_multicore_MPCtes_DW.obj_lj.SPOld[raspberrypi_multicore_MPCtest_B.vcol];
      } else {
        raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[3 *
          raspberrypi_multicore_MPCtest_B.i_f2] =
          raspberrypi_multicore_MPCtes_DW.UnitDelay_DSTATE_l[3 *
          raspberrypi_multicore_MPCtest_B.i_f2];
        raspberrypi_multicore_MPCtest_B.vcol = 3 *
          raspberrypi_multicore_MPCtest_B.i_f2 + 1;
        raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[raspberrypi_multicore_MPCtest_B.vcol]
          =
          raspberrypi_multicore_MPCtes_DW.UnitDelay_DSTATE_l[raspberrypi_multicore_MPCtest_B.vcol];
        raspberrypi_multicore_MPCtest_B.vcol = 3 *
          raspberrypi_multicore_MPCtest_B.i_f2 + 2;
        raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[raspberrypi_multicore_MPCtest_B.vcol]
          =
          raspberrypi_multicore_MPCtes_DW.UnitDelay_DSTATE_l[raspberrypi_multicore_MPCtest_B.vcol];
      }

      /*                      SP(:,i)=pArray_W(:,i); */
    } else {
      raspberrypi_multicore_MPCtest_B.pxNew[raspberrypi_multicore_MPCtest_B.i_f2]
        = 0.0;
      raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[3 *
        raspberrypi_multicore_MPCtest_B.i_f2] =
        raspberrypi_multicore_MPCtes_DW.UnitDelay_DSTATE_l[3 *
        raspberrypi_multicore_MPCtest_B.i_f2];
      raspberrypi_multicore_MPCtest_B.vcol = 3 *
        raspberrypi_multicore_MPCtest_B.i_f2 + 1;
      raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[raspberrypi_multicore_MPCtest_B.vcol]
        =
        raspberrypi_multicore_MPCtes_DW.UnitDelay_DSTATE_l[raspberrypi_multicore_MPCtest_B.vcol];
      raspberrypi_multicore_MPCtest_B.vcol = 3 *
        raspberrypi_multicore_MPCtest_B.i_f2 + 2;
      raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[raspberrypi_multicore_MPCtest_B.vcol]
        =
        raspberrypi_multicore_MPCtes_DW.UnitDelay_DSTATE_l[raspberrypi_multicore_MPCtest_B.vcol];
    }
  }

  if (raspberrypi_multicore_MPCtest_B.estEN > 0.5) {
    memcpy(&raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[0],
           &raspberrypi_multicore_MPCtes_DW.obj_lj.SPOld[0], 12U * sizeof(real_T));
  }

  raspberrypi_multicore_MPCtes_DW.obj_lj.SWOld[0] =
    raspberrypi_multicore_MPCtest_B.pxNew[0];
  raspberrypi_multicore_MPCtes_DW.obj_lj.SWOld[1] =
    raspberrypi_multicore_MPCtest_B.pxNew[1];
  raspberrypi_multicore_MPCtes_DW.obj_lj.SWOld[2] =
    raspberrypi_multicore_MPCtest_B.pxNew[2];
  raspberrypi_multicore_MPCtes_DW.obj_lj.SWOld[3] =
    raspberrypi_multicore_MPCtest_B.pxNew[3];

  /* MATLABSystem: '<S15>/MATLAB System' */
  /*  omegaB must be the world coordinate */
  /*  pArray_B is the foot-end position in body coordinate */
  raspberrypi_multicore_MPCtest_B.vcol = 1;

  /* KF_R=0.5*eye(6); */
  for (raspberrypi_multicore_MPCtest_B.ar = 0;
       raspberrypi_multicore_MPCtest_B.ar < 12;
       raspberrypi_multicore_MPCtest_B.ar++) {
    /* MATLABSystem: '<S15>/MATLAB System6' */
    raspberrypi_multicore_MPCtest_B.t9 =
      raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[raspberrypi_multicore_MPCtest_B.ar];
    raspberrypi_multicore_MPCtes_DW.obj_lj.SPOld[raspberrypi_multicore_MPCtest_B.ar]
      = raspberrypi_multicore_MPCtest_B.t9;

    /* MATLABSystem: '<S15>/MATLAB System' incorporates:
     *  MATLABSystem: '<S15>/MATLAB System6'
     */
    raspberrypi_multicore_MPCtest_B.temp[raspberrypi_multicore_MPCtest_B.ar] =
      raspberrypi_multicore_MPCtest_B.t9;
    raspberrypi_multicore_MPCtest_B.b_pArray_B[raspberrypi_multicore_MPCtest_B.ar]
      =
      raspberrypi_multicore_MPCtest_B.pArray_B[raspberrypi_multicore_MPCtest_B.ar]
      / 1000.0;
  }

  /* MATLABSystem: '<S15>/MATLAB System' incorporates:
   *  Constant: '<Root>/Constant5'
   *  Constant: '<S15>/Constant11'
   *  MATLAB Function: '<S15>/MATLAB Function'
   *  MATLABSystem: '<Root>/MATLAB System1'
   *  MATLABSystem: '<S15>/MATLAB System6'
   */
  /*  3D rotation matrix, vb=M*v: */
  /*  rotate a vector in one frame, */
  /*  or change the vector 'v' in rotated frame to 'vb' in world frame */
  /*  3D rotation matrix, from world to body */
  /*  3D rotation matrix, from world to body */
  raspberrypi_multicore_MPCtest_B.t9 = sin(raspberrypi_multicore_MPCtest_B.v1[2]);
  raspberrypi_multicore_MPCtest_B.t6_e = cos(raspberrypi_multicore_MPCtest_B.v1
    [2]);
  raspberrypi_multicore_MPCtest_B.R[0] = raspberrypi_multicore_MPCtest_B.t6_e;
  raspberrypi_multicore_MPCtest_B.R[3] = -raspberrypi_multicore_MPCtest_B.t9;
  raspberrypi_multicore_MPCtest_B.R[6] = 0.0;
  raspberrypi_multicore_MPCtest_B.R[1] = raspberrypi_multicore_MPCtest_B.t9;
  raspberrypi_multicore_MPCtest_B.R[4] = raspberrypi_multicore_MPCtest_B.t6_e;
  raspberrypi_multicore_MPCtest_B.R[7] = 0.0;
  raspberrypi_multicore_MPCtest_B.U_a[0] = raspberrypi_multicore_MPCtest_B.s_k;
  raspberrypi_multicore_MPCtest_B.U_a[3] = 0.0;
  raspberrypi_multicore_MPCtest_B.U_a[6] =
    raspberrypi_multicore_MPCtest_B.Switch1;
  raspberrypi_multicore_MPCtest_B.R[2] = 0.0;
  raspberrypi_multicore_MPCtest_B.U_a[1] = 0.0;
  raspberrypi_multicore_MPCtest_B.R[5] = 0.0;
  raspberrypi_multicore_MPCtest_B.U_a[4] = 1.0;
  raspberrypi_multicore_MPCtest_B.R[8] = 1.0;
  raspberrypi_multicore_MPCtest_B.U_a[7] = 0.0;
  raspberrypi_multicore_MPCtest_B.U_a[2] = -sin
    (raspberrypi_multicore_MPCtest_B.v1[1]);
  raspberrypi_multicore_MPCtest_B.U_a[5] = 0.0;
  raspberrypi_multicore_MPCtest_B.U_a[8] = raspberrypi_multicore_MPCtest_B.s_k;
  for (raspberrypi_multicore_MPCtest_B.ar = 0;
       raspberrypi_multicore_MPCtest_B.ar < 3;
       raspberrypi_multicore_MPCtest_B.ar++) {
    for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
         raspberrypi_multicore_MPCtest_B.i_f2 < 3;
         raspberrypi_multicore_MPCtest_B.i_f2++) {
      raspberrypi_multicore_MPCtest_B.c_h = raspberrypi_multicore_MPCtest_B.ar +
        3 * raspberrypi_multicore_MPCtest_B.i_f2;
      raspberrypi_multicore_MPCtest_B.V[raspberrypi_multicore_MPCtest_B.c_h] =
        0.0;
      raspberrypi_multicore_MPCtest_B.V[raspberrypi_multicore_MPCtest_B.c_h] +=
        raspberrypi_multicore_MPCtest_B.U_a[3 *
        raspberrypi_multicore_MPCtest_B.i_f2] *
        raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.ar];
      raspberrypi_multicore_MPCtest_B.V[raspberrypi_multicore_MPCtest_B.c_h] +=
        raspberrypi_multicore_MPCtest_B.U_a[3 *
        raspberrypi_multicore_MPCtest_B.i_f2 + 1] *
        raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.ar + 3];
      raspberrypi_multicore_MPCtest_B.V[raspberrypi_multicore_MPCtest_B.c_h] +=
        raspberrypi_multicore_MPCtest_B.U_a[3 *
        raspberrypi_multicore_MPCtest_B.i_f2 + 2] *
        raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.ar + 6];
    }

    raspberrypi_multicore_MPCtest_B.MRz[3 * raspberrypi_multicore_MPCtest_B.ar] =
      d[raspberrypi_multicore_MPCtest_B.ar];
  }

  raspberrypi_multicore_MPCtest_B.MRz[1] = 0.0;
  raspberrypi_multicore_MPCtest_B.MRz[4] = raspberrypi_multicore_MPCtest_B.t7;
  raspberrypi_multicore_MPCtest_B.MRz[7] = -sin
    (raspberrypi_multicore_MPCtest_B.v1[0]);
  raspberrypi_multicore_MPCtest_B.MRz[2] = 0.0;
  raspberrypi_multicore_MPCtest_B.MRz[5] = raspberrypi_multicore_MPCtest_B.t6;
  raspberrypi_multicore_MPCtest_B.MRz[8] = raspberrypi_multicore_MPCtest_B.t7;
  for (raspberrypi_multicore_MPCtest_B.ar = 0;
       raspberrypi_multicore_MPCtest_B.ar < 3;
       raspberrypi_multicore_MPCtest_B.ar++) {
    for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
         raspberrypi_multicore_MPCtest_B.i_f2 < 3;
         raspberrypi_multicore_MPCtest_B.i_f2++) {
      raspberrypi_multicore_MPCtest_B.c_h = raspberrypi_multicore_MPCtest_B.i_f2
        + 3 * raspberrypi_multicore_MPCtest_B.ar;
      raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.c_h] =
        0.0;
      raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.c_h] +=
        raspberrypi_multicore_MPCtest_B.MRz[3 *
        raspberrypi_multicore_MPCtest_B.ar] *
        raspberrypi_multicore_MPCtest_B.V[raspberrypi_multicore_MPCtest_B.i_f2];
      raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.c_h] +=
        raspberrypi_multicore_MPCtest_B.MRz[3 *
        raspberrypi_multicore_MPCtest_B.ar + 1] *
        raspberrypi_multicore_MPCtest_B.V[raspberrypi_multicore_MPCtest_B.i_f2 +
        3];
      raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.c_h] +=
        raspberrypi_multicore_MPCtest_B.MRz[3 *
        raspberrypi_multicore_MPCtest_B.ar + 2] *
        raspberrypi_multicore_MPCtest_B.V[raspberrypi_multicore_MPCtest_B.i_f2 +
        6];
    }
  }

  for (raspberrypi_multicore_MPCtest_B.ar = 0;
       raspberrypi_multicore_MPCtest_B.ar < 12;
       raspberrypi_multicore_MPCtest_B.ar++) {
    raspberrypi_multicore_MPCtest_B.vArray[raspberrypi_multicore_MPCtest_B.ar] =
      (raspberrypi_multicore_MPCtest_B.b_pArray_B[raspberrypi_multicore_MPCtest_B.ar]
       - raspberrypi_multicore_MPCtes_DW.obj_b.pArrayOld[raspberrypi_multicore_MPCtest_B.ar])
      / raspberrypi_multicore_MPCtest_P.Ts_DynSim;
  }

  if (raspberrypi_multicore_MPCtes_DW.obj_b.iniCount < 0.5) {
    memset(&raspberrypi_multicore_MPCtest_B.vArray[0], 0, 12U * sizeof(real_T));
    raspberrypi_multicore_MPCtes_DW.obj_b.iniCount = 1.0;
  }

  for (raspberrypi_multicore_MPCtest_B.ar = 0;
       raspberrypi_multicore_MPCtest_B.ar < 4;
       raspberrypi_multicore_MPCtest_B.ar++) {
    for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
         raspberrypi_multicore_MPCtest_B.i_f2 < 3;
         raspberrypi_multicore_MPCtest_B.i_f2++) {
      raspberrypi_multicore_MPCtest_B.c_h = raspberrypi_multicore_MPCtest_B.i_f2
        + 3 * raspberrypi_multicore_MPCtest_B.ar;
      raspberrypi_multicore_MPCtest_B.vBM[raspberrypi_multicore_MPCtest_B.c_h] =
        0.0;
      raspberrypi_multicore_MPCtest_B.vBM[raspberrypi_multicore_MPCtest_B.c_h] +=
        raspberrypi_multicore_MPCtest_B.vArray[3 *
        raspberrypi_multicore_MPCtest_B.ar] *
        raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.i_f2];
      raspberrypi_multicore_MPCtest_B.vBM[raspberrypi_multicore_MPCtest_B.c_h] +=
        raspberrypi_multicore_MPCtest_B.vArray[3 *
        raspberrypi_multicore_MPCtest_B.ar + 1] *
        raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.i_f2 +
        3];
      raspberrypi_multicore_MPCtest_B.vBM[raspberrypi_multicore_MPCtest_B.c_h] +=
        raspberrypi_multicore_MPCtest_B.vArray[3 *
        raspberrypi_multicore_MPCtest_B.ar + 2] *
        raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.i_f2 +
        6];
    }
  }

  memcpy(&raspberrypi_multicore_MPCtest_B.vArray[0],
         &raspberrypi_multicore_MPCtest_B.vBM[0], 12U * sizeof(real_T));
  for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
       raspberrypi_multicore_MPCtest_B.i_f2 < 4;
       raspberrypi_multicore_MPCtest_B.i_f2++) {
    for (raspberrypi_multicore_MPCtest_B.ar = 0;
         raspberrypi_multicore_MPCtest_B.ar < 3;
         raspberrypi_multicore_MPCtest_B.ar++) {
      raspberrypi_multicore_MPCtest_B.a[raspberrypi_multicore_MPCtest_B.ar] =
        raspberrypi_multicore_MPCtest_B.b_pArray_B[3 *
        raspberrypi_multicore_MPCtest_B.i_f2 + 2] *
        raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.ar + 6]
        + (raspberrypi_multicore_MPCtest_B.b_pArray_B[3 *
           raspberrypi_multicore_MPCtest_B.i_f2 + 1] *
           raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.ar
           + 3] + raspberrypi_multicore_MPCtest_B.b_pArray_B[3 *
           raspberrypi_multicore_MPCtest_B.i_f2] *
           raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.ar]);
    }

    raspberrypi_multicore_MPCtest_B.vBM[3 * raspberrypi_multicore_MPCtest_B.i_f2]
      = -raspberrypi_multicore_MPCtest_B.vArray[3 *
      raspberrypi_multicore_MPCtest_B.i_f2] -
      (raspberrypi_multicore_MPCtest_B.deltaP3[1] *
       raspberrypi_multicore_MPCtest_B.a[2] - raspberrypi_multicore_MPCtest_B.a
       [1] * raspberrypi_multicore_MPCtest_B.deltaP3[2]);
    raspberrypi_multicore_MPCtest_B.ar = 3 *
      raspberrypi_multicore_MPCtest_B.i_f2 + 1;
    raspberrypi_multicore_MPCtest_B.vBM[raspberrypi_multicore_MPCtest_B.ar] =
      -raspberrypi_multicore_MPCtest_B.vArray[raspberrypi_multicore_MPCtest_B.ar]
      - (raspberrypi_multicore_MPCtest_B.a[0] *
         raspberrypi_multicore_MPCtest_B.deltaP3[2] -
         raspberrypi_multicore_MPCtest_B.deltaP3[0] *
         raspberrypi_multicore_MPCtest_B.a[2]);
    raspberrypi_multicore_MPCtest_B.ar = 3 *
      raspberrypi_multicore_MPCtest_B.i_f2 + 2;
    raspberrypi_multicore_MPCtest_B.vBM[raspberrypi_multicore_MPCtest_B.ar] =
      -raspberrypi_multicore_MPCtest_B.vArray[raspberrypi_multicore_MPCtest_B.ar]
      - (raspberrypi_multicore_MPCtest_B.deltaP3[0] *
         raspberrypi_multicore_MPCtest_B.a[1] -
         raspberrypi_multicore_MPCtest_B.a[0] *
         raspberrypi_multicore_MPCtest_B.deltaP3[1]);
  }

  raspberrypi_multicore_MPCtest_B.a[0] = 0.0;
  raspberrypi_multicore_MPCtest_B.a[1] = 0.0;
  raspberrypi_multicore_MPCtest_B.a[2] = 0.0;
  raspberrypi_multicore_MPCtest_B.Angle2new[0] = 0.0;
  raspberrypi_multicore_MPCtest_B.Angle2new[1] = 0.0;
  raspberrypi_multicore_MPCtest_B.Angle2new[2] = 0.0;
  raspberrypi_multicore_MPCtest_B.t7_o = 0.0;

  /*  number of supported legs */
  for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
       raspberrypi_multicore_MPCtest_B.i_f2 < 4;
       raspberrypi_multicore_MPCtest_B.i_f2++) {
    if (raspberrypi_multicore_MPCtest_B.pxNew[raspberrypi_multicore_MPCtest_B.i_f2]
        > 0.5) {
      raspberrypi_multicore_MPCtest_B.Angle2new[0] +=
        raspberrypi_multicore_MPCtest_B.vBM[3 *
        raspberrypi_multicore_MPCtest_B.i_f2];
      raspberrypi_multicore_MPCtest_B.c_h = 3 *
        raspberrypi_multicore_MPCtest_B.i_f2 + 1;
      raspberrypi_multicore_MPCtest_B.Angle2new[1] +=
        raspberrypi_multicore_MPCtest_B.vBM[raspberrypi_multicore_MPCtest_B.c_h];
      raspberrypi_multicore_MPCtest_B.ia = 3 *
        raspberrypi_multicore_MPCtest_B.i_f2 + 2;
      raspberrypi_multicore_MPCtest_B.Angle2new[2] +=
        raspberrypi_multicore_MPCtest_B.vBM[raspberrypi_multicore_MPCtest_B.ia];
      raspberrypi_multicore_MPCtest_B.temp[raspberrypi_multicore_MPCtest_B.ia] =
        (raspberrypi_multicore_MPCtest_B.temp[3 *
         raspberrypi_multicore_MPCtest_B.i_f2] *
         raspberrypi_multicore_MPCtest_P.Constant5_Value[1] +
         raspberrypi_multicore_MPCtest_P.Constant5_Value[0]) +
        raspberrypi_multicore_MPCtest_B.temp[raspberrypi_multicore_MPCtest_B.c_h]
        * raspberrypi_multicore_MPCtest_P.Constant5_Value[2];
      for (raspberrypi_multicore_MPCtest_B.ar = 0;
           raspberrypi_multicore_MPCtest_B.ar < 9;
           raspberrypi_multicore_MPCtest_B.ar++) {
        raspberrypi_multicore_MPCtest_B.U_a[raspberrypi_multicore_MPCtest_B.ar] =
          -raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.ar];
      }

      for (raspberrypi_multicore_MPCtest_B.ar = 0;
           raspberrypi_multicore_MPCtest_B.ar < 3;
           raspberrypi_multicore_MPCtest_B.ar++) {
        raspberrypi_multicore_MPCtest_B.a[raspberrypi_multicore_MPCtest_B.ar] =
          ((raspberrypi_multicore_MPCtest_B.b_pArray_B[raspberrypi_multicore_MPCtest_B.c_h]
            * raspberrypi_multicore_MPCtest_B.U_a[raspberrypi_multicore_MPCtest_B.ar
            + 3] + raspberrypi_multicore_MPCtest_B.b_pArray_B[3 *
            raspberrypi_multicore_MPCtest_B.i_f2] *
            raspberrypi_multicore_MPCtest_B.U_a[raspberrypi_multicore_MPCtest_B.ar])
           + raspberrypi_multicore_MPCtest_B.b_pArray_B[raspberrypi_multicore_MPCtest_B.ia]
           * raspberrypi_multicore_MPCtest_B.U_a[raspberrypi_multicore_MPCtest_B.ar
           + 6]) + (raspberrypi_multicore_MPCtest_B.temp[3 *
                    raspberrypi_multicore_MPCtest_B.i_f2 +
                    raspberrypi_multicore_MPCtest_B.ar] +
                    raspberrypi_multicore_MPCtest_B.a[raspberrypi_multicore_MPCtest_B.ar]);
      }

      raspberrypi_multicore_MPCtest_B.t7_o++;
    }
  }

  if (raspberrypi_multicore_MPCtest_B.t7_o > 0.5) {
    raspberrypi_multicore_MPCtest_B.Angle2new[0] /=
      raspberrypi_multicore_MPCtest_B.t7_o;
    raspberrypi_multicore_MPCtest_B.a[0] /= raspberrypi_multicore_MPCtest_B.t7_o;
    raspberrypi_multicore_MPCtest_B.Angle2new[1] /=
      raspberrypi_multicore_MPCtest_B.t7_o;
    raspberrypi_multicore_MPCtest_B.a[1] /= raspberrypi_multicore_MPCtest_B.t7_o;
    raspberrypi_multicore_MPCtest_B.Angle2new[2] /=
      raspberrypi_multicore_MPCtest_B.t7_o;
    raspberrypi_multicore_MPCtest_B.a[2] /= raspberrypi_multicore_MPCtest_B.t7_o;
  } else {
    raspberrypi_multicore_MPCtest_B.Angle2new[0] =
      raspberrypi_multicore_MPCtes_DW.obj_b.ymOld[3];
    raspberrypi_multicore_MPCtest_B.a[0] =
      raspberrypi_multicore_MPCtes_DW.obj_b.ymOld[0];
    raspberrypi_multicore_MPCtest_B.Angle2new[1] =
      raspberrypi_multicore_MPCtes_DW.obj_b.ymOld[4];
    raspberrypi_multicore_MPCtest_B.a[1] =
      raspberrypi_multicore_MPCtes_DW.obj_b.ymOld[1];
    raspberrypi_multicore_MPCtest_B.Angle2new[2] =
      raspberrypi_multicore_MPCtes_DW.obj_b.ymOld[5];
    raspberrypi_multicore_MPCtest_B.a[2] =
      raspberrypi_multicore_MPCtes_DW.obj_b.ymOld[2];

    /* KF_R=10^-2*eye(6); */
    raspberrypi_multicore_MPCtest_B.vcol = 0;
  }

  for (raspberrypi_multicore_MPCtest_B.ar = 0;
       raspberrypi_multicore_MPCtest_B.ar < 20;
       raspberrypi_multicore_MPCtest_B.ar++) {
    raspberrypi_multicore_MPCtest_B.i_f2 = ((raspberrypi_multicore_MPCtest_B.ar
      + 2) - 1) * 3;
    raspberrypi_multicore_MPCtes_DW.obj_b.vCoMRec[3 *
      raspberrypi_multicore_MPCtest_B.ar] =
      raspberrypi_multicore_MPCtes_DW.obj_b.vCoMRec[raspberrypi_multicore_MPCtest_B.i_f2];
    raspberrypi_multicore_MPCtes_DW.obj_b.vCoMRec[3 *
      raspberrypi_multicore_MPCtest_B.ar + 1] =
      raspberrypi_multicore_MPCtes_DW.obj_b.vCoMRec[raspberrypi_multicore_MPCtest_B.i_f2
      + 1];
    raspberrypi_multicore_MPCtes_DW.obj_b.vCoMRec[3 *
      raspberrypi_multicore_MPCtest_B.ar + 2] =
      raspberrypi_multicore_MPCtes_DW.obj_b.vCoMRec[raspberrypi_multicore_MPCtest_B.i_f2
      + 2];
  }

  /* vCoMFiltered=sgolayfilt(obj.vCoMRec',3,21); */
  /* vCoMFiltered=vCoMFiltered'; */
  /*              ym=[pCoM;vCoMFiltered(:,end)]; */
  raspberrypi_multicore_MPCtes_DW.obj_b.vCoMRec[60] =
    raspberrypi_multicore_MPCtest_B.Angle2new[0];
  raspberrypi_multicore_MPCtest_B.estDis[0] = raspberrypi_multicore_MPCtest_B.a
    [0];
  raspberrypi_multicore_MPCtest_B.estDis[3] =
    raspberrypi_multicore_MPCtest_B.Angle2new[0];
  raspberrypi_multicore_MPCtes_DW.obj_b.vCoMRec[61] =
    raspberrypi_multicore_MPCtest_B.Angle2new[1];
  raspberrypi_multicore_MPCtest_B.estDis[1] = raspberrypi_multicore_MPCtest_B.a
    [1];
  raspberrypi_multicore_MPCtest_B.estDis[4] =
    raspberrypi_multicore_MPCtest_B.Angle2new[1];
  raspberrypi_multicore_MPCtes_DW.obj_b.vCoMRec[62] =
    raspberrypi_multicore_MPCtest_B.Angle2new[2];
  raspberrypi_multicore_MPCtest_B.estDis[2] = raspberrypi_multicore_MPCtest_B.a
    [2];
  raspberrypi_multicore_MPCtest_B.estDis[5] =
    raspberrypi_multicore_MPCtest_B.Angle2new[2];
  memcpy(&raspberrypi_multicore_MPCtes_DW.obj_b.pArrayOld[0],
         &raspberrypi_multicore_MPCtest_B.b_pArray_B[0], 12U * sizeof(real_T));
  for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
       raspberrypi_multicore_MPCtest_B.i_f2 < 6;
       raspberrypi_multicore_MPCtest_B.i_f2++) {
    raspberrypi_multicore_MPCtes_DW.obj_b.ymOld[raspberrypi_multicore_MPCtest_B.i_f2]
      =
      raspberrypi_multicore_MPCtest_B.estDis[raspberrypi_multicore_MPCtest_B.i_f2];
  }

  /* MATLABSystem: '<S15>/MATLAB System3' */
  if (raspberrypi_multicore_MPCtes_DW.obj_c.Ts !=
      raspberrypi_multicore_MPCtest_P.Ts_DynSim) {
    raspberrypi_multicore_MPCtes_DW.obj_c.Ts =
      raspberrypi_multicore_MPCtest_P.Ts_DynSim;
  }

  /* MATLAB Function: '<S15>/MATLAB Function1' */
  raspberrypi_multicore_MPCtest_B.U_a[0] = 0.0;
  raspberrypi_multicore_MPCtest_B.U_a[1] = 0.0;
  raspberrypi_multicore_MPCtest_B.U_a[2] = -raspberrypi_multicore_MPCtest_B.t8;
  raspberrypi_multicore_MPCtest_B.U_a[3] = 0.0;
  raspberrypi_multicore_MPCtest_B.U_a[6] = 0.0;
  raspberrypi_multicore_MPCtest_B.U_a[4] = 0.0;
  raspberrypi_multicore_MPCtest_B.U_a[7] = 0.0;
  raspberrypi_multicore_MPCtest_B.U_a[5] = 0.0;
  raspberrypi_multicore_MPCtest_B.U_a[8] = 0.0;

  /* MATLABSystem: '<S15>/MATLAB System3' incorporates:
   *  Constant: '<S15>/Constant1'
   *  MATLAB Function: '<S15>/MATLAB Function1'
   *  MATLABSystem: '<Root>/MATLAB System1'
   *  MATLABSystem: '<S15>/MATLAB System'
   */
  KalmanFilter_DIY_Offrm_stepImpl(&raspberrypi_multicore_MPCtes_DW.obj_c,
    raspberrypi_multicore_MPCtest_B.deltaP2,
    raspberrypi_multicore_MPCtest_B.estDis, raspberrypi_multicore_MPCtest_B.U_a,
    tmp_1, raspberrypi_multicore_MPCtest_P.Constant1_Value, tmp_2, (real_T)
    raspberrypi_multicore_MPCtest_B.estEN, (real_T)
    raspberrypi_multicore_MPCtest_B.vcol, raspberrypi_multicore_MPCtest_B.R,
    raspberrypi_multicore_MPCtest_B.b_varargout_2_b);

  /* MATLAB Function: '<S15>/MATLAB Function' incorporates:
   *  MATLABSystem: '<Root>/MATLAB System1'
   *  MATLABSystem: '<S15>/MATLAB System3'
   */
  raspberrypi_multicore_MPCtest_B.a[0] = raspberrypi_multicore_MPCtest_B.R[0];
  raspberrypi_multicore_MPCtest_B.a[1] = raspberrypi_multicore_MPCtest_B.R[1];
  raspberrypi_multicore_MPCtest_B.a[2] = raspberrypi_multicore_MPCtest_B.R[2];
  raspberrypi_multicore_MPCtest_B.U_a[0] = raspberrypi_multicore_MPCtest_B.t6_e;
  raspberrypi_multicore_MPCtest_B.U_a[3] = -sin
    (raspberrypi_multicore_MPCtest_B.v1[2]);
  raspberrypi_multicore_MPCtest_B.U_a[6] = 0.0;
  raspberrypi_multicore_MPCtest_B.U_a[1] = raspberrypi_multicore_MPCtest_B.t9;
  raspberrypi_multicore_MPCtest_B.U_a[4] = raspberrypi_multicore_MPCtest_B.t6_e;
  raspberrypi_multicore_MPCtest_B.U_a[7] = 0.0;
  raspberrypi_multicore_MPCtest_B.MRz[0] = raspberrypi_multicore_MPCtest_B.s_k;
  raspberrypi_multicore_MPCtest_B.MRz[3] = 0.0;
  raspberrypi_multicore_MPCtest_B.MRz[6] =
    raspberrypi_multicore_MPCtest_B.Switch1;
  raspberrypi_multicore_MPCtest_B.U_a[2] = 0.0;
  raspberrypi_multicore_MPCtest_B.MRz[1] = 0.0;
  raspberrypi_multicore_MPCtest_B.U_a[5] = 0.0;
  raspberrypi_multicore_MPCtest_B.MRz[4] = 1.0;
  raspberrypi_multicore_MPCtest_B.U_a[8] = 1.0;
  raspberrypi_multicore_MPCtest_B.MRz[7] = 0.0;
  raspberrypi_multicore_MPCtest_B.MRz[2] = -sin
    (raspberrypi_multicore_MPCtest_B.v1[1]);
  raspberrypi_multicore_MPCtest_B.MRz[5] = 0.0;
  raspberrypi_multicore_MPCtest_B.MRz[8] = raspberrypi_multicore_MPCtest_B.s_k;
  for (raspberrypi_multicore_MPCtest_B.ar = 0;
       raspberrypi_multicore_MPCtest_B.ar < 3;
       raspberrypi_multicore_MPCtest_B.ar++) {
    for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
         raspberrypi_multicore_MPCtest_B.i_f2 < 3;
         raspberrypi_multicore_MPCtest_B.i_f2++) {
      raspberrypi_multicore_MPCtest_B.vcol = raspberrypi_multicore_MPCtest_B.ar
        + 3 * raspberrypi_multicore_MPCtest_B.i_f2;
      raspberrypi_multicore_MPCtest_B.V[raspberrypi_multicore_MPCtest_B.vcol] =
        0.0;
      raspberrypi_multicore_MPCtest_B.V[raspberrypi_multicore_MPCtest_B.vcol] +=
        raspberrypi_multicore_MPCtest_B.MRz[3 *
        raspberrypi_multicore_MPCtest_B.i_f2] *
        raspberrypi_multicore_MPCtest_B.U_a[raspberrypi_multicore_MPCtest_B.ar];
      raspberrypi_multicore_MPCtest_B.V[raspberrypi_multicore_MPCtest_B.vcol] +=
        raspberrypi_multicore_MPCtest_B.MRz[3 *
        raspberrypi_multicore_MPCtest_B.i_f2 + 1] *
        raspberrypi_multicore_MPCtest_B.U_a[raspberrypi_multicore_MPCtest_B.ar +
        3];
      raspberrypi_multicore_MPCtest_B.V[raspberrypi_multicore_MPCtest_B.vcol] +=
        raspberrypi_multicore_MPCtest_B.MRz[3 *
        raspberrypi_multicore_MPCtest_B.i_f2 + 2] *
        raspberrypi_multicore_MPCtest_B.U_a[raspberrypi_multicore_MPCtest_B.ar +
        6];
    }

    raspberrypi_multicore_MPCtest_B.MRz_i[3 * raspberrypi_multicore_MPCtest_B.ar]
      = d[raspberrypi_multicore_MPCtest_B.ar];
  }

  raspberrypi_multicore_MPCtest_B.MRz_i[1] = 0.0;
  raspberrypi_multicore_MPCtest_B.MRz_i[4] = raspberrypi_multicore_MPCtest_B.t7;
  raspberrypi_multicore_MPCtest_B.MRz_i[7] = -sin
    (raspberrypi_multicore_MPCtest_B.v1[0]);
  raspberrypi_multicore_MPCtest_B.MRz_i[2] = 0.0;
  raspberrypi_multicore_MPCtest_B.MRz_i[5] = raspberrypi_multicore_MPCtest_B.t6;
  raspberrypi_multicore_MPCtest_B.MRz_i[8] = raspberrypi_multicore_MPCtest_B.t7;
  for (raspberrypi_multicore_MPCtest_B.ar = 0;
       raspberrypi_multicore_MPCtest_B.ar < 3;
       raspberrypi_multicore_MPCtest_B.ar++) {
    for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
         raspberrypi_multicore_MPCtest_B.i_f2 < 3;
         raspberrypi_multicore_MPCtest_B.i_f2++) {
      raspberrypi_multicore_MPCtest_B.vcol = raspberrypi_multicore_MPCtest_B.ar
        + 3 * raspberrypi_multicore_MPCtest_B.i_f2;
      raspberrypi_multicore_MPCtest_B.U_a[raspberrypi_multicore_MPCtest_B.vcol] =
        0.0;
      raspberrypi_multicore_MPCtest_B.U_a[raspberrypi_multicore_MPCtest_B.vcol] +=
        raspberrypi_multicore_MPCtest_B.MRz_i[3 *
        raspberrypi_multicore_MPCtest_B.i_f2] *
        raspberrypi_multicore_MPCtest_B.V[raspberrypi_multicore_MPCtest_B.ar];
      raspberrypi_multicore_MPCtest_B.U_a[raspberrypi_multicore_MPCtest_B.vcol] +=
        raspberrypi_multicore_MPCtest_B.MRz_i[3 *
        raspberrypi_multicore_MPCtest_B.i_f2 + 1] *
        raspberrypi_multicore_MPCtest_B.V[raspberrypi_multicore_MPCtest_B.ar + 3];
      raspberrypi_multicore_MPCtest_B.U_a[raspberrypi_multicore_MPCtest_B.vcol] +=
        raspberrypi_multicore_MPCtest_B.MRz_i[3 *
        raspberrypi_multicore_MPCtest_B.i_f2 + 2] *
        raspberrypi_multicore_MPCtest_B.V[raspberrypi_multicore_MPCtest_B.ar + 6];
    }

    for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
         raspberrypi_multicore_MPCtest_B.i_f2 < 4;
         raspberrypi_multicore_MPCtest_B.i_f2++) {
      raspberrypi_multicore_MPCtest_B.vcol = raspberrypi_multicore_MPCtest_B.ar
        + 3 * raspberrypi_multicore_MPCtest_B.i_f2;
      raspberrypi_multicore_MPCtest_B.vBM[raspberrypi_multicore_MPCtest_B.vcol] =
        0.0;
      raspberrypi_multicore_MPCtest_B.vBM[raspberrypi_multicore_MPCtest_B.vcol] +=
        raspberrypi_multicore_MPCtest_B.pArray_float_tmp[3 *
        raspberrypi_multicore_MPCtest_B.i_f2] *
        raspberrypi_multicore_MPCtest_B.U_a[raspberrypi_multicore_MPCtest_B.ar];
      raspberrypi_multicore_MPCtest_B.vBM[raspberrypi_multicore_MPCtest_B.vcol] +=
        raspberrypi_multicore_MPCtest_B.pArray_float_tmp[3 *
        raspberrypi_multicore_MPCtest_B.i_f2 + 1] *
        raspberrypi_multicore_MPCtest_B.U_a[raspberrypi_multicore_MPCtest_B.ar +
        3];
      raspberrypi_multicore_MPCtest_B.vBM[raspberrypi_multicore_MPCtest_B.vcol] +=
        raspberrypi_multicore_MPCtest_B.pArray_float_tmp[3 *
        raspberrypi_multicore_MPCtest_B.i_f2 + 2] *
        raspberrypi_multicore_MPCtest_B.U_a[raspberrypi_multicore_MPCtest_B.ar +
        6];
    }
  }

  for (raspberrypi_multicore_MPCtest_B.ar = 0;
       raspberrypi_multicore_MPCtest_B.ar < 4;
       raspberrypi_multicore_MPCtest_B.ar++) {
    raspberrypi_multicore_MPCtest_B.pArray_float_tmp[3 *
      raspberrypi_multicore_MPCtest_B.ar] = raspberrypi_multicore_MPCtest_B.a[0];
    raspberrypi_multicore_MPCtest_B.pArray_float_tmp[3 *
      raspberrypi_multicore_MPCtest_B.ar + 1] =
      raspberrypi_multicore_MPCtest_B.a[1];
    raspberrypi_multicore_MPCtest_B.pArray_float_tmp[3 *
      raspberrypi_multicore_MPCtest_B.ar + 2] =
      raspberrypi_multicore_MPCtest_B.a[2];
  }

  for (raspberrypi_multicore_MPCtest_B.ar = 0;
       raspberrypi_multicore_MPCtest_B.ar < 12;
       raspberrypi_multicore_MPCtest_B.ar++) {
    raspberrypi_multicore_MPCtest_B.temp[raspberrypi_multicore_MPCtest_B.ar] =
      raspberrypi_multicore_MPCtest_B.vBM[raspberrypi_multicore_MPCtest_B.ar] +
      raspberrypi_multicore_MPCtest_B.pArray_float_tmp[raspberrypi_multicore_MPCtest_B.ar];
  }

  /* MATLAB Function: '<Root>/MATLAB Function4' */
  raspberrypi_multicore_MPCtest_B.pyNew[0] = 1.0;

  /* SignalConversion generated from: '<Root>/To File12' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function4'
   *  MATLABSystem: '<S15>/MATLAB System6'
   */
  raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_h[0] =
    raspberrypi_multicore_MPCtest_B.pxNew[0];

  /* MATLAB Function: '<Root>/MATLAB Function4' */
  raspberrypi_multicore_MPCtest_B.pyNew[1] = 0.0;

  /* SignalConversion generated from: '<Root>/To File12' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function4'
   *  MATLABSystem: '<S15>/MATLAB System6'
   */
  raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_h[1] =
    raspberrypi_multicore_MPCtest_B.pxNew[1];

  /* MATLAB Function: '<Root>/MATLAB Function4' */
  raspberrypi_multicore_MPCtest_B.pyNew[2] = 0.0;

  /* SignalConversion generated from: '<Root>/To File12' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function4'
   *  MATLABSystem: '<S15>/MATLAB System6'
   */
  raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_h[2] =
    raspberrypi_multicore_MPCtest_B.pxNew[2];

  /* MATLAB Function: '<Root>/MATLAB Function4' */
  raspberrypi_multicore_MPCtest_B.pyNew[3] = 1.0;

  /* SignalConversion generated from: '<Root>/To File12' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function4'
   *  MATLABSystem: '<S15>/MATLAB System6'
   */
  raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_h[3] =
    raspberrypi_multicore_MPCtest_B.pxNew[3];

  /* MATLAB Function: '<Root>/MATLAB Function4' incorporates:
   *  MATLABSystem: '<Root>/MATLAB System11'
   *  MATLABSystem: '<S15>/MATLAB System6'
   */
  if (raspberrypi_multicore_MPCtest_B.b_varargout_4_a > 3.1415926535897931) {
    raspberrypi_multicore_MPCtest_B.s--;
    raspberrypi_multicore_MPCtest_B.pyNew[0] = 0.0;
    raspberrypi_multicore_MPCtest_B.pyNew[1] = 1.0;
    raspberrypi_multicore_MPCtest_B.pyNew[2] = 1.0;
    raspberrypi_multicore_MPCtest_B.pyNew[3] = 0.0;
  }

  if (fabs(raspberrypi_multicore_MPCtest_B.MATLABSystem11_o2 - 1.0) < 0.1) {
    raspberrypi_multicore_MPCtest_B.pyNew[0] = 1.0;
    raspberrypi_multicore_MPCtest_B.pyNew[1] = 1.0;
    raspberrypi_multicore_MPCtest_B.pyNew[2] = 1.0;
    raspberrypi_multicore_MPCtest_B.pyNew[3] = 1.0;
  }

  if ((raspberrypi_multicore_MPCtest_B.pyNew[0] < 0.5) &&
      (raspberrypi_multicore_MPCtest_B.s > 0.5) &&
      (raspberrypi_multicore_MPCtest_B.pxNew[0] > 0.5)) {
    /* SignalConversion generated from: '<Root>/To File12' */
    raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_h[0] = 1.0;
  }

  if ((raspberrypi_multicore_MPCtest_B.pyNew[1] < 0.5) &&
      (raspberrypi_multicore_MPCtest_B.s > 0.5) &&
      (raspberrypi_multicore_MPCtest_B.pxNew[1] > 0.5)) {
    /* SignalConversion generated from: '<Root>/To File12' */
    raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_h[1] = 1.0;
  }

  if ((raspberrypi_multicore_MPCtest_B.pyNew[2] < 0.5) &&
      (raspberrypi_multicore_MPCtest_B.s > 0.5) &&
      (raspberrypi_multicore_MPCtest_B.pxNew[2] > 0.5)) {
    /* SignalConversion generated from: '<Root>/To File12' */
    raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_h[2] = 1.0;
  }

  if ((raspberrypi_multicore_MPCtest_B.pyNew[3] < 0.5) &&
      (raspberrypi_multicore_MPCtest_B.s > 0.5) &&
      (raspberrypi_multicore_MPCtest_B.pxNew[3] > 0.5)) {
    /* SignalConversion generated from: '<Root>/To File12' */
    raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_h[3] = 1.0;
  }

  /* MATLABSystem: '<Root>/MATLAB System18' incorporates:
   *  MATLABSystem: '<S15>/MATLAB System6'
   */
  if (raspberrypi_multicore_MPCtes_DW.obj_kb.lateral_width !=
      raspberrypi_multicore_MPCtest_P.MATLABSystem18_lateral_width) {
    raspberrypi_multicore_MPCtes_DW.obj_kb.lateral_width =
      raspberrypi_multicore_MPCtest_P.MATLABSystem18_lateral_width;
  }

  if (raspberrypi_multicore_MPCtes_DW.obj_kb.sagetial_width !=
      raspberrypi_multicore_MPCtest_P.MATLABSystem18_sagetial_width) {
    raspberrypi_multicore_MPCtes_DW.obj_kb.sagetial_width =
      raspberrypi_multicore_MPCtest_P.MATLABSystem18_sagetial_width;
  }

  if (raspberrypi_multicore_MPCtes_DW.obj_kb.roll_Off !=
      raspberrypi_multicore_MPCtest_P.MATLABSystem18_roll_Off) {
    raspberrypi_multicore_MPCtes_DW.obj_kb.roll_Off =
      raspberrypi_multicore_MPCtest_P.MATLABSystem18_roll_Off;
  }

  /*  round to 0.1 mm */
  for (raspberrypi_multicore_MPCtest_B.ar = 0;
       raspberrypi_multicore_MPCtest_B.ar < 12;
       raspberrypi_multicore_MPCtest_B.ar++) {
    raspberrypi_multicore_MPCtes_DW.UnitDelay5_DSTATE[raspberrypi_multicore_MPCtest_B.ar]
      =
      raspberrypi_multicore_MPCtes_DW.obj_kb.pW_LS[raspberrypi_multicore_MPCtest_B.ar];
    raspberrypi_multicore_MPCtest_B.pArray_float_tmp[raspberrypi_multicore_MPCtest_B.ar]
      = rt_roundd_snf
      (raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[raspberrypi_multicore_MPCtest_B.ar]
       * 10000.0) / 10000.0;
  }

  for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
       raspberrypi_multicore_MPCtest_B.i_f2 < 4;
       raspberrypi_multicore_MPCtest_B.i_f2++) {
    if ((raspberrypi_multicore_MPCtest_B.pyNew[raspberrypi_multicore_MPCtest_B.i_f2]
         > 0.5) &&
        (raspberrypi_multicore_MPCtes_DW.obj_kb.LegState_Old[raspberrypi_multicore_MPCtest_B.i_f2]
         < 0.5)) {
      raspberrypi_multicore_MPCtes_DW.UnitDelay5_DSTATE[3 *
        raspberrypi_multicore_MPCtest_B.i_f2] =
        raspberrypi_multicore_MPCtest_B.pArray_float_tmp[3 *
        raspberrypi_multicore_MPCtest_B.i_f2];
      raspberrypi_multicore_MPCtest_B.ar = 3 *
        raspberrypi_multicore_MPCtest_B.i_f2 + 1;
      raspberrypi_multicore_MPCtes_DW.UnitDelay5_DSTATE[raspberrypi_multicore_MPCtest_B.ar]
        =
        raspberrypi_multicore_MPCtest_B.pArray_float_tmp[raspberrypi_multicore_MPCtest_B.ar];
      raspberrypi_multicore_MPCtest_B.ar = 3 *
        raspberrypi_multicore_MPCtest_B.i_f2 + 2;
      raspberrypi_multicore_MPCtes_DW.UnitDelay5_DSTATE[raspberrypi_multicore_MPCtest_B.ar]
        =
        raspberrypi_multicore_MPCtest_B.pArray_float_tmp[raspberrypi_multicore_MPCtest_B.ar];
    }
  }

  if (raspberrypi_multicore_MPCtest_B.MATLABSystem11_o2 > 0.5) {
    memcpy(&raspberrypi_multicore_MPCtest_B.pArray_float_tmp[0],
           &raspberrypi_multicore_MPCtes_DW.obj_kb.pWnorm[0], 12U * sizeof
           (real_T));
    memcpy(&raspberrypi_multicore_MPCtes_DW.UnitDelay5_DSTATE[0],
           &raspberrypi_multicore_MPCtes_DW.obj_kb.pWnorm[0], 12U * sizeof
           (real_T));
  }

  memcpy(&raspberrypi_multicore_MPCtes_DW.obj_kb.pW_LS[0],
         &raspberrypi_multicore_MPCtes_DW.UnitDelay5_DSTATE[0], 12U * sizeof
         (real_T));
  raspberrypi_multicore_MPCtes_DW.obj_kb.LegState_Old[0] =
    raspberrypi_multicore_MPCtest_B.pyNew[0];
  raspberrypi_multicore_MPCtes_DW.obj_kb.LegState_Old[1] =
    raspberrypi_multicore_MPCtest_B.pyNew[1];
  raspberrypi_multicore_MPCtes_DW.obj_kb.LegState_Old[2] =
    raspberrypi_multicore_MPCtest_B.pyNew[2];
  raspberrypi_multicore_MPCtes_DW.obj_kb.LegState_Old[3] =
    raspberrypi_multicore_MPCtest_B.pyNew[3];

  /* MATLABSystem: '<Root>/MATLAB System20' */
  if (raspberrypi_multicore_MPCtes_DW.obj_n.Ts !=
      raspberrypi_multicore_MPCtest_P.Ts_DynSim) {
    raspberrypi_multicore_MPCtes_DW.obj_n.Ts =
      raspberrypi_multicore_MPCtest_P.Ts_DynSim;
  }

  if (raspberrypi_multicore_MPCtes_DW.obj_n.m !=
      raspberrypi_multicore_MPCtest_P.MATLABSystem20_m) {
    raspberrypi_multicore_MPCtes_DW.obj_n.m =
      raspberrypi_multicore_MPCtest_P.MATLABSystem20_m;
  }

  rEQ0 = false;
  p = true;
  raspberrypi_multicore_MPCtest_B.ar = 0;
  exitg1 = false;
  while ((!exitg1) && (raspberrypi_multicore_MPCtest_B.ar < 9)) {
    if (!(raspberrypi_multicore_MPCtes_DW.obj_n.Inorm[raspberrypi_multicore_MPCtest_B.ar]
          ==
          raspberrypi_multicore_MPCtest_P.MATLABSystem20_Inorm[raspberrypi_multicore_MPCtest_B.ar]))
    {
      p = false;
      exitg1 = true;
    } else {
      raspberrypi_multicore_MPCtest_B.ar++;
    }
  }

  if (p) {
    rEQ0 = true;
  }

  if (!rEQ0) {
    memcpy(&raspberrypi_multicore_MPCtes_DW.obj_n.Inorm[0],
           &raspberrypi_multicore_MPCtest_P.MATLABSystem20_Inorm[0], 9U * sizeof
           (real_T));
  }

  /* SignalConversion generated from: '<Root>/MATLAB System20' incorporates:
   *  Constant: '<Root>/Constant'
   *  MATLAB Function: '<S15>/MATLAB Function'
   *  MATLABSystem: '<Root>/MATLAB System1'
   *  MATLABSystem: '<S15>/MATLAB System3'
   */
  raspberrypi_multicore_MPCtest_B.b_varargout_4[0] =
    raspberrypi_multicore_MPCtest_B.R[0];
  raspberrypi_multicore_MPCtest_B.b_varargout_4[3] =
    raspberrypi_multicore_MPCtest_B.v1[0];
  raspberrypi_multicore_MPCtest_B.b_varargout_4[6] =
    raspberrypi_multicore_MPCtest_B.R[3];
  raspberrypi_multicore_MPCtest_B.b_varargout_4[9] =
    raspberrypi_multicore_MPCtest_B.deltaP3[0];
  raspberrypi_multicore_MPCtest_B.b_varargout_4[1] =
    raspberrypi_multicore_MPCtest_B.R[1];
  raspberrypi_multicore_MPCtest_B.b_varargout_4[4] =
    raspberrypi_multicore_MPCtest_B.v1[1];
  raspberrypi_multicore_MPCtest_B.b_varargout_4[7] =
    raspberrypi_multicore_MPCtest_B.R[4];
  raspberrypi_multicore_MPCtest_B.b_varargout_4[10] =
    raspberrypi_multicore_MPCtest_B.deltaP3[1];
  raspberrypi_multicore_MPCtest_B.b_varargout_4[2] =
    raspberrypi_multicore_MPCtest_B.R[2];
  raspberrypi_multicore_MPCtest_B.b_varargout_4[5] =
    raspberrypi_multicore_MPCtest_B.v1[2];
  raspberrypi_multicore_MPCtest_B.b_varargout_4[8] =
    raspberrypi_multicore_MPCtest_B.R[5];
  raspberrypi_multicore_MPCtest_B.b_varargout_4[11] =
    raspberrypi_multicore_MPCtest_B.deltaP3[2];
  raspberrypi_multicore_MPCtest_B.b_varargout_4[12] =
    raspberrypi_multicore_MPCtest_P.Constant_Value_n;

  /* MATLABSystem: '<Root>/MATLAB System20' incorporates:
   *  Constant: '<Root>/Q'
   *  Constant: '<Root>/R'
   *  MATLABSystem: '<Root>/MATLAB System10'
   *  MATLABSystem: '<Root>/MATLAB System18'
   */
  raspberrypi_mul_SystemCore_step(&raspberrypi_multicore_MPCtes_DW.obj_n,
    raspberrypi_multicore_MPCtest_B.MATLABSystem10_o1,
    raspberrypi_multicore_MPCtest_B.b_varargout_4,
    raspberrypi_multicore_MPCtest_B.pArray_float_tmp,
    raspberrypi_multicore_MPCtest_P.Q_Value,
    raspberrypi_multicore_MPCtest_P.R_Value, (real_T)
    raspberrypi_multicore_MPCtest_B.estEN,
    raspberrypi_multicore_MPCtest_B.b_varargout_1_c,
    raspberrypi_multicore_MPCtest_B.b_varargout_2);

  /* SignalConversion generated from: '<Root>/To File11' incorporates:
   *  DataTypeConversion: '<S16>/Cast To Double'
   *  MATLABSystem: '<S52>/Digital Read'
   *  MATLABSystem: '<S53>/Digital Read'
   *  MATLABSystem: '<S54>/Digital Read'
   *  MATLABSystem: '<S55>/Digital Read'
   */
  memcpy(&raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFile[0],
         &raspberrypi_multicore_MPCtest_B.pArray_B[0], 12U * sizeof(real_T));
  raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFile[12] =
    raspberrypi_multicore_MPCtest_B.estEN;
  raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFile[13] =
    rtb_DigitalRead_pz_0;
  raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFile[14] =
    rtb_DigitalRead_p_0;
  raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFile[15] =
    rtb_DigitalRead_k_0;
  raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFile[16] =
    rtb_DigitalRead_0;

  /* ToFile: '<Root>/To File11' */
  if (tid == 0 ) {
    {
      if (!(++raspberrypi_multicore_MPCtes_DW.ToFile11_IWORK.Decimation % 1) &&
          (raspberrypi_multicore_MPCtes_DW.ToFile11_IWORK.Count * (17 + 1)) + 1 <
          100000000 ) {
        FILE *fp = (FILE *)
          raspberrypi_multicore_MPCtes_DW.ToFile11_PWORK.FilePtr;
        if (fp != (NULL)) {
          real_T u[17 + 1];
          raspberrypi_multicore_MPCtes_DW.ToFile11_IWORK.Decimation = 0;
          u[0] = raspberrypi_multicore_MPCtes_M->Timing.taskTime0;
          u[1] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFile[0];
          u[2] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFile[1];
          u[3] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFile[2];
          u[4] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFile[3];
          u[5] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFile[4];
          u[6] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFile[5];
          u[7] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFile[6];
          u[8] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFile[7];
          u[9] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFile[8];
          u[10] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFile[9];
          u[11] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFile[10];
          u[12] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFile[11];
          u[13] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFile[12];
          u[14] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFile[13];
          u[15] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFile[14];
          u[16] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFile[15];
          u[17] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFile[16];
          if (fwrite(u, sizeof(real_T), 17 + 1, fp) != 17 + 1) {
            rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                              "Error writing to MAT-file multicore_dataRecEstB.mat");
            return;
          }

          if (((++raspberrypi_multicore_MPCtes_DW.ToFile11_IWORK.Count) * (17 +
                1))+1 >= 100000000) {
            (void)fprintf(stdout,
                          "*** The ToFile block will stop logging data before\n"
                          "    the simulation has ended, because it has reached\n"
                          "    the maximum number of elements (100000000)\n"
                          "    allowed in MAT-file multicore_dataRecEstB.mat.\n");
          }
        }
      }
    }
  }

  /* SignalConversion generated from: '<Root>/To File6' incorporates:
   *  MATLABSystem: '<S15>/MATLAB System6'
   */
  raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_f[0] =
    raspberrypi_multicore_MPCtest_B.pxNew[0];
  raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_f[1] =
    raspberrypi_multicore_MPCtest_B.pxNew[1];
  raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_f[2] =
    raspberrypi_multicore_MPCtest_B.pxNew[2];
  raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_f[3] =
    raspberrypi_multicore_MPCtest_B.pxNew[3];
  memcpy(&raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_f[4],
         &raspberrypi_multicore_MPCtest_B.PendAllLocal_st_tmp[0], 12U * sizeof
         (real_T));

  /* ToFile: '<Root>/To File6' */
  if (tid == 0 ) {
    {
      if (!(++raspberrypi_multicore_MPCtes_DW.ToFile6_IWORK.Decimation % 1) &&
          (raspberrypi_multicore_MPCtes_DW.ToFile6_IWORK.Count * (16 + 1)) + 1 <
          100000000 ) {
        FILE *fp = (FILE *)
          raspberrypi_multicore_MPCtes_DW.ToFile6_PWORK.FilePtr;
        if (fp != (NULL)) {
          real_T u[16 + 1];
          raspberrypi_multicore_MPCtes_DW.ToFile6_IWORK.Decimation = 0;
          u[0] = raspberrypi_multicore_MPCtes_M->Timing.taskTime0;
          u[1] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_f[0];
          u[2] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_f[1];
          u[3] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_f[2];
          u[4] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_f[3];
          u[5] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_f[4];
          u[6] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_f[5];
          u[7] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_f[6];
          u[8] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_f[7];
          u[9] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_f[8];
          u[10] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_f[9];
          u[11] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_f[10];
          u[12] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_f[11];
          u[13] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_f[12];
          u[14] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_f[13];
          u[15] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_f[14];
          u[16] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_f[15];
          if (fwrite(u, sizeof(real_T), 16 + 1, fp) != 16 + 1) {
            rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                              "Error writing to MAT-file multicore_dataRec7.mat");
            return;
          }

          if (((++raspberrypi_multicore_MPCtes_DW.ToFile6_IWORK.Count) * (16 + 1))
              +1 >= 100000000) {
            (void)fprintf(stdout,
                          "*** The ToFile block will stop logging data before\n"
                          "    the simulation has ended, because it has reached\n"
                          "    the maximum number of elements (100000000)\n"
                          "    allowed in MAT-file multicore_dataRec7.mat.\n");
          }
        }
      }
    }
  }

  /* MATLAB Function: '<Root>/MATLAB Function7' incorporates:
   *  MATLABSystem: '<Root>/MATLAB System20'
   */
  for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
       raspberrypi_multicore_MPCtest_B.i_f2 < 6;
       raspberrypi_multicore_MPCtest_B.i_f2++) {
    raspberrypi_multicore_MPCtest_B.estDis[raspberrypi_multicore_MPCtest_B.i_f2]
      =
      raspberrypi_multicore_MPCtest_B.b_varargout_1_c[raspberrypi_multicore_MPCtest_B.i_f2
      + 13];
  }

  /* End of MATLAB Function: '<Root>/MATLAB Function7' */

  /* Switch: '<Root>/Switch1' incorporates:
   *  MATLAB Function: '<S15>/MATLAB Function'
   *  MATLABSystem: '<Root>/MATLAB System1'
   *  MATLABSystem: '<S15>/MATLAB System3'
   *  UnitDelay: '<Root>/Unit Delay7'
   */
  if (raspberrypi_multicore_MPCtest_B.mpcSTOP >
      raspberrypi_multicore_MPCtest_P.Switch1_Threshold_k) {
    raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[0] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay7_DSTATE[0];
    raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[3] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay7_DSTATE[3];
    raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[6] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay7_DSTATE[6];
    raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[9] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay7_DSTATE[9];
    raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[1] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay7_DSTATE[1];
    raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[4] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay7_DSTATE[4];
    raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[7] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay7_DSTATE[7];
    raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[10] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay7_DSTATE[10];
    raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[2] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay7_DSTATE[2];
    raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[5] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay7_DSTATE[5];
    raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[8] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay7_DSTATE[8];
    raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[11] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay7_DSTATE[11];
  } else {
    raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[0] =
      raspberrypi_multicore_MPCtest_B.R[0];
    raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[3] =
      raspberrypi_multicore_MPCtest_B.v1[0];
    raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[6] =
      raspberrypi_multicore_MPCtest_B.R[3];
    raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[9] =
      raspberrypi_multicore_MPCtest_B.deltaP3[0];
    raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[1] =
      raspberrypi_multicore_MPCtest_B.R[1];
    raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[4] =
      raspberrypi_multicore_MPCtest_B.v1[1];
    raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[7] =
      raspberrypi_multicore_MPCtest_B.R[4];
    raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[10] =
      raspberrypi_multicore_MPCtest_B.deltaP3[1];
    raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[2] =
      raspberrypi_multicore_MPCtest_B.R[2];
    raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[5] =
      raspberrypi_multicore_MPCtest_B.v1[2];
    raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[8] =
      raspberrypi_multicore_MPCtest_B.R[5];
    raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[11] =
      raspberrypi_multicore_MPCtest_B.deltaP3[2];
  }

  /* End of Switch: '<Root>/Switch1' */

  /* RateTransition: '<Root>/Rate Transition1' incorporates:
   *  Constant: '<Root>/Constant25'
   *  MATLAB Function: '<Root>/MATLAB Function'
   *  MATLAB Function: '<Root>/MATLAB Function2'
   *  MATLAB Function: '<Root>/MATLAB Function5'
   *  MATLABSystem: '<Root>/MATLAB System11'
   *  MATLABSystem: '<Root>/MATLAB System18'
   *  MATLABSystem: '<Root>/MATLAB System3'
   */
  rtw_pthread_mutex_lock
    (raspberrypi_multicore_MPCtes_DW.RateTransition1_d0_SEMAPHORE);
  wrBufIdx = (int8_T)(raspberrypi_multicore_MPCtes_DW.RateTransition1_LstBufWR +
                      1);
  if (wrBufIdx == 3) {
    wrBufIdx = 0;
  }

  if (wrBufIdx == raspberrypi_multicore_MPCtes_DW.RateTransition1_RDBuf) {
    wrBufIdx++;
    if (wrBufIdx == 3) {
      wrBufIdx = 0;
    }
  }

  rtw_pthread_mutex_unlock
    (raspberrypi_multicore_MPCtes_DW.RateTransition1_d0_SEMAPHORE);
  switch (wrBufIdx) {
   case 0:
    /* MATLAB Function: '<Root>/MATLAB Function2' */
    raspberrypi_multicore_MPCtest_B.R[0] =
      raspberrypi_multicore_MPCtest_B.rtb_headG_tmp_tmp;
    raspberrypi_multicore_MPCtest_B.R[3] =
      -raspberrypi_multicore_MPCtest_B.rtb_headG_tmp_tmp_f;
    raspberrypi_multicore_MPCtest_B.R[6] = 0.0;
    raspberrypi_multicore_MPCtest_B.R[1] =
      raspberrypi_multicore_MPCtest_B.rtb_headG_tmp_tmp_f;
    raspberrypi_multicore_MPCtest_B.R[4] =
      raspberrypi_multicore_MPCtest_B.rtb_headG_tmp_tmp;
    raspberrypi_multicore_MPCtest_B.R[7] = 0.0;
    raspberrypi_multicore_MPCtest_B.U_a[0] =
      raspberrypi_multicore_MPCtest_B.rtb_headG_tmp_i;
    raspberrypi_multicore_MPCtest_B.U_a[3] = 0.0;
    raspberrypi_multicore_MPCtest_B.U_a[6] =
      raspberrypi_multicore_MPCtest_B.rtb_headG_tmp;
    raspberrypi_multicore_MPCtest_B.R[2] = 0.0;
    raspberrypi_multicore_MPCtest_B.U_a[1] = 0.0;
    raspberrypi_multicore_MPCtest_B.R[5] = 0.0;
    raspberrypi_multicore_MPCtest_B.U_a[4] = 1.0;
    raspberrypi_multicore_MPCtest_B.R[8] = 1.0;
    raspberrypi_multicore_MPCtest_B.U_a[7] = 0.0;
    raspberrypi_multicore_MPCtest_B.U_a[2] =
      -raspberrypi_multicore_MPCtest_B.rtb_headG_tmp;
    raspberrypi_multicore_MPCtest_B.U_a[5] = 0.0;
    raspberrypi_multicore_MPCtest_B.U_a[8] =
      raspberrypi_multicore_MPCtest_B.rtb_headG_tmp_i;
    for (raspberrypi_multicore_MPCtest_B.ar = 0;
         raspberrypi_multicore_MPCtest_B.ar < 3;
         raspberrypi_multicore_MPCtest_B.ar++) {
      for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
           raspberrypi_multicore_MPCtest_B.i_f2 < 3;
           raspberrypi_multicore_MPCtest_B.i_f2++) {
        raspberrypi_multicore_MPCtest_B.vcol =
          raspberrypi_multicore_MPCtest_B.ar + 3 *
          raspberrypi_multicore_MPCtest_B.i_f2;
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.vcol]
          = 0.0;
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.vcol]
          += raspberrypi_multicore_MPCtest_B.U_a[3 *
          raspberrypi_multicore_MPCtest_B.i_f2] *
          raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.ar];
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.vcol]
          += raspberrypi_multicore_MPCtest_B.U_a[3 *
          raspberrypi_multicore_MPCtest_B.i_f2 + 1] *
          raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.ar +
          3];
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.vcol]
          += raspberrypi_multicore_MPCtest_B.U_a[3 *
          raspberrypi_multicore_MPCtest_B.i_f2 + 2] *
          raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.ar +
          6];
      }

      raspberrypi_multicore_MPCtest_B.V[3 * raspberrypi_multicore_MPCtest_B.ar] =
        d[raspberrypi_multicore_MPCtest_B.ar];
    }

    raspberrypi_multicore_MPCtest_B.V[1] = 0.0;
    raspberrypi_multicore_MPCtest_B.V[4] =
      raspberrypi_multicore_MPCtest_B.rtb_headG_tmp_m;
    raspberrypi_multicore_MPCtest_B.V[7] =
      -raspberrypi_multicore_MPCtest_B.rtb_headG_tmp_o;
    raspberrypi_multicore_MPCtest_B.V[2] = 0.0;
    raspberrypi_multicore_MPCtest_B.V[5] =
      raspberrypi_multicore_MPCtest_B.rtb_headG_tmp_o;
    raspberrypi_multicore_MPCtest_B.V[8] =
      raspberrypi_multicore_MPCtest_B.rtb_headG_tmp_m;
    for (raspberrypi_multicore_MPCtest_B.ar = 0;
         raspberrypi_multicore_MPCtest_B.ar < 3;
         raspberrypi_multicore_MPCtest_B.ar++) {
      raspberrypi_multicore_MPCtest_B.pL_sw_c[raspberrypi_multicore_MPCtest_B.ar]
        = 0.0;
      for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
           raspberrypi_multicore_MPCtest_B.i_f2 < 3;
           raspberrypi_multicore_MPCtest_B.i_f2++) {
        raspberrypi_multicore_MPCtest_B.vcol =
          raspberrypi_multicore_MPCtest_B.ar + 3 *
          raspberrypi_multicore_MPCtest_B.i_f2;
        raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.vcol] =
          0.0;
        raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.vcol] +=
          raspberrypi_multicore_MPCtest_B.V[3 *
          raspberrypi_multicore_MPCtest_B.i_f2] *
          raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar];
        raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.vcol] +=
          raspberrypi_multicore_MPCtest_B.V[3 *
          raspberrypi_multicore_MPCtest_B.i_f2 + 1] *
          raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar
          + 3];
        raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.vcol] +=
          raspberrypi_multicore_MPCtest_B.V[3 *
          raspberrypi_multicore_MPCtest_B.i_f2 + 2] *
          raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar
          + 6];
        raspberrypi_multicore_MPCtest_B.pL_sw_c[raspberrypi_multicore_MPCtest_B.ar]
          +=
          raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.vcol]
          * (real_T)d[raspberrypi_multicore_MPCtest_B.i_f2];
      }
    }

    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf0[0] =
      raspberrypi_multicore_MPCtest_P.Constant25_Value;
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf0[1] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[0];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf0[2] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[1];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf0[3] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[2];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf0[4] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[6];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf0[5] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[7];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf0[6] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[8];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf0[7] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[3];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf0[8] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[4];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf0[9] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[5];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf0[10] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[9];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf0[11] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[10];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf0[12] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[11];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf0[13] =
      raspberrypi_multicore_MPCtest_B.pyNew[0];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf0[14] =
      raspberrypi_multicore_MPCtest_B.pyNew[1];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf0[15] =
      raspberrypi_multicore_MPCtest_B.pyNew[2];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf0[16] =
      raspberrypi_multicore_MPCtest_B.pyNew[3];
    for (raspberrypi_multicore_MPCtest_B.ar = 0;
         raspberrypi_multicore_MPCtest_B.ar < 12;
         raspberrypi_multicore_MPCtest_B.ar++) {
      raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf0[raspberrypi_multicore_MPCtest_B.ar
        + 17] =
        raspberrypi_multicore_MPCtest_B.pArray_float_tmp[raspberrypi_multicore_MPCtest_B.ar];
    }

    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf0[29] =
      raspberrypi_multicore_MPCtest_B.absx;
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf0[30] =
      raspberrypi_multicore_MPCtest_B.b_varargout_5_p;
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf0[31] =
      raspberrypi_multicore_MPCtest_B.b_varargout_10_idx_0;
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf0[32] =
      raspberrypi_multicore_MPCtest_B.q;
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf0[33] =
      raspberrypi_multicore_MPCtest_B.v2_idx_1;
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf0[34] =
      raspberrypi_multicore_MPCtest_B.v2_idx_2;
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf0[35] =
      raspberrypi_multicore_MPCtest_B.b_varargout_5_p *
      raspberrypi_multicore_MPCtest_B.v2_idx_2 -
      raspberrypi_multicore_MPCtest_B.v2_idx_1 *
      raspberrypi_multicore_MPCtest_B.b_varargout_10_idx_0;
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf0[36] =
      raspberrypi_multicore_MPCtest_B.q *
      raspberrypi_multicore_MPCtest_B.b_varargout_10_idx_0 -
      raspberrypi_multicore_MPCtest_B.absx *
      raspberrypi_multicore_MPCtest_B.v2_idx_2;
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf0[37] =
      raspberrypi_multicore_MPCtest_B.absx *
      raspberrypi_multicore_MPCtest_B.v2_idx_1 -
      raspberrypi_multicore_MPCtest_B.q *
      raspberrypi_multicore_MPCtest_B.b_varargout_5_p;
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf0[38] =
      raspberrypi_multicore_MPCtest_B.surP[0];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf0[39] =
      raspberrypi_multicore_MPCtest_B.surP[1];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf0[40] =
      raspberrypi_multicore_MPCtest_B.surP[2];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf0[41] =
      raspberrypi_multicore_MPCtest_B.pL_sw_c[0];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf0[42] =
      raspberrypi_multicore_MPCtest_B.pL_sw_c[1];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf0[43] =
      raspberrypi_multicore_MPCtest_B.pL_sw_c[2];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf0[44] =
      raspberrypi_multicore_MPCtest_B.mpcSTOP;
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf0[45] =
      raspberrypi_multicore_MPCtest_B.b_varargout_4_a;
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf0[46] =
      raspberrypi_multicore_MPCtest_B.b_varargout_9_idx_0;
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf0[47] =
      -raspberrypi_multicore_MPCtest_B.b_varargout_9_idx_1;
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf0[48] =
      -raspberrypi_multicore_MPCtest_B.b_varargout_10_idx_1;
    for (raspberrypi_multicore_MPCtest_B.ar = 0;
         raspberrypi_multicore_MPCtest_B.ar < 6;
         raspberrypi_multicore_MPCtest_B.ar++) {
      raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf0[raspberrypi_multicore_MPCtest_B.ar
        + 49] =
        raspberrypi_multicore_MPCtest_B.estDis[raspberrypi_multicore_MPCtest_B.ar];
    }
    break;

   case 1:
    /* MATLAB Function: '<Root>/MATLAB Function2' */
    raspberrypi_multicore_MPCtest_B.R[0] =
      raspberrypi_multicore_MPCtest_B.rtb_headG_tmp_tmp;
    raspberrypi_multicore_MPCtest_B.R[3] =
      -raspberrypi_multicore_MPCtest_B.rtb_headG_tmp_tmp_f;
    raspberrypi_multicore_MPCtest_B.R[6] = 0.0;
    raspberrypi_multicore_MPCtest_B.R[1] =
      raspberrypi_multicore_MPCtest_B.rtb_headG_tmp_tmp_f;
    raspberrypi_multicore_MPCtest_B.R[4] =
      raspberrypi_multicore_MPCtest_B.rtb_headG_tmp_tmp;
    raspberrypi_multicore_MPCtest_B.R[7] = 0.0;
    raspberrypi_multicore_MPCtest_B.U_a[0] =
      raspberrypi_multicore_MPCtest_B.rtb_headG_tmp_i;
    raspberrypi_multicore_MPCtest_B.U_a[3] = 0.0;
    raspberrypi_multicore_MPCtest_B.U_a[6] =
      raspberrypi_multicore_MPCtest_B.rtb_headG_tmp;
    raspberrypi_multicore_MPCtest_B.R[2] = 0.0;
    raspberrypi_multicore_MPCtest_B.U_a[1] = 0.0;
    raspberrypi_multicore_MPCtest_B.R[5] = 0.0;
    raspberrypi_multicore_MPCtest_B.U_a[4] = 1.0;
    raspberrypi_multicore_MPCtest_B.R[8] = 1.0;
    raspberrypi_multicore_MPCtest_B.U_a[7] = 0.0;
    raspberrypi_multicore_MPCtest_B.U_a[2] =
      -raspberrypi_multicore_MPCtest_B.rtb_headG_tmp;
    raspberrypi_multicore_MPCtest_B.U_a[5] = 0.0;
    raspberrypi_multicore_MPCtest_B.U_a[8] =
      raspberrypi_multicore_MPCtest_B.rtb_headG_tmp_i;
    for (raspberrypi_multicore_MPCtest_B.ar = 0;
         raspberrypi_multicore_MPCtest_B.ar < 3;
         raspberrypi_multicore_MPCtest_B.ar++) {
      for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
           raspberrypi_multicore_MPCtest_B.i_f2 < 3;
           raspberrypi_multicore_MPCtest_B.i_f2++) {
        raspberrypi_multicore_MPCtest_B.vcol =
          raspberrypi_multicore_MPCtest_B.ar + 3 *
          raspberrypi_multicore_MPCtest_B.i_f2;
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.vcol]
          = 0.0;
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.vcol]
          += raspberrypi_multicore_MPCtest_B.U_a[3 *
          raspberrypi_multicore_MPCtest_B.i_f2] *
          raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.ar];
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.vcol]
          += raspberrypi_multicore_MPCtest_B.U_a[3 *
          raspberrypi_multicore_MPCtest_B.i_f2 + 1] *
          raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.ar +
          3];
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.vcol]
          += raspberrypi_multicore_MPCtest_B.U_a[3 *
          raspberrypi_multicore_MPCtest_B.i_f2 + 2] *
          raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.ar +
          6];
      }

      raspberrypi_multicore_MPCtest_B.V[3 * raspberrypi_multicore_MPCtest_B.ar] =
        d[raspberrypi_multicore_MPCtest_B.ar];
    }

    raspberrypi_multicore_MPCtest_B.V[1] = 0.0;
    raspberrypi_multicore_MPCtest_B.V[4] =
      raspberrypi_multicore_MPCtest_B.rtb_headG_tmp_m;
    raspberrypi_multicore_MPCtest_B.V[7] =
      -raspberrypi_multicore_MPCtest_B.rtb_headG_tmp_o;
    raspberrypi_multicore_MPCtest_B.V[2] = 0.0;
    raspberrypi_multicore_MPCtest_B.V[5] =
      raspberrypi_multicore_MPCtest_B.rtb_headG_tmp_o;
    raspberrypi_multicore_MPCtest_B.V[8] =
      raspberrypi_multicore_MPCtest_B.rtb_headG_tmp_m;
    for (raspberrypi_multicore_MPCtest_B.ar = 0;
         raspberrypi_multicore_MPCtest_B.ar < 3;
         raspberrypi_multicore_MPCtest_B.ar++) {
      raspberrypi_multicore_MPCtest_B.pL_sw_c[raspberrypi_multicore_MPCtest_B.ar]
        = 0.0;
      for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
           raspberrypi_multicore_MPCtest_B.i_f2 < 3;
           raspberrypi_multicore_MPCtest_B.i_f2++) {
        raspberrypi_multicore_MPCtest_B.vcol =
          raspberrypi_multicore_MPCtest_B.ar + 3 *
          raspberrypi_multicore_MPCtest_B.i_f2;
        raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.vcol] =
          0.0;
        raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.vcol] +=
          raspberrypi_multicore_MPCtest_B.V[3 *
          raspberrypi_multicore_MPCtest_B.i_f2] *
          raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar];
        raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.vcol] +=
          raspberrypi_multicore_MPCtest_B.V[3 *
          raspberrypi_multicore_MPCtest_B.i_f2 + 1] *
          raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar
          + 3];
        raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.vcol] +=
          raspberrypi_multicore_MPCtest_B.V[3 *
          raspberrypi_multicore_MPCtest_B.i_f2 + 2] *
          raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar
          + 6];
        raspberrypi_multicore_MPCtest_B.pL_sw_c[raspberrypi_multicore_MPCtest_B.ar]
          +=
          raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.vcol]
          * (real_T)d[raspberrypi_multicore_MPCtest_B.i_f2];
      }
    }

    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf1[0] =
      raspberrypi_multicore_MPCtest_P.Constant25_Value;
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf1[1] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[0];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf1[2] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[1];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf1[3] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[2];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf1[4] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[6];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf1[5] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[7];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf1[6] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[8];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf1[7] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[3];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf1[8] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[4];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf1[9] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[5];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf1[10] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[9];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf1[11] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[10];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf1[12] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[11];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf1[13] =
      raspberrypi_multicore_MPCtest_B.pyNew[0];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf1[14] =
      raspberrypi_multicore_MPCtest_B.pyNew[1];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf1[15] =
      raspberrypi_multicore_MPCtest_B.pyNew[2];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf1[16] =
      raspberrypi_multicore_MPCtest_B.pyNew[3];
    for (raspberrypi_multicore_MPCtest_B.ar = 0;
         raspberrypi_multicore_MPCtest_B.ar < 12;
         raspberrypi_multicore_MPCtest_B.ar++) {
      raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf1[raspberrypi_multicore_MPCtest_B.ar
        + 17] =
        raspberrypi_multicore_MPCtest_B.pArray_float_tmp[raspberrypi_multicore_MPCtest_B.ar];
    }

    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf1[29] =
      raspberrypi_multicore_MPCtest_B.absx;
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf1[30] =
      raspberrypi_multicore_MPCtest_B.b_varargout_5_p;
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf1[31] =
      raspberrypi_multicore_MPCtest_B.b_varargout_10_idx_0;
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf1[32] =
      raspberrypi_multicore_MPCtest_B.q;
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf1[33] =
      raspberrypi_multicore_MPCtest_B.v2_idx_1;
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf1[34] =
      raspberrypi_multicore_MPCtest_B.v2_idx_2;
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf1[35] =
      raspberrypi_multicore_MPCtest_B.b_varargout_5_p *
      raspberrypi_multicore_MPCtest_B.v2_idx_2 -
      raspberrypi_multicore_MPCtest_B.v2_idx_1 *
      raspberrypi_multicore_MPCtest_B.b_varargout_10_idx_0;
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf1[36] =
      raspberrypi_multicore_MPCtest_B.q *
      raspberrypi_multicore_MPCtest_B.b_varargout_10_idx_0 -
      raspberrypi_multicore_MPCtest_B.absx *
      raspberrypi_multicore_MPCtest_B.v2_idx_2;
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf1[37] =
      raspberrypi_multicore_MPCtest_B.absx *
      raspberrypi_multicore_MPCtest_B.v2_idx_1 -
      raspberrypi_multicore_MPCtest_B.q *
      raspberrypi_multicore_MPCtest_B.b_varargout_5_p;
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf1[38] =
      raspberrypi_multicore_MPCtest_B.surP[0];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf1[39] =
      raspberrypi_multicore_MPCtest_B.surP[1];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf1[40] =
      raspberrypi_multicore_MPCtest_B.surP[2];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf1[41] =
      raspberrypi_multicore_MPCtest_B.pL_sw_c[0];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf1[42] =
      raspberrypi_multicore_MPCtest_B.pL_sw_c[1];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf1[43] =
      raspberrypi_multicore_MPCtest_B.pL_sw_c[2];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf1[44] =
      raspberrypi_multicore_MPCtest_B.mpcSTOP;
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf1[45] =
      raspberrypi_multicore_MPCtest_B.b_varargout_4_a;
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf1[46] =
      raspberrypi_multicore_MPCtest_B.b_varargout_9_idx_0;
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf1[47] =
      -raspberrypi_multicore_MPCtest_B.b_varargout_9_idx_1;
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf1[48] =
      -raspberrypi_multicore_MPCtest_B.b_varargout_10_idx_1;
    for (raspberrypi_multicore_MPCtest_B.ar = 0;
         raspberrypi_multicore_MPCtest_B.ar < 6;
         raspberrypi_multicore_MPCtest_B.ar++) {
      raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf1[raspberrypi_multicore_MPCtest_B.ar
        + 49] =
        raspberrypi_multicore_MPCtest_B.estDis[raspberrypi_multicore_MPCtest_B.ar];
    }
    break;

   case 2:
    /* MATLAB Function: '<Root>/MATLAB Function2' */
    raspberrypi_multicore_MPCtest_B.R[0] =
      raspberrypi_multicore_MPCtest_B.rtb_headG_tmp_tmp;
    raspberrypi_multicore_MPCtest_B.R[3] =
      -raspberrypi_multicore_MPCtest_B.rtb_headG_tmp_tmp_f;
    raspberrypi_multicore_MPCtest_B.R[6] = 0.0;
    raspberrypi_multicore_MPCtest_B.R[1] =
      raspberrypi_multicore_MPCtest_B.rtb_headG_tmp_tmp_f;
    raspberrypi_multicore_MPCtest_B.R[4] =
      raspberrypi_multicore_MPCtest_B.rtb_headG_tmp_tmp;
    raspberrypi_multicore_MPCtest_B.R[7] = 0.0;
    raspberrypi_multicore_MPCtest_B.U_a[0] =
      raspberrypi_multicore_MPCtest_B.rtb_headG_tmp_i;
    raspberrypi_multicore_MPCtest_B.U_a[3] = 0.0;
    raspberrypi_multicore_MPCtest_B.U_a[6] =
      raspberrypi_multicore_MPCtest_B.rtb_headG_tmp;
    raspberrypi_multicore_MPCtest_B.R[2] = 0.0;
    raspberrypi_multicore_MPCtest_B.U_a[1] = 0.0;
    raspberrypi_multicore_MPCtest_B.R[5] = 0.0;
    raspberrypi_multicore_MPCtest_B.U_a[4] = 1.0;
    raspberrypi_multicore_MPCtest_B.R[8] = 1.0;
    raspberrypi_multicore_MPCtest_B.U_a[7] = 0.0;
    raspberrypi_multicore_MPCtest_B.U_a[2] =
      -raspberrypi_multicore_MPCtest_B.rtb_headG_tmp;
    raspberrypi_multicore_MPCtest_B.U_a[5] = 0.0;
    raspberrypi_multicore_MPCtest_B.U_a[8] =
      raspberrypi_multicore_MPCtest_B.rtb_headG_tmp_i;
    for (raspberrypi_multicore_MPCtest_B.ar = 0;
         raspberrypi_multicore_MPCtest_B.ar < 3;
         raspberrypi_multicore_MPCtest_B.ar++) {
      for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
           raspberrypi_multicore_MPCtest_B.i_f2 < 3;
           raspberrypi_multicore_MPCtest_B.i_f2++) {
        raspberrypi_multicore_MPCtest_B.vcol =
          raspberrypi_multicore_MPCtest_B.ar + 3 *
          raspberrypi_multicore_MPCtest_B.i_f2;
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.vcol]
          = 0.0;
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.vcol]
          += raspberrypi_multicore_MPCtest_B.U_a[3 *
          raspberrypi_multicore_MPCtest_B.i_f2] *
          raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.ar];
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.vcol]
          += raspberrypi_multicore_MPCtest_B.U_a[3 *
          raspberrypi_multicore_MPCtest_B.i_f2 + 1] *
          raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.ar +
          3];
        raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.vcol]
          += raspberrypi_multicore_MPCtest_B.U_a[3 *
          raspberrypi_multicore_MPCtest_B.i_f2 + 2] *
          raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.ar +
          6];
      }

      raspberrypi_multicore_MPCtest_B.V[3 * raspberrypi_multicore_MPCtest_B.ar] =
        d[raspberrypi_multicore_MPCtest_B.ar];
    }

    raspberrypi_multicore_MPCtest_B.V[1] = 0.0;
    raspberrypi_multicore_MPCtest_B.V[4] =
      raspberrypi_multicore_MPCtest_B.rtb_headG_tmp_m;
    raspberrypi_multicore_MPCtest_B.V[7] =
      -raspberrypi_multicore_MPCtest_B.rtb_headG_tmp_o;
    raspberrypi_multicore_MPCtest_B.V[2] = 0.0;
    raspberrypi_multicore_MPCtest_B.V[5] =
      raspberrypi_multicore_MPCtest_B.rtb_headG_tmp_o;
    raspberrypi_multicore_MPCtest_B.V[8] =
      raspberrypi_multicore_MPCtest_B.rtb_headG_tmp_m;
    for (raspberrypi_multicore_MPCtest_B.ar = 0;
         raspberrypi_multicore_MPCtest_B.ar < 3;
         raspberrypi_multicore_MPCtest_B.ar++) {
      raspberrypi_multicore_MPCtest_B.pL_sw_c[raspberrypi_multicore_MPCtest_B.ar]
        = 0.0;
      for (raspberrypi_multicore_MPCtest_B.i_f2 = 0;
           raspberrypi_multicore_MPCtest_B.i_f2 < 3;
           raspberrypi_multicore_MPCtest_B.i_f2++) {
        raspberrypi_multicore_MPCtest_B.vcol =
          raspberrypi_multicore_MPCtest_B.ar + 3 *
          raspberrypi_multicore_MPCtest_B.i_f2;
        raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.vcol] =
          0.0;
        raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.vcol] +=
          raspberrypi_multicore_MPCtest_B.V[3 *
          raspberrypi_multicore_MPCtest_B.i_f2] *
          raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar];
        raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.vcol] +=
          raspberrypi_multicore_MPCtest_B.V[3 *
          raspberrypi_multicore_MPCtest_B.i_f2 + 1] *
          raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar
          + 3];
        raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.vcol] +=
          raspberrypi_multicore_MPCtest_B.V[3 *
          raspberrypi_multicore_MPCtest_B.i_f2 + 2] *
          raspberrypi_multicore_MPCtest_B.MRz[raspberrypi_multicore_MPCtest_B.ar
          + 6];
        raspberrypi_multicore_MPCtest_B.pL_sw_c[raspberrypi_multicore_MPCtest_B.ar]
          +=
          raspberrypi_multicore_MPCtest_B.R[raspberrypi_multicore_MPCtest_B.vcol]
          * (real_T)d[raspberrypi_multicore_MPCtest_B.i_f2];
      }
    }

    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf2[0] =
      raspberrypi_multicore_MPCtest_P.Constant25_Value;
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf2[1] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[0];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf2[2] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[1];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf2[3] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[2];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf2[4] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[6];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf2[5] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[7];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf2[6] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[8];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf2[7] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[3];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf2[8] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[4];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf2[9] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[5];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf2[10] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[9];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf2[11] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[10];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf2[12] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[11];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf2[13] =
      raspberrypi_multicore_MPCtest_B.pyNew[0];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf2[14] =
      raspberrypi_multicore_MPCtest_B.pyNew[1];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf2[15] =
      raspberrypi_multicore_MPCtest_B.pyNew[2];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf2[16] =
      raspberrypi_multicore_MPCtest_B.pyNew[3];
    for (raspberrypi_multicore_MPCtest_B.ar = 0;
         raspberrypi_multicore_MPCtest_B.ar < 12;
         raspberrypi_multicore_MPCtest_B.ar++) {
      raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf2[raspberrypi_multicore_MPCtest_B.ar
        + 17] =
        raspberrypi_multicore_MPCtest_B.pArray_float_tmp[raspberrypi_multicore_MPCtest_B.ar];
    }

    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf2[29] =
      raspberrypi_multicore_MPCtest_B.absx;
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf2[30] =
      raspberrypi_multicore_MPCtest_B.b_varargout_5_p;
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf2[31] =
      raspberrypi_multicore_MPCtest_B.b_varargout_10_idx_0;
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf2[32] =
      raspberrypi_multicore_MPCtest_B.q;
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf2[33] =
      raspberrypi_multicore_MPCtest_B.v2_idx_1;
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf2[34] =
      raspberrypi_multicore_MPCtest_B.v2_idx_2;
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf2[35] =
      raspberrypi_multicore_MPCtest_B.b_varargout_5_p *
      raspberrypi_multicore_MPCtest_B.v2_idx_2 -
      raspberrypi_multicore_MPCtest_B.v2_idx_1 *
      raspberrypi_multicore_MPCtest_B.b_varargout_10_idx_0;
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf2[36] =
      raspberrypi_multicore_MPCtest_B.q *
      raspberrypi_multicore_MPCtest_B.b_varargout_10_idx_0 -
      raspberrypi_multicore_MPCtest_B.absx *
      raspberrypi_multicore_MPCtest_B.v2_idx_2;
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf2[37] =
      raspberrypi_multicore_MPCtest_B.absx *
      raspberrypi_multicore_MPCtest_B.v2_idx_1 -
      raspberrypi_multicore_MPCtest_B.q *
      raspberrypi_multicore_MPCtest_B.b_varargout_5_p;
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf2[38] =
      raspberrypi_multicore_MPCtest_B.surP[0];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf2[39] =
      raspberrypi_multicore_MPCtest_B.surP[1];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf2[40] =
      raspberrypi_multicore_MPCtest_B.surP[2];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf2[41] =
      raspberrypi_multicore_MPCtest_B.pL_sw_c[0];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf2[42] =
      raspberrypi_multicore_MPCtest_B.pL_sw_c[1];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf2[43] =
      raspberrypi_multicore_MPCtest_B.pL_sw_c[2];
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf2[44] =
      raspberrypi_multicore_MPCtest_B.mpcSTOP;
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf2[45] =
      raspberrypi_multicore_MPCtest_B.b_varargout_4_a;
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf2[46] =
      raspberrypi_multicore_MPCtest_B.b_varargout_9_idx_0;
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf2[47] =
      -raspberrypi_multicore_MPCtest_B.b_varargout_9_idx_1;
    raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf2[48] =
      -raspberrypi_multicore_MPCtest_B.b_varargout_10_idx_1;
    for (raspberrypi_multicore_MPCtest_B.ar = 0;
         raspberrypi_multicore_MPCtest_B.ar < 6;
         raspberrypi_multicore_MPCtest_B.ar++) {
      raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf2[raspberrypi_multicore_MPCtest_B.ar
        + 49] =
        raspberrypi_multicore_MPCtest_B.estDis[raspberrypi_multicore_MPCtest_B.ar];
    }
    break;
  }

  raspberrypi_multicore_MPCtes_DW.RateTransition1_LstBufWR = wrBufIdx;

  /* End of RateTransition: '<Root>/Rate Transition1' */

  /* ToFile: '<Root>/To File3' */
  if (tid == 0 ) {
    {
      if (!(++raspberrypi_multicore_MPCtes_DW.ToFile3_IWORK.Decimation % 1) &&
          (raspberrypi_multicore_MPCtes_DW.ToFile3_IWORK.Count * (12 + 1)) + 1 <
          100000000 ) {
        FILE *fp = (FILE *)
          raspberrypi_multicore_MPCtes_DW.ToFile3_PWORK.FilePtr;
        if (fp != (NULL)) {
          real_T u[12 + 1];
          raspberrypi_multicore_MPCtes_DW.ToFile3_IWORK.Decimation = 0;
          u[0] = raspberrypi_multicore_MPCtes_M->Timing.taskTime0;
          u[1] = raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[0];
          u[2] = raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[1];
          u[3] = raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[2];
          u[4] = raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[3];
          u[5] = raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[4];
          u[6] = raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[5];
          u[7] = raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[6];
          u[8] = raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[7];
          u[9] = raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[8];
          u[10] = raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[9];
          u[11] = raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[10];
          u[12] = raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[11];
          if (fwrite(u, sizeof(real_T), 12 + 1, fp) != 12 + 1) {
            rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                              "Error writing to MAT-file multicore_dataRec4.mat");
            return;
          }

          if (((++raspberrypi_multicore_MPCtes_DW.ToFile3_IWORK.Count) * (12 + 1))
              +1 >= 100000000) {
            (void)fprintf(stdout,
                          "*** The ToFile block will stop logging data before\n"
                          "    the simulation has ended, because it has reached\n"
                          "    the maximum number of elements (100000000)\n"
                          "    allowed in MAT-file multicore_dataRec4.mat.\n");
          }
        }
      }
    }
  }

  /* ToFile: '<Root>/To File13' */
  if (tid == 0 ) {
    {
      if (!(++raspberrypi_multicore_MPCtes_DW.ToFile13_IWORK.Decimation % 1) &&
          (raspberrypi_multicore_MPCtes_DW.ToFile13_IWORK.Count * (12 + 1)) + 1 <
          100000000 ) {
        FILE *fp = (FILE *)
          raspberrypi_multicore_MPCtes_DW.ToFile13_PWORK.FilePtr;
        if (fp != (NULL)) {
          real_T u[12 + 1];
          raspberrypi_multicore_MPCtes_DW.ToFile13_IWORK.Decimation = 0;
          u[0] = raspberrypi_multicore_MPCtes_M->Timing.taskTime0;
          u[1] = raspberrypi_multicore_MPCtest_B.pL_sw[0];
          u[2] = raspberrypi_multicore_MPCtest_B.pL_sw[1];
          u[3] = raspberrypi_multicore_MPCtest_B.pL_sw[2];
          u[4] = raspberrypi_multicore_MPCtest_B.pL_sw[3];
          u[5] = raspberrypi_multicore_MPCtest_B.pL_sw[4];
          u[6] = raspberrypi_multicore_MPCtest_B.pL_sw[5];
          u[7] = raspberrypi_multicore_MPCtest_B.pL_sw[6];
          u[8] = raspberrypi_multicore_MPCtest_B.pL_sw[7];
          u[9] = raspberrypi_multicore_MPCtest_B.pL_sw[8];
          u[10] = raspberrypi_multicore_MPCtest_B.pL_sw[9];
          u[11] = raspberrypi_multicore_MPCtest_B.pL_sw[10];
          u[12] = raspberrypi_multicore_MPCtest_B.pL_sw[11];
          if (fwrite(u, sizeof(real_T), 12 + 1, fp) != 12 + 1) {
            rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                              "Error writing to MAT-file multicore_dataRec12.mat");
            return;
          }

          if (((++raspberrypi_multicore_MPCtes_DW.ToFile13_IWORK.Count) * (12 +
                1))+1 >= 100000000) {
            (void)fprintf(stdout,
                          "*** The ToFile block will stop logging data before\n"
                          "    the simulation has ended, because it has reached\n"
                          "    the maximum number of elements (100000000)\n"
                          "    allowed in MAT-file multicore_dataRec12.mat.\n");
          }
        }
      }
    }
  }

  /* ToFile: '<Root>/To File4' */
  if (tid == 0 ) {
    {
      if (!(++raspberrypi_multicore_MPCtes_DW.ToFile4_IWORK.Decimation % 1) &&
          (raspberrypi_multicore_MPCtes_DW.ToFile4_IWORK.Count * (12 + 1)) + 1 <
          100000000 ) {
        FILE *fp = (FILE *)
          raspberrypi_multicore_MPCtes_DW.ToFile4_PWORK.FilePtr;
        if (fp != (NULL)) {
          real_T u[12 + 1];
          raspberrypi_multicore_MPCtes_DW.ToFile4_IWORK.Decimation = 0;
          u[0] = raspberrypi_multicore_MPCtes_M->Timing.taskTime0;
          u[1] = raspberrypi_multicore_MPCtest_B.desAllL[0];
          u[2] = raspberrypi_multicore_MPCtest_B.desAllL[1];
          u[3] = raspberrypi_multicore_MPCtest_B.desAllL[2];
          u[4] = raspberrypi_multicore_MPCtest_B.desAllL[3];
          u[5] = raspberrypi_multicore_MPCtest_B.desAllL[4];
          u[6] = raspberrypi_multicore_MPCtest_B.desAllL[5];
          u[7] = raspberrypi_multicore_MPCtest_B.desAllL[6];
          u[8] = raspberrypi_multicore_MPCtest_B.desAllL[7];
          u[9] = raspberrypi_multicore_MPCtest_B.desAllL[8];
          u[10] = raspberrypi_multicore_MPCtest_B.desAllL[9];
          u[11] = raspberrypi_multicore_MPCtest_B.desAllL[10];
          u[12] = raspberrypi_multicore_MPCtest_B.desAllL[11];
          if (fwrite(u, sizeof(real_T), 12 + 1, fp) != 12 + 1) {
            rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                              "Error writing to MAT-file multicore_dataRec5.mat");
            return;
          }

          if (((++raspberrypi_multicore_MPCtes_DW.ToFile4_IWORK.Count) * (12 + 1))
              +1 >= 100000000) {
            (void)fprintf(stdout,
                          "*** The ToFile block will stop logging data before\n"
                          "    the simulation has ended, because it has reached\n"
                          "    the maximum number of elements (100000000)\n"
                          "    allowed in MAT-file multicore_dataRec5.mat.\n");
          }
        }
      }
    }
  }

  /* ToFile: '<Root>/To File16' */
  if (tid == 0 ) {
    {
      if (!(++raspberrypi_multicore_MPCtes_DW.ToFile16_IWORK.Decimation % 1) &&
          (raspberrypi_multicore_MPCtes_DW.ToFile16_IWORK.Count * (12 + 1)) + 1 <
          100000000 ) {
        FILE *fp = (FILE *)
          raspberrypi_multicore_MPCtes_DW.ToFile16_PWORK.FilePtr;
        if (fp != (NULL)) {
          real_T u[12 + 1];
          raspberrypi_multicore_MPCtes_DW.ToFile16_IWORK.Decimation = 0;
          u[0] = raspberrypi_multicore_MPCtes_M->Timing.taskTime0;
          u[1] = raspberrypi_multicore_MPCtest_B.W[0];
          u[2] = raspberrypi_multicore_MPCtest_B.W[1];
          u[3] = raspberrypi_multicore_MPCtest_B.W[2];
          u[4] = raspberrypi_multicore_MPCtest_B.W[3];
          u[5] = raspberrypi_multicore_MPCtest_B.W[4];
          u[6] = raspberrypi_multicore_MPCtest_B.W[5];
          u[7] = raspberrypi_multicore_MPCtest_B.W[6];
          u[8] = raspberrypi_multicore_MPCtest_B.W[7];
          u[9] = raspberrypi_multicore_MPCtest_B.W[8];
          u[10] = raspberrypi_multicore_MPCtest_B.W[9];
          u[11] = raspberrypi_multicore_MPCtest_B.W[10];
          u[12] = raspberrypi_multicore_MPCtest_B.W[11];
          if (fwrite(u, sizeof(real_T), 12 + 1, fp) != 12 + 1) {
            rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                              "Error writing to MAT-file multicore_dataRec15.mat");
            return;
          }

          if (((++raspberrypi_multicore_MPCtes_DW.ToFile16_IWORK.Count) * (12 +
                1))+1 >= 100000000) {
            (void)fprintf(stdout,
                          "*** The ToFile block will stop logging data before\n"
                          "    the simulation has ended, because it has reached\n"
                          "    the maximum number of elements (100000000)\n"
                          "    allowed in MAT-file multicore_dataRec15.mat.\n");
          }
        }
      }
    }
  }

  /* ToFile: '<Root>/To File9' */
  if (tid == 0 ) {
    {
      if (!(++raspberrypi_multicore_MPCtes_DW.ToFile9_IWORK.Decimation % 1) &&
          (raspberrypi_multicore_MPCtes_DW.ToFile9_IWORK.Count * (12 + 1)) + 1 <
          100000000 ) {
        FILE *fp = (FILE *)
          raspberrypi_multicore_MPCtes_DW.ToFile9_PWORK.FilePtr;
        if (fp != (NULL)) {
          real_T u[12 + 1];
          raspberrypi_multicore_MPCtes_DW.ToFile9_IWORK.Decimation = 0;
          u[0] = raspberrypi_multicore_MPCtes_M->Timing.taskTime0;
          u[1] = raspberrypi_multicore_MPCtest_B.pArray_L_Adm[0];
          u[2] = raspberrypi_multicore_MPCtest_B.pArray_L_Adm[1];
          u[3] = raspberrypi_multicore_MPCtest_B.pArray_L_Adm[2];
          u[4] = raspberrypi_multicore_MPCtest_B.pArray_L_Adm[3];
          u[5] = raspberrypi_multicore_MPCtest_B.pArray_L_Adm[4];
          u[6] = raspberrypi_multicore_MPCtest_B.pArray_L_Adm[5];
          u[7] = raspberrypi_multicore_MPCtest_B.pArray_L_Adm[6];
          u[8] = raspberrypi_multicore_MPCtest_B.pArray_L_Adm[7];
          u[9] = raspberrypi_multicore_MPCtest_B.pArray_L_Adm[8];
          u[10] = raspberrypi_multicore_MPCtest_B.pArray_L_Adm[9];
          u[11] = raspberrypi_multicore_MPCtest_B.pArray_L_Adm[10];
          u[12] = raspberrypi_multicore_MPCtest_B.pArray_L_Adm[11];
          if (fwrite(u, sizeof(real_T), 12 + 1, fp) != 12 + 1) {
            rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                              "Error writing to MAT-file multicore_dataRec10.mat");
            return;
          }

          if (((++raspberrypi_multicore_MPCtes_DW.ToFile9_IWORK.Count) * (12 + 1))
              +1 >= 100000000) {
            (void)fprintf(stdout,
                          "*** The ToFile block will stop logging data before\n"
                          "    the simulation has ended, because it has reached\n"
                          "    the maximum number of elements (100000000)\n"
                          "    allowed in MAT-file multicore_dataRec10.mat.\n");
          }
        }
      }
    }
  }

  /* ToFile: '<Root>/To File15' */
  if (tid == 0 ) {
    {
      if (!(++raspberrypi_multicore_MPCtes_DW.ToFile15_IWORK.Decimation % 1) &&
          (raspberrypi_multicore_MPCtes_DW.ToFile15_IWORK.Count * (12 + 1)) + 1 <
          100000000 ) {
        FILE *fp = (FILE *)
          raspberrypi_multicore_MPCtes_DW.ToFile15_PWORK.FilePtr;
        if (fp != (NULL)) {
          real_T u[12 + 1];
          raspberrypi_multicore_MPCtes_DW.ToFile15_IWORK.Decimation = 0;
          u[0] = raspberrypi_multicore_MPCtes_M->Timing.taskTime0;
          u[1] = rtb_MATLABSystem10_o5_0[0];
          u[2] = rtb_MATLABSystem10_o5_0[1];
          u[3] = rtb_MATLABSystem10_o5_0[2];
          u[4] = rtb_MATLABSystem10_o5_0[3];
          u[5] = rtb_MATLABSystem10_o5_0[4];
          u[6] = rtb_MATLABSystem10_o5_0[5];
          u[7] = rtb_MATLABSystem10_o5_0[6];
          u[8] = rtb_MATLABSystem10_o5_0[7];
          u[9] = rtb_MATLABSystem10_o5_0[8];
          u[10] = rtb_MATLABSystem10_o5_0[9];
          u[11] = rtb_MATLABSystem10_o5_0[10];
          u[12] = rtb_MATLABSystem10_o5_0[11];
          if (fwrite(u, sizeof(real_T), 12 + 1, fp) != 12 + 1) {
            rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                              "Error writing to MAT-file multicore_dataRec14.mat");
            return;
          }

          if (((++raspberrypi_multicore_MPCtes_DW.ToFile15_IWORK.Count) * (12 +
                1))+1 >= 100000000) {
            (void)fprintf(stdout,
                          "*** The ToFile block will stop logging data before\n"
                          "    the simulation has ended, because it has reached\n"
                          "    the maximum number of elements (100000000)\n"
                          "    allowed in MAT-file multicore_dataRec14.mat.\n");
          }
        }
      }
    }
  }

  /* ToFile: '<Root>/To File18' */
  if (tid == 0 ) {
    {
      if (!(++raspberrypi_multicore_MPCtes_DW.ToFile18_IWORK.Decimation % 1) &&
          (raspberrypi_multicore_MPCtes_DW.ToFile18_IWORK.Count * (12 + 1)) + 1 <
          100000000 ) {
        FILE *fp = (FILE *)
          raspberrypi_multicore_MPCtes_DW.ToFile18_PWORK.FilePtr;
        if (fp != (NULL)) {
          real_T u[12 + 1];
          raspberrypi_multicore_MPCtes_DW.ToFile18_IWORK.Decimation = 0;
          u[0] = raspberrypi_multicore_MPCtes_M->Timing.taskTime0;
          u[1] = raspberrypi_multicore_MPCtest_B.MATLABSystem10_o1[0];
          u[2] = raspberrypi_multicore_MPCtest_B.MATLABSystem10_o1[1];
          u[3] = raspberrypi_multicore_MPCtest_B.MATLABSystem10_o1[2];
          u[4] = raspberrypi_multicore_MPCtest_B.MATLABSystem10_o1[3];
          u[5] = raspberrypi_multicore_MPCtest_B.MATLABSystem10_o1[4];
          u[6] = raspberrypi_multicore_MPCtest_B.MATLABSystem10_o1[5];
          u[7] = raspberrypi_multicore_MPCtest_B.MATLABSystem10_o1[6];
          u[8] = raspberrypi_multicore_MPCtest_B.MATLABSystem10_o1[7];
          u[9] = raspberrypi_multicore_MPCtest_B.MATLABSystem10_o1[8];
          u[10] = raspberrypi_multicore_MPCtest_B.MATLABSystem10_o1[9];
          u[11] = raspberrypi_multicore_MPCtest_B.MATLABSystem10_o1[10];
          u[12] = raspberrypi_multicore_MPCtest_B.MATLABSystem10_o1[11];
          if (fwrite(u, sizeof(real_T), 12 + 1, fp) != 12 + 1) {
            rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                              "Error writing to MAT-file multicore_dataRecEstC.mat");
            return;
          }

          if (((++raspberrypi_multicore_MPCtes_DW.ToFile18_IWORK.Count) * (12 +
                1))+1 >= 100000000) {
            (void)fprintf(stdout,
                          "*** The ToFile block will stop logging data before\n"
                          "    the simulation has ended, because it has reached\n"
                          "    the maximum number of elements (100000000)\n"
                          "    allowed in MAT-file multicore_dataRecEstC.mat.\n");
          }
        }
      }
    }
  }

  /* ToFile: '<Root>/To File2' */
  if (tid == 0 ) {
    {
      if (!(++raspberrypi_multicore_MPCtes_DW.ToFile2_IWORK.Decimation % 1) &&
          (raspberrypi_multicore_MPCtes_DW.ToFile2_IWORK.Count * (12 + 1)) + 1 <
          100000000 ) {
        FILE *fp = (FILE *)
          raspberrypi_multicore_MPCtes_DW.ToFile2_PWORK.FilePtr;
        if (fp != (NULL)) {
          real_T u[12 + 1];
          raspberrypi_multicore_MPCtes_DW.ToFile2_IWORK.Decimation = 0;
          u[0] = raspberrypi_multicore_MPCtes_M->Timing.taskTime0;
          u[1] = raspberrypi_multicore_MPCtest_B.MATLABSystem10_o1[0];
          u[2] = raspberrypi_multicore_MPCtest_B.MATLABSystem10_o1[1];
          u[3] = raspberrypi_multicore_MPCtest_B.MATLABSystem10_o1[2];
          u[4] = raspberrypi_multicore_MPCtest_B.MATLABSystem10_o1[3];
          u[5] = raspberrypi_multicore_MPCtest_B.MATLABSystem10_o1[4];
          u[6] = raspberrypi_multicore_MPCtest_B.MATLABSystem10_o1[5];
          u[7] = raspberrypi_multicore_MPCtest_B.MATLABSystem10_o1[6];
          u[8] = raspberrypi_multicore_MPCtest_B.MATLABSystem10_o1[7];
          u[9] = raspberrypi_multicore_MPCtest_B.MATLABSystem10_o1[8];
          u[10] = raspberrypi_multicore_MPCtest_B.MATLABSystem10_o1[9];
          u[11] = raspberrypi_multicore_MPCtest_B.MATLABSystem10_o1[10];
          u[12] = raspberrypi_multicore_MPCtest_B.MATLABSystem10_o1[11];
          if (fwrite(u, sizeof(real_T), 12 + 1, fp) != 12 + 1) {
            rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                              "Error writing to MAT-file multicore_dataRec3.mat");
            return;
          }

          if (((++raspberrypi_multicore_MPCtes_DW.ToFile2_IWORK.Count) * (12 + 1))
              +1 >= 100000000) {
            (void)fprintf(stdout,
                          "*** The ToFile block will stop logging data before\n"
                          "    the simulation has ended, because it has reached\n"
                          "    the maximum number of elements (100000000)\n"
                          "    allowed in MAT-file multicore_dataRec3.mat.\n");
          }
        }
      }
    }
  }

  /* SignalConversion generated from: '<Root>/To File10' incorporates:
   *  MATLABSystem: '<Root>/MATLAB System1'
   */
  raspberrypi_multicore_MPCtest_B.R[0] =
    raspberrypi_multicore_MPCtest_B.deltaP2[0];
  raspberrypi_multicore_MPCtest_B.R[3] =
    raspberrypi_multicore_MPCtest_B.deltaP3[0];
  raspberrypi_multicore_MPCtest_B.R[6] = raspberrypi_multicore_MPCtest_B.v1[0];
  raspberrypi_multicore_MPCtest_B.R[1] =
    raspberrypi_multicore_MPCtest_B.deltaP2[1];
  raspberrypi_multicore_MPCtest_B.R[4] =
    raspberrypi_multicore_MPCtest_B.deltaP3[1];
  raspberrypi_multicore_MPCtest_B.R[7] = raspberrypi_multicore_MPCtest_B.v1[1];
  raspberrypi_multicore_MPCtest_B.R[2] =
    raspberrypi_multicore_MPCtest_B.deltaP2[2];
  raspberrypi_multicore_MPCtest_B.R[5] =
    raspberrypi_multicore_MPCtest_B.deltaP3[2];
  raspberrypi_multicore_MPCtest_B.R[8] = raspberrypi_multicore_MPCtest_B.v1[2];

  /* ToFile: '<Root>/To File10' */
  if (tid == 0 ) {
    {
      if (!(++raspberrypi_multicore_MPCtes_DW.ToFile10_IWORK.Decimation % 1) &&
          (raspberrypi_multicore_MPCtes_DW.ToFile10_IWORK.Count * (9 + 1)) + 1 <
          100000000 ) {
        FILE *fp = (FILE *)
          raspberrypi_multicore_MPCtes_DW.ToFile10_PWORK.FilePtr;
        if (fp != (NULL)) {
          real_T u[9 + 1];
          raspberrypi_multicore_MPCtes_DW.ToFile10_IWORK.Decimation = 0;
          u[0] = raspberrypi_multicore_MPCtes_M->Timing.taskTime0;
          u[1] = raspberrypi_multicore_MPCtest_B.R[0];
          u[2] = raspberrypi_multicore_MPCtest_B.R[1];
          u[3] = raspberrypi_multicore_MPCtest_B.R[2];
          u[4] = raspberrypi_multicore_MPCtest_B.R[3];
          u[5] = raspberrypi_multicore_MPCtest_B.R[4];
          u[6] = raspberrypi_multicore_MPCtest_B.R[5];
          u[7] = raspberrypi_multicore_MPCtest_B.R[6];
          u[8] = raspberrypi_multicore_MPCtest_B.R[7];
          u[9] = raspberrypi_multicore_MPCtest_B.R[8];
          if (fwrite(u, sizeof(real_T), 9 + 1, fp) != 9 + 1) {
            rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                              "Error writing to MAT-file multicore_dataRecEstA.mat");
            return;
          }

          if (((++raspberrypi_multicore_MPCtes_DW.ToFile10_IWORK.Count) * (9 + 1))
              +1 >= 100000000) {
            (void)fprintf(stdout,
                          "*** The ToFile block will stop logging data before\n"
                          "    the simulation has ended, because it has reached\n"
                          "    the maximum number of elements (100000000)\n"
                          "    allowed in MAT-file multicore_dataRecEstA.mat.\n");
          }
        }
      }
    }
  }

  /* SignalConversion generated from: '<Root>/To File12' */
  raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_h[4] =
    raspberrypi_multicore_MPCtest_B.pyNew[0];
  raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_h[5] =
    raspberrypi_multicore_MPCtest_B.pyNew[1];
  raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_h[6] =
    raspberrypi_multicore_MPCtest_B.pyNew[2];
  raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_h[7] =
    raspberrypi_multicore_MPCtest_B.pyNew[3];

  /* ToFile: '<Root>/To File12' */
  if (tid == 0 ) {
    {
      if (!(++raspberrypi_multicore_MPCtes_DW.ToFile12_IWORK.Decimation % 1) &&
          (raspberrypi_multicore_MPCtes_DW.ToFile12_IWORK.Count * (8 + 1)) + 1 <
          100000000 ) {
        FILE *fp = (FILE *)
          raspberrypi_multicore_MPCtes_DW.ToFile12_PWORK.FilePtr;
        if (fp != (NULL)) {
          real_T u[8 + 1];
          raspberrypi_multicore_MPCtes_DW.ToFile12_IWORK.Decimation = 0;
          u[0] = raspberrypi_multicore_MPCtes_M->Timing.taskTime0;
          u[1] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_h[0];
          u[2] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_h[1];
          u[3] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_h[2];
          u[4] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_h[3];
          u[5] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_h[4];
          u[6] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_h[5];
          u[7] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_h[6];
          u[8] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_h[7];
          if (fwrite(u, sizeof(real_T), 8 + 1, fp) != 8 + 1) {
            rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                              "Error writing to MAT-file multicore_dataRec11.mat");
            return;
          }

          if (((++raspberrypi_multicore_MPCtes_DW.ToFile12_IWORK.Count) * (8 + 1))
              +1 >= 100000000) {
            (void)fprintf(stdout,
                          "*** The ToFile block will stop logging data before\n"
                          "    the simulation has ended, because it has reached\n"
                          "    the maximum number of elements (100000000)\n"
                          "    allowed in MAT-file multicore_dataRec11.mat.\n");
          }
        }
      }
    }
  }

  /* ToFile: '<Root>/To File19' */
  if (tid == 0 ) {
    {
      if (!(++raspberrypi_multicore_MPCtes_DW.ToFile19_IWORK.Decimation % 1) &&
          (raspberrypi_multicore_MPCtes_DW.ToFile19_IWORK.Count * (6 + 1)) + 1 <
          100000000 ) {
        FILE *fp = (FILE *)
          raspberrypi_multicore_MPCtes_DW.ToFile19_PWORK.FilePtr;
        if (fp != (NULL)) {
          real_T u[6 + 1];
          raspberrypi_multicore_MPCtes_DW.ToFile19_IWORK.Decimation = 0;
          u[0] = raspberrypi_multicore_MPCtes_M->Timing.taskTime0;
          u[1] = raspberrypi_multicore_MPCtest_B.estDis[0];
          u[2] = raspberrypi_multicore_MPCtest_B.estDis[1];
          u[3] = raspberrypi_multicore_MPCtest_B.estDis[2];
          u[4] = raspberrypi_multicore_MPCtest_B.estDis[3];
          u[5] = raspberrypi_multicore_MPCtest_B.estDis[4];
          u[6] = raspberrypi_multicore_MPCtest_B.estDis[5];
          if (fwrite(u, sizeof(real_T), 6 + 1, fp) != 6 + 1) {
            rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                              "Error writing to MAT-file multicore_dataRec17.mat");
            return;
          }

          if (((++raspberrypi_multicore_MPCtes_DW.ToFile19_IWORK.Count) * (6 + 1))
              +1 >= 100000000) {
            (void)fprintf(stdout,
                          "*** The ToFile block will stop logging data before\n"
                          "    the simulation has ended, because it has reached\n"
                          "    the maximum number of elements (100000000)\n"
                          "    allowed in MAT-file multicore_dataRec17.mat.\n");
          }
        }
      }
    }
  }

  /* SignalConversion generated from: '<Root>/To File5' */
  raspberrypi_multicore_MPCtest_B.pxNew[0] =
    raspberrypi_multicore_MPCtest_B.estEN;
  raspberrypi_multicore_MPCtest_B.pxNew[1] =
    raspberrypi_multicore_MPCtest_B.oscEN;
  raspberrypi_multicore_MPCtest_B.pxNew[2] =
    raspberrypi_multicore_MPCtest_B.mpcSTOP;
  raspberrypi_multicore_MPCtest_B.pxNew[3] =
    raspberrypi_multicore_MPCtest_B.MATLABSystem8;

  /* ToFile: '<Root>/To File5' */
  if (tid == 0 ) {
    {
      if (!(++raspberrypi_multicore_MPCtes_DW.ToFile5_IWORK.Decimation % 1) &&
          (raspberrypi_multicore_MPCtes_DW.ToFile5_IWORK.Count * (4 + 1)) + 1 <
          100000000 ) {
        FILE *fp = (FILE *)
          raspberrypi_multicore_MPCtes_DW.ToFile5_PWORK.FilePtr;
        if (fp != (NULL)) {
          real_T u[4 + 1];
          raspberrypi_multicore_MPCtes_DW.ToFile5_IWORK.Decimation = 0;
          u[0] = raspberrypi_multicore_MPCtes_M->Timing.taskTime0;
          u[1] = raspberrypi_multicore_MPCtest_B.pxNew[0];
          u[2] = raspberrypi_multicore_MPCtest_B.pxNew[1];
          u[3] = raspberrypi_multicore_MPCtest_B.pxNew[2];
          u[4] = raspberrypi_multicore_MPCtest_B.pxNew[3];
          if (fwrite(u, sizeof(real_T), 4 + 1, fp) != 4 + 1) {
            rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                              "Error writing to MAT-file multicore_dataRec6.mat");
            return;
          }

          if (((++raspberrypi_multicore_MPCtes_DW.ToFile5_IWORK.Count) * (4 + 1))
              +1 >= 100000000) {
            (void)fprintf(stdout,
                          "*** The ToFile block will stop logging data before\n"
                          "    the simulation has ended, because it has reached\n"
                          "    the maximum number of elements (100000000)\n"
                          "    allowed in MAT-file multicore_dataRec6.mat.\n");
          }
        }
      }
    }
  }

  /* SignalConversion generated from: '<Root>/To File14' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function'
   *  MATLABSystem: '<Root>/MATLAB System3'
   */
  raspberrypi_multicore_MPCtest_B.surP[0] =
    raspberrypi_multicore_MPCtest_B.b_varargout_9_idx_0;
  raspberrypi_multicore_MPCtest_B.surP[1] =
    -raspberrypi_multicore_MPCtest_B.b_varargout_9_idx_1;
  raspberrypi_multicore_MPCtest_B.surP[2] =
    -raspberrypi_multicore_MPCtest_B.b_varargout_10_idx_1;

  /* ToFile: '<Root>/To File14' */
  if (tid == 0 ) {
    {
      if (!(++raspberrypi_multicore_MPCtes_DW.ToFile14_IWORK.Decimation % 1) &&
          (raspberrypi_multicore_MPCtes_DW.ToFile14_IWORK.Count * (3 + 1)) + 1 <
          100000000 ) {
        FILE *fp = (FILE *)
          raspberrypi_multicore_MPCtes_DW.ToFile14_PWORK.FilePtr;
        if (fp != (NULL)) {
          real_T u[3 + 1];
          raspberrypi_multicore_MPCtes_DW.ToFile14_IWORK.Decimation = 0;
          u[0] = raspberrypi_multicore_MPCtes_M->Timing.taskTime0;
          u[1] = raspberrypi_multicore_MPCtest_B.surP[0];
          u[2] = raspberrypi_multicore_MPCtest_B.surP[1];
          u[3] = raspberrypi_multicore_MPCtest_B.surP[2];
          if (fwrite(u, sizeof(real_T), 3 + 1, fp) != 3 + 1) {
            rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                              "Error writing to MAT-file multicore_dataRec13.mat");
            return;
          }

          if (((++raspberrypi_multicore_MPCtes_DW.ToFile14_IWORK.Count) * (3 + 1))
              +1 >= 100000000) {
            (void)fprintf(stdout,
                          "*** The ToFile block will stop logging data before\n"
                          "    the simulation has ended, because it has reached\n"
                          "    the maximum number of elements (100000000)\n"
                          "    allowed in MAT-file multicore_dataRec13.mat.\n");
          }
        }
      }
    }
  }

  /* Update for Memory: '<S14>/Memory1' */
  raspberrypi_multicore_MPCtes_DW.Memory1_PreviousInput =
    raspberrypi_multicore_MPCtest_B.err;

  /* Update for UnitDelay: '<S15>/Unit Delay' incorporates:
   *  MATLAB Function: '<S15>/MATLAB Function'
   */
  raspberrypi_multicore_MPCtes_DW.UnitDelay_DSTATE_l[0] =
    raspberrypi_multicore_MPCtest_B.temp[0];
  raspberrypi_multicore_MPCtes_DW.UnitDelay_DSTATE_l[3] =
    raspberrypi_multicore_MPCtest_B.temp[3];
  raspberrypi_multicore_MPCtes_DW.UnitDelay_DSTATE_l[6] =
    raspberrypi_multicore_MPCtest_B.temp[6];
  raspberrypi_multicore_MPCtes_DW.UnitDelay_DSTATE_l[9] =
    raspberrypi_multicore_MPCtest_B.temp[9];
  raspberrypi_multicore_MPCtes_DW.UnitDelay_DSTATE_l[1] =
    raspberrypi_multicore_MPCtest_B.temp[1];
  raspberrypi_multicore_MPCtes_DW.UnitDelay_DSTATE_l[4] =
    raspberrypi_multicore_MPCtest_B.temp[4];
  raspberrypi_multicore_MPCtes_DW.UnitDelay_DSTATE_l[7] =
    raspberrypi_multicore_MPCtest_B.temp[7];
  raspberrypi_multicore_MPCtes_DW.UnitDelay_DSTATE_l[10] =
    raspberrypi_multicore_MPCtest_B.temp[10];
  raspberrypi_multicore_MPCtes_DW.UnitDelay_DSTATE_l[2] =
    raspberrypi_multicore_MPCtest_B.temp[2];
  raspberrypi_multicore_MPCtes_DW.UnitDelay_DSTATE_l[5] =
    raspberrypi_multicore_MPCtest_B.temp[5];
  raspberrypi_multicore_MPCtes_DW.UnitDelay_DSTATE_l[8] =
    raspberrypi_multicore_MPCtest_B.temp[8];
  raspberrypi_multicore_MPCtes_DW.UnitDelay_DSTATE_l[11] =
    raspberrypi_multicore_MPCtest_B.temp[11];

  /* Update for UnitDelay: '<Root>/Unit Delay7' incorporates:
   *  Switch: '<Root>/Switch1'
   */
  memcpy(&raspberrypi_multicore_MPCtes_DW.UnitDelay7_DSTATE[0],
         &raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[0], 12U * sizeof
         (real_T));

  /* External mode */
  rtExtModeUploadCheckTrigger(2);
  rtExtModeUpload(0, (real_T)raspberrypi_multicore_MPCtes_M->Timing.taskTime0);

  /* signal main to stop simulation */
  {                                    /* Sample time: [0.005s, 0.0s] */
    if ((rtmGetTFinal(raspberrypi_multicore_MPCtes_M)!=-1) &&
        !((rtmGetTFinal(raspberrypi_multicore_MPCtes_M)-
           raspberrypi_multicore_MPCtes_M->Timing.taskTime0) >
          raspberrypi_multicore_MPCtes_M->Timing.taskTime0 * (DBL_EPSILON))) {
      rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M, "Simulation finished");
    }

    if (rtmGetStopRequested(raspberrypi_multicore_MPCtes_M)) {
      rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M, "Simulation finished");
    }
  }

  /* Update absolute time */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++raspberrypi_multicore_MPCtes_M->Timing.clockTick0)) {
    ++raspberrypi_multicore_MPCtes_M->Timing.clockTickH0;
  }

  raspberrypi_multicore_MPCtes_M->Timing.taskTime0 =
    raspberrypi_multicore_MPCtes_M->Timing.clockTick0 *
    raspberrypi_multicore_MPCtes_M->Timing.stepSize0 +
    raspberrypi_multicore_MPCtes_M->Timing.clockTickH0 *
    raspberrypi_multicore_MPCtes_M->Timing.stepSize0 * 4294967296.0;

  /* If subsystem generates rate grouping Output functions,
   * when tid is used in Output function for one rate,
   * all Output functions include tid as a local variable.
   * As result, some Output functions may have unused tid.
   */
  UNUSED_PARAMETER(tid);
}

/* Model step function for TID1 */
void raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis_step1(void) /* Sample time: [0.025s, 0.0s] */
{
  int_T tid = 1;
  int8_T wrBufIdx;
  boolean_T p;
  boolean_T p_0;
  static const real_T g[169] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.025, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.025, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.025, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.025, -0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.025, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.025, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    -0.00031250000000000006, 0.0, 0.0, 0.0, 0.0, 0.0, -0.025, 0.0, 0.0, 0.0, 1.0
  };

  static const int8_T k[169] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

  static const real_T tmp[169] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T h[416] = { 0.00010416666666666669, 0.0, 0.0, 0.0,
    -0.0025709862844647486, -0.0010549824786535711, 0.0083333333333333332, 0.0,
    0.0, 0.0, -0.20567890275717984, -0.084398598292285687, 0.0, 0.0,
    0.00010416666666666669, 0.0, 0.012872628726287265, 0.0,
    0.0013005281081881451, 0.0, 0.0083333333333333332, 0.0, 1.0298102981029811,
    0.0, 0.10404224865505159, 0.0, 0.0, 0.0, 0.00010416666666666669,
    0.0057926829268292691, -0.0014262208125399183, 0.0, 0.0, 0.0,
    0.0083333333333333332, 0.46341463414634143, -0.11409766500319346, 0.0, 0.0,
    0.00010416666666666669, 0.0, 0.0, 0.0, -0.0025709862844647486,
    0.0010549824786535711, 0.0083333333333333332, 0.0, 0.0, 0.0,
    -0.20567890275717984, 0.084398598292285687, 0.0, 0.0, 0.00010416666666666669,
    0.0, 0.012872628726287265, 0.0, 0.0013005281081881451, 0.0,
    0.0083333333333333332, 0.0, 1.0298102981029811, 0.0, 0.10404224865505159,
    0.0, 0.0, 0.0, 0.00010416666666666669, -0.0057926829268292691,
    -0.0014262208125399183, 0.0, 0.0, 0.0, 0.0083333333333333332,
    -0.46341463414634143, -0.11409766500319346, 0.0, 0.0, 0.00010416666666666669,
    0.0, 0.0, 0.0, -0.0025709862844647486, -0.0010549824786535711,
    0.0083333333333333332, 0.0, 0.0, 0.0, -0.20567890275717984,
    -0.084398598292285687, 0.0, 0.0, 0.00010416666666666669, 0.0,
    0.012872628726287265, 0.0, -0.0013005281081881451, 0.0,
    0.0083333333333333332, 0.0, 1.0298102981029811, 0.0, -0.10404224865505159,
    0.0, 0.0, 0.0, 0.00010416666666666669, 0.0057926829268292691,
    0.0014262208125399183, 0.0, 0.0, 0.0, 0.0083333333333333332,
    0.46341463414634143, 0.11409766500319346, 0.0, 0.0, 0.00010416666666666669,
    0.0, 0.0, 0.0, -0.0025709862844647486, 0.0010549824786535711,
    0.0083333333333333332, 0.0, 0.0, 0.0, -0.20567890275717984,
    0.084398598292285687, 0.0, 0.0, 0.00010416666666666669, 0.0,
    0.012872628726287265, 0.0, -0.0013005281081881451, 0.0,
    0.0083333333333333332, 0.0, 1.0298102981029811, 0.0, -0.10404224865505159,
    0.0, 0.0, 0.0, 0.00010416666666666669, -0.0057926829268292691,
    0.0014262208125399183, 0.0, 0.0, 0.0, 0.0083333333333333332,
    -0.46341463414634143, 0.11409766500319346, 0.0, 0.0, 0.00062500000000000012,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.00062500000000000012, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.00062500000000000012, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.031250000000000007, 0.0, 0.0, 0.0, 0.0, 0.0,
    2.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.031250000000000007, 0.0, 0.0, 0.0,
    0.0, 0.0, 2.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.031250000000000007, 0.0,
    0.0, 0.0, 0.0, 0.0, 2.5, 0.0, 0.0, 0.0, 0.0, 1.033006674779847E-18,
    -6.4941969577607583E-19, 0.0, 0.0, 0.0, 0.0, 1.3051890191934769E-16,
    8.0678566067114554E-18, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  static const real_T l[416] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0 };

  static const real_T n[18] = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 2.0, 2.0, 2.0, 100.0, 100.0, 100.0 };

  static const real_T o[252] = { 4.81, 4.81, 4.81, 4.81, 4.81, 4.81, 0.03, 0.03,
    0.03, 0.03, 0.03, 0.03, 42.65, 42.65, 42.65, 42.65, 42.65, 42.65, 42.65,
    42.65, 7.3500000000000005, 7.3500000000000005, 7.3500000000000005,
    7.3500000000000005, 7.3500000000000005, 7.3500000000000005,
    7.3500000000000005, 7.3500000000000005, 3.6750000000000003,
    3.6750000000000003, 3.6750000000000003, 3.6750000000000003,
    3.6750000000000003, 3.6750000000000003, 3.6750000000000003,
    3.6750000000000003, 3.6750000000000003, 3.6750000000000003,
    3.6750000000000003, 3.6750000000000003, 3.6750000000000003,
    3.6750000000000003, 3.6750000000000003, 3.6750000000000003, 10.0, 10.0, 10.0,
    10.0, 10.0, 10.0, 10.0, 10.0, 52.65, 52.65, 52.65, 52.65, 12.350000000000001,
    12.350000000000001, 12.350000000000001, 12.350000000000001,
    3.6750000000000003, 3.6750000000000003, 3.6750000000000003,
    3.6750000000000003, 3.6750000000000003, 3.6750000000000003,
    3.6750000000000003, 3.6750000000000003, 3.6750000000000003,
    3.6750000000000003, 3.6750000000000003, 3.6750000000000003,
    3.6750000000000003, 3.6750000000000003, 3.6750000000000003,
    3.6750000000000003, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 52.65,
    52.65, 52.65, 52.65, 12.350000000000001, 12.350000000000001,
    12.350000000000001, 12.350000000000001, 3.6750000000000003,
    3.6750000000000003, 3.6750000000000003, 3.6750000000000003,
    3.6750000000000003, 3.6750000000000003, 3.6750000000000003,
    3.6750000000000003, 3.6750000000000003, 3.6750000000000003,
    3.6750000000000003, 3.6750000000000003, 3.6750000000000003,
    3.6750000000000003, 3.6750000000000003, 3.6750000000000003, 10.0, 10.0, 10.0,
    10.0, 10.0, 10.0, 10.0, 10.0, 52.65, 52.65, 52.65, 52.65, 12.350000000000001,
    12.350000000000001, 12.350000000000001, 12.350000000000001,
    3.6750000000000003, 3.6750000000000003, 3.6750000000000003,
    3.6750000000000003, 3.6750000000000003, 3.6750000000000003,
    3.6750000000000003, 3.6750000000000003, 3.6750000000000003,
    3.6750000000000003, 3.6750000000000003, 3.6750000000000003,
    3.6750000000000003, 3.6750000000000003, 3.6750000000000003,
    3.6750000000000003, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 52.65,
    52.65, 52.65, 52.65, 12.350000000000001, 12.350000000000001,
    12.350000000000001, 12.350000000000001, 3.6750000000000003,
    3.6750000000000003, 3.6750000000000003, 3.6750000000000003,
    3.6750000000000003, 3.6750000000000003, 3.6750000000000003,
    3.6750000000000003, 3.6750000000000003, 3.6750000000000003,
    3.6750000000000003, 3.6750000000000003, 3.6750000000000003,
    3.6750000000000003, 3.6750000000000003, 3.6750000000000003, 10.0, 10.0, 10.0,
    10.0, 10.0, 10.0, 10.0, 10.0, 52.65, 52.65, 52.65, 52.65, 12.350000000000001,
    12.350000000000001, 12.350000000000001, 12.350000000000001,
    3.6750000000000003, 3.6750000000000003, 3.6750000000000003,
    3.6750000000000003, 3.6750000000000003, 3.6750000000000003,
    3.6750000000000003, 3.6750000000000003, 3.6750000000000003,
    3.6750000000000003, 3.6750000000000003, 3.6750000000000003,
    3.6750000000000003, 3.6750000000000003, 3.6750000000000003,
    3.6750000000000003, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 52.65,
    52.65, 52.65, 52.65, 12.350000000000001, 12.350000000000001,
    12.350000000000001, 12.350000000000001, 3.6750000000000003,
    3.6750000000000003, 3.6750000000000003, 3.6750000000000003,
    3.6750000000000003, 3.6750000000000003, 3.6750000000000003,
    3.6750000000000003, 3.6750000000000003, 3.6750000000000003,
    3.6750000000000003, 3.6750000000000003, 3.6750000000000003,
    3.6750000000000003, 3.6750000000000003, 3.6750000000000003, 10.0, 10.0, 10.0,
    10.0, 10.0, 10.0, 10.0, 10.0, 52.65, 52.65, 52.65, 52.65, 12.350000000000001,
    12.350000000000001, 12.350000000000001, 12.350000000000001 };

  static const real_T s[13] = { 0.0, 0.0, 0.19, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 9.8 };

  static const real_T q[28] = { 3.0, 16.0, 29.0, 42.0, 55.0, 68.0, 81.0, 94.0,
    107.0, 120.0, 133.0, 146.0, 159.0, 162.0, 165.0, 168.0, 171.0, 174.0, 177.0,
    180.0, 231.0, 234.0, 237.0, 240.0, 243.0, 246.0, 249.0, 252.0 };

  static const real_T r[12] = { 0.0, 0.0, 7.3500000000000005, 0.0, 0.0,
    7.3500000000000005, 0.0, 0.0, 7.3500000000000005, 0.0, 0.0,
    7.3500000000000005 };

  static const real_T b_RMDscale[6] = { 0.5, 0.5, 0.5, 0.01, 0.01, 0.01 };

  static const real_T b[12] = { 0.0, 0.0, 7.5, 0.0, 0.0, 7.5, 0.0, 0.0, 7.5, 0.0,
    0.0, 7.5 };

  static const real_T y[3276] = { -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, -0.025, -0.05, -0.075000000000000011, -0.1, -0.125,
    -0.15, 0.025, 0.05, 0.075000000000000011, 0.1, 0.125, 0.15, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.00031250000000000006, 0.0012500000000000002,
    0.0028125000000000008, 0.005000000000000001, 0.0078125000000000017,
    0.011250000000000003, -0.00031250000000000006, -0.0012500000000000002,
    -0.0028125000000000008, -0.005000000000000001, -0.0078125000000000017,
    -0.011250000000000003, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0 };

  static const real_T ab[3024] = { -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.00010416666666666669, -0.00041666666666666669,
    -0.0009375, -0.0016666666666666668, -0.002604166666666667,
    -0.0037500000000000003, 0.00010416666666666669, 0.00041666666666666669,
    0.0009375, 0.0016666666666666668, 0.002604166666666667,
    0.0037500000000000003, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0,
    0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0,
    0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0,
    0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0,
    1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0,
    0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -1.0, 1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.5, 0.0,
    0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0,
    0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0, -1.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.00010416666666666669, -0.00041666666666666669,
    -0.0009375, -0.0016666666666666668, -0.002604166666666667,
    -0.0037500000000000003, 0.00010416666666666669, 0.00041666666666666669,
    0.0009375, 0.0016666666666666668, 0.002604166666666667,
    0.0037500000000000003, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0,
    0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0,
    1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5,
    0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -1.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0,
    0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, -1.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.5,
    0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0,
    0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, -1.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0,
    0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, -1.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.00010416666666666669, -0.00041666666666666669,
    -0.0009375, -0.0016666666666666668, -0.002604166666666667,
    -0.0037500000000000003, 0.00010416666666666669, 0.00041666666666666669,
    0.0009375, 0.0016666666666666668, 0.002604166666666667,
    0.0037500000000000003, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0,
    0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    -1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0,
    0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, -1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.5, 0.0,
    0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0,
    0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, -1.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5,
    0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0,
    -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.00010416666666666669, -0.00041666666666666669,
    -0.0009375, -0.0016666666666666668, -0.002604166666666667,
    -0.0037500000000000003, 0.00010416666666666669, 0.00041666666666666669,
    0.0009375, 0.0016666666666666668, 0.002604166666666667,
    0.0037500000000000003, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5,
    0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0,
    0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 1.0,
    -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.5,
    0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0,
    0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0,
    0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0,
    0.5, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0,
    0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0,
    0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -1.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T bb[12348] = { -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.00062500000000000012, -0.0018750000000000004,
    -0.0031250000000000006, -0.0043750000000000013, -0.0056250000000000007,
    -0.0068750000000000009, 0.00062500000000000012, 0.0018750000000000004,
    0.0031250000000000006, 0.0043750000000000013, 0.0056250000000000007,
    0.0068750000000000009, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.00062500000000000012,
    -0.0018750000000000004, -0.0031250000000000006, -0.0043750000000000013,
    -0.0056250000000000007, 0.0, 0.00062500000000000012, 0.0018750000000000004,
    0.0031250000000000006, 0.0043750000000000013, 0.0056250000000000007, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -0.0, -0.0, -0.00062500000000000012, -0.0018750000000000004,
    -0.0031250000000000006, -0.0043750000000000013, 0.0, 0.0,
    0.00062500000000000012, 0.0018750000000000004, 0.0031250000000000006,
    0.0043750000000000013, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.00062500000000000012,
    -0.0018750000000000004, -0.0031250000000000006, 0.0, 0.0, 0.0,
    0.00062500000000000012, 0.0018750000000000004, 0.0031250000000000006, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -0.0, -0.0, -0.0, -0.0, -0.00062500000000000012, -0.0018750000000000004,
    0.0, 0.0, 0.0, 0.0, 0.00062500000000000012, 0.0018750000000000004, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.00062500000000000012, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.00062500000000000012, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  static const real_T cb[625] = { 6.0381202732736439, -0.012565420022470157,
    3.3369962284602122, 6.0077342502770374, -0.012565420022470157,
    3.3369962284602122, 6.0281202732736441, 0.012565420022470157,
    -3.3369962284602122, 6.0077342502770374, 0.012565420022470157,
    -3.3369962284602122, 3.9322418885594712, -0.0082058631429608227,
    2.1767732784277816, 3.9189287709366418, -0.0082058631429608227,
    2.1767732784277816, 3.9322418885594712, 0.0082058631429608227,
    -2.1767732784277816, 3.9189287709366418, 0.0082058631429608227,
    -2.1767732784277816, 0.0, -0.012565420022470157, 84.85763673729393,
    38.173354906140531, 0.012565420022470157, 84.847636737293925,
    -38.173354906140531, -0.012565420022470157, 84.816656730969555,
    38.173354906140531, 0.012565420022470157, 84.816656730969555,
    -38.173354906140531, -0.0082058631429608227, 55.34790609117259,
    24.901280818295987, 0.0082058631429608227, 55.34790609117259,
    -24.901280818295987, -0.0082058631429608227, 55.327674559587372,
    24.901280818295987, 0.0082058631429608227, 55.327674559587372,
    -24.901280818295987, 0.0, 3.3369962284602122, 38.173354906140524,
    19.128032277100875, 3.3369962284602122, 38.173354906140524,
    -15.2379871384256, 3.3369962284602122, 38.173354906140524,
    15.415722777314487, 3.3369962284602122, 38.173354906140524,
    -18.940296638211983, 2.176773278427782, 24.901280818295987,
    12.471081938798452, 2.176773278427782, 24.901280818295987,
    -9.9400707976679357, 2.176773278427782, 24.901280818295987,
    10.056009269890156, 2.176773278427782, 24.901280818295987,
    -12.355143466576232, 0.0, 6.0077342502770374, 0.012565420022470157,
    3.3369962284602122, 6.0381202732736439, 0.012565420022470157,
    3.3369962284602122, 6.0077342502770374, -0.012565420022470157,
    -3.3369962284602122, 6.0281202732736441, -0.012565420022470157,
    -3.3369962284602122, 3.9189287709366418, 0.0082058631429608227,
    2.1767732784277816, 3.9322418885594712, 0.0082058631429608227,
    2.1767732784277816, 3.9189287709366418, -0.0082058631429608227,
    -2.1767732784277816, 3.9322418885594712, -0.0082058631429608227,
    -2.1767732784277816, 0.0, -0.012565420022470157, 84.847636737293925,
    38.173354906140531, 0.012565420022470157, 84.85763673729393,
    -38.173354906140531, -0.012565420022470157, 84.816656730969555,
    38.173354906140531, 0.012565420022470157, 84.816656730969555,
    -38.173354906140531, -0.0082058631429608227, 55.34790609117259,
    24.901280818295987, 0.0082058631429608227, 55.34790609117259,
    -24.901280818295987, -0.0082058631429608227, 55.327674559587372,
    24.901280818295987, 0.0082058631429608227, 55.327674559587372,
    -24.901280818295987, 0.0, 3.3369962284602122, -38.173354906140524,
    -15.2379871384256, 3.3369962284602122, -38.173354906140524,
    19.128032277100875, 3.3369962284602122, -38.173354906140524,
    -18.940296638211983, 3.3369962284602122, -38.173354906140524,
    15.415722777314487, 2.176773278427782, -24.901280818295987,
    -9.9400707976679357, 2.176773278427782, -24.901280818295987,
    12.471081938798452, 2.176773278427782, -24.901280818295987,
    -12.355143466576232, 2.176773278427782, -24.901280818295987,
    10.056009269890156, 0.0, 6.0281202732736441, -0.012565420022470157,
    3.3369962284602122, 6.0077342502770374, -0.012565420022470157,
    3.3369962284602122, 6.0381202732736439, 0.012565420022470157,
    -3.3369962284602122, 6.0077342502770374, 0.012565420022470157,
    -3.3369962284602122, 3.9322418885594712, -0.0082058631429608227,
    2.1767732784277816, 3.9189287709366418, -0.0082058631429608227,
    2.1767732784277816, 3.9322418885594712, 0.0082058631429608227,
    -2.1767732784277816, 3.9189287709366418, 0.0082058631429608227,
    -2.1767732784277816, 0.0, 0.012565420022470157, 84.816656730969555,
    38.173354906140531, -0.012565420022470157, 84.816656730969555,
    -38.173354906140531, 0.012565420022470157, 84.85763673729393,
    38.173354906140531, -0.012565420022470157, 84.847636737293925,
    -38.173354906140531, 0.0082058631429608227, 55.327674559587372,
    24.901280818295987, -0.0082058631429608227, 55.327674559587372,
    -24.901280818295987, 0.0082058631429608227, 55.34790609117259,
    24.901280818295987, -0.0082058631429608227, 55.34790609117259,
    -24.901280818295987, 0.0, -3.3369962284602122, 38.173354906140524,
    15.415722777314487, -3.3369962284602122, 38.173354906140524,
    -18.940296638211983, -3.3369962284602122, 38.173354906140524,
    19.128032277100875, -3.3369962284602122, 38.173354906140524,
    -15.2379871384256, -2.176773278427782, 24.901280818295987,
    10.056009269890156, -2.176773278427782, 24.901280818295987,
    -12.355143466576232, -2.176773278427782, 24.901280818295987,
    12.471081938798452, -2.176773278427782, 24.901280818295987,
    -9.9400707976679357, 0.0, 6.0077342502770374, 0.012565420022470157,
    3.3369962284602122, 6.0281202732736441, 0.012565420022470157,
    3.3369962284602122, 6.0077342502770374, -0.012565420022470157,
    -3.3369962284602122, 6.0381202732736439, -0.012565420022470157,
    -3.3369962284602122, 3.9189287709366418, 0.0082058631429608227,
    2.1767732784277816, 3.9322418885594712, 0.0082058631429608227,
    2.1767732784277816, 3.9189287709366418, -0.0082058631429608227,
    -2.1767732784277816, 3.9322418885594712, -0.0082058631429608227,
    -2.1767732784277816, 0.0, 0.012565420022470157, 84.816656730969555,
    38.173354906140531, -0.012565420022470157, 84.816656730969555,
    -38.173354906140531, 0.012565420022470157, 84.847636737293925,
    38.173354906140531, -0.012565420022470157, 84.85763673729393,
    -38.173354906140531, 0.0082058631429608227, 55.327674559587372,
    24.901280818295987, -0.0082058631429608227, 55.327674559587372,
    -24.901280818295987, 0.0082058631429608227, 55.34790609117259,
    24.901280818295987, -0.0082058631429608227, 55.34790609117259,
    -24.901280818295987, 0.0, -3.3369962284602122, -38.173354906140524,
    -18.940296638211983, -3.3369962284602122, -38.173354906140524,
    15.415722777314487, -3.3369962284602122, -38.173354906140524,
    -15.2379871384256, -3.3369962284602122, -38.173354906140524,
    19.128032277100875, -2.176773278427782, -24.901280818295987,
    -12.355143466576232, -2.176773278427782, -24.901280818295987,
    10.056009269890156, -2.176773278427782, -24.901280818295987,
    -9.9400707976679357, -2.176773278427782, -24.901280818295987,
    12.471081938798452, 0.0, 3.9322418885594712, -0.0082058631429608227,
    2.176773278427782, 3.9189287709366418, -0.0082058631429608227,
    2.176773278427782, 3.9322418885594712, 0.0082058631429608227,
    -2.176773278427782, 3.9189287709366418, 0.0082058631429608227,
    -2.176773278427782, 2.6041574121695912, -0.0054211821914236895,
    1.4360456593769877, 2.5853621355592176, -0.0054211821914236895,
    1.4360456593769877, 2.5941574121695914, 0.0054211821914236895,
    -1.4360456593769877, 2.5853621355592176, 0.0054211821914236895,
    -1.4360456593769877, 0.0, -0.0082058631429608227, 55.34790609117259,
    24.901280818295987, 0.0082058631429608227, 55.34790609117259,
    -24.901280818295987, -0.0082058631429608227, 55.327674559587365,
    24.901280818295987, 0.0082058631429608227, 55.327674559587365,
    -24.901280818295987, -0.0054211821914236895, 36.52415264853002,
    16.427883163130414, 0.0054211821914236895, 36.514152648530022,
    -16.427883163130414, -0.0054211821914236895, 36.500786739688479,
    16.427883163130414, 0.0054211821914236895, 36.500786739688479,
    -16.427883163130414, 0.0, 2.1767732784277816, 24.901280818295987,
    12.471081938798452, 2.1767732784277816, 24.901280818295987,
    -9.9400707976679339, 2.1767732784277816, 24.901280818295987,
    10.056009269890156, 2.1767732784277816, 24.901280818295987,
    -12.35514346657623, 1.4360456593769877, 16.42788316313041, 8.23741742705489,
    1.4360456593769877, 16.42788316313041, -6.55767741976248, 1.4360456593769877,
    16.42788316313041, 6.6341625586513686, 1.4360456593769877, 16.42788316313041,
    -8.150932288166, 0.0, 3.9189287709366418, 0.0082058631429608227,
    2.176773278427782, 3.9322418885594712, 0.0082058631429608227,
    2.176773278427782, 3.9189287709366418, -0.0082058631429608227,
    -2.176773278427782, 3.9322418885594712, -0.0082058631429608227,
    -2.176773278427782, 2.5853621355592176, 0.0054211821914236895,
    1.4360456593769877, 2.6041574121695912, 0.0054211821914236895,
    1.4360456593769877, 2.5853621355592176, -0.0054211821914236895,
    -1.4360456593769877, 2.5941574121695914, -0.0054211821914236895,
    -1.4360456593769877, 0.0, -0.0082058631429608227, 55.34790609117259,
    24.901280818295987, 0.0082058631429608227, 55.34790609117259,
    -24.901280818295987, -0.0082058631429608227, 55.327674559587365,
    24.901280818295987, 0.0082058631429608227, 55.327674559587365,
    -24.901280818295987, -0.0054211821914236895, 36.514152648530022,
    16.427883163130414, 0.0054211821914236895, 36.52415264853002,
    -16.427883163130414, -0.0054211821914236895, 36.500786739688479,
    16.427883163130414, 0.0054211821914236895, 36.500786739688479,
    -16.427883163130414, 0.0, 2.1767732784277816, -24.901280818295987,
    -9.9400707976679339, 2.1767732784277816, -24.901280818295987,
    12.471081938798452, 2.1767732784277816, -24.901280818295987,
    -12.35514346657623, 2.1767732784277816, -24.901280818295987,
    10.056009269890156, 1.4360456593769877, -16.42788316313041,
    -6.55767741976248, 1.4360456593769877, -16.42788316313041, 8.23741742705489,
    1.4360456593769877, -16.42788316313041, -8.150932288166, 1.4360456593769877,
    -16.42788316313041, 6.6341625586513686, 0.0, 3.9322418885594712,
    -0.0082058631429608227, 2.176773278427782, 3.9189287709366418,
    -0.0082058631429608227, 2.176773278427782, 3.9322418885594712,
    0.0082058631429608227, -2.176773278427782, 3.9189287709366418,
    0.0082058631429608227, -2.176773278427782, 2.5941574121695914,
    -0.0054211821914236895, 1.4360456593769877, 2.5853621355592176,
    -0.0054211821914236895, 1.4360456593769877, 2.6041574121695912,
    0.0054211821914236895, -1.4360456593769877, 2.5853621355592176,
    0.0054211821914236895, -1.4360456593769877, 0.0, 0.0082058631429608227,
    55.327674559587365, 24.901280818295987, -0.0082058631429608227,
    55.327674559587365, -24.901280818295987, 0.0082058631429608227,
    55.34790609117259, 24.901280818295987, -0.0082058631429608227,
    55.34790609117259, -24.901280818295987, 0.0054211821914236895,
    36.500786739688479, 16.427883163130414, -0.0054211821914236895,
    36.500786739688479, -16.427883163130414, 0.0054211821914236895,
    36.52415264853002, 16.427883163130414, -0.0054211821914236895,
    36.514152648530022, -16.427883163130414, 0.0, -2.1767732784277816,
    24.901280818295987, 10.056009269890156, -2.1767732784277816,
    24.901280818295987, -12.35514346657623, -2.1767732784277816,
    24.901280818295987, 12.471081938798452, -2.1767732784277816,
    24.901280818295987, -9.9400707976679339, -1.4360456593769877,
    16.42788316313041, 6.6341625586513686, -1.4360456593769877,
    16.42788316313041, -8.150932288166, -1.4360456593769877, 16.42788316313041,
    8.23741742705489, -1.4360456593769877, 16.42788316313041, -6.55767741976248,
    0.0, 3.9189287709366418, 0.0082058631429608227, 2.176773278427782,
    3.9322418885594712, 0.0082058631429608227, 2.176773278427782,
    3.9189287709366418, -0.0082058631429608227, -2.176773278427782,
    3.9322418885594712, -0.0082058631429608227, -2.176773278427782,
    2.5853621355592176, 0.0054211821914236895, 1.4360456593769877,
    2.5941574121695914, 0.0054211821914236895, 1.4360456593769877,
    2.5853621355592176, -0.0054211821914236895, -1.4360456593769877,
    2.6041574121695912, -0.0054211821914236895, -1.4360456593769877, 0.0,
    0.0082058631429608227, 55.327674559587365, 24.901280818295987,
    -0.0082058631429608227, 55.327674559587365, -24.901280818295987,
    0.0082058631429608227, 55.34790609117259, 24.901280818295987,
    -0.0082058631429608227, 55.34790609117259, -24.901280818295987,
    0.0054211821914236895, 36.500786739688479, 16.427883163130414,
    -0.0054211821914236895, 36.500786739688479, -16.427883163130414,
    0.0054211821914236895, 36.514152648530022, 16.427883163130414,
    -0.0054211821914236895, 36.52415264853002, -16.427883163130414, 0.0,
    -2.1767732784277816, -24.901280818295987, -12.35514346657623,
    -2.1767732784277816, -24.901280818295987, 10.056009269890156,
    -2.1767732784277816, -24.901280818295987, -9.9400707976679339,
    -2.1767732784277816, -24.901280818295987, 12.471081938798452,
    -1.4360456593769877, -16.42788316313041, -8.150932288166,
    -1.4360456593769877, -16.42788316313041, 6.6341625586513686,
    -1.4360456593769877, -16.42788316313041, -6.55767741976248,
    -1.4360456593769877, -16.42788316313041, 8.23741742705489, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0E+6 };

  static const real_T eb[6300] = { -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, 1.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0,
    -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -1.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, 1.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0,
    -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -1.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0,
    1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0,
    1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0,
    1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.00010416666666666669, -0.00041666666666666669, -0.0009375,
    -0.0016666666666666668, -0.002604166666666667, -0.0037500000000000003,
    0.00010416666666666669, 0.00041666666666666669, 0.0009375,
    0.0016666666666666668, 0.002604166666666667, 0.0037500000000000003, -1.0,
    -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0,
    0.5, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, 1.0, -0.0, -0.0, -1.0,
    -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0,
    -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -1.0, 1.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0,
    0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, 1.0, -0.0, -0.0, -1.0, -0.0,
    -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0,
    -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -1.0, 1.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, 0.5,
    -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, 1.0, -0.0, -0.0, -1.0, -0.0, -0.0,
    -0.0, 1.0, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0,
    0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0,
    1.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, 0.5, -0.0,
    -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, 1.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0,
    1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, 1.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0,
    -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -1.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, 1.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0,
    -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -1.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, 1.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0,
    1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -1.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0,
    1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.00010416666666666669, -0.00041666666666666669, -0.0009375,
    -0.0016666666666666668, -0.002604166666666667, -0.0037500000000000003,
    0.00010416666666666669, 0.00041666666666666669, 0.0009375,
    0.0016666666666666668, 0.002604166666666667, 0.0037500000000000003, -0.0,
    -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0,
    -0.0, 0.5, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, 1.0, -0.0,
    -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5,
    -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -1.0, 1.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0,
    -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0,
    0.5, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, 1.0, -0.0, -1.0,
    -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0,
    -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -1.0, 1.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0,
    0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, 1.0, -0.0, -1.0, -0.0,
    -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0,
    -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -1.0, 1.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, 0.5,
    -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, 1.0, -0.0, -1.0, -0.0, -0.0,
    -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.0, -1.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0,
    -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, 1.0, -1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.0, -1.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0,
    -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, 1.0, -1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.0, -1.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0,
    -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.0, -1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.0, -1.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0,
    -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0,
    1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.00010416666666666669, -0.00041666666666666669, -0.0009375,
    -0.0016666666666666668, -0.002604166666666667, -0.0037500000000000003,
    0.00010416666666666669, 0.00041666666666666669, 0.0009375,
    0.0016666666666666668, 0.002604166666666667, 0.0037500000000000003, -0.0,
    -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
    0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0,
    -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.0, -1.0, -0.0,
    -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0,
    0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, 1.0, -1.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0,
    -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0,
    -0.0, 0.5, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.0, -1.0, -0.0, -0.0,
    -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5,
    -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, 1.0, -1.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0,
    -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0,
    0.5, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.0, -1.0, -0.0, -0.0, -1.0,
    -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0,
    -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, 1.0, -1.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0,
    0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.0, -1.0, -0.0, -0.0, -1.0, -0.0,
    -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.0, -1.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.0, -1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.0, -1.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0,
    -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.0,
    -1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 1.0, -1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0,
    -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.0, -1.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, 1.0, -1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0,
    -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0,
    -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0,
    1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.00010416666666666669, -0.00041666666666666669, -0.0009375,
    -0.0016666666666666668, -0.002604166666666667, -0.0037500000000000003,
    0.00010416666666666669, 0.00041666666666666669, 0.0009375,
    0.0016666666666666668, 0.002604166666666667, 0.0037500000000000003, -0.0,
    -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    1.0, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5,
    -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, -0.0, 1.0, -1.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0,
    -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0,
    -0.0, 1.0, -1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0,
    -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0,
    -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, -0.0, 1.0, -1.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0,
    0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, -0.0,
    1.0, -1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0,
    -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0,
    -0.0, 0.5, -0.0, -0.0, -0.0, -0.0, 1.0, -1.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5,
    -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, -0.0, 1.0,
    -1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0,
    -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0,
    0.5, -0.0, -0.0, -0.0, -0.0, 1.0, -1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0,
    -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0,
    -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -1.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, 1.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0,
    -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -1.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0,
    1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0,
    1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0,
    1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.00010416666666666669, -0.00041666666666666669, -0.0009375,
    -0.0016666666666666668, -0.002604166666666667, 0.0, 0.00010416666666666669,
    0.00041666666666666669, 0.0009375, 0.0016666666666666668,
    0.002604166666666667, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.5,
    -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, 1.0, -0.0, -0.0, -1.0, -0.0, -0.0,
    -0.0, 1.0, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0,
    0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0,
    1.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, 0.5, -0.0,
    -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, 1.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0,
    1.0, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5,
    -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, 1.0,
    -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0,
    -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -1.0, 1.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0,
    -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0,
    -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, 1.0, -0.0,
    -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, 1.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0,
    1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -1.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0,
    1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, 1.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0,
    -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.00010416666666666669, -0.00041666666666666669,
    -0.0009375, -0.0016666666666666668, -0.002604166666666667, 0.0,
    0.00010416666666666669, 0.00041666666666666669, 0.0009375,
    0.0016666666666666668, 0.002604166666666667, -0.0, -0.0, -0.0, -0.0, -0.0,
    -1.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5,
    -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0,
    1.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0,
    -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, 1.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0,
    -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0,
    -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, 1.0,
    -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0,
    0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -1.0, 1.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0,
    -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0,
    -0.0, 0.5, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, 1.0, -0.0,
    -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5,
    -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -1.0, 1.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.0, -1.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, 1.0, -1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.0, -1.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, 1.0, -1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.0, -1.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0,
    -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, 1.0, -1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0,
    -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0,
    -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.00010416666666666669,
    -0.00041666666666666669, -0.0009375, -0.0016666666666666668,
    -0.002604166666666667, 0.0, 0.00010416666666666669, 0.00041666666666666669,
    0.0009375, 0.0016666666666666668, 0.002604166666666667, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -1.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0,
    -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, 1.0, -1.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0,
    0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.0, -1.0, -0.0, -0.0, -1.0, -0.0,
    -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0,
    -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    1.0, -1.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, 0.5,
    -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.0, -1.0, -0.0, -0.0, -1.0, -0.0, -0.0,
    -0.0, 1.0, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0,
    0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.0,
    -1.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, 0.5, -0.0,
    -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, 1.0, -1.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0,
    1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0,
    -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.0, -1.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, 1.0, -1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0,
    -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.0, -1.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, 1.0, -1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0,
    1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 1.0, -1.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, 1.0, -1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0,
    1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.00010416666666666669, -0.00041666666666666669, -0.0009375,
    -0.0016666666666666668, -0.002604166666666667, 0.0, 0.00010416666666666669,
    0.00041666666666666669, 0.0009375, 0.0016666666666666668,
    0.002604166666666667, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0,
    -0.0, 0.5, -0.0, -0.0, -0.0, -0.0, 1.0, -1.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5,
    -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, -0.0, 1.0,
    -1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0,
    -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0,
    0.5, -0.0, -0.0, -0.0, -0.0, 1.0, -1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0,
    -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0,
    -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, -0.0, 1.0, -1.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, -0.0, -0.0, -0.0,
    0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5,
    -0.0, -0.0, -0.0, -0.0, 1.0, -1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0,
    -0.0, -0.0, 1.0, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0,
    -0.0, 0.5, -0.0, -0.0, -0.0, 0.5, -0.0, -0.0, -0.0, -0.0, 1.0, -1.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  static const real_T fb[78] = { 100.0, 100.0, 3600.0, 225.0, 400.0, 4.0, 0.0001,
    0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0, 100.0, 100.0, 3600.0, 225.0,
    400.0, 4.0, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0, 100.0,
    100.0, 3600.0, 225.0, 400.0, 4.0, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001,
    0.0001, 0.0, 100.0, 100.0, 3600.0, 225.0, 400.0, 4.0, 0.0001, 0.0001, 0.0001,
    0.0001, 0.0001, 0.0001, 0.0, 100.0, 100.0, 3600.0, 225.0, 400.0, 4.0, 0.0001,
    0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0, 100.0, 100.0, 3600.0, 225.0,
    400.0, 4.0, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0 };

  static const real_T gb[1728] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  static const real_T lb[864] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  boolean_T exitg1;

  /* RateTransition: '<Root>/Rate Transition1' */
  rtw_pthread_mutex_lock
    (raspberrypi_multicore_MPCtes_DW.RateTransition1_d0_SEMAPHORE);
  raspberrypi_multicore_MPCtes_DW.RateTransition1_RDBuf =
    raspberrypi_multicore_MPCtes_DW.RateTransition1_LstBufWR;
  rtw_pthread_mutex_unlock
    (raspberrypi_multicore_MPCtes_DW.RateTransition1_d0_SEMAPHORE);
  switch (raspberrypi_multicore_MPCtes_DW.RateTransition1_RDBuf) {
   case 0:
    for (raspberrypi_multicore_MPCtest_B.i = 0;
         raspberrypi_multicore_MPCtest_B.i < 55;
         raspberrypi_multicore_MPCtest_B.i++) {
      raspberrypi_multicore_MPCtest_B.RateTransition1[raspberrypi_multicore_MPCtest_B.i]
        =
        raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf0[raspberrypi_multicore_MPCtest_B.i];
    }
    break;

   case 1:
    for (raspberrypi_multicore_MPCtest_B.i = 0;
         raspberrypi_multicore_MPCtest_B.i < 55;
         raspberrypi_multicore_MPCtest_B.i++) {
      raspberrypi_multicore_MPCtest_B.RateTransition1[raspberrypi_multicore_MPCtest_B.i]
        =
        raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf1[raspberrypi_multicore_MPCtest_B.i];
    }
    break;

   case 2:
    for (raspberrypi_multicore_MPCtest_B.i = 0;
         raspberrypi_multicore_MPCtest_B.i < 55;
         raspberrypi_multicore_MPCtest_B.i++) {
      raspberrypi_multicore_MPCtest_B.RateTransition1[raspberrypi_multicore_MPCtest_B.i]
        =
        raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf2[raspberrypi_multicore_MPCtest_B.i];
    }
    break;
  }

  /* End of RateTransition: '<Root>/Rate Transition1' */

  /* MATLABSystem: '<Root>/MATLAB System15' incorporates:
   *  RateTransition: '<Root>/Rate Transition1'
   */
  if (raspberrypi_multicore_MPCtes_DW.obj_e.lateral_width !=
      raspberrypi_multicore_MPCtest_P.MATLABSystem15_lateral_width) {
    raspberrypi_multicore_MPCtes_DW.obj_e.lateral_width =
      raspberrypi_multicore_MPCtest_P.MATLABSystem15_lateral_width;
  }

  if (raspberrypi_multicore_MPCtes_DW.obj_e.sagetial_width !=
      raspberrypi_multicore_MPCtest_P.MATLABSystem15_sagetial_width) {
    raspberrypi_multicore_MPCtes_DW.obj_e.sagetial_width =
      raspberrypi_multicore_MPCtest_P.MATLABSystem15_sagetial_width;
  }

  if (raspberrypi_multicore_MPCtes_DW.obj_e.roll_Off !=
      raspberrypi_multicore_MPCtest_P.MATLABSystem15_roll_Off) {
    raspberrypi_multicore_MPCtes_DW.obj_e.roll_Off =
      raspberrypi_multicore_MPCtest_P.MATLABSystem15_roll_Off;
  }

  if (raspberrypi_multicore_MPCtes_DW.obj_e.hIni !=
      raspberrypi_multicore_MPCtest_P.MATLABSystem15_hIni) {
    raspberrypi_multicore_MPCtes_DW.obj_e.hIni =
      raspberrypi_multicore_MPCtest_P.MATLABSystem15_hIni;
  }

  if (fabs(raspberrypi_multicore_MPCtest_B.RateTransition1[0] - 66.0) < 0.1) {
    raspberrypi_multicore_MPCtest_B.b_varargout_1_idx_0 =
      raspberrypi_multicore_MPCtest_B.RateTransition1[1];
    raspberrypi_multicore_MPCtest_B.b_varargout_2_idx_0 =
      raspberrypi_multicore_MPCtest_B.RateTransition1[4];
    raspberrypi_multicore_MPCtest_B.b_varargout_3_idx_0 =
      raspberrypi_multicore_MPCtest_B.RateTransition1[7];
    raspberrypi_multicore_MPCtest_B.b_varargout_4_idx_0 =
      raspberrypi_multicore_MPCtest_B.RateTransition1[10];
    raspberrypi_multicore_MPCtest_B.b_varargout_1_idx_1 =
      raspberrypi_multicore_MPCtest_B.RateTransition1[2];
    raspberrypi_multicore_MPCtest_B.b_varargout_2_idx_1 =
      raspberrypi_multicore_MPCtest_B.RateTransition1[5];
    raspberrypi_multicore_MPCtest_B.b_varargout_3_idx_1 =
      raspberrypi_multicore_MPCtest_B.RateTransition1[8];
    raspberrypi_multicore_MPCtest_B.b_varargout_4_idx_1 =
      raspberrypi_multicore_MPCtest_B.RateTransition1[11];
    raspberrypi_multicore_MPCtest_B.b_varargout_1_idx_2 =
      raspberrypi_multicore_MPCtest_B.RateTransition1[3];
    raspberrypi_multicore_MPCtest_B.b_varargout_2_idx_2 =
      raspberrypi_multicore_MPCtest_B.RateTransition1[6];
    raspberrypi_multicore_MPCtest_B.b_varargout_3_idx_2 =
      raspberrypi_multicore_MPCtest_B.RateTransition1[9];
    raspberrypi_multicore_MPCtest_B.b_varargout_4_idx_2 =
      raspberrypi_multicore_MPCtest_B.RateTransition1[12];
    raspberrypi_multicore_MPCtest_B.b_varargout_5[0] =
      raspberrypi_multicore_MPCtest_B.RateTransition1[13];
    raspberrypi_multicore_MPCtest_B.b_varargout_5[1] =
      raspberrypi_multicore_MPCtest_B.RateTransition1[14];
    raspberrypi_multicore_MPCtest_B.b_varargout_5[2] =
      raspberrypi_multicore_MPCtest_B.RateTransition1[15];
    raspberrypi_multicore_MPCtest_B.b_varargout_5[3] =
      raspberrypi_multicore_MPCtest_B.RateTransition1[16];
    for (raspberrypi_multicore_MPCtest_B.b_k = 0;
         raspberrypi_multicore_MPCtest_B.b_k < 12;
         raspberrypi_multicore_MPCtest_B.b_k++) {
      raspberrypi_multicore_MPCtest_B.b_varargout_6_e[raspberrypi_multicore_MPCtest_B.b_k]
        = raspberrypi_multicore_MPCtest_B.RateTransition1
        [(raspberrypi_multicore_MPCtest_B.b_k + 18) - 1];
    }

    raspberrypi_multicore_MPCtest_B.b_varargout_7_m[0] =
      raspberrypi_multicore_MPCtest_B.RateTransition1[29];
    raspberrypi_multicore_MPCtest_B.b_varargout_8_l[0] =
      raspberrypi_multicore_MPCtest_B.RateTransition1[32];
    raspberrypi_multicore_MPCtest_B.b_varargout_9_o[0] =
      raspberrypi_multicore_MPCtest_B.RateTransition1[35];
    raspberrypi_multicore_MPCtest_B.b_varargout_10[0] =
      raspberrypi_multicore_MPCtest_B.RateTransition1[38];
    raspberrypi_multicore_MPCtest_B.surVhead_idx_0 =
      raspberrypi_multicore_MPCtest_B.RateTransition1[41];
    raspberrypi_multicore_MPCtest_B.b_varargout_7_m[1] =
      raspberrypi_multicore_MPCtest_B.RateTransition1[30];
    raspberrypi_multicore_MPCtest_B.b_varargout_8_l[1] =
      raspberrypi_multicore_MPCtest_B.RateTransition1[33];
    raspberrypi_multicore_MPCtest_B.b_varargout_9_o[1] =
      raspberrypi_multicore_MPCtest_B.RateTransition1[36];
    raspberrypi_multicore_MPCtest_B.b_varargout_10[1] =
      raspberrypi_multicore_MPCtest_B.RateTransition1[39];
    raspberrypi_multicore_MPCtest_B.surVhead_idx_1 =
      raspberrypi_multicore_MPCtest_B.RateTransition1[42];
    raspberrypi_multicore_MPCtest_B.b_varargout_7_m[2] =
      raspberrypi_multicore_MPCtest_B.RateTransition1[31];
    raspberrypi_multicore_MPCtest_B.b_varargout_8_l[2] =
      raspberrypi_multicore_MPCtest_B.RateTransition1[34];
    raspberrypi_multicore_MPCtest_B.b_varargout_9_o[2] =
      raspberrypi_multicore_MPCtest_B.RateTransition1[37];
    raspberrypi_multicore_MPCtest_B.b_varargout_10[2] =
      raspberrypi_multicore_MPCtest_B.RateTransition1[40];
    raspberrypi_multicore_MPCtest_B.surVhead_idx_2 =
      raspberrypi_multicore_MPCtest_B.RateTransition1[43];
    raspberrypi_multicore_MPCtest_B.b_varargout_12 =
      raspberrypi_multicore_MPCtest_B.RateTransition1[44];
    raspberrypi_multicore_MPCtest_B.b_varargout_13 =
      raspberrypi_multicore_MPCtest_B.RateTransition1[45];
    raspberrypi_multicore_MPCtest_B.b_varargout_14 =
      raspberrypi_multicore_MPCtest_B.RateTransition1[46];
    raspberrypi_multicore_MPCtest_B.b_varargout_15 =
      raspberrypi_multicore_MPCtest_B.RateTransition1[47];
    raspberrypi_multicore_MPCtest_B.b_varargout_16 =
      raspberrypi_multicore_MPCtest_B.RateTransition1[48];
    for (raspberrypi_multicore_MPCtest_B.b_k = 0;
         raspberrypi_multicore_MPCtest_B.b_k < 6;
         raspberrypi_multicore_MPCtest_B.b_k++) {
      raspberrypi_multicore_MPCtest_B.b_varargout_17[raspberrypi_multicore_MPCtest_B.b_k]
        = raspberrypi_multicore_MPCtest_B.RateTransition1
        [(raspberrypi_multicore_MPCtest_B.b_k + 50) - 1];
    }
  } else {
    raspberrypi_multicore_MPCtest_B.b_varargout_1_idx_0 =
      raspberrypi_multicore_MPCtes_DW.obj_e.pCoM_Old[0];
    raspberrypi_multicore_MPCtest_B.b_varargout_2_idx_0 =
      raspberrypi_multicore_MPCtes_DW.obj_e.vCoM_Old[0];
    raspberrypi_multicore_MPCtest_B.b_varargout_3_idx_0 =
      raspberrypi_multicore_MPCtes_DW.obj_e.RPYnew_Old[0];
    raspberrypi_multicore_MPCtest_B.b_varargout_4_idx_0 =
      raspberrypi_multicore_MPCtes_DW.obj_e.OmegaW_Old[0];
    raspberrypi_multicore_MPCtest_B.b_varargout_1_idx_1 =
      raspberrypi_multicore_MPCtes_DW.obj_e.pCoM_Old[1];
    raspberrypi_multicore_MPCtest_B.b_varargout_2_idx_1 =
      raspberrypi_multicore_MPCtes_DW.obj_e.vCoM_Old[1];
    raspberrypi_multicore_MPCtest_B.b_varargout_3_idx_1 =
      raspberrypi_multicore_MPCtes_DW.obj_e.RPYnew_Old[1];
    raspberrypi_multicore_MPCtest_B.b_varargout_4_idx_1 =
      raspberrypi_multicore_MPCtes_DW.obj_e.OmegaW_Old[1];
    raspberrypi_multicore_MPCtest_B.b_varargout_1_idx_2 =
      raspberrypi_multicore_MPCtes_DW.obj_e.pCoM_Old[2];
    raspberrypi_multicore_MPCtest_B.b_varargout_2_idx_2 =
      raspberrypi_multicore_MPCtes_DW.obj_e.vCoM_Old[2];
    raspberrypi_multicore_MPCtest_B.b_varargout_3_idx_2 =
      raspberrypi_multicore_MPCtes_DW.obj_e.RPYnew_Old[2];
    raspberrypi_multicore_MPCtest_B.b_varargout_4_idx_2 =
      raspberrypi_multicore_MPCtes_DW.obj_e.OmegaW_Old[2];
    raspberrypi_multicore_MPCtest_B.b_varargout_5[0] =
      raspberrypi_multicore_MPCtes_DW.obj_e.SPLeg_Old[0];
    raspberrypi_multicore_MPCtest_B.b_varargout_5[1] =
      raspberrypi_multicore_MPCtes_DW.obj_e.SPLeg_Old[1];
    raspberrypi_multicore_MPCtest_B.b_varargout_5[2] =
      raspberrypi_multicore_MPCtes_DW.obj_e.SPLeg_Old[2];
    raspberrypi_multicore_MPCtest_B.b_varargout_5[3] =
      raspberrypi_multicore_MPCtes_DW.obj_e.SPLeg_Old[3];
    memcpy(&raspberrypi_multicore_MPCtest_B.b_varargout_6_e[0],
           &raspberrypi_multicore_MPCtes_DW.obj_e.SP_Old[0], 12U * sizeof(real_T));
    raspberrypi_multicore_MPCtest_B.b_varargout_7_m[0] =
      raspberrypi_multicore_MPCtes_DW.obj_e.surVN_Old[0];
    raspberrypi_multicore_MPCtest_B.b_varargout_8_l[0] =
      raspberrypi_multicore_MPCtes_DW.obj_e.surV1_Old[0];
    raspberrypi_multicore_MPCtest_B.b_varargout_9_o[0] =
      raspberrypi_multicore_MPCtes_DW.obj_e.surV2_Old[0];
    raspberrypi_multicore_MPCtest_B.b_varargout_10[0] =
      raspberrypi_multicore_MPCtes_DW.obj_e.surP_Old[0];
    raspberrypi_multicore_MPCtest_B.surVhead_idx_0 =
      raspberrypi_multicore_MPCtes_DW.obj_e.headG_Old[0];
    raspberrypi_multicore_MPCtest_B.b_varargout_7_m[1] =
      raspberrypi_multicore_MPCtes_DW.obj_e.surVN_Old[1];
    raspberrypi_multicore_MPCtest_B.b_varargout_8_l[1] =
      raspberrypi_multicore_MPCtes_DW.obj_e.surV1_Old[1];
    raspberrypi_multicore_MPCtest_B.b_varargout_9_o[1] =
      raspberrypi_multicore_MPCtes_DW.obj_e.surV2_Old[1];
    raspberrypi_multicore_MPCtest_B.b_varargout_10[1] =
      raspberrypi_multicore_MPCtes_DW.obj_e.surP_Old[1];
    raspberrypi_multicore_MPCtest_B.surVhead_idx_1 =
      raspberrypi_multicore_MPCtes_DW.obj_e.headG_Old[1];
    raspberrypi_multicore_MPCtest_B.b_varargout_7_m[2] =
      raspberrypi_multicore_MPCtes_DW.obj_e.surVN_Old[2];
    raspberrypi_multicore_MPCtest_B.b_varargout_8_l[2] =
      raspberrypi_multicore_MPCtes_DW.obj_e.surV1_Old[2];
    raspberrypi_multicore_MPCtest_B.b_varargout_9_o[2] =
      raspberrypi_multicore_MPCtes_DW.obj_e.surV2_Old[2];
    raspberrypi_multicore_MPCtest_B.b_varargout_10[2] =
      raspberrypi_multicore_MPCtes_DW.obj_e.surP_Old[2];
    raspberrypi_multicore_MPCtest_B.surVhead_idx_2 =
      raspberrypi_multicore_MPCtes_DW.obj_e.headG_Old[2];
    raspberrypi_multicore_MPCtest_B.b_varargout_12 = 1.0;
    raspberrypi_multicore_MPCtest_B.b_varargout_13 =
      raspberrypi_multicore_MPCtes_DW.obj_e.phi_Old;
    raspberrypi_multicore_MPCtest_B.b_varargout_14 =
      raspberrypi_multicore_MPCtes_DW.obj_e.vxPercent_Old;
    raspberrypi_multicore_MPCtest_B.b_varargout_15 =
      raspberrypi_multicore_MPCtes_DW.obj_e.vyPercent_Old;
    raspberrypi_multicore_MPCtest_B.b_varargout_16 =
      raspberrypi_multicore_MPCtes_DW.obj_e.wzPercent_Old;
    for (raspberrypi_multicore_MPCtest_B.i = 0;
         raspberrypi_multicore_MPCtest_B.i < 6;
         raspberrypi_multicore_MPCtest_B.i++) {
      raspberrypi_multicore_MPCtest_B.b_varargout_17[raspberrypi_multicore_MPCtest_B.i]
        =
        raspberrypi_multicore_MPCtes_DW.obj_e.estDis_Old[raspberrypi_multicore_MPCtest_B.i];
    }
  }

  raspberrypi_multicore_MPCtes_DW.obj_e.pCoM_Old[0] =
    raspberrypi_multicore_MPCtest_B.b_varargout_1_idx_0;
  raspberrypi_multicore_MPCtes_DW.obj_e.vCoM_Old[0] =
    raspberrypi_multicore_MPCtest_B.b_varargout_2_idx_0;
  raspberrypi_multicore_MPCtes_DW.obj_e.RPYnew_Old[0] =
    raspberrypi_multicore_MPCtest_B.b_varargout_3_idx_0;
  raspberrypi_multicore_MPCtes_DW.obj_e.OmegaW_Old[0] =
    raspberrypi_multicore_MPCtest_B.b_varargout_4_idx_0;
  raspberrypi_multicore_MPCtes_DW.obj_e.pCoM_Old[1] =
    raspberrypi_multicore_MPCtest_B.b_varargout_1_idx_1;
  raspberrypi_multicore_MPCtes_DW.obj_e.vCoM_Old[1] =
    raspberrypi_multicore_MPCtest_B.b_varargout_2_idx_1;
  raspberrypi_multicore_MPCtes_DW.obj_e.RPYnew_Old[1] =
    raspberrypi_multicore_MPCtest_B.b_varargout_3_idx_1;
  raspberrypi_multicore_MPCtes_DW.obj_e.OmegaW_Old[1] =
    raspberrypi_multicore_MPCtest_B.b_varargout_4_idx_1;
  raspberrypi_multicore_MPCtes_DW.obj_e.pCoM_Old[2] =
    raspberrypi_multicore_MPCtest_B.b_varargout_1_idx_2;
  raspberrypi_multicore_MPCtes_DW.obj_e.vCoM_Old[2] =
    raspberrypi_multicore_MPCtest_B.b_varargout_2_idx_2;
  raspberrypi_multicore_MPCtes_DW.obj_e.RPYnew_Old[2] =
    raspberrypi_multicore_MPCtest_B.b_varargout_3_idx_2;
  raspberrypi_multicore_MPCtes_DW.obj_e.OmegaW_Old[2] =
    raspberrypi_multicore_MPCtest_B.b_varargout_4_idx_2;
  raspberrypi_multicore_MPCtes_DW.obj_e.SPLeg_Old[0] =
    raspberrypi_multicore_MPCtest_B.b_varargout_5[0];
  raspberrypi_multicore_MPCtes_DW.obj_e.SPLeg_Old[1] =
    raspberrypi_multicore_MPCtest_B.b_varargout_5[1];
  raspberrypi_multicore_MPCtes_DW.obj_e.SPLeg_Old[2] =
    raspberrypi_multicore_MPCtest_B.b_varargout_5[2];
  raspberrypi_multicore_MPCtes_DW.obj_e.SPLeg_Old[3] =
    raspberrypi_multicore_MPCtest_B.b_varargout_5[3];
  memcpy(&raspberrypi_multicore_MPCtes_DW.obj_e.SP_Old[0],
         &raspberrypi_multicore_MPCtest_B.b_varargout_6_e[0], 12U * sizeof
         (real_T));
  raspberrypi_multicore_MPCtes_DW.obj_e.surVN_Old[0] =
    raspberrypi_multicore_MPCtest_B.b_varargout_7_m[0];
  raspberrypi_multicore_MPCtes_DW.obj_e.surV1_Old[0] =
    raspberrypi_multicore_MPCtest_B.b_varargout_8_l[0];
  raspberrypi_multicore_MPCtes_DW.obj_e.surV2_Old[0] =
    raspberrypi_multicore_MPCtest_B.b_varargout_9_o[0];
  raspberrypi_multicore_MPCtes_DW.obj_e.surP_Old[0] =
    raspberrypi_multicore_MPCtest_B.b_varargout_10[0];
  raspberrypi_multicore_MPCtes_DW.obj_e.headG_Old[0] =
    raspberrypi_multicore_MPCtest_B.surVhead_idx_0;
  raspberrypi_multicore_MPCtes_DW.obj_e.surVN_Old[1] =
    raspberrypi_multicore_MPCtest_B.b_varargout_7_m[1];
  raspberrypi_multicore_MPCtes_DW.obj_e.surV1_Old[1] =
    raspberrypi_multicore_MPCtest_B.b_varargout_8_l[1];
  raspberrypi_multicore_MPCtes_DW.obj_e.surV2_Old[1] =
    raspberrypi_multicore_MPCtest_B.b_varargout_9_o[1];
  raspberrypi_multicore_MPCtes_DW.obj_e.surP_Old[1] =
    raspberrypi_multicore_MPCtest_B.b_varargout_10[1];
  raspberrypi_multicore_MPCtes_DW.obj_e.headG_Old[1] =
    raspberrypi_multicore_MPCtest_B.surVhead_idx_1;
  raspberrypi_multicore_MPCtes_DW.obj_e.surVN_Old[2] =
    raspberrypi_multicore_MPCtest_B.b_varargout_7_m[2];
  raspberrypi_multicore_MPCtes_DW.obj_e.surV1_Old[2] =
    raspberrypi_multicore_MPCtest_B.b_varargout_8_l[2];
  raspberrypi_multicore_MPCtes_DW.obj_e.surV2_Old[2] =
    raspberrypi_multicore_MPCtest_B.b_varargout_9_o[2];
  raspberrypi_multicore_MPCtes_DW.obj_e.surP_Old[2] =
    raspberrypi_multicore_MPCtest_B.b_varargout_10[2];
  raspberrypi_multicore_MPCtes_DW.obj_e.headG_Old[2] =
    raspberrypi_multicore_MPCtest_B.surVhead_idx_2;
  raspberrypi_multicore_MPCtes_DW.obj_e.phi_Old =
    raspberrypi_multicore_MPCtest_B.b_varargout_13;
  for (raspberrypi_multicore_MPCtest_B.i = 0; raspberrypi_multicore_MPCtest_B.i <
       6; raspberrypi_multicore_MPCtest_B.i++) {
    raspberrypi_multicore_MPCtes_DW.obj_e.estDis_Old[raspberrypi_multicore_MPCtest_B.i]
      =
      raspberrypi_multicore_MPCtest_B.b_varargout_17[raspberrypi_multicore_MPCtest_B.i];
  }

  /* MATLAB Function: '<Root>/online Constraints1' incorporates:
   *  MATLABSystem: '<Root>/MATLAB System15'
   */
  memset(&raspberrypi_multicore_MPCtest_B.Eb2_l[0], 0, 96U * sizeof(real_T));
  for (raspberrypi_multicore_MPCtest_B.i = 0; raspberrypi_multicore_MPCtest_B.i <
       4; raspberrypi_multicore_MPCtest_B.i++) {
    raspberrypi_multicore_MPCtest_B.b_k = (raspberrypi_multicore_MPCtest_B.i + 1)
      * 3 - 3;
    raspberrypi_multicore_MPCtest_B.Eb2_tmp_i =
      raspberrypi_multicore_MPCtest_B.i + (raspberrypi_multicore_MPCtest_B.b_k <<
      3);
    raspberrypi_multicore_MPCtest_B.Eb2_l[raspberrypi_multicore_MPCtest_B.Eb2_tmp_i]
      = -0.5 * raspberrypi_multicore_MPCtest_B.b_varargout_7_m[0] +
      raspberrypi_multicore_MPCtest_B.b_varargout_8_l[0];
    raspberrypi_multicore_MPCtest_B.Eb2_l[raspberrypi_multicore_MPCtest_B.Eb2_tmp_i
      + 4] = -0.5 * raspberrypi_multicore_MPCtest_B.b_varargout_7_m[0] -
      raspberrypi_multicore_MPCtest_B.b_varargout_8_l[0];
    raspberrypi_multicore_MPCtest_B.Eb2_tmp_i =
      raspberrypi_multicore_MPCtest_B.i + ((raspberrypi_multicore_MPCtest_B.b_k
      + 1) << 3);
    raspberrypi_multicore_MPCtest_B.Eb2_l[raspberrypi_multicore_MPCtest_B.Eb2_tmp_i]
      = -0.5 * raspberrypi_multicore_MPCtest_B.b_varargout_7_m[1] +
      raspberrypi_multicore_MPCtest_B.b_varargout_8_l[1];
    raspberrypi_multicore_MPCtest_B.Eb2_l[raspberrypi_multicore_MPCtest_B.Eb2_tmp_i
      + 4] = -0.5 * raspberrypi_multicore_MPCtest_B.b_varargout_7_m[1] -
      raspberrypi_multicore_MPCtest_B.b_varargout_8_l[1];
    raspberrypi_multicore_MPCtest_B.Eb2_tmp_i =
      raspberrypi_multicore_MPCtest_B.i + ((raspberrypi_multicore_MPCtest_B.b_k
      + 2) << 3);
    raspberrypi_multicore_MPCtest_B.Eb2_l[raspberrypi_multicore_MPCtest_B.Eb2_tmp_i]
      = -0.5 * raspberrypi_multicore_MPCtest_B.b_varargout_7_m[2] +
      raspberrypi_multicore_MPCtest_B.b_varargout_8_l[2];
    raspberrypi_multicore_MPCtest_B.Eb2_l[raspberrypi_multicore_MPCtest_B.Eb2_tmp_i
      + 4] = -0.5 * raspberrypi_multicore_MPCtest_B.b_varargout_7_m[2] -
      raspberrypi_multicore_MPCtest_B.b_varargout_8_l[2];
  }

  memset(&raspberrypi_multicore_MPCtest_B.Eb3[0], 0, 96U * sizeof(real_T));
  for (raspberrypi_multicore_MPCtest_B.i = 0; raspberrypi_multicore_MPCtest_B.i <
       4; raspberrypi_multicore_MPCtest_B.i++) {
    raspberrypi_multicore_MPCtest_B.b_k = (raspberrypi_multicore_MPCtest_B.i + 1)
      * 3 - 3;
    raspberrypi_multicore_MPCtest_B.Eb2_tmp_i =
      raspberrypi_multicore_MPCtest_B.i + (raspberrypi_multicore_MPCtest_B.b_k <<
      3);
    raspberrypi_multicore_MPCtest_B.Eb3[raspberrypi_multicore_MPCtest_B.Eb2_tmp_i]
      = -0.5 * raspberrypi_multicore_MPCtest_B.b_varargout_7_m[0] +
      raspberrypi_multicore_MPCtest_B.b_varargout_9_o[0];
    raspberrypi_multicore_MPCtest_B.Eb3[raspberrypi_multicore_MPCtest_B.Eb2_tmp_i
      + 4] = -0.5 * raspberrypi_multicore_MPCtest_B.b_varargout_7_m[0] -
      raspberrypi_multicore_MPCtest_B.b_varargout_9_o[0];
    raspberrypi_multicore_MPCtest_B.Eb2_tmp_i =
      raspberrypi_multicore_MPCtest_B.i + ((raspberrypi_multicore_MPCtest_B.b_k
      + 1) << 3);
    raspberrypi_multicore_MPCtest_B.Eb3[raspberrypi_multicore_MPCtest_B.Eb2_tmp_i]
      = -0.5 * raspberrypi_multicore_MPCtest_B.b_varargout_7_m[1] +
      raspberrypi_multicore_MPCtest_B.b_varargout_9_o[1];
    raspberrypi_multicore_MPCtest_B.Eb3[raspberrypi_multicore_MPCtest_B.Eb2_tmp_i
      + 4] = -0.5 * raspberrypi_multicore_MPCtest_B.b_varargout_7_m[1] -
      raspberrypi_multicore_MPCtest_B.b_varargout_9_o[1];
    raspberrypi_multicore_MPCtest_B.Eb3[raspberrypi_multicore_MPCtest_B.i +
      ((raspberrypi_multicore_MPCtest_B.b_k + 2) << 3)] = -0.5 *
      raspberrypi_multicore_MPCtest_B.b_varargout_7_m[2] +
      raspberrypi_multicore_MPCtest_B.b_varargout_9_o[2];
    raspberrypi_multicore_MPCtest_B.Eb3[(raspberrypi_multicore_MPCtest_B.i +
      ((raspberrypi_multicore_MPCtest_B.b_k + 2) << 3)) + 4] = -0.5 *
      raspberrypi_multicore_MPCtest_B.b_varargout_7_m[2] -
      raspberrypi_multicore_MPCtest_B.b_varargout_9_o[2];
  }

  raspberrypi_multicore_MPCtest_B.surVhead =
    (raspberrypi_multicore_MPCtest_B.surVhead_idx_0 *
     raspberrypi_multicore_MPCtest_B.b_varargout_7_m[0] +
     raspberrypi_multicore_MPCtest_B.surVhead_idx_1 *
     raspberrypi_multicore_MPCtest_B.b_varargout_7_m[1]) +
    raspberrypi_multicore_MPCtest_B.surVhead_idx_2 *
    raspberrypi_multicore_MPCtest_B.b_varargout_7_m[2];
  raspberrypi_multicore_MPCtest_B.scale = 3.3121686421112381E-170;
  raspberrypi_multicore_MPCtest_B.surVhead_p =
    raspberrypi_multicore_MPCtest_B.surVhead_idx_0 -
    raspberrypi_multicore_MPCtest_B.surVhead *
    raspberrypi_multicore_MPCtest_B.b_varargout_7_m[0];
  raspberrypi_multicore_MPCtest_B.absxk = fabs
    (raspberrypi_multicore_MPCtest_B.surVhead_p);
  if (raspberrypi_multicore_MPCtest_B.absxk > 3.3121686421112381E-170) {
    raspberrypi_multicore_MPCtest_B.b_y = 1.0;
    raspberrypi_multicore_MPCtest_B.scale =
      raspberrypi_multicore_MPCtest_B.absxk;
  } else {
    raspberrypi_multicore_MPCtest_B.t = raspberrypi_multicore_MPCtest_B.absxk /
      3.3121686421112381E-170;
    raspberrypi_multicore_MPCtest_B.b_y = raspberrypi_multicore_MPCtest_B.t *
      raspberrypi_multicore_MPCtest_B.t;
  }

  raspberrypi_multicore_MPCtest_B.surVhead_idx_0 =
    raspberrypi_multicore_MPCtest_B.surVhead_p;
  raspberrypi_multicore_MPCtest_B.surVhead_p =
    raspberrypi_multicore_MPCtest_B.surVhead_idx_1 -
    raspberrypi_multicore_MPCtest_B.surVhead *
    raspberrypi_multicore_MPCtest_B.b_varargout_7_m[1];
  raspberrypi_multicore_MPCtest_B.absxk = fabs
    (raspberrypi_multicore_MPCtest_B.surVhead_p);
  if (raspberrypi_multicore_MPCtest_B.absxk >
      raspberrypi_multicore_MPCtest_B.scale) {
    raspberrypi_multicore_MPCtest_B.t = raspberrypi_multicore_MPCtest_B.scale /
      raspberrypi_multicore_MPCtest_B.absxk;
    raspberrypi_multicore_MPCtest_B.b_y = raspberrypi_multicore_MPCtest_B.b_y *
      raspberrypi_multicore_MPCtest_B.t * raspberrypi_multicore_MPCtest_B.t +
      1.0;
    raspberrypi_multicore_MPCtest_B.scale =
      raspberrypi_multicore_MPCtest_B.absxk;
  } else {
    raspberrypi_multicore_MPCtest_B.t = raspberrypi_multicore_MPCtest_B.absxk /
      raspberrypi_multicore_MPCtest_B.scale;
    raspberrypi_multicore_MPCtest_B.b_y += raspberrypi_multicore_MPCtest_B.t *
      raspberrypi_multicore_MPCtest_B.t;
  }

  raspberrypi_multicore_MPCtest_B.surVhead_idx_1 =
    raspberrypi_multicore_MPCtest_B.surVhead_p;
  raspberrypi_multicore_MPCtest_B.surVhead_p =
    raspberrypi_multicore_MPCtest_B.surVhead_idx_2 -
    raspberrypi_multicore_MPCtest_B.surVhead *
    raspberrypi_multicore_MPCtest_B.b_varargout_7_m[2];
  raspberrypi_multicore_MPCtest_B.absxk = fabs
    (raspberrypi_multicore_MPCtest_B.surVhead_p);
  if (raspberrypi_multicore_MPCtest_B.absxk >
      raspberrypi_multicore_MPCtest_B.scale) {
    raspberrypi_multicore_MPCtest_B.t = raspberrypi_multicore_MPCtest_B.scale /
      raspberrypi_multicore_MPCtest_B.absxk;
    raspberrypi_multicore_MPCtest_B.b_y = raspberrypi_multicore_MPCtest_B.b_y *
      raspberrypi_multicore_MPCtest_B.t * raspberrypi_multicore_MPCtest_B.t +
      1.0;
    raspberrypi_multicore_MPCtest_B.scale =
      raspberrypi_multicore_MPCtest_B.absxk;
  } else {
    raspberrypi_multicore_MPCtest_B.t = raspberrypi_multicore_MPCtest_B.absxk /
      raspberrypi_multicore_MPCtest_B.scale;
    raspberrypi_multicore_MPCtest_B.b_y += raspberrypi_multicore_MPCtest_B.t *
      raspberrypi_multicore_MPCtest_B.t;
  }

  raspberrypi_multicore_MPCtest_B.b_y = raspberrypi_multicore_MPCtest_B.scale *
    sqrt(raspberrypi_multicore_MPCtest_B.b_y);
  raspberrypi_multicore_MPCtest_B.surVhead =
    raspberrypi_multicore_MPCtest_B.surVhead_idx_0 /
    raspberrypi_multicore_MPCtest_B.b_y;
  raspberrypi_multicore_MPCtest_B.surVhead_idx_0 =
    raspberrypi_multicore_MPCtest_B.surVhead_idx_1 /
    raspberrypi_multicore_MPCtest_B.b_y;
  raspberrypi_multicore_MPCtest_B.surVhead_p /=
    raspberrypi_multicore_MPCtest_B.b_y;
  memset(&raspberrypi_multicore_MPCtest_B.Eb4[0], 0, 48U * sizeof(real_T));
  raspberrypi_multicore_MPCtest_B.Eb4[0] =
    raspberrypi_multicore_MPCtest_B.surVhead;
  raspberrypi_multicore_MPCtest_B.Eb4[36] =
    -raspberrypi_multicore_MPCtest_B.surVhead;
  raspberrypi_multicore_MPCtest_B.Eb4[1] =
    -raspberrypi_multicore_MPCtest_B.surVhead;
  raspberrypi_multicore_MPCtest_B.Eb4[37] =
    raspberrypi_multicore_MPCtest_B.surVhead;
  raspberrypi_multicore_MPCtest_B.Eb4[14] =
    raspberrypi_multicore_MPCtest_B.surVhead;
  raspberrypi_multicore_MPCtest_B.Eb4[26] =
    -raspberrypi_multicore_MPCtest_B.surVhead;
  raspberrypi_multicore_MPCtest_B.Eb4[15] =
    -raspberrypi_multicore_MPCtest_B.surVhead;
  raspberrypi_multicore_MPCtest_B.Eb4[27] =
    raspberrypi_multicore_MPCtest_B.surVhead;
  raspberrypi_multicore_MPCtest_B.Eb4[4] =
    raspberrypi_multicore_MPCtest_B.surVhead_idx_0;
  raspberrypi_multicore_MPCtest_B.Eb4[40] =
    -raspberrypi_multicore_MPCtest_B.surVhead_idx_0;
  raspberrypi_multicore_MPCtest_B.Eb4[5] =
    -raspberrypi_multicore_MPCtest_B.surVhead_idx_0;
  raspberrypi_multicore_MPCtest_B.Eb4[41] =
    raspberrypi_multicore_MPCtest_B.surVhead_idx_0;
  raspberrypi_multicore_MPCtest_B.Eb4[18] =
    raspberrypi_multicore_MPCtest_B.surVhead_idx_0;
  raspberrypi_multicore_MPCtest_B.Eb4[30] =
    -raspberrypi_multicore_MPCtest_B.surVhead_idx_0;
  raspberrypi_multicore_MPCtest_B.Eb4[19] =
    -raspberrypi_multicore_MPCtest_B.surVhead_idx_0;
  raspberrypi_multicore_MPCtest_B.Eb4[31] =
    raspberrypi_multicore_MPCtest_B.surVhead_idx_0;
  raspberrypi_multicore_MPCtest_B.Eb4[8] =
    raspberrypi_multicore_MPCtest_B.surVhead_p;
  raspberrypi_multicore_MPCtest_B.Eb4[44] =
    -raspberrypi_multicore_MPCtest_B.surVhead_p;
  raspberrypi_multicore_MPCtest_B.Eb4[9] =
    -raspberrypi_multicore_MPCtest_B.surVhead_p;
  raspberrypi_multicore_MPCtest_B.Eb4[45] =
    raspberrypi_multicore_MPCtest_B.surVhead_p;
  raspberrypi_multicore_MPCtest_B.Eb4[22] =
    raspberrypi_multicore_MPCtest_B.surVhead_p;
  raspberrypi_multicore_MPCtest_B.Eb4[34] =
    -raspberrypi_multicore_MPCtest_B.surVhead_p;
  raspberrypi_multicore_MPCtest_B.Eb4[23] =
    -raspberrypi_multicore_MPCtest_B.surVhead_p;
  raspberrypi_multicore_MPCtest_B.Eb4[35] =
    raspberrypi_multicore_MPCtest_B.surVhead_p;
  memset(&raspberrypi_multicore_MPCtest_B.Eb5[0], 0, 48U * sizeof(real_T));
  raspberrypi_multicore_MPCtest_B.Eb5[0] =
    raspberrypi_multicore_MPCtest_B.b_varargout_7_m[0];
  raspberrypi_multicore_MPCtest_B.Eb5[36] =
    -raspberrypi_multicore_MPCtest_B.b_varargout_7_m[0];
  raspberrypi_multicore_MPCtest_B.Eb5[1] =
    -raspberrypi_multicore_MPCtest_B.b_varargout_7_m[0];
  raspberrypi_multicore_MPCtest_B.Eb5[37] =
    raspberrypi_multicore_MPCtest_B.b_varargout_7_m[0];
  raspberrypi_multicore_MPCtest_B.Eb5[14] =
    raspberrypi_multicore_MPCtest_B.b_varargout_7_m[0];
  raspberrypi_multicore_MPCtest_B.Eb5[26] =
    -raspberrypi_multicore_MPCtest_B.b_varargout_7_m[0];
  raspberrypi_multicore_MPCtest_B.Eb5[15] =
    -raspberrypi_multicore_MPCtest_B.b_varargout_7_m[0];
  raspberrypi_multicore_MPCtest_B.Eb5[27] =
    raspberrypi_multicore_MPCtest_B.b_varargout_7_m[0];
  raspberrypi_multicore_MPCtest_B.Eb5[4] =
    raspberrypi_multicore_MPCtest_B.b_varargout_7_m[1];
  raspberrypi_multicore_MPCtest_B.Eb5[40] =
    -raspberrypi_multicore_MPCtest_B.b_varargout_7_m[1];
  raspberrypi_multicore_MPCtest_B.Eb5[5] =
    -raspberrypi_multicore_MPCtest_B.b_varargout_7_m[1];
  raspberrypi_multicore_MPCtest_B.Eb5[41] =
    raspberrypi_multicore_MPCtest_B.b_varargout_7_m[1];
  raspberrypi_multicore_MPCtest_B.Eb5[18] =
    raspberrypi_multicore_MPCtest_B.b_varargout_7_m[1];
  raspberrypi_multicore_MPCtest_B.Eb5[30] =
    -raspberrypi_multicore_MPCtest_B.b_varargout_7_m[1];
  raspberrypi_multicore_MPCtest_B.Eb5[19] =
    -raspberrypi_multicore_MPCtest_B.b_varargout_7_m[1];
  raspberrypi_multicore_MPCtest_B.Eb5[31] =
    raspberrypi_multicore_MPCtest_B.b_varargout_7_m[1];
  raspberrypi_multicore_MPCtest_B.Eb5[8] =
    raspberrypi_multicore_MPCtest_B.b_varargout_7_m[2];
  raspberrypi_multicore_MPCtest_B.Eb5[44] =
    -raspberrypi_multicore_MPCtest_B.b_varargout_7_m[2];
  raspberrypi_multicore_MPCtest_B.Eb5[9] =
    -raspberrypi_multicore_MPCtest_B.b_varargout_7_m[2];
  raspberrypi_multicore_MPCtest_B.Eb5[45] =
    raspberrypi_multicore_MPCtest_B.b_varargout_7_m[2];
  raspberrypi_multicore_MPCtest_B.Eb5[22] =
    raspberrypi_multicore_MPCtest_B.b_varargout_7_m[2];
  raspberrypi_multicore_MPCtest_B.Eb5[34] =
    -raspberrypi_multicore_MPCtest_B.b_varargout_7_m[2];
  raspberrypi_multicore_MPCtest_B.Eb5[23] =
    -raspberrypi_multicore_MPCtest_B.b_varargout_7_m[2];
  raspberrypi_multicore_MPCtest_B.Eb5[35] =
    raspberrypi_multicore_MPCtest_B.b_varargout_7_m[2];
  memset(&raspberrypi_multicore_MPCtest_B.Eb1[0], 0, 96U * sizeof(real_T));
  memset(&raspberrypi_multicore_MPCtest_B.Gb1[0], 0, sizeof(real_T) << 3U);
  for (raspberrypi_multicore_MPCtest_B.i = 0; raspberrypi_multicore_MPCtest_B.i <
       4; raspberrypi_multicore_MPCtest_B.i++) {
    raspberrypi_multicore_MPCtest_B.b_k = (raspberrypi_multicore_MPCtest_B.i + 1)
      * 3 - 3;
    raspberrypi_multicore_MPCtest_B.Eb2_tmp_i =
      raspberrypi_multicore_MPCtest_B.i + (raspberrypi_multicore_MPCtest_B.b_k <<
      3);
    raspberrypi_multicore_MPCtest_B.Eb1[raspberrypi_multicore_MPCtest_B.Eb2_tmp_i]
      = raspberrypi_multicore_MPCtest_B.b_varargout_7_m[0];
    raspberrypi_multicore_MPCtest_B.Eb1[raspberrypi_multicore_MPCtest_B.Eb2_tmp_i
      + 4] = -raspberrypi_multicore_MPCtest_B.b_varargout_7_m[0];
    raspberrypi_multicore_MPCtest_B.Eb2_tmp_i =
      raspberrypi_multicore_MPCtest_B.i + ((raspberrypi_multicore_MPCtest_B.b_k
      + 1) << 3);
    raspberrypi_multicore_MPCtest_B.Eb1[raspberrypi_multicore_MPCtest_B.Eb2_tmp_i]
      = raspberrypi_multicore_MPCtest_B.b_varargout_7_m[1];
    raspberrypi_multicore_MPCtest_B.Eb1[raspberrypi_multicore_MPCtest_B.Eb2_tmp_i
      + 4] = -raspberrypi_multicore_MPCtest_B.b_varargout_7_m[1];
    raspberrypi_multicore_MPCtest_B.Eb2_tmp_i =
      raspberrypi_multicore_MPCtest_B.i + ((raspberrypi_multicore_MPCtest_B.b_k
      + 2) << 3);
    raspberrypi_multicore_MPCtest_B.Eb1[raspberrypi_multicore_MPCtest_B.Eb2_tmp_i]
      = raspberrypi_multicore_MPCtest_B.b_varargout_7_m[2];
    raspberrypi_multicore_MPCtest_B.Eb1[raspberrypi_multicore_MPCtest_B.Eb2_tmp_i
      + 4] = -raspberrypi_multicore_MPCtest_B.b_varargout_7_m[2];
    raspberrypi_multicore_MPCtest_B.Gb1[raspberrypi_multicore_MPCtest_B.i] =
      60.0;
    raspberrypi_multicore_MPCtest_B.Gb1[raspberrypi_multicore_MPCtest_B.i + 4] =
      5.0;
    if (raspberrypi_multicore_MPCtest_B.b_varargout_5[raspberrypi_multicore_MPCtest_B.i]
        < 0.5) {
      raspberrypi_multicore_MPCtest_B.Gb1[raspberrypi_multicore_MPCtest_B.i] =
        0.0;
      raspberrypi_multicore_MPCtest_B.Gb1[raspberrypi_multicore_MPCtest_B.i + 4]
        = 0.0;
    }
  }

  /* SignalConversion generated from: '<Root>/MATLAB System21' incorporates:
   *  Constant: '<Root>/Constant2'
   *  MATLABSystem: '<Root>/MATLAB System15'
   */
  raspberrypi_multicore_MPCtest_B.X_FB[0] =
    raspberrypi_multicore_MPCtest_B.b_varargout_1_idx_0;
  raspberrypi_multicore_MPCtest_B.X_FB[3] =
    raspberrypi_multicore_MPCtest_B.b_varargout_3_idx_0;
  raspberrypi_multicore_MPCtest_B.X_FB[6] =
    raspberrypi_multicore_MPCtest_B.b_varargout_2_idx_0;
  raspberrypi_multicore_MPCtest_B.X_FB[9] =
    raspberrypi_multicore_MPCtest_B.b_varargout_4_idx_0;
  raspberrypi_multicore_MPCtest_B.X_FB[1] =
    raspberrypi_multicore_MPCtest_B.b_varargout_1_idx_1;
  raspberrypi_multicore_MPCtest_B.X_FB[4] =
    raspberrypi_multicore_MPCtest_B.b_varargout_3_idx_1;
  raspberrypi_multicore_MPCtest_B.X_FB[7] =
    raspberrypi_multicore_MPCtest_B.b_varargout_2_idx_1;
  raspberrypi_multicore_MPCtest_B.X_FB[10] =
    raspberrypi_multicore_MPCtest_B.b_varargout_4_idx_1;
  raspberrypi_multicore_MPCtest_B.X_FB[2] =
    raspberrypi_multicore_MPCtest_B.b_varargout_1_idx_2;
  raspberrypi_multicore_MPCtest_B.X_FB[5] =
    raspberrypi_multicore_MPCtest_B.b_varargout_3_idx_2;
  raspberrypi_multicore_MPCtest_B.X_FB[8] =
    raspberrypi_multicore_MPCtest_B.b_varargout_2_idx_2;
  raspberrypi_multicore_MPCtest_B.X_FB[11] =
    raspberrypi_multicore_MPCtest_B.b_varargout_4_idx_2;
  raspberrypi_multicore_MPCtest_B.X_FB[12] =
    raspberrypi_multicore_MPCtest_P.Constant2_Value;

  /* MATLABSystem: '<Root>/MATLAB System6' incorporates:
   *  MATLABSystem: '<Root>/MATLAB System15'
   */
  if (raspberrypi_multicore_MPCtes_DW.obj_as.vxMax !=
      raspberrypi_multicore_MPCtest_P.MATLABSystem6_vxMax) {
    raspberrypi_multicore_MPCtes_DW.obj_as.vxMax =
      raspberrypi_multicore_MPCtest_P.MATLABSystem6_vxMax;
  }

  if (raspberrypi_multicore_MPCtes_DW.obj_as.vyMax !=
      raspberrypi_multicore_MPCtest_P.MATLABSystem6_vyMax) {
    raspberrypi_multicore_MPCtes_DW.obj_as.vyMax =
      raspberrypi_multicore_MPCtest_P.MATLABSystem6_vyMax;
  }

  if (raspberrypi_multicore_MPCtes_DW.obj_as.wzMax !=
      raspberrypi_multicore_MPCtest_P.MATLABSystem6_wzMax) {
    raspberrypi_multicore_MPCtes_DW.obj_as.wzMax =
      raspberrypi_multicore_MPCtest_P.MATLABSystem6_wzMax;
  }

  if (fabs(raspberrypi_multicore_MPCtest_B.b_varargout_12 - 1.0) < 0.1) {
    raspberrypi_multicore_MPCtest_B.b_varargout_14 = 0.0;
    raspberrypi_multicore_MPCtest_B.b_varargout_15 = 0.0;
    raspberrypi_multicore_MPCtest_B.b_varargout_16 = 0.0;
  } else {
    raspberrypi_multicore_MPCtest_B.b_varargout_14 *=
      raspberrypi_multicore_MPCtes_DW.obj_as.vxMax;
    raspberrypi_multicore_MPCtest_B.b_varargout_15 *=
      raspberrypi_multicore_MPCtes_DW.obj_as.vyMax;
    raspberrypi_multicore_MPCtest_B.b_varargout_16 *=
      raspberrypi_multicore_MPCtes_DW.obj_as.wzMax;
  }

  if (fabs(raspberrypi_multicore_MPCtest_B.b_varargout_14) >
      raspberrypi_multicore_MPCtes_DW.obj_as.vxMax) {
    if (raspberrypi_multicore_MPCtest_B.b_varargout_14 < 0.0) {
      raspberrypi_multicore_MPCtest_B.b_varargout_14 = -1.0;
    } else if (raspberrypi_multicore_MPCtest_B.b_varargout_14 > 0.0) {
      raspberrypi_multicore_MPCtest_B.b_varargout_14 = 1.0;
    } else if (raspberrypi_multicore_MPCtest_B.b_varargout_14 == 0.0) {
      raspberrypi_multicore_MPCtest_B.b_varargout_14 = 0.0;
    } else {
      raspberrypi_multicore_MPCtest_B.b_varargout_14 = (rtNaN);
    }

    raspberrypi_multicore_MPCtest_B.b_varargout_14 *=
      raspberrypi_multicore_MPCtes_DW.obj_as.vxMax;
  }

  if (fabs(raspberrypi_multicore_MPCtest_B.b_varargout_15) >
      raspberrypi_multicore_MPCtes_DW.obj_as.vyMax) {
    if (raspberrypi_multicore_MPCtest_B.b_varargout_15 < 0.0) {
      raspberrypi_multicore_MPCtest_B.b_varargout_15 = -1.0;
    } else if (raspberrypi_multicore_MPCtest_B.b_varargout_15 > 0.0) {
      raspberrypi_multicore_MPCtest_B.b_varargout_15 = 1.0;
    } else if (raspberrypi_multicore_MPCtest_B.b_varargout_15 == 0.0) {
      raspberrypi_multicore_MPCtest_B.b_varargout_15 = 0.0;
    } else {
      raspberrypi_multicore_MPCtest_B.b_varargout_15 = (rtNaN);
    }

    raspberrypi_multicore_MPCtest_B.b_varargout_15 *=
      raspberrypi_multicore_MPCtes_DW.obj_as.vyMax;
  }

  if (fabs(raspberrypi_multicore_MPCtest_B.b_varargout_16) >
      raspberrypi_multicore_MPCtes_DW.obj_as.wzMax) {
    if (raspberrypi_multicore_MPCtest_B.b_varargout_16 < 0.0) {
      raspberrypi_multicore_MPCtest_B.b_varargout_16 = -1.0;
    } else if (raspberrypi_multicore_MPCtest_B.b_varargout_16 > 0.0) {
      raspberrypi_multicore_MPCtest_B.b_varargout_16 = 1.0;
    } else if (raspberrypi_multicore_MPCtest_B.b_varargout_16 == 0.0) {
      raspberrypi_multicore_MPCtest_B.b_varargout_16 = 0.0;
    } else {
      raspberrypi_multicore_MPCtest_B.b_varargout_16 = (rtNaN);
    }

    raspberrypi_multicore_MPCtest_B.b_varargout_16 *=
      raspberrypi_multicore_MPCtes_DW.obj_as.wzMax;
  }

  /* MATLABSystem: '<Root>/MATLAB System7' incorporates:
   *  MATLABSystem: '<Root>/MATLAB System15'
   *  MATLABSystem: '<Root>/MATLAB System6'
   *  SignalConversion generated from: '<Root>/MATLAB System21'
   */
  if (!raspberrypi_multicore_M_isequal(raspberrypi_multicore_MPCtes_DW.obj_cm.r0,
       raspberrypi_multicore_MPCtest_P.MATLABSystem7_r0)) {
    raspberrypi_multicore_MPCtes_DW.obj_cm.r0[0] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem7_r0[0];
    raspberrypi_multicore_MPCtes_DW.obj_cm.r0[1] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem7_r0[1];
    raspberrypi_multicore_MPCtes_DW.obj_cm.r0[2] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem7_r0[2];
  }

  if (!raspberrypi_multicore_M_isequal
      (raspberrypi_multicore_MPCtes_DW.obj_cm.theta0,
       raspberrypi_multicore_MPCtest_P.MATLABSystem7_theta0)) {
    raspberrypi_multicore_MPCtes_DW.obj_cm.theta0[0] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem7_theta0[0];
    raspberrypi_multicore_MPCtes_DW.obj_cm.theta0[1] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem7_theta0[1];
    raspberrypi_multicore_MPCtes_DW.obj_cm.theta0[2] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem7_theta0[2];
  }

  if (!raspberrypi_multicore_M_isequal
      (raspberrypi_multicore_MPCtes_DW.obj_cm.dr0,
       raspberrypi_multicore_MPCtest_P.MATLABSystem7_dr0)) {
    raspberrypi_multicore_MPCtes_DW.obj_cm.dr0[0] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem7_dr0[0];
    raspberrypi_multicore_MPCtes_DW.obj_cm.dr0[1] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem7_dr0[1];
    raspberrypi_multicore_MPCtes_DW.obj_cm.dr0[2] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem7_dr0[2];
  }

  if (!raspberrypi_multicore_M_isequal
      (raspberrypi_multicore_MPCtes_DW.obj_cm.omega0,
       raspberrypi_multicore_MPCtest_P.MATLABSystem7_omega0)) {
    raspberrypi_multicore_MPCtes_DW.obj_cm.omega0[0] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem7_omega0[0];
    raspberrypi_multicore_MPCtes_DW.obj_cm.omega0[1] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem7_omega0[1];
    raspberrypi_multicore_MPCtes_DW.obj_cm.omega0[2] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem7_omega0[2];
  }

  if (raspberrypi_multicore_MPCtes_DW.obj_cm.desHeight !=
      raspberrypi_multicore_MPCtest_P.MATLABSystem7_desHeight) {
    raspberrypi_multicore_MPCtes_DW.obj_cm.desHeight =
      raspberrypi_multicore_MPCtest_P.MATLABSystem7_desHeight;
  }

  if (!raspberrypi_multicore_M_isequal
      (raspberrypi_multicore_MPCtes_DW.obj_cm.sitaErr_K,
       raspberrypi_multicore_MPCtest_P.MATLABSystem7_sitaErr_K)) {
    raspberrypi_multicore_MPCtes_DW.obj_cm.sitaErr_K[0] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem7_sitaErr_K[0];
    raspberrypi_multicore_MPCtes_DW.obj_cm.sitaErr_K[1] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem7_sitaErr_K[1];
    raspberrypi_multicore_MPCtes_DW.obj_cm.sitaErr_K[2] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem7_sitaErr_K[2];
  }

  if (raspberrypi_multicore_MPCtes_DW.obj_cm.dt !=
      raspberrypi_multicore_MPCtest_P.Ts) {
    raspberrypi_multicore_MPCtes_DW.obj_cm.dt =
      raspberrypi_multicore_MPCtest_P.Ts;
  }

  raspberrypi__SystemCore_step_am(&raspberrypi_multicore_MPCtes_DW.obj_cm,
    raspberrypi_multicore_MPCtest_B.b_varargout_14,
    raspberrypi_multicore_MPCtest_B.b_varargout_15,
    raspberrypi_multicore_MPCtest_B.b_varargout_16,
    raspberrypi_multicore_MPCtest_B.b_varargout_7_m,
    raspberrypi_multicore_MPCtest_B.b_varargout_10,
    raspberrypi_multicore_MPCtest_B.X_FB,
    raspberrypi_multicore_MPCtest_B.b_varargout_12,
    raspberrypi_multicore_MPCtest_B.b_varargout_1_a,
    raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_n,
    raspberrypi_multicore_MPCtest_B.b_varargout_3,
    raspberrypi_multicore_MPCtest_B.b_varargout_8_l,
    raspberrypi_multicore_MPCtest_B.b_varargout_9_o);

  /* MATLABSystem: '<Root>/MATLAB System21' incorporates:
   *  MATLABSystem: '<Root>/MATLAB System15'
   *  SignalConversion generated from: '<Root>/MATLAB System21'
   */
  p = false;
  p_0 = true;
  raspberrypi_multicore_MPCtest_B.b_k = 0;
  exitg1 = false;
  while ((!exitg1) && (raspberrypi_multicore_MPCtest_B.b_k < 9)) {
    if (!(raspberrypi_multicore_MPCtes_DW.obj_g.Inorm[raspberrypi_multicore_MPCtest_B.b_k]
          ==
          raspberrypi_multicore_MPCtest_P.MATLABSystem21_Inorm[raspberrypi_multicore_MPCtest_B.b_k]))
    {
      p_0 = false;
      exitg1 = true;
    } else {
      raspberrypi_multicore_MPCtest_B.b_k++;
    }
  }

  if (p_0) {
    p = true;
  }

  if (!p) {
    memcpy(&raspberrypi_multicore_MPCtes_DW.obj_g.Inorm[0],
           &raspberrypi_multicore_MPCtest_P.MATLABSystem21_Inorm[0], 9U * sizeof
           (real_T));
  }

  if (raspberrypi_multicore_MPCtes_DW.obj_g.m !=
      raspberrypi_multicore_MPCtest_P.MATLABSystem21_m) {
    raspberrypi_multicore_MPCtes_DW.obj_g.m =
      raspberrypi_multicore_MPCtest_P.MATLABSystem21_m;
  }

  if (raspberrypi_multicore_MPCtes_DW.obj_g.Ts !=
      raspberrypi_multicore_MPCtest_P.Ts) {
    raspberrypi_multicore_MPCtes_DW.obj_g.Ts =
      raspberrypi_multicore_MPCtest_P.Ts;
  }

  if (raspberrypi_multicore_MPCtes_DW.obj_g.hIni !=
      raspberrypi_multicore_MPCtest_P.MATLABSystem21_hIni) {
    raspberrypi_multicore_MPCtes_DW.obj_g.hIni =
      raspberrypi_multicore_MPCtest_P.MATLABSystem21_hIni;
  }

  raspberrypi_m_SystemCore_step_a(&raspberrypi_multicore_MPCtes_DW.obj_g,
    raspberrypi_multicore_MPCtest_B.b_varargout_6_e,
    raspberrypi_multicore_MPCtest_B.X_FB,
    raspberrypi_multicore_MPCtest_B.b_varargout_5,
    raspberrypi_multicore_MPCtest_B.b_varargout_1,
    raspberrypi_multicore_MPCtest_B.b_varargout_2_d,
    raspberrypi_multicore_MPCtest_B.xest,
    raspberrypi_multicore_MPCtest_B.b_varargout_6,
    raspberrypi_multicore_MPCtest_B.b_varargout_7,
    raspberrypi_multicore_MPCtest_B.b_varargout_8,
    raspberrypi_multicore_MPCtest_B.b_varargout_9);

  /* MATLAB Function: '<S46>/FixedHorizonOptimizer' */
  memset(&raspberrypi_multicore_MPCtest_B.Bu[0], 0, 1092U * sizeof(real_T));
  memset(&raspberrypi_multicore_MPCtest_B.Bv[0], 0, 637U * sizeof(real_T));
  memset(&raspberrypi_multicore_MPCtest_B.Dv_g[0], 0, 637U * sizeof(real_T));
  for (raspberrypi_multicore_MPCtest_B.b_k = 0;
       raspberrypi_multicore_MPCtest_B.b_k < 169;
       raspberrypi_multicore_MPCtest_B.b_k++) {
    raspberrypi_multicore_MPCtest_B.b_A[raspberrypi_multicore_MPCtest_B.b_k] =
      g[raspberrypi_multicore_MPCtest_B.b_k];
    raspberrypi_multicore_MPCtest_B.b_C_l[raspberrypi_multicore_MPCtest_B.b_k] =
      k[raspberrypi_multicore_MPCtest_B.b_k];
  }

  /* MATLABSystem: '<Root>/MATLAB System21' */
  memset(&raspberrypi_multicore_MPCtest_B.dv9[0], 0, 234U * sizeof(real_T));

  /* MATLAB Function: '<S46>/FixedHorizonOptimizer' */
  for (raspberrypi_multicore_MPCtest_B.b_k = 0;
       raspberrypi_multicore_MPCtest_B.b_k < 12;
       raspberrypi_multicore_MPCtest_B.b_k++) {
    raspberrypi_multicore_MPCtest_B.dv21[raspberrypi_multicore_MPCtest_B.b_k] =
      (real_T)raspberrypi_multicore_MPCtest_B.b_k + 1.0;
  }

  for (raspberrypi_multicore_MPCtest_B.b_k = 0;
       raspberrypi_multicore_MPCtest_B.b_k < 6;
       raspberrypi_multicore_MPCtest_B.b_k++) {
    raspberrypi_multicore_MPCtest_B.dv26[raspberrypi_multicore_MPCtest_B.b_k] =
      (real_T)raspberrypi_multicore_MPCtest_B.b_k + 13.0;
  }

  for (raspberrypi_multicore_MPCtest_B.b_k = 0;
       raspberrypi_multicore_MPCtest_B.b_k < 13;
       raspberrypi_multicore_MPCtest_B.b_k++) {
    raspberrypi_multicore_MPCtest_B.dv16[raspberrypi_multicore_MPCtest_B.b_k] =
      (real_T)raspberrypi_multicore_MPCtest_B.b_k + 1.0;
    raspberrypi_multicore_MPCtest_B.dv17[raspberrypi_multicore_MPCtest_B.b_k] =
      1.0;
  }

  /* MATLABSystem: '<Root>/MATLAB System21' */
  memcpy(&raspberrypi_multicore_MPCtest_B.dv11[0], &tmp[0], 169U * sizeof(real_T));

  /* MATLAB Function: '<S46>/FixedHorizonOptimizer' incorporates:
   *  Constant: '<Root>/Constant2'
   *  MATLABSystem: '<Root>/MATLAB System15'
   *  MATLABSystem: '<Root>/MATLAB System21'
   *  MATLABSystem: '<Root>/MATLAB System7'
   */
  memcpy(&raspberrypi_multicore_MPCtest_B.h[0], &h[0], 416U * sizeof(real_T));
  memcpy(&raspberrypi_multicore_MPCtest_B.l[0], &l[0], 416U * sizeof(real_T));
  raspberrypi_mul_mpc_plantupdate(raspberrypi_multicore_MPCtest_B.b_varargout_1,
    raspberrypi_multicore_MPCtest_B.b_varargout_2_d,
    raspberrypi_multicore_MPCtest_B.dv11, raspberrypi_multicore_MPCtest_B.dv9,
    raspberrypi_multicore_MPCtest_B.b_A, raspberrypi_multicore_MPCtest_B.h,
    raspberrypi_multicore_MPCtest_B.b_C_l, raspberrypi_multicore_MPCtest_B.l,
    raspberrypi_multicore_MPCtest_B.dv21, raspberrypi_multicore_MPCtest_B.dv26,
    raspberrypi_multicore_MPCtest_B.dv16, n,
    raspberrypi_multicore_MPCtest_B.dv17, &raspberrypi_multicore_MPCtest_B.Bu[0],
    &raspberrypi_multicore_MPCtest_B.Bv[0], raspberrypi_multicore_MPCtest_B.d,
    &raspberrypi_multicore_MPCtest_B.Dv_g[0],
    raspberrypi_multicore_MPCtest_B.yseq, raspberrypi_multicore_MPCtest_B.Qk,
    raspberrypi_multicore_MPCtest_B.Rk, raspberrypi_multicore_MPCtest_B.Nk);
  memcpy(&raspberrypi_multicore_MPCtest_B.b_Mlim[0], &o[0], 252U * sizeof(real_T));
  memset(&raspberrypi_multicore_MPCtest_B.b_utarget[0], 0, 72U * sizeof(real_T));
  memcpy(&raspberrypi_multicore_MPCtest_B.xk1[0], &s[0], 13U * sizeof(real_T));
  for (raspberrypi_multicore_MPCtest_B.b_k = 0;
       raspberrypi_multicore_MPCtest_B.b_k < 12;
       raspberrypi_multicore_MPCtest_B.b_k++) {
    raspberrypi_multicore_MPCtest_B.dv21[raspberrypi_multicore_MPCtest_B.b_k] =
      (real_T)raspberrypi_multicore_MPCtest_B.b_k + 1.0;
  }

  for (raspberrypi_multicore_MPCtest_B.b_k = 0;
       raspberrypi_multicore_MPCtest_B.b_k < 6;
       raspberrypi_multicore_MPCtest_B.b_k++) {
    raspberrypi_multicore_MPCtest_B.dv26[raspberrypi_multicore_MPCtest_B.b_k] =
      (real_T)raspberrypi_multicore_MPCtest_B.b_k + 13.0;
  }

  for (raspberrypi_multicore_MPCtest_B.b_k = 0;
       raspberrypi_multicore_MPCtest_B.b_k < 13;
       raspberrypi_multicore_MPCtest_B.b_k++) {
    raspberrypi_multicore_MPCtest_B.dv16[raspberrypi_multicore_MPCtest_B.b_k] =
      1.0;
    raspberrypi_multicore_MPCtest_B.dv17[raspberrypi_multicore_MPCtest_B.b_k] =
      (real_T)raspberrypi_multicore_MPCtest_B.b_k + 1.0;
  }

  raspberry_mpc_updateFromNominal(raspberrypi_multicore_MPCtest_B.b_Mlim, q,
    raspberrypi_multicore_MPCtest_B.b_varargout_6, n, r,
    raspberrypi_multicore_MPCtest_B.dv21, raspberrypi_multicore_MPCtest_B.dv26,
    raspberrypi_multicore_MPCtest_B.b_utarget,
    raspberrypi_multicore_MPCtest_B.b_varargout_7,
    raspberrypi_multicore_MPCtest_B.dv16, s,
    raspberrypi_multicore_MPCtest_B.dv17, raspberrypi_multicore_MPCtest_B.xest,
    raspberrypi_multicore_MPCtest_B.xk1,
    raspberrypi_multicore_MPCtest_B.b_varargout_8,
    raspberrypi_multicore_MPCtest_B.Bv, raspberrypi_multicore_MPCtest_B.b_uoff,
    raspberrypi_multicore_MPCtest_B.b_voff,
    raspberrypi_multicore_MPCtest_B.b_yoff,
    raspberrypi_multicore_MPCtest_B.b_myoff);
  memset(&raspberrypi_multicore_MPCtest_B.vseq[0], 0, 49U * sizeof(real_T));
  for (raspberrypi_multicore_MPCtest_B.i = 0; raspberrypi_multicore_MPCtest_B.i <
       7; raspberrypi_multicore_MPCtest_B.i++) {
    raspberrypi_multicore_MPCtest_B.vseq[raspberrypi_multicore_MPCtest_B.i * 7 +
      6] = 1.0;
  }

  for (raspberrypi_multicore_MPCtest_B.i = 0; raspberrypi_multicore_MPCtest_B.i <
       6; raspberrypi_multicore_MPCtest_B.i++) {
    for (raspberrypi_multicore_MPCtest_B.b_k = 0;
         raspberrypi_multicore_MPCtest_B.b_k < 13;
         raspberrypi_multicore_MPCtest_B.b_k++) {
      raspberrypi_multicore_MPCtest_B.b_varargout_3[raspberrypi_multicore_MPCtest_B.b_k
        + raspberrypi_multicore_MPCtest_B.i * 13] =
        raspberrypi_multicore_MPCtest_B.b_varargout_1_a[6 *
        raspberrypi_multicore_MPCtest_B.b_k + raspberrypi_multicore_MPCtest_B.i]
        - raspberrypi_multicore_MPCtest_B.b_yoff[raspberrypi_multicore_MPCtest_B.b_k];
    }
  }

  for (raspberrypi_multicore_MPCtest_B.i = 0; raspberrypi_multicore_MPCtest_B.i <
       7; raspberrypi_multicore_MPCtest_B.i++) {
    for (raspberrypi_multicore_MPCtest_B.b_k = 0;
         raspberrypi_multicore_MPCtest_B.b_k < 6;
         raspberrypi_multicore_MPCtest_B.b_k++) {
      raspberrypi_multicore_MPCtest_B.vseq[raspberrypi_multicore_MPCtest_B.b_k +
        raspberrypi_multicore_MPCtest_B.i * 7] =
        b_RMDscale[raspberrypi_multicore_MPCtest_B.b_k] *
        raspberrypi_multicore_MPCtest_B.b_varargout_17[raspberrypi_multicore_MPCtest_B.b_k]
        - raspberrypi_multicore_MPCtest_B.b_voff[raspberrypi_multicore_MPCtest_B.b_k];
    }
  }

  raspberrypi_multicore_MPCtest_B.xest[0] =
    raspberrypi_multicore_MPCtest_B.b_varargout_1_idx_0 -
    raspberrypi_multicore_MPCtest_B.xk1[0];
  raspberrypi_multicore_MPCtest_B.xest[3] =
    raspberrypi_multicore_MPCtest_B.b_varargout_3_idx_0 -
    raspberrypi_multicore_MPCtest_B.xk1[3];
  raspberrypi_multicore_MPCtest_B.xest[6] =
    raspberrypi_multicore_MPCtest_B.b_varargout_2_idx_0 -
    raspberrypi_multicore_MPCtest_B.xk1[6];
  raspberrypi_multicore_MPCtest_B.xest[9] =
    raspberrypi_multicore_MPCtest_B.b_varargout_4_idx_0 -
    raspberrypi_multicore_MPCtest_B.xk1[9];
  raspberrypi_multicore_MPCtest_B.xest[1] =
    raspberrypi_multicore_MPCtest_B.b_varargout_1_idx_1 -
    raspberrypi_multicore_MPCtest_B.xk1[1];
  raspberrypi_multicore_MPCtest_B.xest[4] =
    raspberrypi_multicore_MPCtest_B.b_varargout_3_idx_1 -
    raspberrypi_multicore_MPCtest_B.xk1[4];
  raspberrypi_multicore_MPCtest_B.xest[7] =
    raspberrypi_multicore_MPCtest_B.b_varargout_2_idx_1 -
    raspberrypi_multicore_MPCtest_B.xk1[7];
  raspberrypi_multicore_MPCtest_B.xest[10] =
    raspberrypi_multicore_MPCtest_B.b_varargout_4_idx_1 -
    raspberrypi_multicore_MPCtest_B.xk1[10];
  raspberrypi_multicore_MPCtest_B.xest[2] =
    raspberrypi_multicore_MPCtest_B.b_varargout_1_idx_2 -
    raspberrypi_multicore_MPCtest_B.xk1[2];
  raspberrypi_multicore_MPCtest_B.xest[5] =
    raspberrypi_multicore_MPCtest_B.b_varargout_3_idx_2 -
    raspberrypi_multicore_MPCtest_B.xk1[5];
  raspberrypi_multicore_MPCtest_B.xest[8] =
    raspberrypi_multicore_MPCtest_B.b_varargout_2_idx_2 -
    raspberrypi_multicore_MPCtest_B.xk1[8];
  raspberrypi_multicore_MPCtest_B.xest[11] =
    raspberrypi_multicore_MPCtest_B.b_varargout_4_idx_2 -
    raspberrypi_multicore_MPCtest_B.xk1[11];
  raspberrypi_multicore_MPCtest_B.xest[12] =
    raspberrypi_multicore_MPCtest_P.Constant2_Value -
    raspberrypi_multicore_MPCtest_B.xk1[12];

  /* MATLAB Function: '<Root>/MATLAB Function8' incorporates:
   *  MATLABSystem: '<Root>/MATLAB System15'
   */
  p = (raspberrypi_multicore_MPCtest_B.b_varargout_12 > 0.5);

  /* MATLAB Function: '<Root>/online Constraints1' */
  for (raspberrypi_multicore_MPCtest_B.b_k = 0;
       raspberrypi_multicore_MPCtest_B.b_k < 12;
       raspberrypi_multicore_MPCtest_B.b_k++) {
    for (raspberrypi_multicore_MPCtest_B.i = 0;
         raspberrypi_multicore_MPCtest_B.i < 8;
         raspberrypi_multicore_MPCtest_B.i++) {
      raspberrypi_multicore_MPCtest_B.Eb2_tmp_i =
        (raspberrypi_multicore_MPCtest_B.b_k << 3) +
        raspberrypi_multicore_MPCtest_B.i;
      raspberrypi_multicore_MPCtest_B.Eb2_tmp =
        raspberrypi_multicore_MPCtest_B.i + (raspberrypi_multicore_MPCtest_B.b_k
        << 5);
      raspberrypi_multicore_MPCtest_B.Eb2[raspberrypi_multicore_MPCtest_B.Eb2_tmp]
        =
        raspberrypi_multicore_MPCtest_B.Eb2_l[raspberrypi_multicore_MPCtest_B.Eb2_tmp_i];
      raspberrypi_multicore_MPCtest_B.Eb2[raspberrypi_multicore_MPCtest_B.Eb2_tmp
        + 8] =
        raspberrypi_multicore_MPCtest_B.Eb3[raspberrypi_multicore_MPCtest_B.Eb2_tmp_i];
    }

    raspberrypi_multicore_MPCtest_B.Eb2_tmp_i =
      raspberrypi_multicore_MPCtest_B.b_k << 2;
    raspberrypi_multicore_MPCtest_B.Eb2_tmp =
      raspberrypi_multicore_MPCtest_B.b_k << 5;
    raspberrypi_multicore_MPCtest_B.Eb2[raspberrypi_multicore_MPCtest_B.Eb2_tmp
      + 16] =
      raspberrypi_multicore_MPCtest_B.Eb4[raspberrypi_multicore_MPCtest_B.Eb2_tmp_i];
    raspberrypi_multicore_MPCtest_B.Eb2[raspberrypi_multicore_MPCtest_B.Eb2_tmp
      + 20] =
      raspberrypi_multicore_MPCtest_B.Eb5[raspberrypi_multicore_MPCtest_B.Eb2_tmp_i];
    raspberrypi_multicore_MPCtest_B.Eb2[raspberrypi_multicore_MPCtest_B.Eb2_tmp
      + 17] =
      raspberrypi_multicore_MPCtest_B.Eb4[raspberrypi_multicore_MPCtest_B.Eb2_tmp_i
      + 1];
    raspberrypi_multicore_MPCtest_B.Eb2[raspberrypi_multicore_MPCtest_B.Eb2_tmp
      + 21] =
      raspberrypi_multicore_MPCtest_B.Eb5[raspberrypi_multicore_MPCtest_B.Eb2_tmp_i
      + 1];
    raspberrypi_multicore_MPCtest_B.Eb2[raspberrypi_multicore_MPCtest_B.Eb2_tmp
      + 18] =
      raspberrypi_multicore_MPCtest_B.Eb4[raspberrypi_multicore_MPCtest_B.Eb2_tmp_i
      + 2];
    raspberrypi_multicore_MPCtest_B.Eb2[raspberrypi_multicore_MPCtest_B.Eb2_tmp
      + 22] =
      raspberrypi_multicore_MPCtest_B.Eb5[raspberrypi_multicore_MPCtest_B.Eb2_tmp_i
      + 2];
    raspberrypi_multicore_MPCtest_B.Eb2[raspberrypi_multicore_MPCtest_B.Eb2_tmp
      + 19] =
      raspberrypi_multicore_MPCtest_B.Eb4[raspberrypi_multicore_MPCtest_B.Eb2_tmp_i
      + 3];
    raspberrypi_multicore_MPCtest_B.Eb2[raspberrypi_multicore_MPCtest_B.Eb2_tmp
      + 23] =
      raspberrypi_multicore_MPCtest_B.Eb5[raspberrypi_multicore_MPCtest_B.Eb2_tmp_i
      + 3];
    memcpy
      (&raspberrypi_multicore_MPCtest_B.Eb2[raspberrypi_multicore_MPCtest_B.Eb2_tmp
       + 24],
       &raspberrypi_multicore_MPCtest_B.Eb1[raspberrypi_multicore_MPCtest_B.b_k <<
       3], sizeof(real_T) << 3U);
  }

  /* Gain: '<S18>/umin_scale4' */
  for (raspberrypi_multicore_MPCtest_B.b_k = 0;
       raspberrypi_multicore_MPCtest_B.b_k < 384;
       raspberrypi_multicore_MPCtest_B.b_k++) {
    raspberrypi_multicore_MPCtest_B.dv6[raspberrypi_multicore_MPCtest_B.b_k] =
      raspberrypi_multicore_MPCtest_P.umin_scale4_Gain[raspberrypi_multicore_MPCtest_B.b_k]
      * raspberrypi_multicore_MPCtest_B.Eb2[raspberrypi_multicore_MPCtest_B.b_k];
  }

  /* Gain: '<S18>/ymin_scale1' incorporates:
   *  MATLAB Function: '<Root>/online Constraints1'
   */
  for (raspberrypi_multicore_MPCtest_B.b_k = 0;
       raspberrypi_multicore_MPCtest_B.b_k < 416;
       raspberrypi_multicore_MPCtest_B.b_k++) {
    raspberrypi_multicore_MPCtest_B.h[raspberrypi_multicore_MPCtest_B.b_k] =
      raspberrypi_multicore_MPCtest_P.ymin_scale1_Gain[raspberrypi_multicore_MPCtest_B.b_k]
      * 0.0;
  }

  /* Gain: '<S18>/ymin_scale2' incorporates:
   *  MATLAB Function: '<Root>/online Constraints1'
   */
  for (raspberrypi_multicore_MPCtest_B.b_k = 0;
       raspberrypi_multicore_MPCtest_B.b_k < 192;
       raspberrypi_multicore_MPCtest_B.b_k++) {
    raspberrypi_multicore_MPCtest_B.dv10[raspberrypi_multicore_MPCtest_B.b_k] =
      raspberrypi_multicore_MPCtest_P.ymin_scale2_Gain[raspberrypi_multicore_MPCtest_B.b_k]
      * 0.0;
  }

  /* MATLAB Function: '<S46>/FixedHorizonOptimizer' incorporates:
   *  Gain: '<S18>/ext.mv_scale'
   */
  for (raspberrypi_multicore_MPCtest_B.b_k = 0;
       raspberrypi_multicore_MPCtest_B.b_k < 12;
       raspberrypi_multicore_MPCtest_B.b_k++) {
    /* MATLAB Function: '<Root>/MATLAB Function8' incorporates:
     *  Gain: '<S18>/ext.mv_scale'
     *  UnitDelay: '<Root>/Unit Delay2'
     */
    if (p) {
      raspberrypi_multicore_MPCtest_B.b_varargout_1_idx_0 =
        b[raspberrypi_multicore_MPCtest_B.b_k];
    } else {
      raspberrypi_multicore_MPCtest_B.b_varargout_1_idx_0 =
        raspberrypi_multicore_MPCtes_DW.UnitDelay2_DSTATE[raspberrypi_multicore_MPCtest_B.b_k];
    }

    raspberrypi_multicore_MPCtest_B.dv21[raspberrypi_multicore_MPCtest_B.b_k] =
      raspberrypi_multicore_MPCtest_P.extmv_scale_Gain[raspberrypi_multicore_MPCtest_B.b_k]
      * raspberrypi_multicore_MPCtest_B.b_varargout_1_idx_0 -
      raspberrypi_multicore_MPCtest_B.b_uoff[raspberrypi_multicore_MPCtest_B.b_k];
  }

  for (raspberrypi_multicore_MPCtest_B.b_k = 0;
       raspberrypi_multicore_MPCtest_B.b_k < 12;
       raspberrypi_multicore_MPCtest_B.b_k++) {
    raspberrypi_multicore_MPCtest_B.dv22[raspberrypi_multicore_MPCtest_B.b_k] =
      0.010000000000000002;
    raspberrypi_multicore_MPCtest_B.dv23[raspberrypi_multicore_MPCtest_B.b_k] =
      0.0;
  }

  /* MATLAB Function: '<Root>/online Constraints1' */
  raspberrypi_multicore_MPCtest_B.dv15[16] = 10.0;
  raspberrypi_multicore_MPCtest_B.dv15[20] = 10.0;
  raspberrypi_multicore_MPCtest_B.dv15[17] = 10.0;
  raspberrypi_multicore_MPCtest_B.dv15[21] = 10.0;
  raspberrypi_multicore_MPCtest_B.dv15[18] = 10.0;
  raspberrypi_multicore_MPCtest_B.dv15[22] = 10.0;
  raspberrypi_multicore_MPCtest_B.dv15[19] = 10.0;
  raspberrypi_multicore_MPCtest_B.dv15[23] = 10.0;
  for (raspberrypi_multicore_MPCtest_B.b_k = 0;
       raspberrypi_multicore_MPCtest_B.b_k < 8;
       raspberrypi_multicore_MPCtest_B.b_k++) {
    raspberrypi_multicore_MPCtest_B.dv15[raspberrypi_multicore_MPCtest_B.b_k] =
      0.0;
    raspberrypi_multicore_MPCtest_B.dv15[raspberrypi_multicore_MPCtest_B.b_k + 8]
      = 0.0;
    raspberrypi_multicore_MPCtest_B.dv15[raspberrypi_multicore_MPCtest_B.b_k +
      24] =
      raspberrypi_multicore_MPCtest_B.Gb1[raspberrypi_multicore_MPCtest_B.b_k];
  }

  /* Memory: '<S18>/Memory' */
  memcpy(&raspberrypi_multicore_MPCtest_B.bv[0],
         &raspberrypi_multicore_MPCtes_DW.Memory_PreviousInput[0], 252U * sizeof
         (boolean_T));

  /* MATLAB Function: '<S46>/FixedHorizonOptimizer' */
  memcpy(&raspberrypi_multicore_MPCtest_B.y[0], &y[0], 3276U * sizeof(real_T));
  memcpy(&raspberrypi_multicore_MPCtest_B.ab[0], &ab[0], 3024U * sizeof(real_T));
  memcpy(&raspberrypi_multicore_MPCtest_B.bb[0], &bb[0], 12348U * sizeof(real_T));
  memcpy(&raspberrypi_multicore_MPCtest_B.cb[0], &cb[0], 625U * sizeof(real_T));
  memcpy(&raspberrypi_multicore_MPCtest_B.eb[0], &eb[0], 6300U * sizeof(real_T));

  /* Update for Memory: '<S18>/Memory' incorporates:
   *  Gain: '<S18>/umin_scale4'
   *  Gain: '<S18>/ymin_scale1'
   *  Gain: '<S18>/ymin_scale2'
   *  MATLAB Function: '<S46>/FixedHorizonOptimizer'
   *  MATLABSystem: '<Root>/MATLAB System15'
   */
  raspberrypi__mpcblock_optimizer(raspberrypi_multicore_MPCtest_B.b_varargout_3,
    raspberrypi_multicore_MPCtest_B.vseq,
    raspberrypi_multicore_MPCtest_B.b_varargout_12,
    raspberrypi_multicore_MPCtest_B.xest, raspberrypi_multicore_MPCtest_B.dv21,
    raspberrypi_multicore_MPCtest_B.bv, raspberrypi_multicore_MPCtest_B.b_Mlim,
    raspberrypi_multicore_MPCtest_B.y, raspberrypi_multicore_MPCtest_B.ab,
    raspberrypi_multicore_MPCtest_B.bb,
    raspberrypi_multicore_MPCtest_B.b_utarget,
    raspberrypi_multicore_MPCtest_B.b_uoff,
    raspberrypi_multicore_MPCtest_B.b_voff,
    raspberrypi_multicore_MPCtest_B.b_yoff, raspberrypi_multic_enable_value,
    raspberrypi_multicore_MPCtest_B.cb, raspberrypi_multicore_MPCtest_B.eb, fb,
    raspberrypi_multicore_MPCtest_B.dv22, gb,
    raspberrypi_multicore_MPCtest_B.dv23, lb,
    raspberrypi_multicore_MPCtest_B.b_A, raspberrypi_multicore_MPCtest_B.Bu,
    raspberrypi_multicore_MPCtest_B.Bv, raspberrypi_multicore_MPCtest_B.b_C_l,
    raspberrypi_multicore_MPCtest_B.Dv_g, q, raspberrypi_multicore_MPCtest_B.dv6,
    raspberrypi_multicore_MPCtest_B.h, raspberrypi_multicore_MPCtest_B.dv10,
    raspberrypi_multicore_MPCtest_B.dv15,
    raspberrypi_multicore_MPCtes_DW.UnitDelay2_DSTATE,
    raspberrypi_multicore_MPCtest_B.useq,
    &raspberrypi_multicore_MPCtest_B.b_varargout_1_idx_0,
    raspberrypi_multicore_MPCtes_DW.Memory_PreviousInput);

  /* MATLAB Function: '<S46>/FixedHorizonOptimizer' */
  raspberrypi_mpc_computeSequence(raspberrypi_multicore_MPCtest_B.xest,
    raspberrypi_multicore_MPCtest_B.useq, raspberrypi_multicore_MPCtest_B.vseq,
    raspberrypi_multicore_MPCtest_B.b_uoff,
    raspberrypi_multicore_MPCtest_B.b_yoff, raspberrypi_multicore_MPCtest_B.xk1,
    raspberrypi_multicore_MPCtest_B.b_A, raspberrypi_multicore_MPCtest_B.Bu,
    raspberrypi_multicore_MPCtest_B.Bv, raspberrypi_multicore_MPCtest_B.b_C_l,
    raspberrypi_multicore_MPCtest_B.Dv_g, raspberrypi_multicore_MPCtest_B.yseq,
    raspberrypi_multicore_MPCtest_B.xseq);

  /* SignalConversion generated from: '<Root>/To File8' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function6'
   *  MATLABSystem: '<Root>/MATLAB System7'
   */
  for (raspberrypi_multicore_MPCtest_B.b_k = 0;
       raspberrypi_multicore_MPCtest_B.b_k < 13;
       raspberrypi_multicore_MPCtest_B.b_k++) {
    raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_p[raspberrypi_multicore_MPCtest_B.b_k]
      = raspberrypi_multicore_MPCtest_B.b_varargout_1_a[6 *
      raspberrypi_multicore_MPCtest_B.b_k + 1];
    raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_p[raspberrypi_multicore_MPCtest_B.b_k
      + 13] =
      raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_n[raspberrypi_multicore_MPCtest_B.b_k];
  }

  /* End of SignalConversion generated from: '<Root>/To File8' */

  /* ToFile: '<Root>/To File8' */
  if (tid == 1 ) {
    {
      if (!(++raspberrypi_multicore_MPCtes_DW.ToFile8_IWORK.Decimation % 1) &&
          (raspberrypi_multicore_MPCtes_DW.ToFile8_IWORK.Count * (26 + 1)) + 1 <
          100000000 ) {
        FILE *fp = (FILE *)
          raspberrypi_multicore_MPCtes_DW.ToFile8_PWORK.FilePtr;
        if (fp != (NULL)) {
          real_T u[26 + 1];
          raspberrypi_multicore_MPCtes_DW.ToFile8_IWORK.Decimation = 0;
          u[0] = (((raspberrypi_multicore_MPCtes_M->Timing.clockTick1+
                    raspberrypi_multicore_MPCtes_M->Timing.clockTickH1*
                    4294967296.0)) * 0.025);
          u[1] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_p[0];
          u[2] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_p[1];
          u[3] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_p[2];
          u[4] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_p[3];
          u[5] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_p[4];
          u[6] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_p[5];
          u[7] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_p[6];
          u[8] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_p[7];
          u[9] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_p[8];
          u[10] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_p[9];
          u[11] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_p[10];
          u[12] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_p[11];
          u[13] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_p[12];
          u[14] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_p[13];
          u[15] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_p[14];
          u[16] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_p[15];
          u[17] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_p[16];
          u[18] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_p[17];
          u[19] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_p[18];
          u[20] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_p[19];
          u[21] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_p[20];
          u[22] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_p[21];
          u[23] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_p[22];
          u[24] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_p[23];
          u[25] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_p[24];
          u[26] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_p[25];
          if (fwrite(u, sizeof(real_T), 26 + 1, fp) != 26 + 1) {
            rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                              "Error writing to MAT-file multicore_dataRec9.mat");
            return;
          }

          if (((++raspberrypi_multicore_MPCtes_DW.ToFile8_IWORK.Count) * (26 + 1))
              +1 >= 100000000) {
            (void)fprintf(stdout,
                          "*** The ToFile block will stop logging data before\n"
                          "    the simulation has ended, because it has reached\n"
                          "    the maximum number of elements (100000000)\n"
                          "    allowed in MAT-file multicore_dataRec9.mat.\n");
          }
        }
      }
    }
  }

  /* MATLAB Function: '<Root>/MATLAB Function1' */
  for (raspberrypi_multicore_MPCtest_B.b_k = 0;
       raspberrypi_multicore_MPCtest_B.b_k < 13;
       raspberrypi_multicore_MPCtest_B.b_k++) {
    raspberrypi_multicore_MPCtest_B.xk1[raspberrypi_multicore_MPCtest_B.b_k] =
      raspberrypi_multicore_MPCtest_B.xseq[7 *
      raspberrypi_multicore_MPCtest_B.b_k + 1];
  }

  /* End of MATLAB Function: '<Root>/MATLAB Function1' */

  /* ToFile: '<Root>/To File7' */
  if (tid == 1 ) {
    {
      if (!(++raspberrypi_multicore_MPCtes_DW.ToFile7_IWORK.Decimation % 1) &&
          (raspberrypi_multicore_MPCtes_DW.ToFile7_IWORK.Count * (13 + 1)) + 1 <
          100000000 ) {
        FILE *fp = (FILE *)
          raspberrypi_multicore_MPCtes_DW.ToFile7_PWORK.FilePtr;
        if (fp != (NULL)) {
          real_T u[13 + 1];
          raspberrypi_multicore_MPCtes_DW.ToFile7_IWORK.Decimation = 0;
          u[0] = (((raspberrypi_multicore_MPCtes_M->Timing.clockTick1+
                    raspberrypi_multicore_MPCtes_M->Timing.clockTickH1*
                    4294967296.0)) * 0.025);
          u[1] = raspberrypi_multicore_MPCtest_B.xk1[0];
          u[2] = raspberrypi_multicore_MPCtest_B.xk1[1];
          u[3] = raspberrypi_multicore_MPCtest_B.xk1[2];
          u[4] = raspberrypi_multicore_MPCtest_B.xk1[3];
          u[5] = raspberrypi_multicore_MPCtest_B.xk1[4];
          u[6] = raspberrypi_multicore_MPCtest_B.xk1[5];
          u[7] = raspberrypi_multicore_MPCtest_B.xk1[6];
          u[8] = raspberrypi_multicore_MPCtest_B.xk1[7];
          u[9] = raspberrypi_multicore_MPCtest_B.xk1[8];
          u[10] = raspberrypi_multicore_MPCtest_B.xk1[9];
          u[11] = raspberrypi_multicore_MPCtest_B.xk1[10];
          u[12] = raspberrypi_multicore_MPCtest_B.xk1[11];
          u[13] = raspberrypi_multicore_MPCtest_B.xk1[12];
          if (fwrite(u, sizeof(real_T), 13 + 1, fp) != 13 + 1) {
            rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                              "Error writing to MAT-file multicore_dataRec8.mat");
            return;
          }

          if (((++raspberrypi_multicore_MPCtes_DW.ToFile7_IWORK.Count) * (13 + 1))
              +1 >= 100000000) {
            (void)fprintf(stdout,
                          "*** The ToFile block will stop logging data before\n"
                          "    the simulation has ended, because it has reached\n"
                          "    the maximum number of elements (100000000)\n"
                          "    allowed in MAT-file multicore_dataRec8.mat.\n");
          }
        }
      }
    }
  }

  /* Gain: '<S18>/u_scale' */
  for (raspberrypi_multicore_MPCtest_B.b_k = 0;
       raspberrypi_multicore_MPCtest_B.b_k < 12;
       raspberrypi_multicore_MPCtest_B.b_k++) {
    raspberrypi_multicore_MPCtes_DW.UnitDelay2_DSTATE[raspberrypi_multicore_MPCtest_B.b_k]
      *=
      raspberrypi_multicore_MPCtest_P.u_scale_Gain[raspberrypi_multicore_MPCtest_B.b_k];
  }

  /* End of Gain: '<S18>/u_scale' */

  /* Sum: '<Root>/Add' incorporates:
   *  Constant: '<Root>/Constant1'
   *  UnitDelay: '<Root>/Unit Delay1'
   */
  raspberrypi_multicore_MPCtes_DW.UnitDelay1_DSTATE +=
    raspberrypi_multicore_MPCtest_P.Constant1_Value_o;

  /* RateTransition: '<Root>/Rate Transition' incorporates:
   *  DigitalClock: '<Root>/Digital Clock2'
   *  MATLABSystem: '<Root>/MATLAB System15'
   *  MATLABSystem: '<Root>/MATLAB System21'
   *  MATLABSystem: '<Root>/MATLAB System7'
   */
  rtw_pthread_mutex_lock
    (raspberrypi_multicore_MPCtes_DW.RateTransition_d0_SEMAPHORE);
  wrBufIdx = (int8_T)(raspberrypi_multicore_MPCtes_DW.RateTransition_LstBufWR +
                      1);
  if (wrBufIdx == 3) {
    wrBufIdx = 0;
  }

  if (wrBufIdx == raspberrypi_multicore_MPCtes_DW.RateTransition_RDBuf) {
    wrBufIdx++;
    if (wrBufIdx == 3) {
      wrBufIdx = 0;
    }
  }

  rtw_pthread_mutex_unlock
    (raspberrypi_multicore_MPCtes_DW.RateTransition_d0_SEMAPHORE);
  raspberrypi_multicore_MPCtest_B.i = wrBufIdx * 67;
  raspberrypi_multicore_MPCtes_DW.RateTransition_Buf[raspberrypi_multicore_MPCtest_B.i]
    = raspberrypi_multicore_MPCtest_P.Constant3_Value;
  for (raspberrypi_multicore_MPCtest_B.b_k = 0;
       raspberrypi_multicore_MPCtest_B.b_k < 12;
       raspberrypi_multicore_MPCtest_B.b_k++) {
    raspberrypi_multicore_MPCtes_DW.RateTransition_Buf
      [(raspberrypi_multicore_MPCtest_B.b_k + raspberrypi_multicore_MPCtest_B.i)
      + 1] =
      raspberrypi_multicore_MPCtes_DW.UnitDelay2_DSTATE[raspberrypi_multicore_MPCtest_B.b_k];
  }

  for (raspberrypi_multicore_MPCtest_B.b_k = 0;
       raspberrypi_multicore_MPCtest_B.b_k < 13;
       raspberrypi_multicore_MPCtest_B.b_k++) {
    raspberrypi_multicore_MPCtes_DW.RateTransition_Buf
      [(raspberrypi_multicore_MPCtest_B.b_k + raspberrypi_multicore_MPCtest_B.i)
      + 13] =
      raspberrypi_multicore_MPCtest_B.xk1[raspberrypi_multicore_MPCtest_B.b_k];
  }

  for (raspberrypi_multicore_MPCtest_B.b_k = 0;
       raspberrypi_multicore_MPCtest_B.b_k < 9;
       raspberrypi_multicore_MPCtest_B.b_k++) {
    raspberrypi_multicore_MPCtes_DW.RateTransition_Buf
      [(raspberrypi_multicore_MPCtest_B.b_k + raspberrypi_multicore_MPCtest_B.i)
      + 26] =
      raspberrypi_multicore_MPCtest_B.b_varargout_9[raspberrypi_multicore_MPCtest_B.b_k];
  }

  for (raspberrypi_multicore_MPCtest_B.b_k = 0;
       raspberrypi_multicore_MPCtest_B.b_k < 13;
       raspberrypi_multicore_MPCtest_B.b_k++) {
    raspberrypi_multicore_MPCtes_DW.RateTransition_Buf
      [(raspberrypi_multicore_MPCtest_B.b_k + raspberrypi_multicore_MPCtest_B.i)
      + 35] =
      raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_n[raspberrypi_multicore_MPCtest_B.b_k];
  }

  for (raspberrypi_multicore_MPCtest_B.b_k = 0;
       raspberrypi_multicore_MPCtest_B.b_k < 12;
       raspberrypi_multicore_MPCtest_B.b_k++) {
    raspberrypi_multicore_MPCtes_DW.RateTransition_Buf
      [(raspberrypi_multicore_MPCtest_B.b_k + raspberrypi_multicore_MPCtest_B.i)
      + 48] =
      raspberrypi_multicore_MPCtest_B.b_varargout_6_e[raspberrypi_multicore_MPCtest_B.b_k];
  }

  raspberrypi_multicore_MPCtes_DW.RateTransition_Buf[raspberrypi_multicore_MPCtest_B.i
    + 60] = (((raspberrypi_multicore_MPCtes_M->Timing.clockTick1+
               raspberrypi_multicore_MPCtes_M->Timing.clockTickH1* 4294967296.0))
             * 0.025);
  raspberrypi_multicore_MPCtes_DW.RateTransition_Buf[raspberrypi_multicore_MPCtest_B.i
    + 61] = raspberrypi_multicore_MPCtes_DW.UnitDelay1_DSTATE;
  raspberrypi_multicore_MPCtes_DW.RateTransition_Buf[raspberrypi_multicore_MPCtest_B.i
    + 62] = raspberrypi_multicore_MPCtest_B.b_varargout_5[0];
  raspberrypi_multicore_MPCtes_DW.RateTransition_Buf[raspberrypi_multicore_MPCtest_B.i
    + 63] = raspberrypi_multicore_MPCtest_B.b_varargout_5[1];
  raspberrypi_multicore_MPCtes_DW.RateTransition_Buf[raspberrypi_multicore_MPCtest_B.i
    + 64] = raspberrypi_multicore_MPCtest_B.b_varargout_5[2];
  raspberrypi_multicore_MPCtes_DW.RateTransition_Buf[raspberrypi_multicore_MPCtest_B.i
    + 65] = raspberrypi_multicore_MPCtest_B.b_varargout_5[3];
  raspberrypi_multicore_MPCtes_DW.RateTransition_Buf[raspberrypi_multicore_MPCtest_B.i
    + 66] = raspberrypi_multicore_MPCtest_B.b_varargout_13;
  raspberrypi_multicore_MPCtes_DW.RateTransition_LstBufWR = wrBufIdx;

  /* End of RateTransition: '<Root>/Rate Transition' */

  /* SignalConversion generated from: '<Root>/To File1' incorporates:
   *  MATLAB Function: '<S46>/FixedHorizonOptimizer'
   */
  memcpy(&raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_n[0],
         &raspberrypi_multicore_MPCtes_DW.UnitDelay2_DSTATE[0], 12U * sizeof
         (real_T));
  raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_n[12] =
    raspberrypi_multicore_MPCtest_B.b_varargout_1_idx_0;

  /* ToFile: '<Root>/To File1' */
  if (tid == 1 ) {
    {
      if (!(++raspberrypi_multicore_MPCtes_DW.ToFile1_IWORK.Decimation % 1) &&
          (raspberrypi_multicore_MPCtes_DW.ToFile1_IWORK.Count * (13 + 1)) + 1 <
          100000000 ) {
        FILE *fp = (FILE *)
          raspberrypi_multicore_MPCtes_DW.ToFile1_PWORK.FilePtr;
        if (fp != (NULL)) {
          real_T u[13 + 1];
          raspberrypi_multicore_MPCtes_DW.ToFile1_IWORK.Decimation = 0;
          u[0] = (((raspberrypi_multicore_MPCtes_M->Timing.clockTick1+
                    raspberrypi_multicore_MPCtes_M->Timing.clockTickH1*
                    4294967296.0)) * 0.025);
          u[1] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_n[0];
          u[2] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_n[1];
          u[3] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_n[2];
          u[4] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_n[3];
          u[5] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_n[4];
          u[6] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_n[5];
          u[7] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_n[6];
          u[8] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_n[7];
          u[9] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_n[8];
          u[10] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_n[9];
          u[11] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_n[10];
          u[12] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_n[11];
          u[13] = raspberrypi_multicore_MPCtest_B.TmpSignalConversionAtToFi_n[12];
          if (fwrite(u, sizeof(real_T), 13 + 1, fp) != 13 + 1) {
            rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                              "Error writing to MAT-file multicore_dataRec2.mat");
            return;
          }

          if (((++raspberrypi_multicore_MPCtes_DW.ToFile1_IWORK.Count) * (13 + 1))
              +1 >= 100000000) {
            (void)fprintf(stdout,
                          "*** The ToFile block will stop logging data before\n"
                          "    the simulation has ended, because it has reached\n"
                          "    the maximum number of elements (100000000)\n"
                          "    allowed in MAT-file multicore_dataRec2.mat.\n");
          }
        }
      }
    }
  }

  /* ToFile: '<Root>/To File' */
  if (tid == 1 ) {
    {
      if (!(++raspberrypi_multicore_MPCtes_DW.ToFile_IWORK.Decimation % 1) &&
          (raspberrypi_multicore_MPCtes_DW.ToFile_IWORK.Count * (13 + 1)) + 1 <
          100000000 ) {
        FILE *fp = (FILE *) raspberrypi_multicore_MPCtes_DW.ToFile_PWORK.FilePtr;
        if (fp != (NULL)) {
          real_T u[13 + 1];
          raspberrypi_multicore_MPCtes_DW.ToFile_IWORK.Decimation = 0;
          u[0] = (((raspberrypi_multicore_MPCtes_M->Timing.clockTick1+
                    raspberrypi_multicore_MPCtes_M->Timing.clockTickH1*
                    4294967296.0)) * 0.025);
          u[1] = raspberrypi_multicore_MPCtest_B.X_FB[0];
          u[2] = raspberrypi_multicore_MPCtest_B.X_FB[1];
          u[3] = raspberrypi_multicore_MPCtest_B.X_FB[2];
          u[4] = raspberrypi_multicore_MPCtest_B.X_FB[3];
          u[5] = raspberrypi_multicore_MPCtest_B.X_FB[4];
          u[6] = raspberrypi_multicore_MPCtest_B.X_FB[5];
          u[7] = raspberrypi_multicore_MPCtest_B.X_FB[6];
          u[8] = raspberrypi_multicore_MPCtest_B.X_FB[7];
          u[9] = raspberrypi_multicore_MPCtest_B.X_FB[8];
          u[10] = raspberrypi_multicore_MPCtest_B.X_FB[9];
          u[11] = raspberrypi_multicore_MPCtest_B.X_FB[10];
          u[12] = raspberrypi_multicore_MPCtest_B.X_FB[11];
          u[13] = raspberrypi_multicore_MPCtest_B.X_FB[12];
          if (fwrite(u, sizeof(real_T), 13 + 1, fp) != 13 + 1) {
            rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                              "Error writing to MAT-file multicore_dataRec1.mat");
            return;
          }

          if (((++raspberrypi_multicore_MPCtes_DW.ToFile_IWORK.Count) * (13 + 1))
              +1 >= 100000000) {
            (void)fprintf(stdout,
                          "*** The ToFile block will stop logging data before\n"
                          "    the simulation has ended, because it has reached\n"
                          "    the maximum number of elements (100000000)\n"
                          "    allowed in MAT-file multicore_dataRec1.mat.\n");
          }
        }
      }
    }
  }

  rtExtModeUpload(1, (real_T)
                  (((raspberrypi_multicore_MPCtes_M->Timing.clockTick1+
                     raspberrypi_multicore_MPCtes_M->Timing.clockTickH1*
                     4294967296.0)) * 0.025));

  /* Update absolute time */
  /* The "clockTick1" counts the number of times the code of this task has
   * been executed. The resolution of this integer timer is 0.025, which is the step size
   * of the task. Size of "clockTick1" ensures timer will not overflow during the
   * application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick1 and the high bits
   * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
   */
  raspberrypi_multicore_MPCtes_M->Timing.clockTick1++;
  if (!raspberrypi_multicore_MPCtes_M->Timing.clockTick1) {
    raspberrypi_multicore_MPCtes_M->Timing.clockTickH1++;
  }

  /* If subsystem generates rate grouping Output functions,
   * when tid is used in Output function for one rate,
   * all Output functions include tid as a local variable.
   * As result, some Output functions may have unused tid.
   */
  UNUSED_PARAMETER(tid);
}

/* Model wrapper function for compatibility with a static main program */
void raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis_step(int_T tid)
{
  switch (tid) {
   case 0 :
    raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis_step0();
    break;

   case 1 :
    raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis_step1();
    break;

   default :
    break;
  }
}

/* Model initialize function */
void raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)raspberrypi_multicore_MPCtes_M, 0,
                sizeof(RT_MODEL_raspberrypi_multicor_T));
  rtmSetTFinal(raspberrypi_multicore_MPCtes_M, 600.0);
  raspberrypi_multicore_MPCtes_M->Timing.stepSize0 = 0.005;

  /* External mode info */
  raspberrypi_multicore_MPCtes_M->Sizes.checksums[0] = (2142260798U);
  raspberrypi_multicore_MPCtes_M->Sizes.checksums[1] = (4200748146U);
  raspberrypi_multicore_MPCtes_M->Sizes.checksums[2] = (39622389U);
  raspberrypi_multicore_MPCtes_M->Sizes.checksums[3] = (4180910345U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[50];
    raspberrypi_multicore_MPCtes_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    systemRan[1] = &rtAlwaysEnabled;
    systemRan[2] = &rtAlwaysEnabled;
    systemRan[3] = &rtAlwaysEnabled;
    systemRan[4] = &rtAlwaysEnabled;
    systemRan[5] = &rtAlwaysEnabled;
    systemRan[6] = &rtAlwaysEnabled;
    systemRan[7] = &rtAlwaysEnabled;
    systemRan[8] = &rtAlwaysEnabled;
    systemRan[9] = &rtAlwaysEnabled;
    systemRan[10] = &rtAlwaysEnabled;
    systemRan[11] = &rtAlwaysEnabled;
    systemRan[12] = &rtAlwaysEnabled;
    systemRan[13] = &rtAlwaysEnabled;
    systemRan[14] = &rtAlwaysEnabled;
    systemRan[15] = &rtAlwaysEnabled;
    systemRan[16] = &rtAlwaysEnabled;
    systemRan[17] = &rtAlwaysEnabled;
    systemRan[18] = &rtAlwaysEnabled;
    systemRan[19] = &rtAlwaysEnabled;
    systemRan[20] = &rtAlwaysEnabled;
    systemRan[21] = &rtAlwaysEnabled;
    systemRan[22] = &rtAlwaysEnabled;
    systemRan[23] = &rtAlwaysEnabled;
    systemRan[24] = &rtAlwaysEnabled;
    systemRan[25] = &rtAlwaysEnabled;
    systemRan[26] = &rtAlwaysEnabled;
    systemRan[27] = &rtAlwaysEnabled;
    systemRan[28] = &rtAlwaysEnabled;
    systemRan[29] = &rtAlwaysEnabled;
    systemRan[30] = &rtAlwaysEnabled;
    systemRan[31] = &rtAlwaysEnabled;
    systemRan[32] = &rtAlwaysEnabled;
    systemRan[33] = &rtAlwaysEnabled;
    systemRan[34] = &rtAlwaysEnabled;
    systemRan[35] = &rtAlwaysEnabled;
    systemRan[36] = &rtAlwaysEnabled;
    systemRan[37] = &rtAlwaysEnabled;
    systemRan[38] = &rtAlwaysEnabled;
    systemRan[39] = &rtAlwaysEnabled;
    systemRan[40] = &rtAlwaysEnabled;
    systemRan[41] = &rtAlwaysEnabled;
    systemRan[42] = &rtAlwaysEnabled;
    systemRan[43] = &rtAlwaysEnabled;
    systemRan[44] = &rtAlwaysEnabled;
    systemRan[45] = &rtAlwaysEnabled;
    systemRan[46] = &rtAlwaysEnabled;
    systemRan[47] = &rtAlwaysEnabled;
    systemRan[48] = &rtAlwaysEnabled;
    systemRan[49] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(raspberrypi_multicore_MPCtes_M->extModeInfo,
      &raspberrypi_multicore_MPCtes_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(raspberrypi_multicore_MPCtes_M->extModeInfo,
                        raspberrypi_multicore_MPCtes_M->Sizes.checksums);
    rteiSetTPtr(raspberrypi_multicore_MPCtes_M->extModeInfo, rtmGetTPtr
                (raspberrypi_multicore_MPCtes_M));
  }

  /* block I/O */
  (void) memset(((void *) &raspberrypi_multicore_MPCtest_B), 0,
                sizeof(B_raspberrypi_multicore_MPCte_T));

  /* states (dwork) */
  (void) memset((void *)&raspberrypi_multicore_MPCtes_DW, 0,
                sizeof(DW_raspberrypi_multicore_MPCt_T));

  {
    int32_T i;
    int32_T i_0;
    int32_T tmp;
    static const int8_T tmp_0[12] = { 0, 1, 0, 0, -1, 0, 0, 1, 0, 0, -1, 0 };

    static const int8_T tmp_1[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

    static const int8_T tmp_2[361] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

    /* Start for MATLABSystem: '<Root>/MATLAB System' */
    /*  Constructor */
    /*  Support name-value pair arguments when constructing the object. */
    raspberrypi_multicore_MPCtes_DW.obj_e1.matlabCodegenIsDeleted = false;
    raspberrypi_multicore_MPCtes_DW.objisempty_k5 = true;
    raspberrypi_multicore_MPCtes_DW.obj_e1.SampleTime =
      raspberrypi_multicore_MPCtest_P.Ts_DynSim;
    raspberrypi_multicore_MPCtes_DW.obj_e1.isInitialized = 1;

    /*         %% Define output properties */
    /*  Call C-function implementing device initialization */
    joystickSetup();
    raspberrypi_multicore_MPCtes_DW.obj_e1.isSetupComplete = true;

    /* Start for MATLABSystem: '<Root>/MATLAB System3' */
    raspberrypi_multicore_MPCtes_DW.objisempty_c = true;

    /*  Perform one-time calculations, such as computing constants */
    raspberrypi_multicore_MPCtes_DW.obj_a.XOld = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj_a.YOld = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj_a.AOld = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj_a.BOld = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj_a.upOld = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj_a.downOld = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj_a.leftOld = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj_a.rightOld = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj_a.count = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj_a.LeftAxis[0] = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj_a.LeftAxis[1] = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj_a.RightAxis[0] = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj_a.RightAxis[1] = 0.0;
    raspber_MATLABSystem4_Start(&raspberrypi_multicore_MPCtes_DW.MATLABSystem4);
    raspber_MATLABSystem4_Start(&raspberrypi_multicore_MPCtes_DW.MATLABSystem9);

    /* Start for RateTransition: '<Root>/Rate Transition' */
    rtw_pthread_mutex_init
      (&raspberrypi_multicore_MPCtes_DW.RateTransition_d0_SEMAPHORE);

    /* Start for MATLABSystem: '<Root>/MATLAB System10' */
    memset(&raspberrypi_multicore_MPCtes_DW.obj_cy.X_mpc_Old[0], 0, 13U * sizeof
           (real_T));
    memset(&raspberrypi_multicore_MPCtes_DW.obj_cy.refP_Old[0], 0, 13U * sizeof
           (real_T));
    raspberrypi_multicore_MPCtes_DW.obj_cy.LegStateMPC_Old[0] = 1.0;
    raspberrypi_multicore_MPCtes_DW.obj_cy.LegStateMPC_Old[1] = 1.0;
    raspberrypi_multicore_MPCtes_DW.obj_cy.LegStateMPC_Old[2] = 1.0;
    raspberrypi_multicore_MPCtes_DW.obj_cy.LegStateMPC_Old[3] = 1.0;
    raspberrypi_multicore_MPCtes_DW.obj_cy.phiSlow_Old = 0.0;
    raspberrypi_multicore_MPCtes_DW.objisempty_my = true;
    memcpy(&raspberrypi_multicore_MPCtes_DW.obj_cy.Inorm[0],
           &raspberrypi_multicore_MPCtest_P.MATLABSystem10_Inorm[0], 9U * sizeof
           (real_T));
    raspberrypi_multicore_MPCtes_DW.obj_cy.m =
      raspberrypi_multicore_MPCtest_P.MATLABSystem10_m;
    raspberrypi_multicore_MPCtes_DW.obj_cy.hIni =
      raspberrypi_multicore_MPCtest_P.MATLABSystem10_hIni;
    raspberrypi_multicore_MPCtes_DW.obj_cy.lateral_width =
      raspberrypi_multicore_MPCtest_P.MATLABSystem10_lateral_width;
    raspberrypi_multicore_MPCtes_DW.obj_cy.sagetial_width =
      raspberrypi_multicore_MPCtest_P.MATLABSystem10_sagetial_width;
    raspberrypi_multicore_MPCtes_DW.obj_cy.roll_Off =
      raspberrypi_multicore_MPCtest_P.MATLABSystem10_roll_Off;
    raspberrypi_multicore_MPCtes_DW.obj_cy.isInitialized = 1;
    ras_UDP_decoder_raspi_setupImpl(&raspberrypi_multicore_MPCtes_DW.obj_cy);

    /* Start for MATLABSystem: '<Root>/MATLAB System8' */
    raspberrypi_multicore_MPCtes_DW.objisempty_m = true;

    /*  Perform one-time calculations, such as computing constants */
    raspberrypi_multicore_MPCtes_DW.obj_d.ESOld = 0.0;

    /* Start for MATLABSystem: '<Root>/MATLAB System11' */
    raspberrypi_multicore_MPCtes_DW.objisempty_o = true;

    /* obj.Count = 0; */
    raspberrypi_multicore_MPCtes_DW.obj_iw.tCount = 0.0;

    /* Start for MATLABSystem: '<Root>/MATLAB System2' */
    raspberrypi_multicore_MPCtes_DW.obj_f.AB = 44.5;
    raspberrypi_multicore_MPCtes_DW.obj_f.BC = 120.0;
    raspberrypi_multicore_MPCtes_DW.obj_f.CDP = 2.8770007389874528;
    raspberrypi_multicore_MPCtes_DW.obj_f.DP = 139.063;
    raspberrypi_multicore_MPCtes_DW.obj_f.OR = 37.0;
    raspberrypi_multicore_MPCtes_DW.objisempty_g3 = true;
    raspberrypi_multicore_MPCtes_DW.obj_f.lateral_width =
      raspberrypi_multicore_MPCtest_P.MATLABSystem2_lateral_width;
    raspberrypi_multicore_MPCtes_DW.obj_f.sagetial_width =
      raspberrypi_multicore_MPCtest_P.MATLABSystem2_sagetial_width;
    raspberrypi_multicore_MPCtes_DW.obj_f.roll_Off =
      raspberrypi_multicore_MPCtest_P.MATLABSystem2_roll_Off;
    raspberrypi_multicore_MPCtes_DW.obj_f.hIni =
      raspberrypi_multicore_MPCtest_P.MATLABSystem2_hIni;
    raspberrypi_multicore_MPCtes_DW.obj_f.m =
      raspberrypi_multicore_MPCtest_P.MATLABSystem2_m;
    raspberrypi_multicore_MPCtes_DW.obj_f.ks1 =
      raspberrypi_multicore_MPCtest_P.MATLABSystem2_ks1;
    raspberrypi_multicore_MPCtes_DW.obj_f.ks2 =
      raspberrypi_multicore_MPCtest_P.MATLABSystem2_ks2;
    raspberrypi_multicore_MPCtes_DW.obj_f.ks3 =
      raspberrypi_multicore_MPCtest_P.MATLABSystem2_ks3;
    raspberrypi_multicore_MPCtes_DW.obj_f.isInitialized = 1;

    /*  Perform one-time calculations, such as computing constants */
    /*              obj.PendAllnorm=[0.1075,0.1075,-0.1075,-0.1075; */
    /*                  0.0750,-0.0750,0.0750,-0.0750; */
    /*                  0,0,0,0]; */
    raspberrypi_multicore_MPCtes_DW.obj_f.PendAllnorm[0] =
      raspberrypi_multicore_MPCtes_DW.obj_f.sagetial_width / 2.0;
    raspberrypi_multicore_MPCtes_DW.obj_f.PendAllnorm[1] =
      raspberrypi_multicore_MPCtes_DW.obj_f.lateral_width / 2.0;
    raspberrypi_multicore_MPCtes_DW.obj_f.PendAllnorm[2] = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj_f.PendAllnorm[3] =
      raspberrypi_multicore_MPCtes_DW.obj_f.sagetial_width / 2.0;
    raspberrypi_multicore_MPCtes_DW.obj_f.PendAllnorm[4] =
      -raspberrypi_multicore_MPCtes_DW.obj_f.lateral_width / 2.0;
    raspberrypi_multicore_MPCtes_DW.obj_f.PendAllnorm[5] = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj_f.PendAllnorm[6] =
      -raspberrypi_multicore_MPCtes_DW.obj_f.sagetial_width / 2.0;
    raspberrypi_multicore_MPCtes_DW.obj_f.PendAllnorm[7] =
      raspberrypi_multicore_MPCtes_DW.obj_f.lateral_width / 2.0;
    raspberrypi_multicore_MPCtes_DW.obj_f.PendAllnorm[8] = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj_f.PendAllnorm[9] =
      -raspberrypi_multicore_MPCtes_DW.obj_f.sagetial_width / 2.0;
    raspberrypi_multicore_MPCtes_DW.obj_f.PendAllnorm[10] =
      -raspberrypi_multicore_MPCtes_DW.obj_f.lateral_width / 2.0;
    raspberrypi_multicore_MPCtes_DW.obj_f.PendAllnorm[11] = 0.0;

    /* Start for MATLABSystem: '<Root>/MATLAB System17' */
    /* +[0;1;0;0;-1;0;0;1;0;0;-1;0]*obj.roll_Off; */
    raspberrypi_multicore_MPCtes_DW.objisempty_ci = true;
    raspberrypi_multicore_MPCtes_DW.obj_l.r0[0] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_r0[0];
    raspberrypi_multicore_MPCtes_DW.obj_l.theta0[0] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_theta0[0];
    raspberrypi_multicore_MPCtes_DW.obj_l.dr0[0] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_dr0[0];
    raspberrypi_multicore_MPCtes_DW.obj_l.omega0[0] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_omega0[0];
    raspberrypi_multicore_MPCtes_DW.obj_l.r0[1] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_r0[1];
    raspberrypi_multicore_MPCtes_DW.obj_l.theta0[1] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_theta0[1];
    raspberrypi_multicore_MPCtes_DW.obj_l.dr0[1] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_dr0[1];
    raspberrypi_multicore_MPCtes_DW.obj_l.omega0[1] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_omega0[1];
    raspberrypi_multicore_MPCtes_DW.obj_l.r0[2] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_r0[2];
    raspberrypi_multicore_MPCtes_DW.obj_l.theta0[2] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_theta0[2];
    raspberrypi_multicore_MPCtes_DW.obj_l.dr0[2] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_dr0[2];
    raspberrypi_multicore_MPCtes_DW.obj_l.omega0[2] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_omega0[2];
    raspberrypi_multicore_MPCtes_DW.obj_l.lateral_width =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_lateral_width;
    raspberrypi_multicore_MPCtes_DW.obj_l.sagetial_width =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_sagetial_width;
    raspberrypi_multicore_MPCtes_DW.obj_l.roll_Off =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_roll_Off;
    raspberrypi_multicore_MPCtes_DW.obj_l.m =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_m;
    raspberrypi_multicore_MPCtes_DW.obj_l.kx =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_kx;
    raspberrypi_multicore_MPCtes_DW.obj_l.ky =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_ky;
    raspberrypi_multicore_MPCtes_DW.obj_l.kRz =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_kRz;
    raspberrypi_multicore_MPCtes_DW.obj_l.T =
      raspberrypi_multicore_MPCtest_P.T_gait;
    raspberrypi_multicore_MPCtes_DW.obj_l.StepH =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_StepH;
    raspberrypi_multicore_MPCtes_DW.obj_l.SampleTime =
      raspberrypi_multicore_MPCtest_P.Ts_DynSim;
    raspberrypi_multicore_MPCtes_DW.obj_l.OffsetTime =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_OffsetTime;
    raspberrypi_multicore_MPCtes_DW.obj_l.TickTime =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_TickTime;
    raspberrypi_multicore_MPCtes_DW.obj_l.startPhase =
      raspberrypi_multicore_MPCtest_P.MATLABSystem17_startPhase;
    raspberrypi_multicore_MPCtes_DW.obj_l.isInitialized = 1;

    /*              obj.PendAllnorm=[0.1075,0.1075,-0.1075,-0.1075; */
    /*                  0.0750,-0.0750,0.0750,-0.0750; */
    /*                  0,0,0,0]; */
    /*  norminal foot position in the world coordinate */
    raspberrypi_multicore_MPCtest_B.yW =
      -raspberrypi_multicore_MPCtes_DW.obj_l.r0[2];

    /*  norminal foot position in the leg coordinate */
    for (i = 0; i < 4; i++) {
      i_0 = tmp_0[3 * i];
      raspberrypi_multicore_MPCtes_DW.obj_l.pLnorm[3 * i] = (real_T)i_0 *
        raspberrypi_multicore_MPCtes_DW.obj_l.roll_Off;
      raspberrypi_multicore_MPCtes_DW.obj_l.PendAllLocalOld[3 * i] = (real_T)i_0
        * raspberrypi_multicore_MPCtes_DW.obj_l.roll_Off;
      tmp = 3 * i + 1;
      i_0 = tmp_0[tmp];
      raspberrypi_multicore_MPCtes_DW.obj_l.pLnorm[tmp] = (real_T)i_0 *
        raspberrypi_multicore_MPCtes_DW.obj_l.roll_Off;
      raspberrypi_multicore_MPCtes_DW.obj_l.PendAllLocalOld[tmp] = (real_T)i_0 *
        raspberrypi_multicore_MPCtes_DW.obj_l.roll_Off;
      tmp = 3 * i + 2;
      i_0 = tmp_0[tmp];
      raspberrypi_multicore_MPCtes_DW.obj_l.pLnorm[tmp] = (real_T)i_0 *
        raspberrypi_multicore_MPCtes_DW.obj_l.roll_Off +
        raspberrypi_multicore_MPCtest_B.yW;
      raspberrypi_multicore_MPCtes_DW.obj_l.PendAllLocalOld[tmp] = (real_T)i_0 *
        raspberrypi_multicore_MPCtes_DW.obj_l.roll_Off +
        raspberrypi_multicore_MPCtest_B.yW;
    }

    memcpy(&raspberrypi_multicore_MPCtes_DW.obj_l.pL_LS[0],
           &raspberrypi_multicore_MPCtes_DW.obj_l.PendAllLocalOld[0], 12U *
           sizeof(real_T));
    raspberrypi_multicore_MPCtes_DW.obj_l.MPC_legStateOld[0] = 1.0;
    raspberrypi_multicore_MPCtes_DW.obj_l.MPC_legStateOld[1] = 1.0;
    raspberrypi_multicore_MPCtes_DW.obj_l.MPC_legStateOld[2] = 1.0;
    raspberrypi_multicore_MPCtes_DW.obj_l.MPC_legStateOld[3] = 1.0;
    raspberrypi_multicore_MPCtes_DW.obj_l.interpol_Count = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj_l.pArray_L_Adm_Old[0] = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj_l.pArray_L_Adm_Old[1] =
      raspberrypi_multicore_MPCtes_DW.obj_l.roll_Off;
    raspberrypi_multicore_MPCtes_DW.obj_l.pArray_L_Adm_Old[2] =
      -raspberrypi_multicore_MPCtes_DW.obj_l.r0[2];
    raspberrypi_multicore_MPCtes_DW.obj_l.pArray_L_Adm_Old[3] = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj_l.pArray_L_Adm_Old[4] =
      -raspberrypi_multicore_MPCtes_DW.obj_l.roll_Off;
    raspberrypi_multicore_MPCtes_DW.obj_l.pArray_L_Adm_Old[5] =
      -raspberrypi_multicore_MPCtes_DW.obj_l.r0[2];
    raspberrypi_multicore_MPCtes_DW.obj_l.pArray_L_Adm_Old[6] = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj_l.pArray_L_Adm_Old[7] =
      raspberrypi_multicore_MPCtes_DW.obj_l.roll_Off;
    raspberrypi_multicore_MPCtes_DW.obj_l.pArray_L_Adm_Old[8] =
      -raspberrypi_multicore_MPCtes_DW.obj_l.r0[2];
    raspberrypi_multicore_MPCtes_DW.obj_l.pArray_L_Adm_Old[9] = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj_l.pArray_L_Adm_Old[10] =
      -raspberrypi_multicore_MPCtes_DW.obj_l.roll_Off;
    raspberrypi_multicore_MPCtes_DW.obj_l.pArray_L_Adm_Old[11] =
      -raspberrypi_multicore_MPCtes_DW.obj_l.r0[2];
    raspberrypi_multicore_MPCtes_DW.obj_l.pArray_L_Adm_Now[0] = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj_l.pArray_L_Adm_Now[1] =
      raspberrypi_multicore_MPCtes_DW.obj_l.roll_Off;
    raspberrypi_multicore_MPCtes_DW.obj_l.pArray_L_Adm_Now[2] =
      -raspberrypi_multicore_MPCtes_DW.obj_l.r0[2];
    raspberrypi_multicore_MPCtes_DW.obj_l.pArray_L_Adm_Now[3] = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj_l.pArray_L_Adm_Now[4] =
      -raspberrypi_multicore_MPCtes_DW.obj_l.roll_Off;
    raspberrypi_multicore_MPCtes_DW.obj_l.pArray_L_Adm_Now[5] =
      -raspberrypi_multicore_MPCtes_DW.obj_l.r0[2];
    raspberrypi_multicore_MPCtes_DW.obj_l.pArray_L_Adm_Now[6] = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj_l.pArray_L_Adm_Now[7] =
      raspberrypi_multicore_MPCtes_DW.obj_l.roll_Off;
    raspberrypi_multicore_MPCtes_DW.obj_l.pArray_L_Adm_Now[8] =
      -raspberrypi_multicore_MPCtes_DW.obj_l.r0[2];
    raspberrypi_multicore_MPCtes_DW.obj_l.pArray_L_Adm_Now[9] = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj_l.pArray_L_Adm_Now[10] =
      -raspberrypi_multicore_MPCtes_DW.obj_l.roll_Off;
    raspberrypi_multicore_MPCtes_DW.obj_l.pArray_L_Adm_Now[11] =
      -raspberrypi_multicore_MPCtes_DW.obj_l.r0[2];
    raspberrypi_multicore_MPCtes_DW.obj_l.MPC_Count_Old = 0.0;
    memset(&raspberrypi_multicore_MPCtes_DW.obj_l.vNowN[0], 0, 60U * sizeof
           (real_T));

    /* End of Start for MATLABSystem: '<Root>/MATLAB System17' */

    /* Start for MATLABSystem: '<Root>/MATLAB System12' */
    raspberrypi_multicore_MPCtes_DW.objisempty_k1 = true;
    raspberrypi_multicore_MPCtes_DW.obj_b0.pArray_L_Old[0] = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj_b0.pArray_L_Old[1] = 37.0;
    raspberrypi_multicore_MPCtes_DW.obj_b0.pArray_L_Old[2] = -190.0;
    raspberrypi_multicore_MPCtes_DW.obj_b0.pArray_L_Old[3] = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj_b0.pArray_L_Old[4] = -37.0;
    raspberrypi_multicore_MPCtes_DW.obj_b0.pArray_L_Old[5] = -190.0;
    raspberrypi_multicore_MPCtes_DW.obj_b0.pArray_L_Old[6] = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj_b0.pArray_L_Old[7] = 37.0;
    raspberrypi_multicore_MPCtes_DW.obj_b0.pArray_L_Old[8] = -190.0;
    raspberrypi_multicore_MPCtes_DW.obj_b0.pArray_L_Old[9] = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj_b0.pArray_L_Old[10] = -37.0;
    raspberrypi_multicore_MPCtes_DW.obj_b0.pArray_L_Old[11] = -190.0;

    /* Start for MATLABSystem: '<Root>/MATLAB System14' */
    raspberrypi_multicore_MPC_IK_IK(&raspberrypi_multicore_MPCtes_DW.obj_k);
    raspberrypi_multicore_MPCtes_DW.objisempty_e = true;
    raspberrypi_multicore_MPCtes_DW.obj_k.LF_Off[0] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem14_LF_Off[0];
    raspberrypi_multicore_MPCtes_DW.obj_k.LF_Off[1] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem14_LF_Off[1];
    raspberrypi_multicore_MPCtes_DW.obj_k.LF_Off[2] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem14_LF_Off[2];
    raspberrypi_multicore_MPCtes_DW.obj_k.RF_Off[0] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem14_RF_Off[0];
    raspberrypi_multicore_MPCtes_DW.obj_k.RF_Off[1] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem14_RF_Off[1];
    raspberrypi_multicore_MPCtes_DW.obj_k.RF_Off[2] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem14_RF_Off[2];
    raspberrypi_multicore_MPCtes_DW.obj_k.LH_Off[0] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem14_LH_Off[0];
    raspberrypi_multicore_MPCtes_DW.obj_k.LH_Off[1] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem14_LH_Off[1];
    raspberrypi_multicore_MPCtes_DW.obj_k.LH_Off[2] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem14_LH_Off[2];
    raspberrypi_multicore_MPCtes_DW.obj_k.RH_Off[0] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem14_RH_Off[0];
    raspberrypi_multicore_MPCtes_DW.obj_k.RH_Off[1] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem14_RH_Off[1];
    raspberrypi_multicore_MPCtes_DW.obj_k.RH_Off[2] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem14_RH_Off[2];
    raspberrypi_multicore_MPCtes_DW.obj_k.isInitialized = 1;

    /* Start for MATLABSystem: '<Root>/MATLAB System16' */
    /*  Perform one-time calculations, such as computing constants */
    raspberrypi_multicore_MPCtes_DW.objisempty_m3 = true;
    raspberrypi_multicore_MPCtes_DW.obj_p.cons_Vel =
      raspberrypi_multicore_MPCtest_P.MATLABSystem16_cons_Vel;
    raspberrypi_multicore_MPCtes_DW.obj_p.dt =
      raspberrypi_multicore_MPCtest_P.Ts_DynSim;
    raspberrypi_multicore_MPCtes_DW.obj_p.isInitialized = 1;

    /*  Perform one-time calculations, such as computing constants */
    memset(&raspberrypi_multicore_MPCtes_DW.obj_p.PosDesOld[0], 0, 12U * sizeof
           (real_T));
    raspberrypi_multicore_MPCtes_DW.obj_p.Ini_count = 0.0;

    /* Start for MATLABSystem: '<S14>/SPI Master Transfer' */
    raspberrypi_multicore_MPCtes_DW.obj_h.isInitialized = 0;
    raspberrypi_multicore_MPCtes_DW.obj_h.matlabCodegenIsDeleted = false;
    raspberrypi_multicore_MPCtes_DW.objisempty_g = true;
    raspberrypi_mu_SystemCore_setup(&raspberrypi_multicore_MPCtes_DW.obj_h);

    /* Start for MATLABSystem: '<Root>/MATLAB System1' */
    for (i = 0; i < 9; i++) {
      raspberrypi_multicore_MPCtes_DW.obj.Roff[i] = tmp_1[i];
    }

    raspberrypi_multicore_MPCtes_DW.obj.accOff[0] = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj.sitaOld[0] = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj.accOff[1] = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj.sitaOld[1] = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj.accOff[2] = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj.sitaOld[2] = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj.yawOff = 0.0;
    memset(&raspberrypi_multicore_MPCtes_DW.obj.accStore[0], 0, 1200U * sizeof
           (real_T));
    raspberrypi_multicore_MPCtes_DW.objisempty_iw = true;

    /* End of Start for MATLABSystem: '<Root>/MATLAB System1' */

    /* Start for MATLABSystem: '<Root>/MATLAB System13' */
    /*  Perform one-time calculations, such as computing constants */
    raspberrypi_multicore_MPCtes_DW.obj_m.BC = 120.0;
    raspberrypi_multicore_MPCtes_DW.obj_m.CDP = 2.8770007389874528;
    raspberrypi_multicore_MPCtes_DW.obj_m.DP = 139.063;
    raspberrypi_multicore_MPCtes_DW.obj_m.OR = 37.0;
    raspberrypi_multicore_MPCtes_DW.objisempty_h = true;

    /* Start for MATLABSystem: '<S52>/Digital Read' */
    /*  Perform one-time calculations, such as computing constants */
    raspberrypi_multicore_MPCtes_DW.obj_ls.matlabCodegenIsDeleted = false;
    raspberrypi_multicore_MPCtes_DW.objisempty_i = true;
    raspberrypi_multicore_MPCtes_DW.obj_ls.SampleTime =
      raspberrypi_multicore_MPCtest_P.DigitalRead_SampleTime;
    raspberrypi_multicore_MPCtes_DW.obj_ls.isInitialized = 1;
    EXT_GPIO_init(24U, false);
    raspberrypi_multicore_MPCtes_DW.obj_ls.isSetupComplete = true;

    /* Start for MATLABSystem: '<S53>/Digital Read' */
    raspberrypi_multicore_MPCtes_DW.obj_cu.matlabCodegenIsDeleted = false;
    raspberrypi_multicore_MPCtes_DW.objisempty_a = true;
    raspberrypi_multicore_MPCtes_DW.obj_cu.SampleTime =
      raspberrypi_multicore_MPCtest_P.DigitalRead_SampleTime_f;
    raspberrypi_multicore_MPCtes_DW.obj_cu.isInitialized = 1;
    EXT_GPIO_init(23U, false);
    raspberrypi_multicore_MPCtes_DW.obj_cu.isSetupComplete = true;

    /* Start for MATLABSystem: '<S54>/Digital Read' */
    raspberrypi_multicore_MPCtes_DW.obj_i.matlabCodegenIsDeleted = false;
    raspberrypi_multicore_MPCtes_DW.objisempty_d = true;
    raspberrypi_multicore_MPCtes_DW.obj_i.SampleTime =
      raspberrypi_multicore_MPCtest_P.DigitalRead_SampleTime_n;
    raspberrypi_multicore_MPCtes_DW.obj_i.isInitialized = 1;
    EXT_GPIO_init(18U, false);
    raspberrypi_multicore_MPCtes_DW.obj_i.isSetupComplete = true;

    /* Start for MATLABSystem: '<S55>/Digital Read' */
    raspberrypi_multicore_MPCtes_DW.obj_fv.matlabCodegenIsDeleted = false;
    raspberrypi_multicore_MPCtes_DW.objisempty = true;
    raspberrypi_multicore_MPCtes_DW.obj_fv.SampleTime =
      raspberrypi_multicore_MPCtest_P.DigitalRead_SampleTime_p;
    raspberrypi_multicore_MPCtes_DW.obj_fv.isInitialized = 1;
    EXT_GPIO_init(4U, false);
    raspberrypi_multicore_MPCtes_DW.obj_fv.isSetupComplete = true;

    /* Start for MATLABSystem: '<S15>/MATLAB System6' */
    raspberrypi_multicore_MPCtes_DW.obj_lj.SWOld[0] = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj_lj.SWOld[1] = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj_lj.SWOld[2] = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj_lj.SWOld[3] = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj_lj.count = 0.0;
    raspberrypi_multicore_MPCtes_DW.objisempty_p = true;

    /* Start for MATLABSystem: '<S15>/MATLAB System' */
    /*  Perform one-time calculations, such as computing constants */
    raspberrypi_multicore_MPCtes_DW.obj_b.yWidth = 0.097;
    raspberrypi_multicore_MPCtes_DW.obj_b.xWidth = 0.21080000000000002;

    /* Start for MATLABSystem: '<S15>/MATLAB System6' */
    memset(&raspberrypi_multicore_MPCtes_DW.obj_lj.SPOld[0], 0, 12U * sizeof
           (real_T));

    /* Start for MATLABSystem: '<S15>/MATLAB System' */
    memset(&raspberrypi_multicore_MPCtes_DW.obj_b.pArrayOld[0], 0, 12U * sizeof
           (real_T));
    memset(&raspberrypi_multicore_MPCtes_DW.obj_b.vCoMRec[0], 0, 63U * sizeof
           (real_T));
    for (i = 0; i < 6; i++) {
      raspberrypi_multicore_MPCtes_DW.obj_b.ymOld[i] = 0.0;
    }

    raspberrypi_multicore_MPCtes_DW.objisempty_k = true;

    /*  Perform one-time calculations, such as computing constants */
    raspberrypi_multicore_MPCtes_DW.obj_b.iniCount = 0.0;

    /* Start for MATLABSystem: '<S15>/MATLAB System3' */
    raspberrypi_multicore_MPCtes_DW.objisempty_i5 = true;
    raspberrypi_multicore_MPCtes_DW.obj_c.Ts =
      raspberrypi_multicore_MPCtest_P.Ts_DynSim;
    raspberrypi_multicore_MPCtes_DW.obj_c.isInitialized = 1;
    KalmanFilter_DIY_Offrm_setupImp(&raspberrypi_multicore_MPCtes_DW.obj_c);

    /* Start for MATLABSystem: '<Root>/MATLAB System18' */
    raspberrypi_multicore_MPCtes_DW.obj_kb.LegState_Old[0] = 1.0;
    raspberrypi_multicore_MPCtes_DW.obj_kb.LegState_Old[1] = 1.0;
    raspberrypi_multicore_MPCtes_DW.obj_kb.LegState_Old[2] = 1.0;
    raspberrypi_multicore_MPCtes_DW.obj_kb.LegState_Old[3] = 1.0;
    raspberrypi_multicore_MPCtes_DW.objisempty_ab = true;
    raspberrypi_multicore_MPCtes_DW.obj_kb.lateral_width =
      raspberrypi_multicore_MPCtest_P.MATLABSystem18_lateral_width;
    raspberrypi_multicore_MPCtes_DW.obj_kb.sagetial_width =
      raspberrypi_multicore_MPCtest_P.MATLABSystem18_sagetial_width;
    raspberrypi_multicore_MPCtes_DW.obj_kb.roll_Off =
      raspberrypi_multicore_MPCtest_P.MATLABSystem18_roll_Off;
    raspberrypi_multicore_MPCtes_DW.obj_kb.isInitialized = 1;

    /*  Perform one-time calculations, such as computing constants */
    /*              obj.PendAllnorm=[0.1075,0.1075,-0.1075,-0.1075; */
    /*                  0.0750,-0.0750,0.0750,-0.0750; */
    /*                  0,0,0,0]; */
    raspberrypi_multicore_MPCtes_DW.obj_kb.pWnorm[0] =
      raspberrypi_multicore_MPCtes_DW.obj_kb.sagetial_width / 2.0;
    raspberrypi_multicore_MPCtes_DW.obj_kb.pWnorm[1] =
      raspberrypi_multicore_MPCtes_DW.obj_kb.lateral_width / 2.0;
    raspberrypi_multicore_MPCtes_DW.obj_kb.pWnorm[2] = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj_kb.pWnorm[3] =
      raspberrypi_multicore_MPCtes_DW.obj_kb.sagetial_width / 2.0;
    raspberrypi_multicore_MPCtes_DW.obj_kb.pWnorm[4] =
      -raspberrypi_multicore_MPCtes_DW.obj_kb.lateral_width / 2.0;
    raspberrypi_multicore_MPCtes_DW.obj_kb.pWnorm[5] = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj_kb.pWnorm[6] =
      -raspberrypi_multicore_MPCtes_DW.obj_kb.sagetial_width / 2.0;
    raspberrypi_multicore_MPCtes_DW.obj_kb.pWnorm[7] =
      raspberrypi_multicore_MPCtes_DW.obj_kb.lateral_width / 2.0;
    raspberrypi_multicore_MPCtes_DW.obj_kb.pWnorm[8] = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj_kb.pWnorm[9] =
      -raspberrypi_multicore_MPCtes_DW.obj_kb.sagetial_width / 2.0;
    raspberrypi_multicore_MPCtes_DW.obj_kb.pWnorm[10] =
      -raspberrypi_multicore_MPCtes_DW.obj_kb.lateral_width / 2.0;
    raspberrypi_multicore_MPCtes_DW.obj_kb.pWnorm[11] = 0.0;
    for (i = 0; i < 12; i++) {
      raspberrypi_multicore_MPCtes_DW.obj_kb.pWnorm[i] += (real_T)tmp_0[i] *
        raspberrypi_multicore_MPCtes_DW.obj_kb.roll_Off;
      raspberrypi_multicore_MPCtes_DW.obj_kb.pW_LS[i] =
        raspberrypi_multicore_MPCtes_DW.obj_kb.pWnorm[i];
    }

    /* End of Start for MATLABSystem: '<Root>/MATLAB System18' */

    /* Start for MATLABSystem: '<Root>/MATLAB System20' */
    raspberrypi_multicore_MPCtes_DW.objisempty_do = true;
    raspberrypi_multicore_MPCtes_DW.obj_n.Ts =
      raspberrypi_multicore_MPCtest_P.Ts_DynSim;
    raspberrypi_multicore_MPCtes_DW.obj_n.m =
      raspberrypi_multicore_MPCtest_P.MATLABSystem20_m;
    memcpy(&raspberrypi_multicore_MPCtes_DW.obj_n.Inorm[0],
           &raspberrypi_multicore_MPCtest_P.MATLABSystem20_Inorm[0], 9U * sizeof
           (real_T));
    raspberrypi_multicore_MPCtes_DW.obj_n.isInitialized = 1;

    /*  Perform one-time calculations, such as computing constants */
    raspberrypi_multicore_MPCtes_DW.obj_n.count = 0.0;
    memset(&raspberrypi_multicore_MPCtes_DW.obj_n.XOld[0], 0, 19U * sizeof
           (real_T));
    for (i = 0; i < 361; i++) {
      raspberrypi_multicore_MPCtes_DW.obj_n.POld[i] = (real_T)tmp_2[i] * 1000.0;
      raspberrypi_multicore_MPCtes_DW.obj_n.G[i] = 0.0;
    }

    for (i = 0; i < 19; i++) {
      raspberrypi_multicore_MPCtes_DW.obj_n.G[i + 19 * i] = 1.0;
    }

    for (i = 0; i < 361; i++) {
      raspberrypi_multicore_MPCtes_DW.obj_n.P0[i] = (real_T)tmp_2[i] * 1000.0;
    }

    /* End of Start for MATLABSystem: '<Root>/MATLAB System20' */

    /* Start for ToFile: '<Root>/To File11' */
    {
      FILE *fp = (NULL);
      char fileName[509] = "multicore_dataRecEstB.mat";
      if ((fp = fopen(fileName, "wb")) == (NULL)) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error creating .mat file multicore_dataRecEstB.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 17 + 1, 0, "rt_pArrayB_EN_SW")) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error writing mat file header to file multicore_dataRecEstB.mat");
        return;
      }

      raspberrypi_multicore_MPCtes_DW.ToFile11_IWORK.Count = 0;
      raspberrypi_multicore_MPCtes_DW.ToFile11_IWORK.Decimation = -1;
      raspberrypi_multicore_MPCtes_DW.ToFile11_PWORK.FilePtr = fp;
    }

    /* Start for ToFile: '<Root>/To File6' */
    {
      FILE *fp = (NULL);
      char fileName[509] = "multicore_dataRec7.mat";
      if ((fp = fopen(fileName, "wb")) == (NULL)) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error creating .mat file multicore_dataRec7.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 16 + 1, 0, "rt_estSPLeg_estSP")) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error writing mat file header to file multicore_dataRec7.mat");
        return;
      }

      raspberrypi_multicore_MPCtes_DW.ToFile6_IWORK.Count = 0;
      raspberrypi_multicore_MPCtes_DW.ToFile6_IWORK.Decimation = -1;
      raspberrypi_multicore_MPCtes_DW.ToFile6_PWORK.FilePtr = fp;
    }

    /* Start for RateTransition: '<Root>/Rate Transition1' */
    rtw_pthread_mutex_init
      (&raspberrypi_multicore_MPCtes_DW.RateTransition1_d0_SEMAPHORE);

    /* Start for ToFile: '<Root>/To File3' */
    {
      FILE *fp = (NULL);
      char fileName[509] = "multicore_dataRec4.mat";
      if ((fp = fopen(fileName, "wb")) == (NULL)) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error creating .mat file multicore_dataRec4.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 12 + 1, 0, "rt_x_FB_fast")) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error writing mat file header to file multicore_dataRec4.mat");
        return;
      }

      raspberrypi_multicore_MPCtes_DW.ToFile3_IWORK.Count = 0;
      raspberrypi_multicore_MPCtes_DW.ToFile3_IWORK.Decimation = -1;
      raspberrypi_multicore_MPCtes_DW.ToFile3_PWORK.FilePtr = fp;
    }

    /* Start for ToFile: '<Root>/To File13' */
    {
      FILE *fp = (NULL);
      char fileName[509] = "multicore_dataRec12.mat";
      if ((fp = fopen(fileName, "wb")) == (NULL)) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error creating .mat file multicore_dataRec12.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 12 + 1, 0, "rt_pLFKfast")) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error writing mat file header to file multicore_dataRec12.mat");
        return;
      }

      raspberrypi_multicore_MPCtes_DW.ToFile13_IWORK.Count = 0;
      raspberrypi_multicore_MPCtes_DW.ToFile13_IWORK.Decimation = -1;
      raspberrypi_multicore_MPCtes_DW.ToFile13_PWORK.FilePtr = fp;
    }

    /* Start for ToFile: '<Root>/To File4' */
    {
      FILE *fp = (NULL);
      char fileName[509] = "multicore_dataRec5.mat";
      if ((fp = fopen(fileName, "wb")) == (NULL)) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error creating .mat file multicore_dataRec5.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 12 + 1, 0, "rt_PendAllLocal")) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error writing mat file header to file multicore_dataRec5.mat");
        return;
      }

      raspberrypi_multicore_MPCtes_DW.ToFile4_IWORK.Count = 0;
      raspberrypi_multicore_MPCtes_DW.ToFile4_IWORK.Decimation = -1;
      raspberrypi_multicore_MPCtes_DW.ToFile4_PWORK.FilePtr = fp;
    }

    /* Start for ToFile: '<Root>/To File16' */
    {
      FILE *fp = (NULL);
      char fileName[509] = "multicore_dataRec15.mat";
      if ((fp = fopen(fileName, "wb")) == (NULL)) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error creating .mat file multicore_dataRec15.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 12 + 1, 0, "rt_pLnor_fast")) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error writing mat file header to file multicore_dataRec15.mat");
        return;
      }

      raspberrypi_multicore_MPCtes_DW.ToFile16_IWORK.Count = 0;
      raspberrypi_multicore_MPCtes_DW.ToFile16_IWORK.Decimation = -1;
      raspberrypi_multicore_MPCtes_DW.ToFile16_PWORK.FilePtr = fp;
    }

    /* Start for ToFile: '<Root>/To File9' */
    {
      FILE *fp = (NULL);
      char fileName[509] = "multicore_dataRec10.mat";
      if ((fp = fopen(fileName, "wb")) == (NULL)) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error creating .mat file multicore_dataRec10.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 12 + 1, 0, "rt_pArrayLAdm")) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error writing mat file header to file multicore_dataRec10.mat");
        return;
      }

      raspberrypi_multicore_MPCtes_DW.ToFile9_IWORK.Count = 0;
      raspberrypi_multicore_MPCtes_DW.ToFile9_IWORK.Decimation = -1;
      raspberrypi_multicore_MPCtes_DW.ToFile9_PWORK.FilePtr = fp;
    }

    /* Start for ToFile: '<Root>/To File15' */
    {
      FILE *fp = (NULL);
      char fileName[509] = "multicore_dataRec14.mat";
      if ((fp = fopen(fileName, "wb")) == (NULL)) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error creating .mat file multicore_dataRec14.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 12 + 1, 0, "rt_spMPC_fast")) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error writing mat file header to file multicore_dataRec14.mat");
        return;
      }

      raspberrypi_multicore_MPCtes_DW.ToFile15_IWORK.Count = 0;
      raspberrypi_multicore_MPCtes_DW.ToFile15_IWORK.Decimation = -1;
      raspberrypi_multicore_MPCtes_DW.ToFile15_PWORK.FilePtr = fp;
    }

    /* Start for ToFile: '<Root>/To File18' */
    {
      FILE *fp = (NULL);
      char fileName[509] = "multicore_dataRecEstC.mat";
      if ((fp = fopen(fileName, "wb")) == (NULL)) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error creating .mat file multicore_dataRecEstC.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 12 + 1, 0, "rt_uMPC")) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error writing mat file header to file multicore_dataRecEstC.mat");
        return;
      }

      raspberrypi_multicore_MPCtes_DW.ToFile18_IWORK.Count = 0;
      raspberrypi_multicore_MPCtes_DW.ToFile18_IWORK.Decimation = -1;
      raspberrypi_multicore_MPCtes_DW.ToFile18_PWORK.FilePtr = fp;
    }

    /* Start for ToFile: '<Root>/To File2' */
    {
      FILE *fp = (NULL);
      char fileName[509] = "multicore_dataRec3.mat";
      if ((fp = fopen(fileName, "wb")) == (NULL)) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error creating .mat file multicore_dataRec3.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 12 + 1, 0, "rt_mvOut_fast")) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error writing mat file header to file multicore_dataRec3.mat");
        return;
      }

      raspberrypi_multicore_MPCtes_DW.ToFile2_IWORK.Count = 0;
      raspberrypi_multicore_MPCtes_DW.ToFile2_IWORK.Decimation = -1;
      raspberrypi_multicore_MPCtes_DW.ToFile2_PWORK.FilePtr = fp;
    }

    /* Start for ToFile: '<Root>/To File10' */
    {
      FILE *fp = (NULL);
      char fileName[509] = "multicore_dataRecEstA.mat";
      if ((fp = fopen(fileName, "wb")) == (NULL)) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error creating .mat file multicore_dataRecEstA.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 9 + 1, 0, "rt_acc_omega_RPYOri")) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error writing mat file header to file multicore_dataRecEstA.mat");
        return;
      }

      raspberrypi_multicore_MPCtes_DW.ToFile10_IWORK.Count = 0;
      raspberrypi_multicore_MPCtes_DW.ToFile10_IWORK.Decimation = -1;
      raspberrypi_multicore_MPCtes_DW.ToFile10_PWORK.FilePtr = fp;
    }

    /* Start for ToFile: '<Root>/To File12' */
    {
      FILE *fp = (NULL);
      char fileName[509] = "multicore_dataRec11.mat";
      if ((fp = fopen(fileName, "wb")) == (NULL)) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error creating .mat file multicore_dataRec11.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 8 + 1, 0, "rt_touchInd_LegState")) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error writing mat file header to file multicore_dataRec11.mat");
        return;
      }

      raspberrypi_multicore_MPCtes_DW.ToFile12_IWORK.Count = 0;
      raspberrypi_multicore_MPCtes_DW.ToFile12_IWORK.Decimation = -1;
      raspberrypi_multicore_MPCtes_DW.ToFile12_PWORK.FilePtr = fp;
    }

    /* Start for ToFile: '<Root>/To File19' */
    {
      FILE *fp = (NULL);
      char fileName[509] = "multicore_dataRec17.mat";
      if ((fp = fopen(fileName, "wb")) == (NULL)) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error creating .mat file multicore_dataRec17.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 6 + 1, 0, "rt_estDis_fast")) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error writing mat file header to file multicore_dataRec17.mat");
        return;
      }

      raspberrypi_multicore_MPCtes_DW.ToFile19_IWORK.Count = 0;
      raspberrypi_multicore_MPCtes_DW.ToFile19_IWORK.Decimation = -1;
      raspberrypi_multicore_MPCtes_DW.ToFile19_PWORK.FilePtr = fp;
    }

    /* Start for ToFile: '<Root>/To File5' */
    {
      FILE *fp = (NULL);
      char fileName[509] = "multicore_dataRec6.mat";
      if ((fp = fopen(fileName, "wb")) == (NULL)) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error creating .mat file multicore_dataRec6.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 4 + 1, 0, "rt_EstOff_OscEN_mpcSTOP_ES")) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error writing mat file header to file multicore_dataRec6.mat");
        return;
      }

      raspberrypi_multicore_MPCtes_DW.ToFile5_IWORK.Count = 0;
      raspberrypi_multicore_MPCtes_DW.ToFile5_IWORK.Decimation = -1;
      raspberrypi_multicore_MPCtes_DW.ToFile5_PWORK.FilePtr = fp;
    }

    /* Start for ToFile: '<Root>/To File14' */
    {
      FILE *fp = (NULL);
      char fileName[509] = "multicore_dataRec13.mat";
      if ((fp = fopen(fileName, "wb")) == (NULL)) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error creating .mat file multicore_dataRec13.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 3 + 1, 0,
           "rt_vxPercent_vyPercent_wzPercent")) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error writing mat file header to file multicore_dataRec13.mat");
        return;
      }

      raspberrypi_multicore_MPCtes_DW.ToFile14_IWORK.Count = 0;
      raspberrypi_multicore_MPCtes_DW.ToFile14_IWORK.Decimation = -1;
      raspberrypi_multicore_MPCtes_DW.ToFile14_PWORK.FilePtr = fp;
    }

    /* Start for MATLABSystem: '<Root>/MATLAB System15' */
    message_decoder_PC_message_deco(&raspberrypi_multicore_MPCtes_DW.obj_e);
    raspberrypi_multicore_MPCtes_DW.objisempty_mb = true;
    raspberrypi_multicore_MPCtes_DW.obj_e.lateral_width =
      raspberrypi_multicore_MPCtest_P.MATLABSystem15_lateral_width;
    raspberrypi_multicore_MPCtes_DW.obj_e.sagetial_width =
      raspberrypi_multicore_MPCtest_P.MATLABSystem15_sagetial_width;
    raspberrypi_multicore_MPCtes_DW.obj_e.roll_Off =
      raspberrypi_multicore_MPCtest_P.MATLABSystem15_roll_Off;
    raspberrypi_multicore_MPCtes_DW.obj_e.hIni =
      raspberrypi_multicore_MPCtest_P.MATLABSystem15_hIni;
    raspberrypi_multicore_MPCtes_DW.obj_e.isInitialized = 1;
    raspberrypi_multicore_MPCtes_DW.obj_e.pCoM_Old[0] = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj_e.pCoM_Old[1] = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj_e.pCoM_Old[2] =
      raspberrypi_multicore_MPCtes_DW.obj_e.hIni;
    raspberrypi_multicore_MPCtest_B.xW =
      raspberrypi_multicore_MPCtes_DW.obj_e.lateral_width;
    raspberrypi_multicore_MPCtest_B.yW =
      raspberrypi_multicore_MPCtes_DW.obj_e.sagetial_width;
    memset(&raspberrypi_multicore_MPCtes_DW.obj_e.SP_Old[0], 0, 12U * sizeof
           (real_T));
    raspberrypi_multicore_MPCtes_DW.obj_e.SP_Old[0] =
      raspberrypi_multicore_MPCtest_B.xW / 2.0;
    raspberrypi_multicore_MPCtes_DW.obj_e.SP_Old[1] =
      raspberrypi_multicore_MPCtest_B.yW / 2.0;
    raspberrypi_multicore_MPCtes_DW.obj_e.SP_Old[2] = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj_e.SP_Old[3] =
      raspberrypi_multicore_MPCtest_B.xW / 2.0;
    raspberrypi_multicore_MPCtest_B.d1 = -raspberrypi_multicore_MPCtest_B.yW /
      2.0;
    raspberrypi_multicore_MPCtes_DW.obj_e.SP_Old[4] =
      raspberrypi_multicore_MPCtest_B.d1;
    raspberrypi_multicore_MPCtes_DW.obj_e.SP_Old[5] = 0.0;
    raspberrypi_multicore_MPCtest_B.xW = -raspberrypi_multicore_MPCtest_B.xW /
      2.0;
    raspberrypi_multicore_MPCtes_DW.obj_e.SP_Old[6] =
      raspberrypi_multicore_MPCtest_B.xW;
    raspberrypi_multicore_MPCtes_DW.obj_e.SP_Old[7] =
      raspberrypi_multicore_MPCtest_B.yW / 2.0;
    raspberrypi_multicore_MPCtes_DW.obj_e.SP_Old[8] = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj_e.SP_Old[9] =
      raspberrypi_multicore_MPCtest_B.xW;
    raspberrypi_multicore_MPCtes_DW.obj_e.SP_Old[10] =
      raspberrypi_multicore_MPCtest_B.d1;
    raspberrypi_multicore_MPCtes_DW.obj_e.SP_Old[11] = 0.0;
    raspberrypi_multicore_MPCtest_B.yW =
      raspberrypi_multicore_MPCtes_DW.obj_e.roll_Off;
    for (i = 0; i < 12; i++) {
      raspberrypi_multicore_MPCtes_DW.obj_e.SP_Old[i] += (real_T)tmp_0[i] *
        raspberrypi_multicore_MPCtest_B.yW;
    }

    raspberrypi_multicore_MPCtes_DW.obj_e.SPLeg_Old[0] = 1.0;
    raspberrypi_multicore_MPCtes_DW.obj_e.SPLeg_Old[1] = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj_e.SPLeg_Old[2] = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj_e.SPLeg_Old[3] = 1.0;

    /* End of Start for MATLABSystem: '<Root>/MATLAB System15' */

    /* Start for MATLABSystem: '<Root>/MATLAB System6' */
    raspberrypi_multicore_MPCtes_DW.objisempty_gx = true;
    raspberrypi_multicore_MPCtes_DW.obj_as.vxMax =
      raspberrypi_multicore_MPCtest_P.MATLABSystem6_vxMax;
    raspberrypi_multicore_MPCtes_DW.obj_as.vyMax =
      raspberrypi_multicore_MPCtest_P.MATLABSystem6_vyMax;
    raspberrypi_multicore_MPCtes_DW.obj_as.wzMax =
      raspberrypi_multicore_MPCtest_P.MATLABSystem6_wzMax;
    raspberrypi_multicore_MPCtes_DW.obj_as.isInitialized = 1;

    /* Start for MATLABSystem: '<Root>/MATLAB System7' */
    raspberrypi_multicore_MPCtes_DW.objisempty_n = true;
    raspberrypi_multicore_MPCtes_DW.obj_cm.desHeight =
      raspberrypi_multicore_MPCtest_P.MATLABSystem7_desHeight;
    raspberrypi_multicore_MPCtes_DW.obj_cm.dt =
      raspberrypi_multicore_MPCtest_P.Ts;
    raspberrypi_multicore_MPCtes_DW.obj_cm.isInitialized = 1;
    raspberrypi_multicore_MPCtes_DW.obj_cm.r0[0] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem7_r0[0];
    raspberrypi_multicore_MPCtes_DW.obj_cm.theta0[0] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem7_theta0[0];
    raspberrypi_multicore_MPCtes_DW.obj_cm.dr0[0] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem7_dr0[0];
    raspberrypi_multicore_MPCtes_DW.obj_cm.omega0[0] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem7_omega0[0];
    raspberrypi_multicore_MPCtes_DW.obj_cm.sitaErr_K[0] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem7_sitaErr_K[0];
    raspberrypi_multicore_MPCtest_B.dv20[0] =
      raspberrypi_multicore_MPCtes_DW.obj_cm.r0[0];
    raspberrypi_multicore_MPCtest_B.dv20[3] =
      raspberrypi_multicore_MPCtes_DW.obj_cm.theta0[0];
    raspberrypi_multicore_MPCtest_B.dv20[6] =
      raspberrypi_multicore_MPCtes_DW.obj_cm.dr0[0];
    raspberrypi_multicore_MPCtest_B.dv20[9] =
      raspberrypi_multicore_MPCtes_DW.obj_cm.omega0[0];
    raspberrypi_multicore_MPCtes_DW.obj_cm.r0[1] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem7_r0[1];
    raspberrypi_multicore_MPCtes_DW.obj_cm.theta0[1] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem7_theta0[1];
    raspberrypi_multicore_MPCtes_DW.obj_cm.dr0[1] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem7_dr0[1];
    raspberrypi_multicore_MPCtes_DW.obj_cm.omega0[1] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem7_omega0[1];
    raspberrypi_multicore_MPCtes_DW.obj_cm.sitaErr_K[1] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem7_sitaErr_K[1];
    raspberrypi_multicore_MPCtest_B.dv20[1] =
      raspberrypi_multicore_MPCtes_DW.obj_cm.r0[1];
    raspberrypi_multicore_MPCtest_B.dv20[4] =
      raspberrypi_multicore_MPCtes_DW.obj_cm.theta0[1];
    raspberrypi_multicore_MPCtest_B.dv20[7] =
      raspberrypi_multicore_MPCtes_DW.obj_cm.dr0[1];
    raspberrypi_multicore_MPCtest_B.dv20[10] =
      raspberrypi_multicore_MPCtes_DW.obj_cm.omega0[1];
    raspberrypi_multicore_MPCtes_DW.obj_cm.r0[2] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem7_r0[2];
    raspberrypi_multicore_MPCtes_DW.obj_cm.theta0[2] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem7_theta0[2];
    raspberrypi_multicore_MPCtes_DW.obj_cm.dr0[2] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem7_dr0[2];
    raspberrypi_multicore_MPCtes_DW.obj_cm.omega0[2] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem7_omega0[2];
    raspberrypi_multicore_MPCtes_DW.obj_cm.sitaErr_K[2] =
      raspberrypi_multicore_MPCtest_P.MATLABSystem7_sitaErr_K[2];
    raspberrypi_multicore_MPCtest_B.dv20[2] =
      raspberrypi_multicore_MPCtes_DW.obj_cm.r0[2];
    raspberrypi_multicore_MPCtest_B.dv20[5] =
      raspberrypi_multicore_MPCtes_DW.obj_cm.theta0[2];
    raspberrypi_multicore_MPCtest_B.dv20[8] =
      raspberrypi_multicore_MPCtes_DW.obj_cm.dr0[2];
    raspberrypi_multicore_MPCtest_B.dv20[11] =
      raspberrypi_multicore_MPCtes_DW.obj_cm.omega0[2];
    raspberrypi_multicore_MPCtest_B.dv20[12] = 9.8;
    for (i = 0; i < 6; i++) {
      for (i_0 = 0; i_0 < 13; i_0++) {
        raspberrypi_multicore_MPCtes_DW.obj_cm.refSeqOld[i + 6 * i_0] =
          raspberrypi_multicore_MPCtest_B.dv20[i_0];
      }
    }

    raspberrypi_multicore_MPCtes_DW.obj_cm.sitaErrOld[0] = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj_cm.sitaErrOld[1] = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj_cm.sitaErrOld[2] = 0.0;
    raspberrypi_multicore_MPCtes_DW.obj_cm.sitaZOld = 0.0;

    /* End of Start for MATLABSystem: '<Root>/MATLAB System7' */

    /* Start for MATLABSystem: '<Root>/MATLAB System21' */
    raspberrypi_multicore_MPCtes_DW.objisempty_f = true;
    memcpy(&raspberrypi_multicore_MPCtes_DW.obj_g.Inorm[0],
           &raspberrypi_multicore_MPCtest_P.MATLABSystem21_Inorm[0], 9U * sizeof
           (real_T));
    raspberrypi_multicore_MPCtes_DW.obj_g.m =
      raspberrypi_multicore_MPCtest_P.MATLABSystem21_m;
    raspberrypi_multicore_MPCtes_DW.obj_g.Ts =
      raspberrypi_multicore_MPCtest_P.Ts;
    raspberrypi_multicore_MPCtes_DW.obj_g.hIni =
      raspberrypi_multicore_MPCtest_P.MATLABSystem21_hIni;
    raspberrypi_multicore_MPCtes_DW.obj_g.isInitialized = 1;

    /* Start for ToFile: '<Root>/To File8' */
    {
      FILE *fp = (NULL);
      char fileName[509] = "multicore_dataRec9.mat";
      if ((fp = fopen(fileName, "wb")) == (NULL)) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error creating .mat file multicore_dataRec9.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 26 + 1, 0, "rt_refSeq2_refP_slow")) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error writing mat file header to file multicore_dataRec9.mat");
        return;
      }

      raspberrypi_multicore_MPCtes_DW.ToFile8_IWORK.Count = 0;
      raspberrypi_multicore_MPCtes_DW.ToFile8_IWORK.Decimation = -1;
      raspberrypi_multicore_MPCtes_DW.ToFile8_PWORK.FilePtr = fp;
    }

    /* Start for ToFile: '<Root>/To File7' */
    {
      FILE *fp = (NULL);
      char fileName[509] = "multicore_dataRec8.mat";
      if ((fp = fopen(fileName, "wb")) == (NULL)) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error creating .mat file multicore_dataRec8.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 13 + 1, 0, "rt_xMPCOut_slow")) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error writing mat file header to file multicore_dataRec8.mat");
        return;
      }

      raspberrypi_multicore_MPCtes_DW.ToFile7_IWORK.Count = 0;
      raspberrypi_multicore_MPCtes_DW.ToFile7_IWORK.Decimation = -1;
      raspberrypi_multicore_MPCtes_DW.ToFile7_PWORK.FilePtr = fp;
    }

    /* Start for ToFile: '<Root>/To File1' */
    {
      FILE *fp = (NULL);
      char fileName[509] = "multicore_dataRec2.mat";
      if ((fp = fopen(fileName, "wb")) == (NULL)) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error creating .mat file multicore_dataRec2.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 13 + 1, 0, "rt_mvOut_qpStatus_slow")) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error writing mat file header to file multicore_dataRec2.mat");
        return;
      }

      raspberrypi_multicore_MPCtes_DW.ToFile1_IWORK.Count = 0;
      raspberrypi_multicore_MPCtes_DW.ToFile1_IWORK.Decimation = -1;
      raspberrypi_multicore_MPCtes_DW.ToFile1_PWORK.FilePtr = fp;
    }

    /* Start for ToFile: '<Root>/To File' */
    {
      FILE *fp = (NULL);
      char fileName[509] = "multicore_dataRec1.mat";
      if ((fp = fopen(fileName, "wb")) == (NULL)) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error creating .mat file multicore_dataRec1.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 13 + 1, 0, "rt_x_FB_slow")) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error writing mat file header to file multicore_dataRec1.mat");
        return;
      }

      raspberrypi_multicore_MPCtes_DW.ToFile_IWORK.Count = 0;
      raspberrypi_multicore_MPCtes_DW.ToFile_IWORK.Decimation = -1;
      raspberrypi_multicore_MPCtes_DW.ToFile_PWORK.FilePtr = fp;
    }
  }

  {
    int32_T i;

    /* InitializeConditions for RateTransition: '<Root>/Rate Transition' */
    for (i = 0; i < 67; i++) {
      raspberrypi_multicore_MPCtes_DW.RateTransition_Buf[i] =
        raspberrypi_multicore_MPCtest_P.RateTransition_InitialCondition[i];
    }

    /* End of InitializeConditions for RateTransition: '<Root>/Rate Transition' */

    /* InitializeConditions for Memory: '<S14>/Memory1' */
    raspberrypi_multicore_MPCtes_DW.Memory1_PreviousInput =
      raspberrypi_multicore_MPCtest_P.Memory1_InitialCondition;

    /* InitializeConditions for UnitDelay: '<Root>/Unit Delay6' */
    memcpy(&raspberrypi_multicore_MPCtes_DW.UnitDelay6_DSTATE[0],
           &raspberrypi_multicore_MPCtest_P.UnitDelay6_InitialCondition[0], 12U *
           sizeof(real_T));

    /* InitializeConditions for UnitDelay: '<Root>/Unit Delay5' */
    memcpy(&raspberrypi_multicore_MPCtes_DW.UnitDelay5_DSTATE[0],
           &raspberrypi_multicore_MPCtest_P.UnitDelay5_InitialCondition[0], 12U *
           sizeof(real_T));

    /* InitializeConditions for UnitDelay: '<Root>/Unit Delay' */
    memcpy(&raspberrypi_multicore_MPCtes_DW.UnitDelay_DSTATE[0],
           &raspberrypi_multicore_MPCtest_P.UnitDelay_InitialCondition[0], 12U *
           sizeof(real_T));

    /* InitializeConditions for UnitDelay: '<S15>/Unit Delay' */
    memcpy(&raspberrypi_multicore_MPCtes_DW.UnitDelay_DSTATE_l[0],
           &raspberrypi_multicore_MPCtest_P.UnitDelay_InitialCondition_j[0], 12U
           * sizeof(real_T));

    /* InitializeConditions for UnitDelay: '<Root>/Unit Delay7' */
    memcpy(&raspberrypi_multicore_MPCtes_DW.UnitDelay7_DSTATE[0],
           &raspberrypi_multicore_MPCtest_P.UnitDelay7_InitialCondition[0], 12U *
           sizeof(real_T));

    /* InitializeConditions for RateTransition: '<Root>/Rate Transition1' */
    for (i = 0; i < 55; i++) {
      raspberrypi_multicore_MPCtes_DW.RateTransition1_Buf0[i] =
        raspberrypi_multicore_MPCtest_P.RateTransition1_InitialConditio[i];
    }

    /* End of InitializeConditions for RateTransition: '<Root>/Rate Transition1' */

    /* InitializeConditions for UnitDelay: '<Root>/Unit Delay2' */
    memcpy(&raspberrypi_multicore_MPCtes_DW.UnitDelay2_DSTATE[0],
           &raspberrypi_multicore_MPCtest_P.UnitDelay2_InitialCondition[0], 12U *
           sizeof(real_T));

    /* InitializeConditions for Memory: '<S18>/Memory' */
    memcpy(&raspberrypi_multicore_MPCtes_DW.Memory_PreviousInput[0],
           &raspberrypi_multicore_MPCtest_P.Memory_InitialCondition[0], 252U *
           sizeof(boolean_T));

    /* InitializeConditions for Sum: '<Root>/Add' incorporates:
     *  UnitDelay: '<Root>/Unit Delay1'
     */
    raspberrypi_multicore_MPCtes_DW.UnitDelay1_DSTATE =
      raspberrypi_multicore_MPCtest_P.UnitDelay1_InitialCondition;

    /* SystemInitialize for Chart: '<Root>/Chart' */
    raspberrypi_multicore_MPCtes_DW.sfEvent = -1;
    raspberrypi_multicore_MPCtes_DW.temporalCounter_i1 = 0U;
    raspberrypi_multicore_MPCtes_DW.is_active_c12_raspberrypi_multi = 0U;
    raspberrypi_multicore_MPCtes_DW.is_c12_raspberrypi_multicore_MP = 0U;
    raspberrypi_multicore_MPCtest_B.oscEN = 0.0;
    raspberrypi_multicore_MPCtest_B.mpcSTOP = 0.0;

    /*  Initialize / reset discrete-state properties */
    /*  Initialize / reset discrete-state properties */
    /*  Initialize / reset discrete-state properties */
    /*  Initialize / reset discrete-state properties */
    /*  Initialize / reset discrete-state properties */
    /*  Initialize / reset discrete-state properties */
    /*  Initialize / reset discrete-state properties */
    /*  Initialize / reset discrete-state properties */
    /*  Initialize / reset discrete-state properties */
    /*  Initialize / reset discrete-state properties */
    /*  Initialize / reset discrete-state properties */
    /*  Initialize / reset discrete-state properties */
    /*  Initialize / reset discrete-state properties */
    /*  Initialize / reset discrete-state properties */
    /*  Initialize / reset discrete-state properties */
    /*  Initialize / reset discrete-state properties */
    /*  Initialize / reset discrete-state properties */
    /*  Initialize / reset discrete-state properties */
    /*  Initialize / reset discrete-state properties */
    /*  Initialize / reset discrete-state properties */
  }
}

/* Model terminate function */
void raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_estDis_terminate(void)
{
  uint32_T MISOPinLoc;
  uint32_T MOSIPinLoc;
  uint32_T PinNameLoc;
  uint32_T SCKPinLoc;

  /* Terminate for MATLABSystem: '<Root>/MATLAB System' */
  if (!raspberrypi_multicore_MPCtes_DW.obj_e1.matlabCodegenIsDeleted) {
    raspberrypi_multicore_MPCtes_DW.obj_e1.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<Root>/MATLAB System' */

  /* Terminate for RateTransition: '<Root>/Rate Transition' */
  rtw_pthread_mutex_destroy
    (raspberrypi_multicore_MPCtes_DW.RateTransition_d0_SEMAPHORE);

  /* Terminate for MATLABSystem: '<S14>/SPI Master Transfer' */
  if (!raspberrypi_multicore_MPCtes_DW.obj_h.matlabCodegenIsDeleted) {
    raspberrypi_multicore_MPCtes_DW.obj_h.matlabCodegenIsDeleted = true;
    if ((raspberrypi_multicore_MPCtes_DW.obj_h.isInitialized == 1) &&
        raspberrypi_multicore_MPCtes_DW.obj_h.isSetupComplete) {
      PinNameLoc = SPI0_CE0;
      MOSIPinLoc = MW_UNDEFINED_VALUE;
      MISOPinLoc = MW_UNDEFINED_VALUE;
      SCKPinLoc = MW_UNDEFINED_VALUE;
      MW_SPI_Close(raspberrypi_multicore_MPCtes_DW.obj_h.MW_SPI_HANDLE,
                   MOSIPinLoc, MISOPinLoc, SCKPinLoc, PinNameLoc);
    }
  }

  /* End of Terminate for MATLABSystem: '<S14>/SPI Master Transfer' */

  /* Terminate for MATLABSystem: '<S52>/Digital Read' */
  if (!raspberrypi_multicore_MPCtes_DW.obj_ls.matlabCodegenIsDeleted) {
    raspberrypi_multicore_MPCtes_DW.obj_ls.matlabCodegenIsDeleted = true;
    if ((raspberrypi_multicore_MPCtes_DW.obj_ls.isInitialized == 1) &&
        raspberrypi_multicore_MPCtes_DW.obj_ls.isSetupComplete) {
      EXT_GPIO_terminate(24U);
    }
  }

  /* End of Terminate for MATLABSystem: '<S52>/Digital Read' */

  /* Terminate for MATLABSystem: '<S53>/Digital Read' */
  if (!raspberrypi_multicore_MPCtes_DW.obj_cu.matlabCodegenIsDeleted) {
    raspberrypi_multicore_MPCtes_DW.obj_cu.matlabCodegenIsDeleted = true;
    if ((raspberrypi_multicore_MPCtes_DW.obj_cu.isInitialized == 1) &&
        raspberrypi_multicore_MPCtes_DW.obj_cu.isSetupComplete) {
      EXT_GPIO_terminate(23U);
    }
  }

  /* End of Terminate for MATLABSystem: '<S53>/Digital Read' */

  /* Terminate for MATLABSystem: '<S54>/Digital Read' */
  if (!raspberrypi_multicore_MPCtes_DW.obj_i.matlabCodegenIsDeleted) {
    raspberrypi_multicore_MPCtes_DW.obj_i.matlabCodegenIsDeleted = true;
    if ((raspberrypi_multicore_MPCtes_DW.obj_i.isInitialized == 1) &&
        raspberrypi_multicore_MPCtes_DW.obj_i.isSetupComplete) {
      EXT_GPIO_terminate(18U);
    }
  }

  /* End of Terminate for MATLABSystem: '<S54>/Digital Read' */

  /* Terminate for MATLABSystem: '<S55>/Digital Read' */
  if (!raspberrypi_multicore_MPCtes_DW.obj_fv.matlabCodegenIsDeleted) {
    raspberrypi_multicore_MPCtes_DW.obj_fv.matlabCodegenIsDeleted = true;
    if ((raspberrypi_multicore_MPCtes_DW.obj_fv.isInitialized == 1) &&
        raspberrypi_multicore_MPCtes_DW.obj_fv.isSetupComplete) {
      EXT_GPIO_terminate(4U);
    }
  }

  /* End of Terminate for MATLABSystem: '<S55>/Digital Read' */

  /* Terminate for ToFile: '<Root>/To File11' */
  {
    FILE *fp = (FILE *) raspberrypi_multicore_MPCtes_DW.ToFile11_PWORK.FilePtr;
    if (fp != (NULL)) {
      char fileName[509] = "multicore_dataRecEstB.mat";
      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error closing MAT-file multicore_dataRecEstB.mat");
        return;
      }

      if ((fp = fopen(fileName, "r+b")) == (NULL)) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error reopening MAT-file multicore_dataRecEstB.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 17 + 1,
           raspberrypi_multicore_MPCtes_DW.ToFile11_IWORK.Count,
           "rt_pArrayB_EN_SW")) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error writing header for rt_pArrayB_EN_SW to MAT-file multicore_dataRecEstB.mat");
      }

      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error closing MAT-file multicore_dataRecEstB.mat");
        return;
      }

      raspberrypi_multicore_MPCtes_DW.ToFile11_PWORK.FilePtr = (NULL);
    }
  }

  /* Terminate for ToFile: '<Root>/To File6' */
  {
    FILE *fp = (FILE *) raspberrypi_multicore_MPCtes_DW.ToFile6_PWORK.FilePtr;
    if (fp != (NULL)) {
      char fileName[509] = "multicore_dataRec7.mat";
      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error closing MAT-file multicore_dataRec7.mat");
        return;
      }

      if ((fp = fopen(fileName, "r+b")) == (NULL)) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error reopening MAT-file multicore_dataRec7.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 16 + 1,
           raspberrypi_multicore_MPCtes_DW.ToFile6_IWORK.Count,
           "rt_estSPLeg_estSP")) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error writing header for rt_estSPLeg_estSP to MAT-file multicore_dataRec7.mat");
      }

      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error closing MAT-file multicore_dataRec7.mat");
        return;
      }

      raspberrypi_multicore_MPCtes_DW.ToFile6_PWORK.FilePtr = (NULL);
    }
  }

  /* Terminate for RateTransition: '<Root>/Rate Transition1' */
  rtw_pthread_mutex_destroy
    (raspberrypi_multicore_MPCtes_DW.RateTransition1_d0_SEMAPHORE);

  /* Terminate for ToFile: '<Root>/To File3' */
  {
    FILE *fp = (FILE *) raspberrypi_multicore_MPCtes_DW.ToFile3_PWORK.FilePtr;
    if (fp != (NULL)) {
      char fileName[509] = "multicore_dataRec4.mat";
      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error closing MAT-file multicore_dataRec4.mat");
        return;
      }

      if ((fp = fopen(fileName, "r+b")) == (NULL)) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error reopening MAT-file multicore_dataRec4.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 12 + 1,
           raspberrypi_multicore_MPCtes_DW.ToFile3_IWORK.Count, "rt_x_FB_fast"))
      {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error writing header for rt_x_FB_fast to MAT-file multicore_dataRec4.mat");
      }

      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error closing MAT-file multicore_dataRec4.mat");
        return;
      }

      raspberrypi_multicore_MPCtes_DW.ToFile3_PWORK.FilePtr = (NULL);
    }
  }

  /* Terminate for ToFile: '<Root>/To File13' */
  {
    FILE *fp = (FILE *) raspberrypi_multicore_MPCtes_DW.ToFile13_PWORK.FilePtr;
    if (fp != (NULL)) {
      char fileName[509] = "multicore_dataRec12.mat";
      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error closing MAT-file multicore_dataRec12.mat");
        return;
      }

      if ((fp = fopen(fileName, "r+b")) == (NULL)) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error reopening MAT-file multicore_dataRec12.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 12 + 1,
           raspberrypi_multicore_MPCtes_DW.ToFile13_IWORK.Count, "rt_pLFKfast"))
      {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error writing header for rt_pLFKfast to MAT-file multicore_dataRec12.mat");
      }

      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error closing MAT-file multicore_dataRec12.mat");
        return;
      }

      raspberrypi_multicore_MPCtes_DW.ToFile13_PWORK.FilePtr = (NULL);
    }
  }

  /* Terminate for ToFile: '<Root>/To File4' */
  {
    FILE *fp = (FILE *) raspberrypi_multicore_MPCtes_DW.ToFile4_PWORK.FilePtr;
    if (fp != (NULL)) {
      char fileName[509] = "multicore_dataRec5.mat";
      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error closing MAT-file multicore_dataRec5.mat");
        return;
      }

      if ((fp = fopen(fileName, "r+b")) == (NULL)) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error reopening MAT-file multicore_dataRec5.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 12 + 1,
           raspberrypi_multicore_MPCtes_DW.ToFile4_IWORK.Count,
           "rt_PendAllLocal")) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error writing header for rt_PendAllLocal to MAT-file multicore_dataRec5.mat");
      }

      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error closing MAT-file multicore_dataRec5.mat");
        return;
      }

      raspberrypi_multicore_MPCtes_DW.ToFile4_PWORK.FilePtr = (NULL);
    }
  }

  /* Terminate for ToFile: '<Root>/To File16' */
  {
    FILE *fp = (FILE *) raspberrypi_multicore_MPCtes_DW.ToFile16_PWORK.FilePtr;
    if (fp != (NULL)) {
      char fileName[509] = "multicore_dataRec15.mat";
      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error closing MAT-file multicore_dataRec15.mat");
        return;
      }

      if ((fp = fopen(fileName, "r+b")) == (NULL)) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error reopening MAT-file multicore_dataRec15.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 12 + 1,
           raspberrypi_multicore_MPCtes_DW.ToFile16_IWORK.Count, "rt_pLnor_fast"))
      {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error writing header for rt_pLnor_fast to MAT-file multicore_dataRec15.mat");
      }

      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error closing MAT-file multicore_dataRec15.mat");
        return;
      }

      raspberrypi_multicore_MPCtes_DW.ToFile16_PWORK.FilePtr = (NULL);
    }
  }

  /* Terminate for ToFile: '<Root>/To File9' */
  {
    FILE *fp = (FILE *) raspberrypi_multicore_MPCtes_DW.ToFile9_PWORK.FilePtr;
    if (fp != (NULL)) {
      char fileName[509] = "multicore_dataRec10.mat";
      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error closing MAT-file multicore_dataRec10.mat");
        return;
      }

      if ((fp = fopen(fileName, "r+b")) == (NULL)) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error reopening MAT-file multicore_dataRec10.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 12 + 1,
           raspberrypi_multicore_MPCtes_DW.ToFile9_IWORK.Count, "rt_pArrayLAdm"))
      {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error writing header for rt_pArrayLAdm to MAT-file multicore_dataRec10.mat");
      }

      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error closing MAT-file multicore_dataRec10.mat");
        return;
      }

      raspberrypi_multicore_MPCtes_DW.ToFile9_PWORK.FilePtr = (NULL);
    }
  }

  /* Terminate for ToFile: '<Root>/To File15' */
  {
    FILE *fp = (FILE *) raspberrypi_multicore_MPCtes_DW.ToFile15_PWORK.FilePtr;
    if (fp != (NULL)) {
      char fileName[509] = "multicore_dataRec14.mat";
      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error closing MAT-file multicore_dataRec14.mat");
        return;
      }

      if ((fp = fopen(fileName, "r+b")) == (NULL)) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error reopening MAT-file multicore_dataRec14.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 12 + 1,
           raspberrypi_multicore_MPCtes_DW.ToFile15_IWORK.Count, "rt_spMPC_fast"))
      {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error writing header for rt_spMPC_fast to MAT-file multicore_dataRec14.mat");
      }

      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error closing MAT-file multicore_dataRec14.mat");
        return;
      }

      raspberrypi_multicore_MPCtes_DW.ToFile15_PWORK.FilePtr = (NULL);
    }
  }

  /* Terminate for ToFile: '<Root>/To File18' */
  {
    FILE *fp = (FILE *) raspberrypi_multicore_MPCtes_DW.ToFile18_PWORK.FilePtr;
    if (fp != (NULL)) {
      char fileName[509] = "multicore_dataRecEstC.mat";
      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error closing MAT-file multicore_dataRecEstC.mat");
        return;
      }

      if ((fp = fopen(fileName, "r+b")) == (NULL)) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error reopening MAT-file multicore_dataRecEstC.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 12 + 1,
           raspberrypi_multicore_MPCtes_DW.ToFile18_IWORK.Count, "rt_uMPC")) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error writing header for rt_uMPC to MAT-file multicore_dataRecEstC.mat");
      }

      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error closing MAT-file multicore_dataRecEstC.mat");
        return;
      }

      raspberrypi_multicore_MPCtes_DW.ToFile18_PWORK.FilePtr = (NULL);
    }
  }

  /* Terminate for ToFile: '<Root>/To File2' */
  {
    FILE *fp = (FILE *) raspberrypi_multicore_MPCtes_DW.ToFile2_PWORK.FilePtr;
    if (fp != (NULL)) {
      char fileName[509] = "multicore_dataRec3.mat";
      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error closing MAT-file multicore_dataRec3.mat");
        return;
      }

      if ((fp = fopen(fileName, "r+b")) == (NULL)) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error reopening MAT-file multicore_dataRec3.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 12 + 1,
           raspberrypi_multicore_MPCtes_DW.ToFile2_IWORK.Count, "rt_mvOut_fast"))
      {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error writing header for rt_mvOut_fast to MAT-file multicore_dataRec3.mat");
      }

      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error closing MAT-file multicore_dataRec3.mat");
        return;
      }

      raspberrypi_multicore_MPCtes_DW.ToFile2_PWORK.FilePtr = (NULL);
    }
  }

  /* Terminate for ToFile: '<Root>/To File10' */
  {
    FILE *fp = (FILE *) raspberrypi_multicore_MPCtes_DW.ToFile10_PWORK.FilePtr;
    if (fp != (NULL)) {
      char fileName[509] = "multicore_dataRecEstA.mat";
      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error closing MAT-file multicore_dataRecEstA.mat");
        return;
      }

      if ((fp = fopen(fileName, "r+b")) == (NULL)) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error reopening MAT-file multicore_dataRecEstA.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 9 + 1,
           raspberrypi_multicore_MPCtes_DW.ToFile10_IWORK.Count,
           "rt_acc_omega_RPYOri")) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error writing header for rt_acc_omega_RPYOri to MAT-file multicore_dataRecEstA.mat");
      }

      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error closing MAT-file multicore_dataRecEstA.mat");
        return;
      }

      raspberrypi_multicore_MPCtes_DW.ToFile10_PWORK.FilePtr = (NULL);
    }
  }

  /* Terminate for ToFile: '<Root>/To File12' */
  {
    FILE *fp = (FILE *) raspberrypi_multicore_MPCtes_DW.ToFile12_PWORK.FilePtr;
    if (fp != (NULL)) {
      char fileName[509] = "multicore_dataRec11.mat";
      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error closing MAT-file multicore_dataRec11.mat");
        return;
      }

      if ((fp = fopen(fileName, "r+b")) == (NULL)) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error reopening MAT-file multicore_dataRec11.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 8 + 1,
           raspberrypi_multicore_MPCtes_DW.ToFile12_IWORK.Count,
           "rt_touchInd_LegState")) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error writing header for rt_touchInd_LegState to MAT-file multicore_dataRec11.mat");
      }

      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error closing MAT-file multicore_dataRec11.mat");
        return;
      }

      raspberrypi_multicore_MPCtes_DW.ToFile12_PWORK.FilePtr = (NULL);
    }
  }

  /* Terminate for ToFile: '<Root>/To File19' */
  {
    FILE *fp = (FILE *) raspberrypi_multicore_MPCtes_DW.ToFile19_PWORK.FilePtr;
    if (fp != (NULL)) {
      char fileName[509] = "multicore_dataRec17.mat";
      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error closing MAT-file multicore_dataRec17.mat");
        return;
      }

      if ((fp = fopen(fileName, "r+b")) == (NULL)) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error reopening MAT-file multicore_dataRec17.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 6 + 1,
           raspberrypi_multicore_MPCtes_DW.ToFile19_IWORK.Count,
           "rt_estDis_fast")) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error writing header for rt_estDis_fast to MAT-file multicore_dataRec17.mat");
      }

      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error closing MAT-file multicore_dataRec17.mat");
        return;
      }

      raspberrypi_multicore_MPCtes_DW.ToFile19_PWORK.FilePtr = (NULL);
    }
  }

  /* Terminate for ToFile: '<Root>/To File5' */
  {
    FILE *fp = (FILE *) raspberrypi_multicore_MPCtes_DW.ToFile5_PWORK.FilePtr;
    if (fp != (NULL)) {
      char fileName[509] = "multicore_dataRec6.mat";
      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error closing MAT-file multicore_dataRec6.mat");
        return;
      }

      if ((fp = fopen(fileName, "r+b")) == (NULL)) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error reopening MAT-file multicore_dataRec6.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 4 + 1,
           raspberrypi_multicore_MPCtes_DW.ToFile5_IWORK.Count,
           "rt_EstOff_OscEN_mpcSTOP_ES")) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error writing header for rt_EstOff_OscEN_mpcSTOP_ES to MAT-file multicore_dataRec6.mat");
      }

      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error closing MAT-file multicore_dataRec6.mat");
        return;
      }

      raspberrypi_multicore_MPCtes_DW.ToFile5_PWORK.FilePtr = (NULL);
    }
  }

  /* Terminate for ToFile: '<Root>/To File14' */
  {
    FILE *fp = (FILE *) raspberrypi_multicore_MPCtes_DW.ToFile14_PWORK.FilePtr;
    if (fp != (NULL)) {
      char fileName[509] = "multicore_dataRec13.mat";
      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error closing MAT-file multicore_dataRec13.mat");
        return;
      }

      if ((fp = fopen(fileName, "r+b")) == (NULL)) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error reopening MAT-file multicore_dataRec13.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 3 + 1,
           raspberrypi_multicore_MPCtes_DW.ToFile14_IWORK.Count,
           "rt_vxPercent_vyPercent_wzPercent")) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error writing header for rt_vxPercent_vyPercent_wzPercent to MAT-file multicore_dataRec13.mat");
      }

      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error closing MAT-file multicore_dataRec13.mat");
        return;
      }

      raspberrypi_multicore_MPCtes_DW.ToFile14_PWORK.FilePtr = (NULL);
    }
  }

  /* Terminate for ToFile: '<Root>/To File8' */
  {
    FILE *fp = (FILE *) raspberrypi_multicore_MPCtes_DW.ToFile8_PWORK.FilePtr;
    if (fp != (NULL)) {
      char fileName[509] = "multicore_dataRec9.mat";
      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error closing MAT-file multicore_dataRec9.mat");
        return;
      }

      if ((fp = fopen(fileName, "r+b")) == (NULL)) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error reopening MAT-file multicore_dataRec9.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 26 + 1,
           raspberrypi_multicore_MPCtes_DW.ToFile8_IWORK.Count,
           "rt_refSeq2_refP_slow")) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error writing header for rt_refSeq2_refP_slow to MAT-file multicore_dataRec9.mat");
      }

      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error closing MAT-file multicore_dataRec9.mat");
        return;
      }

      raspberrypi_multicore_MPCtes_DW.ToFile8_PWORK.FilePtr = (NULL);
    }
  }

  /* Terminate for ToFile: '<Root>/To File7' */
  {
    FILE *fp = (FILE *) raspberrypi_multicore_MPCtes_DW.ToFile7_PWORK.FilePtr;
    if (fp != (NULL)) {
      char fileName[509] = "multicore_dataRec8.mat";
      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error closing MAT-file multicore_dataRec8.mat");
        return;
      }

      if ((fp = fopen(fileName, "r+b")) == (NULL)) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error reopening MAT-file multicore_dataRec8.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 13 + 1,
           raspberrypi_multicore_MPCtes_DW.ToFile7_IWORK.Count,
           "rt_xMPCOut_slow")) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error writing header for rt_xMPCOut_slow to MAT-file multicore_dataRec8.mat");
      }

      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error closing MAT-file multicore_dataRec8.mat");
        return;
      }

      raspberrypi_multicore_MPCtes_DW.ToFile7_PWORK.FilePtr = (NULL);
    }
  }

  /* Terminate for ToFile: '<Root>/To File1' */
  {
    FILE *fp = (FILE *) raspberrypi_multicore_MPCtes_DW.ToFile1_PWORK.FilePtr;
    if (fp != (NULL)) {
      char fileName[509] = "multicore_dataRec2.mat";
      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error closing MAT-file multicore_dataRec2.mat");
        return;
      }

      if ((fp = fopen(fileName, "r+b")) == (NULL)) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error reopening MAT-file multicore_dataRec2.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 13 + 1,
           raspberrypi_multicore_MPCtes_DW.ToFile1_IWORK.Count,
           "rt_mvOut_qpStatus_slow")) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error writing header for rt_mvOut_qpStatus_slow to MAT-file multicore_dataRec2.mat");
      }

      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error closing MAT-file multicore_dataRec2.mat");
        return;
      }

      raspberrypi_multicore_MPCtes_DW.ToFile1_PWORK.FilePtr = (NULL);
    }
  }

  /* Terminate for ToFile: '<Root>/To File' */
  {
    FILE *fp = (FILE *) raspberrypi_multicore_MPCtes_DW.ToFile_PWORK.FilePtr;
    if (fp != (NULL)) {
      char fileName[509] = "multicore_dataRec1.mat";
      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error closing MAT-file multicore_dataRec1.mat");
        return;
      }

      if ((fp = fopen(fileName, "r+b")) == (NULL)) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error reopening MAT-file multicore_dataRec1.mat");
        return;
      }

      if (rt_WriteMat4FileHeader(fp, 13 + 1,
           raspberrypi_multicore_MPCtes_DW.ToFile_IWORK.Count, "rt_x_FB_slow"))
      {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error writing header for rt_x_FB_slow to MAT-file multicore_dataRec1.mat");
      }

      if (fclose(fp) == EOF) {
        rtmSetErrorStatus(raspberrypi_multicore_MPCtes_M,
                          "Error closing MAT-file multicore_dataRec1.mat");
        return;
      }

      raspberrypi_multicore_MPCtes_DW.ToFile_PWORK.FilePtr = (NULL);
    }
  }
}
