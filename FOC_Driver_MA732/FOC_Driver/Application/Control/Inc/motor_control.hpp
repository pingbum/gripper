/**
 * @file motor_control.hpp
 * @brief Transform utilities and space vector modulation.
 */

#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP
#include "arm_math_types.h"

/**
 * @brief Two-axis stationary reference frame representation.
 */
typedef struct {
  float32_t alpha; /**< alpha-axis component */
  float32_t beta;  /**< beta-axis component */
} alpha_beta_t;

/**
 * @brief Two-axis rotating reference frame representation.
 */
typedef struct {
  float32_t d; /**< direct-axis component  */
  float32_t q; /**< quadrature-axis component */
} dq_t;

/**
 * @ingroup park
 * @brief Floating-point Park transform.
 * @param[in] pIalpha_beta Input two-phase vector (alpha-beta).
 * @param[out] pIdq Output vector in the rotor d-q frame.
 * @param[in] sinVal Sine of rotation angle.
 * @param[in] cosVal Cosine of rotation angle.
 */
__STATIC_FORCEINLINE void park_f32(alpha_beta_t *pIalpha_beta, dq_t *pIdq,
                                   float32_t sinVal, float32_t cosVal) {
  /* Calculate pId using the equation, pId = Ialpha * cosVal + Ibeta * sinVal */
  pIdq->d = pIalpha_beta->alpha * cosVal + pIalpha_beta->beta * sinVal;
  /* Calculate pIq using the equation, pIq = - Ialpha * sinVal + Ibeta * cosVal
   */
  pIdq->q = -pIalpha_beta->alpha * sinVal + pIalpha_beta->beta * cosVal;
}

/**
 * @ingroup clarke
 * @brief Floating-point Clarke transform.
 * @param[in] Ia Input phase-a current.
 * @param[in] Ib Input phase-b current.
 * @param[in] Ic Input phase-c current.
 * @param[out] pIalpha_beta Output two-phase orthogonal vector (alpha-beta).
 */
__STATIC_FORCEINLINE void clarke_f32(float32_t Ia, float32_t Ib, float32_t Ic,
                                     alpha_beta_t *pIalpha_beta) {
  /* Calculate pIalpha using the equation, pIalpha = 2/3Ia-1/3Ib-1/3Ic  */
  pIalpha_beta->alpha =
      0.66666666666f * Ia - 0.33333333333f * Ib - 0.33333333333f * Ic;

  /* Calculate pIbeta using the equation, pIbeta = (1/sqrt(3)) * Ib -
   * (1/sqrt(3)) * Ic */
  pIalpha_beta->beta = (0.57735026919f * Ib - 0.57735026919f * Ic);
}

/**
 * @ingroup inv_park
 * @brief Floating-point inverse Park transform.
 * @param[in] pIdq Input vector in the rotor d-q frame.
 * @param[out] pIalpha_beta Output alpha-beta vector.
 * @param[in] sinVal Sine of rotation angle.
 * @param[in] cosVal Cosine of rotation angle.
 */
__STATIC_FORCEINLINE void park_inv_f32(dq_t *pIdq, alpha_beta_t *pIalpha_beta,
                                       float32_t sinVal, float32_t cosVal) {
  /* Calculate pIalpha using the equation, pIalpha = Id * cosVal - Iq * sinVal
   */
  pIalpha_beta->alpha = pIdq->d * cosVal - pIdq->q * sinVal;

  /* Calculate pIbeta using the equation, pIbeta = Id * sinVal + Iq * cosVal */
  pIalpha_beta->beta = pIdq->d * sinVal + pIdq->q * cosVal;
}

// Space vector modulation.
/**
 * @ingroup svmod
 * @brief Floating-point Space Vector Modulation (SVPWM via zero-sequence
 * injection).
 * @param[in] v_alpha_beta Input alpha-beta voltage vector (line-neutral).
 * @param[in] Vdc DC link voltage [V].
 * @param[out] duty Output array of three phase duty cycles (0..1).
 *
 * This implementation uses min/max common-mode (zero-sequence) injection,
 * which is equivalent to symmetric SVPWM in the linear modulation region.
 */
__STATIC_FORCEINLINE void SVM_f32(alpha_beta_t *v_alpha_beta, float32_t Vdc,
                                  float32_t *duty) {
  // 1) inverse Clarke: alpha-beta -> a,b,c (phase voltage references)
  float32_t Va = v_alpha_beta->alpha;
  float32_t Vb = -0.5f * v_alpha_beta->alpha +
                 0.86602540378f * v_alpha_beta->beta; // sqrt(3)/2
  float32_t Vc =
      -0.5f * v_alpha_beta->alpha - 0.86602540378f * v_alpha_beta->beta;

  // 2) common-mode injection (SVPWM): Voff = -(Vmax + Vmin)/2
  float32_t Vmax = Va;
  if (Vb > Vmax)
    Vmax = Vb;
  if (Vc > Vmax)
    Vmax = Vc;

  float32_t Vmin = Va;
  if (Vb < Vmin)
    Vmin = Vb;
  if (Vc < Vmin)
    Vmin = Vc;

  float32_t Voff = -0.5f * (Vmax + Vmin);

  Va += Voff;
  Vb += Voff;
  Vc += Voff;

  // 3) voltage -> duty (d = 0.5 + Vphase/Vdc)
  duty[0] = 0.5f + (Va / Vdc);
  duty[1] = 0.5f + (Vb / Vdc);
  duty[2] = 0.5f + (Vc / Vdc);
}
#endif // MOTOR_CONTROL_HPP
