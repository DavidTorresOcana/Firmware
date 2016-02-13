/**
 * @file dbx_control_params.c
 * Parameter definition for DBX Control.
 *
 * @author David Torres <david.torres@dbxdrones.com>
 *
 */
#include <systemlib/param/param.h>

/**
 * Sensibilidad throtle en m/s2
 *
 * RE-CHECK this.
 *
 * @min
 * @max
 * @group DBX Advanced Controller
 */
PARAM_DEFINE_FLOAT(DBX_Throtle_sens, 5.0f);

/**
 * Sensibildiad ginada en DEG
 *
 * RE-CHECK this.
 *
 * @min
 * @max
 * @group DBX Advanced Controller
 */
PARAM_DEFINE_FLOAT(DBX_Yaw_sens, 20.0f);

/**
 * Sensibilidad de cabeceo y balanceo en DEG
 *
 * RE-CHECK this.
 *
 * @min
 * @max
 * @group DBX Advanced Controller
 */
PARAM_DEFINE_FLOAT(DBX_Atti_sens, 10.0f);

/**
 * Tiempo de respuesta PHI
 *
 * RE-CHECK this.
 *
 * @min
 * @max
 * @group DBX Advanced Controller
 */
PARAM_DEFINE_FLOAT(DBX_phi_tau, 0.7f);
/**
 * Ganancia control PHI
 *
 * RE-CHECK this.
 *
 * @min
 * @max
 * @group DBX Advanced Controller
 */
PARAM_DEFINE_FLOAT(DBX_phi_K_b, 3.0f);

/**
 * Integral control PHI
 *
 * RE-CHECK this.
 *
 * @min
 * @max
 * @group DBX Advanced Controller
 */
PARAM_DEFINE_FLOAT(DBX_phi_f_i, 0.2f);


/**
 * Tiempo de respuesta THETA
 *
 * RE-CHECK this.
 *
 * @min
 * @max
 * @group DBX Advanced Controller
 */
PARAM_DEFINE_FLOAT(DBX_theta_tau, 0.7f);
/**
 * Ganancia control THETA
 *
 * RE-CHECK this.
 *
 * @min
 * @max
 * @group DBX Advanced Controller
 */
PARAM_DEFINE_FLOAT(DBX_theta_K_b, 3.0f);

/**
 * Integral control THETA
 *
 * RE-CHECK this.
 *
 * @min
 * @max
 * @group DBX Advanced Controller
 */
PARAM_DEFINE_FLOAT(DBX_theta_f_i, 0.2f);

/**
 * Tiempo de respuesta PSI
 *
 * RE-CHECK this.
 *
 * @min
 * @max
 * @group DBX Advanced Controller
 */
PARAM_DEFINE_FLOAT(DBX_psi_tau, 4.0f);
/**
 * Ganancia control PSI
 *
 * RE-CHECK this.
 *
 * @min
 * @max
 * @group DBX Advanced Controller
 */
PARAM_DEFINE_FLOAT(DBX_psi_K_b, 2.5f);

/**
 * Integral control PSI
 *
 * RE-CHECK this.
 *
 * @min
 * @max
 * @group DBX Advanced Controller
 */
PARAM_DEFINE_FLOAT(DBX_psi_f_i, 0.1f);

/**
 * Tiempo respuesta control P
 *
 * RE-CHECK this.
 *
 * @min
 * @max
 * @group DBX Advanced Controller
 */
PARAM_DEFINE_FLOAT(DBX_p_tau, 0.3f);

/**
 * Ganancia control P
 *
 * RE-CHECK this.
 *
 * @min
 * @max
 * @group DBX Advanced Controller
 */
PARAM_DEFINE_FLOAT(DBX_p_K_b, 6.0f);

/**
 * Tiempo respuesta control Q
 *
 * RE-CHECK this.
 *
 * @min
 * @max
 * @group DBX Advanced Controller
 */
PARAM_DEFINE_FLOAT(DBX_q_tau, 0.3f);

/**
 * Ganancia control Q
 *
 * RE-CHECK this.
 *
 * @min
 * @max
 * @group DBX Advanced Controller
 */
PARAM_DEFINE_FLOAT(DBX_q_K_b, 6.0f);

/**
 * Tiempo respuesta control R
 *
 * RE-CHECK this.
 *
 * @min
 * @max
 * @group DBX Advanced Controller
 */
PARAM_DEFINE_FLOAT(DBX_r_tau, 0.5f);

/**
 * Ganancia control R
 *
 * RE-CHECK this.
 *
 * @min
 * @max
 * @group DBX Advanced Controller
 */
PARAM_DEFINE_FLOAT(DBX_r_K_b, 5.0f);


/**
 * Deflexion Flaps en DEG
 *
 * RE-CHECK this.
 *
 * @min
 * @max
 * @group DBX Advanced Controller
 */
PARAM_DEFINE_FLOAT(DBX_Flaps_deg, 0.0f);

/**
 * Theta P Gain of PID cotnroller
 *
 * RE-CHECK this.
 *
 * @min
 * @max
 * @group DBX Classic Controller (PIDs)
 */
PARAM_DEFINE_FLOAT(DBXcl_theta_Kp, 3.0f);

/**
 * Phi P Gain of PID cotnroller
 *
 * RE-CHECK this.
 *
 * @min
 * @max
 * @group DBX Classic Controller (PIDs)
 */
PARAM_DEFINE_FLOAT(DBXcl_phi_Kp, 3.0f);

/**
 * Theta I Gain of PID cotnroller
 *
 * RE-CHECK this.
 *
 * @min
 * @max
 * @group DBX Classic Controller (PIDs)
 */
PARAM_DEFINE_FLOAT(DBXcl_theta_Ki, 0.1f);

/**
 * Phi I Gain of PID cotnroller
 *
 * RE-CHECK this.
 *
 * @min
 * @max
 * @group DBX Classic Controller (PIDs)
 */
PARAM_DEFINE_FLOAT(DBXcl_phi_Ki, 0.1f);

/**
 * Theta D Gain of PID cotnroller
 *
 * RE-CHECK this.
 *
 * @min
 * @max
 * @group DBX Classic Controller (PIDs)
 */
PARAM_DEFINE_FLOAT(DBXcl_theta_Kd, 0.0f);

/**
 * Phi D Gain of PID cotnroller
 *
 * RE-CHECK this.
 *
 * @min
 * @max
 * @group DBX Classic Controller (PIDs)
 */
PARAM_DEFINE_FLOAT(DBXcl_phi_Kd, 0.0f);

/**
 * Theta_dot P Gain of PID cotnroller
 *
 * RE-CHECK this.
 *
 * @min
 * @max
 * @group DBX Classic Controller (PIDs)
 */
PARAM_DEFINE_FLOAT(DBXcl_theta_dot_Kp, 0.2f);

/**
 * Phi_dot P Gain of PID cotnroller
 *
 * RE-CHECK this.
 *
 * @min
 * @max
 * @group DBX Classic Controller (PIDs)
 */
PARAM_DEFINE_FLOAT(DBXcl_phi_dot_Kp, 0.2f);

/**
 * Theta_dot I Gain of PID cotnroller
 *
 * RE-CHECK this.
 *
 * @min
 * @max
 * @group DBX Classic Controller (PIDs)
 */
PARAM_DEFINE_FLOAT(DBXcl_theta_dot_Ki, 0.0f);

/**
 * Phi I Gain of PID cotnroller
 *
 * RE-CHECK this.
 *
 * @min
 * @max
 * @group DBX Classic Controller (PIDs)
 */
PARAM_DEFINE_FLOAT(DBXcl_phi_dot_Ki, 0.0f);

// * Params of PID controller*/

/**
 * Theta_dot D Gain of PID cotnroller
 *
 * RE-CHECK this.
 *
 * @min
 * @max
 * @group DBX Classic Controller (PIDs)
 */
PARAM_DEFINE_FLOAT(DBXcl_theta_dot_Kd, 0.0f);

/**
 * Phi_dot D Gain of PID cotnroller
 *
 * RE-CHECK this.
 *
 * @min
 * @max
 * @group DBX Classic Controller (PIDs)
 */
PARAM_DEFINE_FLOAT(DBXcl_phi_dot_Kd, 0.0f);
