/****************************************************************************
 *
 *   Copyright (c) 2016 DBX Drones Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file dbx_control_params.c
 * DBX Control parameters
 *
 * @author Juan Herrero <jn.herrerom@gmail.com>
 */
#include <px4_config.h>
#include <systemlib/param/param.h>

/**
 * Sensibilidad throtle en m/s2
 *
 * RE-CHECK this.
 *
 * @unit unknown
 * @min 0
 * @max 100
 * @group DBX Control
 */
PARAM_DEFINE_FLOAT(DBX_THROTTLE_SENS, 5.0f);

/**
 * Sensibildiad ginada en DEG
 *
 * RE-CHECK this.
 *
 * @unit unknown
 * @min 0
 * @max 100
 * @group DBX Control
 */
PARAM_DEFINE_FLOAT(DBX_YAW_SENS, 20.0f);

/**
 * Sensibilidad de cabeceo y balanceo en DEG
 *
 * RE-CHECK this.
 *
 * @unit unknown
 * @min 0
 * @max 100
 * @group DBX Control
 */
PARAM_DEFINE_FLOAT(DBX_ATTI_SENS, 10.0f);

/**
 * Tiempo de respuesta PHI
 *
 * RE-CHECK this.
 *
 * @unit unknown
 * @min 0
 * @max 100
 * @group DBX Control
 */
PARAM_DEFINE_FLOAT(DBX_PHI_TAU, 0.7f);
/**
 * Ganancia control PHI
 *
 * RE-CHECK this.
 *
 * @unit unknown
 * @min 0
 * @max 100
 * @group DBX Control
 */
PARAM_DEFINE_FLOAT(DBX_PHI_K_B, 3.0f);

/**
 * Integral control PHI
 *
 * RE-CHECK this.
 *
 * @unit unknown
 * @min 0
 * @max 100
 * @group DBX Control
 */
PARAM_DEFINE_FLOAT(DBX_PHI_F_I, 0.2f);


/**
 * Tiempo de respuesta THETA
 *
 * RE-CHECK this.
 *
 * @unit unknown
 * @min 0
 * @max 100
 * @group DBX Control
 */
PARAM_DEFINE_FLOAT(DBX_THETA_TAU, 0.7f);
/**
 * Ganancia control THETA
 *
 * RE-CHECK this.
 *
 * @unit unknown
 * @min 0
 * @max 100
 * @group DBX Control
 */
PARAM_DEFINE_FLOAT(DBX_THETA_K_B, 3.0f);

/**
 * Integral control THETA
 *
 * RE-CHECK this.
 *
 * @unit unknown
 * @min 0
 * @max 100
 * @group DBX Control
 */
PARAM_DEFINE_FLOAT(DBX_THETA_F_I, 0.2f);
DBX_Atti_sens
/**
 * Tiempo de respuesta PSI
 *
 * RE-CHECK this.
 *
 * @unit unknown
 * @min 0
 * @max 100
 * @group DBX Control
 */
PARAM_DEFINE_FLOAT(DBX_PSI_TAU, 4.0f);
/**
 * Ganancia control PSI
 *
 * RE-CHECK this.
 *
 * @unit unknown
 * @min 0
 * @max 100
 * @group DBX Control
 */
PARAM_DEFINE_FLOAT(DBX_PSI_K_B, 2.5f);

/**
 * Integral control PSI
 *
 * RE-CHECK this.
 *
 * @unit unknown
 * @min 0
 * @max 100
 * @group DBX Control
 */
PARAM_DEFINE_FLOAT(DBX_PSI_F_I, 0.1f);

/**
 * Tiempo respuesta control P
 *
 * RE-CHECK this.
 *
 * @unit unknown
 * @min 0
 * @max 100
 * @group DBX Control
 */
PARAM_DEFINE_FLOAT(DBX_P_TAU, 0.3f);

/**
 * Ganancia control P
 *
 * RE-CHECK this.
 *
 * @unit unknown
 * @min 0
 * @max 100
 * @group DBX Control
 */
PARAM_DEFINE_FLOAT(DBX_P_K_B, 6.0f);

/**
 * Tiempo respuesta control Q
 *
 * RE-CHECK this.
 *
 * @unit unknown
 * @min 0
 * @max 100
 * @group DBX Control
 */
PARAM_DEFINE_FLOAT(DBX_Q_TAU, 0.3f);

/**
 * Ganancia control Q
 *
 * RE-CHECK this.
 *
 * @unit unknown
 * @min 0
 * @max 100
 * @group DBX Control
 */
PARAM_DEFINE_FLOAT(DBX_Q_K_B, 6.0f);

/**
 * Tiempo respuesta control R
 *
 * RE-CHECK this.
 *
 * @unit unknown
 * @min 0
 * @max 100
 * @group DBX Control
 */
PARAM_DEFINE_FLOAT(DBX_R_TAU, 0.5f);

/**
 * Ganancia control R
 *
 * RE-CHECK this.
 *
 * @unit unknown
 * @min 0
 * @max 100
 * @group DBX Control
 */
PARAM_DEFINE_FLOAT(DBX_R_K_B, 5.0f);


/**
 * Deflexion Flaps en DEG
 *
 * RE-CHECK this.
 *
 * @unit unknown
 * @min 0
 * @max 100
 * @group DBX Control
 */
PARAM_DEFINE_FLOAT(DBX_FLAPS_DEG, 0.0f);

/**
 * Theta P Gain of PID cotnroller
 *
 * RE-CHECK this.
 *
 * @unit unknown
 * @min 0
 * @max 100
 * @group DBX_CL Control
 */
PARAM_DEFINE_FLOAT(DBX_CL_THETA_KP, 3.0f);

/**
 * Phi P Gain of PID cotnroller
 *
 * RE-CHECK this.
 *
 * @unit unknown
 * @min 0
 * @max 100
 * @group DBX_CL Control
 */
PARAM_DEFINE_FLOAT(DBX_CL_PHI_KP, 3.0f);

/**
 * Theta I Gain of PID cotnroller
 *
 * RE-CHECK this.
 *
 * @unit unknown
 * @min 0
 * @max 100
 * @group DBX_CL Control
 */
PARAM_DEFINE_FLOAT(DBX_CL_THETA_KI, 0.1f);

/**
 * Phi I Gain of PID cotnroller
 *
 * RE-CHECK this.
 *
 * @unit unknown
 * @min 0
 * @max 100
 * @group DBX_CL Control
 */
PARAM_DEFINE_FLOAT(DBX_CL_PHI_KI, 0.1f);

/**
 * Theta D Gain of PID cotnroller
 *
 * RE-CHECK this.
 *
 * @unit unknown
 * @min 0
 * @max 100
 * @group DBX_CL Control
 */
PARAM_DEFINE_FLOAT(DBX_CL_THETA_KD, 0.0f);

/**
 * Phi D Gain of PID cotnroller
 *
 * RE-CHECK this.
 *
 * @unit unknown
 * @min 0
 * @max 100
 * @group DBX_CL Control
 */
PARAM_DEFINE_FLOAT(DBX_CL_PHI_KD, 0.0f);

/**
 * Theta_dot P Gain of PID cotnroller
 *
 * RE-CHECK this.
 *
 * @unit unknown
 * @min 0
 * @max 100
 * @group DBX_CL Control
 */
PARAM_DEFINE_FLOAT(DBX_CL_THETA_DOT_KP, 0.2f);

/**
 * Phi_dot P Gain of PID cotnroller
 *
 * RE-CHECK this.
 *
 * @unit unknown
 * @min 0
 * @max 100
 * @group DBX_CL Control
 */
PARAM_DEFINE_FLOAT(DBX_CL_PHI_DOT_KP, 0.2f);

/**
 * Theta_dot I Gain of PID cotnroller
 *
 * RE-CHECK this.
 *
 * @unit unknown
 * @min 0
 * @max 100
 * @group DBX_CL Control
 */
PARAM_DEFINE_FLOAT(DBX_CL_THETA_DOT_KI, 0.0f);

/**
 * Phi I Gain of PID cotnroller
 *
 * RE-CHECK this.
 *
 * @unit unknown
 * @min 0
 * @max 100
 * @group DBX_CL Control
 */
PARAM_DEFINE_FLOAT(DBX_CL_PHI_DOT_KI, 0.0f);

// * Params of PID controller*/

/**
 * Theta_dot D Gain of PID cotnroller
 *
 * RE-CHECK this.
 *
 * @min
 * @max
 * @group DBX_CL Control
 */
PARAM_DEFINE_FLOAT(DBX_CL_THETA_DOT_KD, 0.0f);

/**
 * Phi_dot D Gain of PID cotnroller
 *
 * RE-CHECK this.
 *
 * @min
 * @max
 * @group DBX_CL Control
 */
PARAM_DEFINE_FLOAT(DBX_CL_PHI_DOT_KD, 0.0f);
