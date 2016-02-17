// Simulink wrapper code for PX4 (PX4FMU & Pixhawk)

#include <nuttx/config.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#define __STDC_FORMAT_MACROS
#include <inttypes.h>
#include "commander/commander_helper.h"

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_global_velocity_setpoint.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/vehicle_local_position.h>

#include <drivers/drv_led.h>
#include <drivers/drv_rgbled.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_rc_input.h>
#include <drivers/drv_tone_alarm.h>
#include <drivers/drv_hrt.h>

#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>

// Include the generated files and code of Simulink: Autopilot
#include "dbx_control_ert_rtw/dbx_control.c"
#include "dbx_control_ert_rtw/dbx_control_data.c" // Aqui se cargan los datos y constantes con que se compila Simulink
#include "dbx_control_params.c"

__EXPORT int dbx_control_main(int argc, char *argv[]);
__EXPORT int simulink_main(int argc, char *argv[]);

const char *dev_rgbled = RGBLED0_DEVICE_PATH;
const char *dev_pwm = PWM_OUTPUT0_DEVICE_PATH;
const char *h_buzzer = TONEALARM0_DEVICE_PATH;

// Declare output devices
int rgbled;
int pwm;
int buzzer;

static int simulink_task;
static bool thread_exit = true;
static bool pwm_enabled;

struct rgbled_rgbset_t{
	uint8_t red;
	uint8_t green;
	uint8_t blue;
};

int step_size = 10; // fundamental sample time (ms). 100Hz
int i = 1;

// GCS parameters
struct {
	float Throtle_sens;
	float Yaw_sens;
	float Atti_sens;
	float phi_tau;
	float phi_K_b;
	float phi_f_i;
	float theta_tau;
	float theta_K_b;
	float theta_f_i;
	float psi_tau;
	float psi_K_b;
	float psi_f_i;
	float p_tau;
	float p_K_b;
	float q_tau;
	float q_K_b;
	float r_tau;
	float r_K_b;
	float Flaps_deg;

	float PID_theta_Kp;
	float PID_phi_Kp;
	float PID_theta_Ki;
	float PID_phi_Ki;
	float PID_theta_Kd;
	float PID_phi_Kd;
	float PID_theta_dot_Kp;
	float PID_phi_dot_Kp;
	float PID_theta_dot_Ki;
	float PID_phi_dot_Ki;
	float PID_theta_dot_Kd;
	float PID_phi_dot_Kd;
}		GCS_parameters;

int simulink_main(int argc, char *argv[])
{	dbx_control_initialize();

	// declare data subscriptions
	int sensors_sub = orb_subscribe(ORB_ID(sensor_combined));
	int pwm_inputs_sub = orb_subscribe(ORB_ID(input_rc));
	int attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	int bat_status_sub = orb_subscribe(ORB_ID(battery_status));
	int airspeed_sub = orb_subscribe(ORB_ID(airspeed));

	int local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));

	//   int att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	//   int pos_sp_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
	//   int local_pos_sp_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
	//   int global_vel_sp_sub = orb_subscribe(ORB_ID(vehicle_global_velocity_setpoint));

	// Declare data structs from subcriptions
	struct sensor_combined_s        sensors;
	struct rc_input_values          pwm_inputs;
	struct vehicle_attitude_s       attitude;
	struct vehicle_gps_position_s 	gps;
	struct battery_status_s         bat_status;
	struct airspeed_s               airspeed;
	struct vehicle_local_position_s local_pos;  /*< vehicle local position */

	// Declare pointers to GCS params
	struct {
		param_t Throtle_sens;
		param_t Yaw_sens;
		param_t Atti_sens;

		param_t phi_tau;
		param_t phi_K_b;
		param_t phi_f_i;

		param_t theta_tau;
		param_t theta_K_b;
		param_t theta_f_i;

		param_t psi_tau;
		param_t psi_K_b;
		param_t psi_f_i;

		param_t p_tau;
		param_t p_K_b;

		param_t q_tau;
		param_t q_K_b;

		param_t r_tau;
		param_t r_K_b;

		param_t Flaps_deg;

		param_t PID_theta_Kp;
		param_t PID_phi_Kp;
		param_t PID_theta_Ki;
		param_t PID_phi_Ki;
		param_t PID_theta_Kd;
		param_t PID_phi_Kd;
		param_t PID_theta_dot_Kp;
		param_t PID_phi_dot_Kp;
		param_t PID_theta_dot_Ki;
		param_t PID_phi_dot_Ki;
		param_t PID_theta_dot_Kd;
		param_t PID_phi_dot_Kd;

	}	GCS_comms_pointers;

	// Get the pointers to GCS params
	GCS_comms_pointers.Throtle_sens = param_find("DBX_THRT_SENS");
	GCS_comms_pointers.Yaw_sens = param_find("DBX_YAW_SENS");
	GCS_comms_pointers.Atti_sens = param_find("DBX_ATTI_SENS");

	GCS_comms_pointers.phi_tau = param_find("DBX_PHI_TAU");
	GCS_comms_pointers.phi_K_b = param_find("DBX_PHI_KB");
	GCS_comms_pointers.phi_f_i = param_find("DBX_PHI_FI");

	GCS_comms_pointers.theta_tau = param_find("DBX_THETA_TAU");
	GCS_comms_pointers.theta_K_b = param_find("DBX_THETA_KB");
	GCS_comms_pointers.theta_f_i = param_find("DBX_THETA_FI");

	GCS_comms_pointers.psi_tau = param_find("DBX_PSI_TAU");
	GCS_comms_pointers.psi_K_b = param_find("DBX_PSI_KB");
	GCS_comms_pointers.psi_f_i = param_find("DBX_PSI_FI");

	GCS_comms_pointers.p_tau = param_find("DBX_P_TAU");
	GCS_comms_pointers.p_K_b = param_find("DBX_P_KB");

	GCS_comms_pointers.q_tau = param_find("DBX_Q_TAU");
	GCS_comms_pointers.q_K_b = param_find("DBX_Q_KB");

	GCS_comms_pointers.r_tau = param_find("DBX_R_TAU");
	GCS_comms_pointers.r_K_b = param_find("DBX_R_KB");

	GCS_comms_pointers.Flaps_deg = param_find("DBX_FLP_DEG");

	GCS_comms_pointers.PID_theta_Kp = param_find("DBCL_THTA_KP");
	GCS_comms_pointers.PID_phi_Kp = param_find("DBCL_PHI_KP");
	GCS_comms_pointers.PID_theta_Ki = param_find("DBCL_THTA_KI");
	GCS_comms_pointers.PID_phi_Ki = param_find("DBCL_PHI_KI");
	GCS_comms_pointers.PID_theta_Kd = param_find("DBCL_THTA_KD");
	GCS_comms_pointers.PID_phi_Kd = param_find("DBCL_PHI_KD");
	GCS_comms_pointers.PID_theta_dot_Kp = param_find("DBCL_THTA_D_KP");
	GCS_comms_pointers.PID_phi_dot_Kp = param_find("DBCL_PHI_D_KP");
	GCS_comms_pointers.PID_theta_dot_Ki = param_find("DBCL_THTA_D_KI");
	GCS_comms_pointers.PID_phi_dot_Ki = param_find("DBCL_PHI_D_KI");
	GCS_comms_pointers.PID_theta_dot_Kd = param_find("DBCL_THTA_D_KD");
	GCS_comms_pointers.PID_phi_dot_Kd = param_find("DBCL_PHI_D_KD");

	// Limiting the update rate
	orb_set_interval(sensors_sub, step_size);
	orb_set_interval(attitude_sub, step_size);
	orb_set_interval(airspeed_sub, step_size);

	// initialize output devices
	rgbled = open(dev_rgbled, 0);
	pwm = open(dev_pwm, 0);
	buzzer = open(h_buzzer, 0);
	// Declare here the other AUX PWM outputs???

	// initialize outputs
	ioctl(rgbled, RGBLED_SET_MODE, (unsigned long)RGBLED_MODE_ON);
	ioctl(pwm, PWM_SERVO_SET_ARM_OK, 0);
	ioctl(pwm, PWM_SERVO_ARM, 0);
	pwm_enabled = 0;

	struct pollfd fds[] = {
		{ .fd = sensors_sub, .events = POLLIN },
	};

	int test_time = 3000; // Test time ms
	if(!dbx_test_rc(test_time)) {
		// primary application thread
		PX4_INFO("Starting DBX Control Thread");
		while (!thread_exit) {
			int poll_return = poll(fds, 1, 1000);
			if (poll_return > 0) {
				if (fds[0].revents & POLLIN) {
					// assign sensor data
					orb_copy(ORB_ID(sensor_combined), sensors_sub, &sensors);
					orb_copy(ORB_ID(vehicle_attitude), attitude_sub, &attitude);
					orb_copy(ORB_ID(vehicle_gps_position), gps_sub, &gps);
					orb_copy(ORB_ID(input_rc), pwm_inputs_sub, &pwm_inputs);
					orb_copy(ORB_ID(battery_status), bat_status_sub, &bat_status);
					orb_copy(ORB_ID(airspeed), airspeed_sub, &airspeed);
					orb_copy(ORB_ID(vehicle_local_position), local_pos_sub, &local_pos);

					// Filtering noise in PWM Radio inputs
					for(int q=0;q<=7;q++) {
						if(pwm_inputs.values[q]<(rc_input_t) 1000) pwm_inputs.values[q]= (rc_input_t) 1000;
						if(pwm_inputs.values[q]>(rc_input_t) 2000) pwm_inputs.values[q]= (rc_input_t) 2000;
					}

					// Declare Simulink inputs
					dbx_control_U.runtime = hrt_absolute_time();
					dbx_control_U.mag_x = sensors.magnetometer_ga[0];
					dbx_control_U.mag_y = sensors.magnetometer_ga[1];
					dbx_control_U.mag_z = sensors.magnetometer_ga[2];
					dbx_control_U.acc_x = sensors.accelerometer_m_s2[0];
					dbx_control_U.acc_y = sensors.accelerometer_m_s2[1];
					dbx_control_U.acc_z = sensors.accelerometer_m_s2[2];
					dbx_control_U.gyro_x = sensors.gyro_rad_s[0];
					dbx_control_U.gyro_y = sensors.gyro_rad_s[1];
					dbx_control_U.gyro_z = sensors.gyro_rad_s[2];
					dbx_control_U.rate_roll = attitude.rollspeed;
					dbx_control_U.rate_pitch = attitude.pitchspeed;
					dbx_control_U.rate_yaw = attitude.yawspeed;
					dbx_control_U.att_roll = attitude.roll;
					dbx_control_U.att_pitch = attitude.pitch;
					dbx_control_U.att_yaw = attitude.yaw;
					dbx_control_U.q0 = attitude.q[0];
					dbx_control_U.q1 = attitude.q[1];
					dbx_control_U.q2 = attitude.q[2];
					dbx_control_U.q3 = attitude.q[3];
					dbx_control_U.baro_alt = sensors.baro_alt_meter[0]; // There are up to 3 baros, Use the one you need
					dbx_control_U.gps_sat = gps.satellites_used;
					dbx_control_U.gps_lat = 0.0000001*(double)gps.lat;
					dbx_control_U.gps_lon = 0.0000001*(double)gps.lon;
					dbx_control_U.gps_alt = 0.001*(double)gps.alt;
					dbx_control_U.gps_vel = gps.vel_m_s;
					dbx_control_U.gps_vel_n = gps.vel_n_m_s;
					dbx_control_U.gps_vel_e = gps.vel_e_m_s;
					dbx_control_U.gps_vel_d = gps.vel_d_m_s;
					dbx_control_U.ch1 = pwm_inputs.values[0];
					dbx_control_U.ch2 = pwm_inputs.values[1];
					dbx_control_U.ch3 = pwm_inputs.values[2];
					dbx_control_U.ch4 = pwm_inputs.values[3];
					dbx_control_U.ch5 = pwm_inputs.values[4];
					dbx_control_U.ch6 = pwm_inputs.values[5];
					dbx_control_U.ch7 = pwm_inputs.values[6];
					dbx_control_U.ch8 = pwm_inputs.values[7];

					/*----- Added inputs ---------*/
					dbx_control_U.gps_pdop = gps.eph; // pdop or hdop
					dbx_control_U.gps_vdop = gps.epv; // vdop
					dbx_control_U.bat_volts = bat_status.voltage_filtered_v; // Batery volts
					dbx_control_U.pitot_diff_pre = sensors.differential_pressure_filtered_pa[0]; // Pitot presion dinamica
					dbx_control_U.TAS_mps = airspeed.true_airspeed_m_s; // TAS estimada

					if (i < 10) { // 10Hz loop
						i = i++;
					} else {
						// check arm state
						if (dbx_control_Y.pwm_arm == 1 && pwm_enabled == 0 && pwm_inputs.channel_count == 8) {
							// arm system
							pwm_enabled = 1;
							ioctl(buzzer, TONE_SET_ALARM, TONE_ARMING_WARNING_TUNE);
							printf("\t  ARMED\t\t  ARMED\t\t  ARMED\t\t  ARMED\t\t  ARMED\t\t  ARMED\t\t  ARMED\t\t  ARMED\n");
						} else if (dbx_control_Y.pwm_arm == 0 && pwm_enabled == 1) {
							// disarm system
							ioctl(pwm, PWM_SERVO_SET(0), 900);
							ioctl(pwm, PWM_SERVO_SET(1), 900);
							ioctl(pwm, PWM_SERVO_SET(2), 900);
							ioctl(pwm, PWM_SERVO_SET(3), 1500);
							ioctl(pwm, PWM_SERVO_SET(4), 1500);
							ioctl(pwm, PWM_SERVO_SET(5), 1500);
							ioctl(pwm, PWM_SERVO_SET(6), 1600);
							ioctl(pwm, PWM_SERVO_SET(7), 1500);
							pwm_enabled = 0;
							ioctl(buzzer, TONE_SET_ALARM, TONE_ARMING_FAILURE_TUNE);
							printf("\tDISARMED\tDISARMED\tDISARMED\tDISARMED\tDISARMED\tDISARMED\tDISARMED\tDISARMED\n");
						}

						// Read GCS parameters
						param_get(GCS_comms_pointers.Throtle_sens, 		&(GCS_parameters.Throtle_sens));
						param_get(GCS_comms_pointers.Yaw_sens,          &(GCS_parameters.Yaw_sens));
						param_get(GCS_comms_pointers.Atti_sens, 		&(GCS_parameters.Atti_sens));
						param_get(GCS_comms_pointers.phi_tau,           &(GCS_parameters.phi_tau));
						param_get(GCS_comms_pointers.phi_K_b,           &(GCS_parameters.phi_K_b));
						param_get(GCS_comms_pointers.phi_f_i,           &(GCS_parameters.phi_f_i));
						param_get(GCS_comms_pointers.theta_tau,         &(GCS_parameters.theta_tau));
						param_get(GCS_comms_pointers.theta_K_b,         &(GCS_parameters.theta_K_b));
						param_get(GCS_comms_pointers.theta_f_i,         &(GCS_parameters.theta_f_i));
						param_get(GCS_comms_pointers.psi_tau,           &(GCS_parameters.psi_tau));
						param_get(GCS_comms_pointers.psi_K_b,           &(GCS_parameters.psi_K_b));
						param_get(GCS_comms_pointers.psi_f_i,           &(GCS_parameters.psi_f_i));
						param_get(GCS_comms_pointers.p_tau,             &(GCS_parameters.p_tau));
						param_get(GCS_comms_pointers.p_K_b,             &(GCS_parameters.p_K_b));
						param_get(GCS_comms_pointers.q_tau,             &(GCS_parameters.q_tau));
						param_get(GCS_comms_pointers.q_K_b,             &(GCS_parameters.q_K_b));
						param_get(GCS_comms_pointers.r_tau,             &(GCS_parameters.r_tau));
						param_get(GCS_comms_pointers.r_K_b,             &(GCS_parameters.r_K_b));
						param_get(GCS_comms_pointers.Flaps_deg,         &(GCS_parameters.Flaps_deg));

						param_get(GCS_comms_pointers.PID_theta_Kp,  	&(GCS_parameters.PID_theta_Kp));
						param_get(GCS_comms_pointers.PID_phi_Kp,  		&(GCS_parameters.PID_phi_Kp));
						param_get(GCS_comms_pointers.PID_theta_Ki, 		&(GCS_parameters.PID_theta_Ki));
						param_get(GCS_comms_pointers.PID_phi_Ki,  		&(GCS_parameters.PID_phi_Ki));
						param_get(GCS_comms_pointers.PID_theta_Kd,  	&(GCS_parameters.PID_theta_Kd));
						param_get(GCS_comms_pointers.PID_phi_Kd,  		&(GCS_parameters.PID_phi_Kd));
						param_get(GCS_comms_pointers.PID_theta_dot_Kp,  &(GCS_parameters.PID_theta_dot_Kp));
						param_get(GCS_comms_pointers.PID_phi_dot_Kp,  	&(GCS_parameters.PID_phi_dot_Kp));
						param_get(GCS_comms_pointers.PID_theta_dot_Ki,  &(GCS_parameters.PID_theta_dot_Ki));
						param_get(GCS_comms_pointers.PID_phi_dot_Ki,  	&(GCS_parameters.PID_phi_dot_Ki));
						param_get(GCS_comms_pointers.PID_theta_dot_Kd,  &(GCS_parameters.PID_theta_dot_Kd));
						param_get(GCS_comms_pointers.PID_phi_dot_Kd,  	&(GCS_parameters.PID_phi_dot_Kd));

						// Declarar las ganancias de Simulink: se podria necesitar Casting!
						dbx_control_P.Throtle_sens 	= GCS_parameters.Throtle_sens;
						dbx_control_P.Yaw_sens  	= GCS_parameters.Yaw_sens;
						dbx_control_P.Roll_pich_sens	= GCS_parameters.Atti_sens;
						dbx_control_P.phi_tau  		= GCS_parameters.phi_tau;
						dbx_control_P.phi_K_b  		= GCS_parameters.phi_K_b;
						dbx_control_P.phi_f_i  		= GCS_parameters.phi_f_i;
						dbx_control_P.theta_tau  	= GCS_parameters.theta_tau;
						dbx_control_P.theta_K_b  	= GCS_parameters.theta_K_b;
						dbx_control_P.theta_f_i  	= GCS_parameters.theta_f_i;
						//             dbx_control_P.psi_tau  = GCS_parameters.psi_tau; // Estas salidas estaban cuando habia control en heading
						//             dbx_control_P.psi_K_b  = GCS_parameters.psi_K_b;  // Descomentarlas si se activa heading control en simulink
						//             dbx_control_P.psi_f_i  = GCS_parameters.psi_f_i;
						dbx_control_P.p_tau  = GCS_parameters.p_tau;
						dbx_control_P.p_K_b  = GCS_parameters.p_K_b;
						dbx_control_P.q_tau  = GCS_parameters.q_tau;
						dbx_control_P.q_K_b  = GCS_parameters.q_K_b;
						dbx_control_P.r_tau  = GCS_parameters.r_tau;
						dbx_control_P.r_K_b  = GCS_parameters.r_K_b;
						dbx_control_P.Flaps_ang_deg  	= GCS_parameters.Flaps_deg;

						dbx_control_P.PID_theta_Kp  	= GCS_parameters.PID_theta_Kp;
						dbx_control_P.PID_phi_Kp  		= GCS_parameters.PID_phi_Kp;

						dbx_control_P.PID_theta_Ki  	= GCS_parameters.PID_theta_Ki;
						dbx_control_P.PID_phi_Ki  		= GCS_parameters.PID_phi_Ki;

						dbx_control_P.PID_theta_Kd  	= GCS_parameters.PID_theta_Kd;
						dbx_control_P.PID_phi_Kd  		= GCS_parameters.PID_phi_Kd;

						dbx_control_P.PID_theta_dot_Kp  = GCS_parameters.PID_theta_dot_Kp;
						dbx_control_P.PID_phi_dot_Kp  	= GCS_parameters.PID_phi_dot_Kp;

						dbx_control_P.PID_theta_dot_Ki  = GCS_parameters.PID_theta_dot_Ki;
						dbx_control_P.PID_phi_dot_Ki  	= GCS_parameters.PID_phi_dot_Ki;

						dbx_control_P.PID_theta_dot_Kd  = GCS_parameters.PID_theta_dot_Kd;
						dbx_control_P.PID_phi_dot_Kd  	= GCS_parameters.PID_phi_dot_Kd;

						// output FMU LED signals
						if (dbx_control_Y.led_blue == 1) {
							led_on(LED_BLUE);
						} else {
							led_off(LED_BLUE);
						}
						if (dbx_control_Y.led_red == 1) {
							led_on(LED_RED);
						} else {
							led_off(LED_RED);
						}
						// output RGBLED signals
						rgbled_rgbset_t rgb_value;
						rgb_value.red = dbx_control_Y.rgb_red;
						rgb_value.green = dbx_control_Y.rgb_green;
						rgb_value.blue = dbx_control_Y.rgb_blue;
						ioctl(rgbled, RGBLED_SET_RGB, (unsigned long)&rgb_value);

						// //print debug data
						// printf("%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%i\n",
						// (double)(dbx_control_U.runtime/1000000),
						// (double)dbx_control_Y.debug1,
						// (double)dbx_control_Y.debug2,
						// (double)dbx_control_Y.debug3,
						// (double)dbx_control_Y.debug4,
						// (double)dbx_control_Y.debug5,
						// (double)dbx_control_Y.debug6,
						// (double)dbx_control_Y.debug7,
						// (double)dbx_control_Y.debug8,
						// pwm_inputs.channel_count);

						// Print Simulink outputs
						/* printf("%i\t%i\t%i\t%i\t%i\t%i\t%i\t%i\t%i\t%i\n",
						(int)(dbx_control_U.runtime/1000000),
						(int)dbx_control_Y.pwm1,
						(int)dbx_control_Y.pwm2,
						(int)dbx_control_Y.pwm3,
						(int)dbx_control_Y.pwm4,
						(int)dbx_control_Y.pwm5,
						(int)dbx_control_Y.pwm6,
						(int)dbx_control_Y.pwm7,
						(int)dbx_control_Y.pwm8,
						pwm_inputs.channel_count); */

						// Print Radio pwm inputs
// 						printf("%i\t%i\t%i\t%i\t%i\t%i\t%i\t%i\t%i\t%i\n",
// 						(int)(dbx_control_U.runtime/1000000),
// 						pwm_inputs.values[0],
// 						pwm_inputs.values[1],
// 						pwm_inputs.values[2],
// 						pwm_inputs.values[3],
// 						pwm_inputs.values[4],
// 						pwm_inputs.values[5],
// 						pwm_inputs.values[6],
// 						pwm_inputs.values[7],
// 						pwm_inputs.channel_count);

						/*
						// Sensors debuging and testing
						printf("%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t\n",
						(double)(dbx_control_U.runtime/1000000),
						(double)local_pos.x,
						(double)local_pos.y,
						(double)local_pos.z,
						(double)gps.epv,
						(double)local_pos.vx,
						(double)local_pos.vy,
						(double)local_pos.vz,
						(double)airspeed.true_airspeed_m_s,
						(double)sensors.baro_alt_meter[0], // TO BE verified: Is it correct altitude?
						(double)sensors.accelerometer_m_s2[2],
						(double)sensors.gyro_rad_s[0],
						(double)bat_status.voltage_filtered_v,
						(double)attitude.pitch,
						(double)attitude.yaw); */
						i = 1;
					}
                    
					// MATLAB INPUT FAILSAFE
					if (pwm_inputs.channel_count != 8) {
						dbx_control_U.ch1 = 1500;
						dbx_control_U.ch2 = 1500;
						dbx_control_U.ch3 = 1000;
						dbx_control_U.ch4 = 1500;
						dbx_control_U.ch5 = 1000;
						dbx_control_U.ch6 = 1000;
						dbx_control_U.ch7 = 0;
					}

					// output pwm signals
					if (pwm_enabled == 1 && pwm_inputs.channel_count == 8) {
						ioctl(pwm, PWM_SERVO_SET(0), dbx_control_Y.pwm1);
						ioctl(pwm, PWM_SERVO_SET(1), dbx_control_Y.pwm2);
						ioctl(pwm, PWM_SERVO_SET(2), dbx_control_Y.pwm3);
						ioctl(pwm, PWM_SERVO_SET(3), dbx_control_Y.pwm4);
						ioctl(pwm, PWM_SERVO_SET(4), dbx_control_Y.pwm5);
						ioctl(pwm, PWM_SERVO_SET(5), dbx_control_Y.pwm6);
						ioctl(pwm, PWM_SERVO_SET(6), dbx_control_Y.pwm7);
						ioctl(pwm, PWM_SERVO_SET(7), dbx_control_Y.pwm8);
                        
                        printf("%i\t%i\t%i\t%i\t%i\t%i\t%i\t%i\t%i\t%i\n",
						(int)(dbx_control_U.runtime/1000000),
						(int)dbx_control_Y.pwm1,
						(int)dbx_control_Y.pwm2,
						(int)dbx_control_Y.pwm3,
						(int)dbx_control_Y.pwm4,
						(int)dbx_control_Y.pwm5,
						(int)dbx_control_Y.pwm6,
						(int)dbx_control_Y.pwm7,
						(int)dbx_control_Y.pwm8,
						pwm_inputs.channel_count);
                        
					} else { // Disarmed or failsafe
						failsafe_pwm_output(pwm);
					}
					// execute simulink code
					dbx_control_step();
				}
			}
		}
	}
	// disable pwm outputs
	failsafe_pwm_output(pwm);
	ioctl(pwm, PWM_SERVO_DISARM, 0);
	// disable LEDs
	led_off(LED_BLUE);
	led_off(LED_RED);
	ioctl(rgbled, RGBLED_SET_MODE, (unsigned long)RGBLED_MODE_OFF);
	// close sensor subscriptions
	close(sensors_sub);
	close(attitude_sub);
	close(pwm_inputs_sub);
	close(gps_sub);
	// terminate application thread
	exit(0);
}

int dbx_control_main(int argc, char *argv[])
{	// start primary application thread
	if (!strcmp(argv[1], "start")) {

		if (!thread_exit) {
			warnx("already running");
			/* this is not an error */
			return 0;
		}

		thread_exit = false;
		simulink_task = px4_task_spawn_cmd("dbx_control",
		SCHED_DEFAULT,
		SCHED_PRIORITY_MAX - 15,
		10240,
		simulink_main,
		NULL);
		PX4_INFO("dbx_control started");
		exit(0);
	}

	// terminate primary application thread
	if (!strcmp(argv[1], "stop")) {
		thread_exit = true;
		exit(0);
	}

	exit(1);
}

int failsafe_pwm_output(int pwm) {
	ioctl(pwm, PWM_SERVO_SET(0), 900);
	ioctl(pwm, PWM_SERVO_SET(1), 900);
	ioctl(pwm, PWM_SERVO_SET(2), 900);
	ioctl(pwm, PWM_SERVO_SET(3), 1500);
	ioctl(pwm, PWM_SERVO_SET(4), 1500);
	ioctl(pwm, PWM_SERVO_SET(5), 1500);
	ioctl(pwm, PWM_SERVO_SET(6), 1600); // Disarmed indication
	ioctl(pwm, PWM_SERVO_SET(7), 1500);
	return(1);
}


int dbx_test_rc(int duration)
{	int _rc_sub = orb_subscribe(ORB_ID(input_rc));
		
	//int buzzer = open(h_buzzer, 0);
	/* read low-level values from FMU or IO RC inputs (PPM, Spektrum, S.Bus) */
	struct rc_input_values	rc_input;
	struct rc_input_values	rc_last;
	orb_copy(ORB_ID(input_rc), _rc_sub, &rc_input);
	PX4_INFO("Input RC read %i", (int)rc_input.values[1]);
	usleep(100000);

	/* open PPM input and expect values close to the output values */
	bool rc_updated;
	orb_check(_rc_sub, &rc_updated);

	PX4_INFO("Reading PPM values - press any key to abort");
	PX4_INFO("This test guarantees: 10 Hz update rates, no glitches (channel values), no channel count changes.");

	if (rc_updated) {
		/* copy initial set */
		for (unsigned j = 0; j < rc_input.channel_count; j++) {
			rc_last.values[j] = rc_input.values[j];
		}

		rc_last.channel_count = rc_input.channel_count;


		struct pollfd rc_fds[] = {
		{
			.fd = _rc_sub,
			.events = POLLIN },
		};
		/* poll descriptor */
		//struct pollfd fds[2];
		//fds[0].fd = _rc_sub;
		//fds[0].events = POLLIN;
		//fds[1].fd = 0;
		//fds[1].events = POLLIN;

		uint64_t rc_start_time = hrt_absolute_time();
		while (hrt_absolute_time() - rc_start_time < (uint64_t)(duration * 1000)) {
			PX4_INFO("I heard: [%" PRIu64 "\t%" PRIu64 "]]", (hrt_absolute_time() - rc_start_time), (uint64_t)(duration * 1000));
			
			int ret = poll(rc_fds, 1, 200);
			PX4_INFO("Poll finished with result %i", ret);
			if (ret > 0) {
				if (rc_fds[0].revents & POLLIN) {
					orb_copy(ORB_ID(input_rc), _rc_sub, &rc_input);
					PX4_INFO("Input RC read %i", (int)rc_input.values[1]);
                    
					// Out-of-bounds check
					for (unsigned i = 0; i < rc_input.channel_count; i++) {
						if (abs(rc_input.values[i] - rc_last.values[i]) > 20) {
							PX4_ERR("comparison fail: RC: %d, expected: %d", rc_input.values[i], rc_last.values[i]);
							(void)close(_rc_sub);
							ioctl(buzzer, TONE_SET_ALARM, TONE_ARMING_FAILURE_TUNE);
							return 1;
						}

						rc_last.values[i] = rc_input.values[i];
					}
                    // Channel count check
					if (rc_last.channel_count != rc_input.channel_count) {
						PX4_ERR("channel count mismatch: last: %d, now: %d", rc_last.channel_count, rc_input.channel_count);
						(void)close(_rc_sub);
						ioctl(buzzer, TONE_SET_ALARM, TONE_ARMING_FAILURE_TUNE);
						return 1;
					}
                    // Frequency check
					if (hrt_absolute_time() - rc_input.timestamp_last_signal > 100000) {
						PX4_ERR("TIMEOUT, less than 10 Hz updates");
						(void)close(_rc_sub);
						ioctl(buzzer, TONE_SET_ALARM, TONE_ARMING_FAILURE_TUNE);
						return 1;
					}
                    
                    // Normal start Radio inputs check
					if ( abs(rc_input.values[2] - 1000)>200 && abs(rc_input.values[4] - 1000)>200 && abs(rc_input.values[5] - 1000)>200) {
						PX4_ERR(" Radio inputs are not safe for starting");
						(void)close(_rc_sub);
						ioctl(buzzer, TONE_SET_ALARM, TONE_ARMING_FAILURE_TUNE);
						return 1;
					}
                    
				} else {
					/* key pressed, bye bye */
					return 1;
				}
			}
		}

	} else {
		PX4_ERR("failed reading RC input data");
		(void)close(_rc_sub);
		ioctl(buzzer, TONE_SET_ALARM, TONE_ARMING_FAILURE_TUNE);
		return 1;
	}
	PX4_INFO("RC IN CONTINUITY TEST PASSED SUCCESSFULLY!");
	ioctl(buzzer, TONE_SET_ALARM, TONE_STARTUP_TUNE);
	return 0;
}
