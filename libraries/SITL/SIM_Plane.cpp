/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  very simple plane simulator class. Not aerodynamically accurate,
  just enough to be able to debug control logic for new frame types
*/

#include "SIM_Plane.h"

#include <stdio.h>
#include <AP_Filesystem/AP_Filesystem_config.h>

using namespace SITL;

Plane::Plane(const char *frame_str) :
    Aircraft(frame_str)
{
    mass = 2.0f;

    /*
       scaling from motor power to Newtons. Allows the plane to hold
       vertically against gravity when the motor is at hover_throttle
    */
    thrust_scale = (mass * GRAVITY_MSS) / hover_throttle;
    frame_height = 0.1f;

    ground_behavior = GROUND_BEHAVIOR_FWD_ONLY;
    lock_step_scheduled = true;

    if (strstr(frame_str, "-heavy")) {
        mass = 8;
    }
    if (strstr(frame_str, "-jet")) {
        // a 22kg "jet", level top speed is 102m/s
        mass = 22;
        thrust_scale = (mass * GRAVITY_MSS) / hover_throttle;
    }
    if (strstr(frame_str, "-revthrust")) {
        reverse_thrust = true;
    }
    if (strstr(frame_str, "-elevon")) {
        elevons = true;
    } else if (strstr(frame_str, "-vtail")) {
        vtail = true;
    } else if (strstr(frame_str, "-dspoilers")) {
        dspoilers = true;
    }
    if (strstr(frame_str, "-elevrev")) {
        reverse_elevator_rudder = true;
    }
    if (strstr(frame_str, "-catapult")) {
        have_launcher = true;
        launch_accel = 15;
        launch_time = 2;
    }
    if (strstr(frame_str, "-bungee")) {
        have_launcher = true;
        launch_accel = 7;
        launch_time = 4;
    }
    if (strstr(frame_str, "-throw")) {
        have_launcher = true;
        launch_accel = 25;
        launch_time = 0.4;
    }
    if (strstr(frame_str, "-tailsitter")) {
        tailsitter = true;
        ground_behavior = GROUND_BEHAVIOR_TAILSITTER;
        thrust_scale *= 1.5;
    }
    if (strstr(frame_str, "-steering")) {
        have_steering = true;
    }

#if AP_FILESYSTEM_FILE_READING_ENABLED
    if (strstr(frame_str, "-3d")) {
        aerobatic = true;
        thrust_scale *= 1.5;
        // setup parameters for plane-3d
        AP_Param::load_defaults_file("@ROMFS/models/plane.parm", false);
        AP_Param::load_defaults_file("@ROMFS/models/plane-3d.parm", false);
    }
#endif

    if (strstr(frame_str, "-ice")) {
        ice_engine = true;
    }

    if (strstr(frame_str, "-soaring")) {
        mass = 2.0;
        coefficient.c_drag_p = 0.05;
    }

    df200_frame = false;
    df200_aero_log_last_ms = 0;

    if (strstr(frame_str, "-df200")) {
        df200_frame = true;
        // DF-200: geometry + longitudinal from xlsx; lateral/dir mostly stock
        // but roll *control* softened for small-I SITL step-up.
        //
        // 083: with stock c_l_deltaa=0.25 on DF wing (S=5.29,b=7.5) and
        // Ix≈20, full aileron ⇒ α̇_roll ~1e4 deg/s² — C1 and Roll lock
        // violently. Keep stock weathercock/damping (Cnb/Clp), but cut
        // aileron effectiveness toward xlsx (raw dCl/dδ·dmax≈0.09).
        // Named df200 (not jt200) to avoid strstr prefix clash with jt20.
        coefficient.s = 5.28846f;
        coefficient.b = 7.5f;
        coefficient.c = 0.70513f;
        coefficient.c_lift_0 = 0.311f;     // CL @ α≈0, β=0
        coefficient.c_lift_a = 6.023f;     // 1/rad, fit α=-2..8°
        coefficient.c_drag_p = 0.03195f;   // min CD near α=-2°
        coefficient.oswald = 0.9f;
        coefficient.alpha_stall = 0.2443f; // ~14°
        coefficient.c_m_a = -0.487f;       // 1/rad, Cm vs α fit
        coefficient.c_m_q = -19.287f;      // pitch damping
        // Clp from 动导数 AVL = -0.5873 (not stock -1.0).
        coefficient.c_l_p = -0.5873f;
        // Isolation (114): VTOL R/P/Y mix already 0 at AS≥25 and Mot span≈0,
        // but still blew at AS≈27. Zero ALL control-surface aero derivatives so
        // aileron/elevator/rudder produce no force/moment. Bare-airframe aero
        // (CL, Cma, Clp, ...) kept. If AS≈27 then flies → surfaces were the
        // problem; if still blows → bare wing/aero or something else.
        // (xlsx aileron map was c_l_deltaa≈0.091; restore after this test.)
        // Surfaces: FULL handover (153). 152 first crossed the old AS≈27 wall
        // (Cnβ restored + Excel Cnr + rudder + coupling loop cut) and reached
        // "Transition airspeed reached 40.1". But in the TIMER phase
        // (tilt>30°, AS≈47) roll diverged with Ail saturated at 570 and Elev
        // at 333 - both still had ZERO aero derivatives, so once the Lua mix
        // faded (R->0 at AS≥25) roll/pitch had no actuator at all.
        // Restore xlsx-fitted aileron/elevator effectiveness (NOT stock -
        // stock c_l_deltaa=0.25 on this wing locked roll violently at small
        // I; xlsx fits: c_l_deltaa≈0.091, c_m_deltae≈0.42). Rudder trio was
        // already restored in the 151 heading-control block below.
        // 155: TIMER roll snap persists even with ALL Lua floors = 0 (mix
        // factors verified R=P=Y≈0 at AS≥25, isolation worked). New per-motor
        // log shows a persistent net rotor Mx≈-25 Nm from imperfect pairwise
        // cancellation (outer pairs at slightly different thrust), while
        // aileron 0.091 gives only La_da≈-5..-9 Nm - cannot hold the drift.
        // Raise aileron effectiveness 0.091 -> 0.15 so the FW roll channel
        // can absorb the residual rotor moment through TIMER to FW done.
        coefficient.c_l_deltaa = 0.15f;
        coefficient.c_m_deltae = 0.42f;
        coefficient.c_l_deltar = -0.037f;
        coefficient.c_n_deltar = 0.1f;
        coefficient.c_y_deltar = -0.2f;
        coefficient.c_lift_deltae = 0.0f;   // keep: camber lift split not fitted
        coefficient.c_drag_deltae = 0.0f;   // keep: elevator drag ignored
        coefficient.c_n_deltaa = 0.0f;      // keep: aileron-yaw not fitted
        coefficient.c_y_deltaa = 0.0f;      // keep: aileron side force ignored
        // aero ref vs hover CG (user may tune CGOffset for trim)
        coefficient.CGOffset = Vector3f(-0.154f, 0.0f, 0.0f);
        // Cross-coupling isolation trial (after 149): pre-onset aero M≈0,
        // rotor seeds only ~6 Nm, yet yaw->roll divergence still started at
        // AS≈27.6 with Tilt=30. Hypothesis: dutch-roll loop gains
        //   β -> roll  (Clβ = c_l_b, stock -0.12)
        //   β -> yaw   (Cnβ = c_n_b, stock 0.25)
        //   r  -> roll (Clr = c_l_r, stock 0.14)
        //   p  -> yaw  (Cnp = c_n_p, stock 0.022)
        // feed each oscillation cycle while roll/yaw damping is weak
        // (VTOL mix faded, surfaces zeroed). Zero all four loop terms, KEEP
        // damping (c_l_p, c_n_r) and side force (c_y_b) so we only remove
        // the mutual excitation, not the brakes.
        // If AS≈27 then survives -> coupling loop confirmed as the wall.
        // If still blows -> look elsewhere (rotor/tilt-stream effects).
        coefficient.c_l_b = 0.0f;
        // 150: with c_n_b=0 too, yaw lost its weathercock restoring moment -
        // RATE.Y saturated into a +/-406 deg/s square wave (no heading sense,
        // only Cnr damping + weak rotor yaw diff). Restore the stock
        // weathercock (c_n_b=0.25) as the minimal yaw restoring term while
        // keeping the cross-coupling terms (c_l_r, c_n_p) and dihedral (c_l_b)
        // at zero. Test: square wave should disappear; divergence (needs
        // Clr/Cnp loop) should NOT return.
        coefficient.c_n_b = 0.25f;
        coefficient.c_l_r = 0.0f;
        coefficient.c_n_p = 0.0f;
        // Heading-control trial (151): Excel 动导数 AVL Cnr = -0.065
        // (偏航阻尼) replaces stock -1.0 - 15x weaker, so yaw becomes a
        // lightly-damped spring(Cnβ)-damper(Cnr) mode. Also re-enable the
        // RUDDER (stock derivatives) so active heading control exists:
        //   c_n_deltar -> yaw moment from rudder
        //   c_y_deltar -> side force from rudder
        //   c_l_deltar -> small roll coupling from rudder (stock, small)
        // Aileron/elevator stay zeroed to keep the roll path isolated.
        // This is now a heading-control-ability test: restoring (Cnβ) +
        // damping (Cnr Excel) + rudder (active) + rotor yaw diff.
        coefficient.c_n_r = -0.065f;
        coefficient.c_n_deltar = 0.1f;
        coefficient.c_y_deltar = -0.2f;
        coefficient.c_l_deltar = -0.037f;
    } else if (strstr(frame_str, "-jt20")) {
        // JT-20 fixed-wing aero from jt_20.sdf AdvancedLiftDragPlugin.
        // Override the default (Skywalker) coefficients with the real airframe's.
        // Geometry: area 0.4761 m^2, AR 10.6383, mac 0.2115.
        coefficient.s = 0.4761f;
        coefficient.b = sqrtf(coefficient.s * 10.6383f); // AR*S, ~2.25 m
        coefficient.c = 0.2115f;
        // lift / drag / moment from SDF plugin
        coefficient.c_lift_0 = 0.403f;
        coefficient.c_lift_a = 7.39f;
        coefficient.c_drag_p = 0.01887f;  // approx CD0
        coefficient.oswald = 0.97f;
        coefficient.alpha_stall = 0.3391f; // 19.4 deg
        coefficient.c_m_a = -1.06f;        // static stability
        coefficient.c_m_q = -22.89f;       // pitch damping
        // aero reference point from SDF (ref_pt -0.126 0 -0.08)
        coefficient.CGOffset = Vector3f(-0.126f, 0.0f, -0.08f);
    }
}

/*
  the following functions are from last_letter
  https://github.com/Georacer/last_letter/blob/master/last_letter/src/aerodynamicsLib.cpp
  many thanks to Georacer!
 */
float Plane::liftCoeff(float alpha) const
{
    const float alpha0 = coefficient.alpha_stall;
    const float M = coefficient.mcoeff;
    const float c_lift_0 = coefficient.c_lift_0;
    const float c_lift_a0 = coefficient.c_lift_a;

    // clamp the value of alpha to avoid exp(90) in calculation of sigmoid
    const float max_alpha_delta = 0.8f;
    if (alpha-alpha0 > max_alpha_delta) {
        alpha = alpha0 + max_alpha_delta;
    } else if (alpha0-alpha > max_alpha_delta) {
        alpha = alpha0 - max_alpha_delta;
    }
	double sigmoid = ( 1+exp(-M*(alpha-alpha0))+exp(M*(alpha+alpha0)) ) / (1+exp(-M*(alpha-alpha0))) / (1+exp(M*(alpha+alpha0)));
	double linear = (1.0-sigmoid) * (c_lift_0 + c_lift_a0*alpha); //Lift at small AoA
	double flatPlate = sigmoid*(2*copysign(1,alpha)*pow(sin(alpha),2)*cos(alpha)); //Lift beyond stall

	float result  = linear+flatPlate;
	return result;
}

float Plane::dragCoeff(float alpha) const
{
    const float b = coefficient.b;
    const float s = coefficient.s;
    const float c_drag_p = coefficient.c_drag_p;
    const float c_lift_0 = coefficient.c_lift_0;
    const float c_lift_a0 = coefficient.c_lift_a;
    const float oswald = coefficient.oswald;
    
	double AR = pow(b,2)/s;
	double c_drag_a = c_drag_p + pow(c_lift_0+c_lift_a0*alpha,2)/(M_PI*oswald*AR);

	return c_drag_a;
}

// Torque calculation function
Vector3f Plane::getTorque(float inputAileron, float inputElevator, float inputRudder, float inputThrust, const Vector3f &force) const
{
    float alpha = angle_of_attack;

	//calculate aerodynamic torque
    float effective_airspeed = airspeed;

    if (tailsitter || aerobatic) {
        /*
          tailsitters get airspeed from prop-wash
         */
        effective_airspeed += inputThrust * 20;

        // reduce effective angle of attack as thrust increases
        alpha *= constrain_float(1 - inputThrust, 0, 1);
    }
    
    const float s = coefficient.s;
    const float c = coefficient.c;
    const float b = coefficient.b;
    const float c_l_0 = coefficient.c_l_0;
    const float c_l_b = coefficient.c_l_b;
    const float c_l_p = coefficient.c_l_p;
    const float c_l_r = coefficient.c_l_r;
    const float c_l_deltaa = coefficient.c_l_deltaa;
    const float c_l_deltar = coefficient.c_l_deltar;
    const float c_m_0 = coefficient.c_m_0;
    const float c_m_a = coefficient.c_m_a;
    const float c_m_q = coefficient.c_m_q;
    const float c_m_deltae = coefficient.c_m_deltae;
    const float c_n_0 = coefficient.c_n_0;
    const float c_n_b = coefficient.c_n_b;
    const float c_n_p = coefficient.c_n_p;
    const float c_n_r = coefficient.c_n_r;
    const float c_n_deltaa = coefficient.c_n_deltaa;
    const float c_n_deltar = coefficient.c_n_deltar;
    const Vector3f &CGOffset = coefficient.CGOffset;
    
    float rho = air_density;

	//read angular rates
	double p = gyro.x;
	double q = gyro.y;
	double r = gyro.z;

	double qbar = 1.0/2.0*rho*pow(effective_airspeed,2)*s; //Calculate dynamic pressure
	double la, na, ma;
	if (is_zero(effective_airspeed))
	{
		la = 0;
		ma = 0;
		na = 0;
	}
	else
	{
		la = qbar*b*(c_l_0 + c_l_b*beta + c_l_p*b*p/(2*effective_airspeed) + c_l_r*b*r/(2*effective_airspeed) + c_l_deltaa*inputAileron + c_l_deltar*inputRudder);
		ma = qbar*c*(c_m_0 + c_m_a*alpha + c_m_q*c*q/(2*effective_airspeed) + c_m_deltae*inputElevator);
		na = qbar*b*(c_n_0 + c_n_b*beta + c_n_p*b*p/(2*effective_airspeed) + c_n_r*b*r/(2*effective_airspeed) + c_n_deltaa*inputAileron + c_n_deltar*inputRudder);
	}


	// Add torque to force misalignment with CG
	// r x F, where r is the distance from CoG to CoL
	la +=  CGOffset.y * force.z - CGOffset.z * force.y;
	ma += -CGOffset.x * force.z + CGOffset.z * force.x;
	na += -CGOffset.y * force.x + CGOffset.x * force.y;

	return Vector3f(la, ma, na);
}

// Force calculation function from last_letter
Vector3f Plane::getForce(float inputAileron, float inputElevator, float inputRudder) const
{
    const float alpha = angle_of_attack;
    const float c_drag_q = coefficient.c_drag_q;
    const float c_lift_q = coefficient.c_lift_q;
    const float s = coefficient.s;
    const float c = coefficient.c;
    const float b = coefficient.b;
    const float c_drag_deltae = coefficient.c_drag_deltae;
    const float c_lift_deltae = coefficient.c_lift_deltae;
    const float c_y_0 = coefficient.c_y_0;
    const float c_y_b = coefficient.c_y_b;
    const float c_y_p = coefficient.c_y_p;
    const float c_y_r = coefficient.c_y_r;
    const float c_y_deltaa = coefficient.c_y_deltaa;
    const float c_y_deltar = coefficient.c_y_deltar;
    
    float rho = air_density;

	//request lift and drag alpha-coefficients from the corresponding functions
	double c_lift_a = liftCoeff(alpha);
	double c_drag_a = dragCoeff(alpha);

	//convert coefficients to the body frame
	double c_x_a = -c_drag_a*cos(alpha)+c_lift_a*sin(alpha);
	double c_x_q = -c_drag_q*cos(alpha)+c_lift_q*sin(alpha);
	double c_z_a = -c_drag_a*sin(alpha)-c_lift_a*cos(alpha);
	double c_z_q = -c_drag_q*sin(alpha)-c_lift_q*cos(alpha);

	//read angular rates
	double p = gyro.x;
	double q = gyro.y;
	double r = gyro.z;

	//calculate aerodynamic force
	double qbar = 1.0/2.0*rho*pow(airspeed,2)*s; //Calculate dynamic pressure
	double ax, ay, az;
	if (is_zero(airspeed))
	{
		ax = 0;
		ay = 0;
		az = 0;
	}
	else
	{
		ax = qbar*(c_x_a + c_x_q*c*q/(2*airspeed) - c_drag_deltae*cos(alpha)*fabs(inputElevator) + c_lift_deltae*sin(alpha)*inputElevator);
		// split c_x_deltae to include "abs" term
		ay = qbar*(c_y_0 + c_y_b*beta + c_y_p*b*p/(2*airspeed) + c_y_r*b*r/(2*airspeed) + c_y_deltaa*inputAileron + c_y_deltar*inputRudder);
		az = qbar*(c_z_a + c_z_q*c*q/(2*airspeed) - c_drag_deltae*sin(alpha)*fabs(inputElevator) - c_lift_deltae*cos(alpha)*inputElevator);
		// split c_z_deltae to include "abs" term
	}
    return Vector3f(ax, ay, az);
}

void Plane::calculate_forces(const struct sitl_input &input, Vector3f &rot_accel)
{
    float aileron  = filtered_servo_angle(input, 0);
    float elevator = filtered_servo_angle(input, 1);
    float rudder   = filtered_servo_angle(input, 3);
    bool launch_triggered = input.servos[6] > 1700;
    float throttle;
    if (reverse_elevator_rudder) {
        elevator = -elevator;
        rudder = -rudder;
    }
    if (elevons) {
        // fake an elevon plane
        float ch1 = aileron;
        float ch2 = elevator;
        aileron  = (ch2-ch1)/2.0f;
        // the minus does away with the need for RC2_REVERSED=-1
        elevator = -(ch2+ch1)/2.0f;

        // assume no rudder
        rudder = 0;
    } else if (vtail) {
        // fake a vtail plane
        float ch1 = elevator;
        float ch2 = rudder;
        // this matches VTAIL_OUTPUT==2
        elevator = (ch2-ch1)/2.0f;
        rudder   = (ch2+ch1)/2.0f;
    } else if (dspoilers) {
        // fake a differential spoiler plane. Use outputs 1, 2, 4 and 5
        float dspoiler1_left = filtered_servo_angle(input, 0);
        float dspoiler1_right = filtered_servo_angle(input, 1);
        float dspoiler2_left = filtered_servo_angle(input, 3);
        float dspoiler2_right = filtered_servo_angle(input, 4);
        float elevon_left  = (dspoiler1_left + dspoiler2_left)/2;
        float elevon_right = (dspoiler1_right + dspoiler2_right)/2;
        aileron  = (elevon_right-elevon_left)/2;
        elevator = (elevon_left+elevon_right)/2;
        rudder = fabsf(dspoiler1_right - dspoiler2_right)/2 - fabsf(dspoiler1_left - dspoiler2_left)/2;
    }
    //printf("Aileron: %.1f elevator: %.1f rudder: %.1f\n", aileron, elevator, rudder);

    if (reverse_thrust) {
        throttle = filtered_servo_angle(input, 2);
    } else {
        throttle = filtered_servo_range(input, 2);
    }
    
    float thrust     = throttle;

    battery_voltage = sitl->batt_voltage - 0.7*throttle;
    battery_current = (battery_voltage/sitl->batt_voltage)*50.0f*sq(throttle);

    if (ice_engine) {
        thrust = icengine.update(input);
    }

    // calculate angle of attack
    angle_of_attack = atan2f(velocity_air_bf.z, velocity_air_bf.x);
    beta = atan2f(velocity_air_bf.y,velocity_air_bf.x);

    if (tailsitter || aerobatic) {
        /*
          tailsitters get 4x the control surfaces
         */
        aileron *= 4;
        elevator *= 4;
        rudder *= 4;
    }
    
    Vector3f force = getForce(aileron, elevator, rudder);
    rot_accel = getTorque(aileron, elevator, rudder, thrust, force);

    // DF-200: dump aero state ~20 Hz when AS in 20..35 to a fixed file so
    // sim_vehicle's separate gnome-terminal does not need stdout redirect.
    //   /tmp/df200_aero.log
    // Columns: time, AS, alpha/beta deg, qbar, body force N, torque Nm, then
    // moment contributions from alpha (Cma), rate damp (Clp/Cmq/Cnr), sideslip
    // (Clb/Cnb), surfaces (delta*), and CGOffset cross product.
    if (df200_frame && airspeed >= 20.0f && airspeed <= 55.0f) {
        const uint64_t now_ms = AP_HAL::millis64();
        if (now_ms - df200_aero_log_last_ms >= 50) {
            df200_aero_log_last_ms = now_ms;
            const float s = coefficient.s;
            const float c = coefficient.c;
            const float b = coefficient.b;
            const float rho = air_density;
            const float qbar = 0.5f * rho * sq(airspeed) * s;
            const float p = gyro.x, q = gyro.y, r = gyro.z;
            const float alpha = angle_of_attack;
            // torque term breakdown (Nm) — same formulas as getTorque()
            const float la_b   = qbar * b * (coefficient.c_l_b * beta);
            const float la_p   = qbar * b * (coefficient.c_l_p * b * p / (2 * airspeed));
            const float la_r   = qbar * b * (coefficient.c_l_r * b * r / (2 * airspeed));
            const float la_da  = qbar * b * (coefficient.c_l_deltaa * aileron);
            const float la_dr  = qbar * b * (coefficient.c_l_deltar * rudder);
            const float ma_a   = qbar * c * (coefficient.c_m_a * alpha);
            const float ma_q   = qbar * c * (coefficient.c_m_q * c * q / (2 * airspeed));
            const float ma_de  = qbar * c * (coefficient.c_m_deltae * elevator);
            const float ma_0   = qbar * c * coefficient.c_m_0;
            const float na_b   = qbar * b * (coefficient.c_n_b * beta);
            const float na_p   = qbar * b * (coefficient.c_n_p * b * p / (2 * airspeed));
            const float na_r   = qbar * b * (coefficient.c_n_r * b * r / (2 * airspeed));
            const float na_da  = qbar * b * (coefficient.c_n_deltaa * aileron);
            const float na_dr  = qbar * b * (coefficient.c_n_deltar * rudder);
            const Vector3f &cg = coefficient.CGOffset;
            const float la_cg = cg.y * force.z - cg.z * force.y;
            const float ma_cg = -cg.x * force.z + cg.z * force.x;
            const float na_cg = -cg.y * force.x + cg.x * force.y;
            const float la = la_b + la_p + la_r + la_da + la_dr + la_cg;
            const float ma = ma_0 + ma_a + ma_q + ma_de + ma_cg;
            const float na = na_b + na_p + na_r + na_da + na_dr + na_cg;
            // attitude + altitude so the log is self-contained (no BIN needed
            // for force/moment analysis). dcm is body->earth; euler from it.
            float roll_rad = 0, pitch_rad = 0, yaw_rad = 0;
            get_dcm().to_euler(&roll_rad, &pitch_rad, &yaw_rad);
            const float roll_deg = degrees(roll_rad);
            const float pitch_deg = degrees(pitch_rad);
            const float yaw_deg = degrees(yaw_rad);
            const float alt_amsl = location.alt * 0.01f;  // cm -> m
            FILE *fp = fopen("/tmp/df200_aero.log", "a");
            if (fp != nullptr) {
                fprintf(fp,
                    "DF200AERO t=%.2f AS=%.1f a=%.1f b=%.1f qbar=%.0f "
                    "F=%.0f,%.0f,%.0f M=%.0f,%.0f,%.0f "
                    "Ma_a=%.0f Ma_q=%.0f Ma_de=%.0f Ma_cg=%.0f "
                    "La_b=%.0f La_p=%.0f La_r=%.0f La_da=%.0f "
                    "Na_b=%.0f Na_r=%.0f Na_dr=%.0f "
                    "ail=%.2f elev=%.2f rud=%.2f "
                    "pqr=%.3f,%.3f,%.3f "
                    "RPY=%.1f,%.1f,%.1f Alt=%.1f\n",
                    now_ms * 1.0e-3,
                    airspeed,
                    degrees(alpha), degrees(beta),
                    qbar,
                    force.x, force.y, force.z,
                    la, ma, na,
                    ma_a, ma_q, ma_de, ma_cg,
                    la_b, la_p, la_r, la_da,
                    na_b, na_r, na_dr,
                    aileron, elevator, rudder,
                    p, q, r,
                    roll_deg, pitch_deg, yaw_deg, alt_amsl);
                fclose(fp);
            }
        }
    }

    if (have_launcher) {
        /*
          simple simulation of a launcher
         */
        if (launch_triggered) {
            uint64_t now = AP_HAL::millis64();
            if (launch_start_ms == 0) {
                launch_start_ms = now;
            }
            if (now - launch_start_ms < launch_time*1000) {
                force.x += mass * launch_accel;
                force.z += mass * launch_accel/3;
            }
        } else {
            // allow reset of catapult
            launch_start_ms = 0;
        }
    }
    
    // simulate engine RPM
    motor_mask |= (1U<<2);
    rpm[2] = thrust * 7000;
    
    // scale thrust to newtons
    thrust *= thrust_scale;

    accel_body = Vector3f(thrust, 0, 0) + force;
    accel_body /= mass;

    // add some noise
    if (thrust_scale > 0) {
        add_noise(fabsf(thrust) / thrust_scale);
    }

    if (on_ground() && !tailsitter) {
        // add some ground friction
        Vector3f vel_body = dcm.transposed() * velocity_ef;
        accel_body.x -= vel_body.x * 0.3f;
    }
}
    
/*
  update the plane simulation by one time step
 */
void Plane::update(const struct sitl_input &input)
{
    Vector3f rot_accel;

    update_wind(input);
    
    calculate_forces(input, rot_accel);
    
    update_dynamics(rot_accel);

    /*
      add in ground steering, this should be replaced with a proper
      calculation of a nose wheel effect
    */
    if (have_steering && on_ground()) {
        const float steering = filtered_servo_angle(input, 4);
        const Vector3f velocity_bf = dcm.transposed() * velocity_ef;
        const float steer_scale = radians(5);
        gyro.z += steering * velocity_bf.x * steer_scale;
    }

    update_external_payload(input);

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}
