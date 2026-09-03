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
  simple quadplane simulator class
*/

#include "SIM_QuadPlane.h"

#include <stdio.h>

using namespace SITL;

QuadPlane::QuadPlane(const char *frame_str) :
    Plane(frame_str)
{
    // default to X frame
    const char *frame_type = "x";
    uint8_t motor_offset = 4;

    ground_behavior = GROUND_BEHAVIOR_NO_MOVEMENT;

    if (strstr(frame_str, "-octa-quad")) {
        frame_type = "octa-quad";
    } else if (strstr(frame_str, "-octaquad")) {
        frame_type = "octa-quad";
    } else if (strstr(frame_str, "-octa")) {
        frame_type = "octa";
    } else if (strstr(frame_str, "-hexax")) {
        frame_type = "hexax";
    } else if (strstr(frame_str, "-hexa")) {
        frame_type = "hexa";
    } else if (strstr(frame_str, "-plus")) {
        frame_type = "+";
    } else if (strstr(frame_str, "-y6")) {
        frame_type = "y6";
    } else if (strstr(frame_str, "-tri")) {
        frame_type = "tri";
    } else if (strstr(frame_str, "-tilttrivec")) {
        frame_type = "tilttrivec";
        // fwd motor gives zero thrust
        thrust_scale = 0;
    } else if (strstr(frame_str, "-tilthvec")) {
        frame_type = "tilthvec";
    } else if (strstr(frame_str, "-df200")) {
        // DF-200: same topology as JT-20, geometry from 数据库_v2.xlsx
        // Named df200 (not jt200) to avoid strstr/find_frame prefix clash with jt20.
        frame_type = "df200";
        thrust_scale = 0;
    } else if (strstr(frame_str, "-jt20")) {
        // JT-20: 4 inner tilting + 4 outer fixed (see SIM_Frame jt20_motors)
        frame_type = "jt20";
        thrust_scale = 0;
    } else if (strstr(frame_str, "-tilttri")) {
        frame_type = "tilttri";
        // fwd motor gives zero thrust
        thrust_scale = 0;
    } else if (strstr(frame_str, "firefly")) {
        frame_type = "firefly";
        // elevon style surfaces
        elevons = true;
        // fwd motor gives zero thrust
        thrust_scale = 0;
        // vtol motors start at 2
        motor_offset = 2;
    } else if (strstr(frame_str, "-tilt")) {
        frame_type = "tilt";
        // fwd motor gives zero thrust
        thrust_scale = 0;
    } else if (strstr(frame_str, "cl84")) {
        frame_type = "tilttri";
        // fwd motor gives zero thrust
        thrust_scale = 0;
    } else if (strstr(frame_str, "-copter_tailsitter")) {
        frame_type = "+";
        copter_tailsitter = true;
        ground_behavior = GROUND_BEHAVIOR_TAILSITTER;
        thrust_scale *= 1.5;
    }
    frame = Frame::find_frame(frame_type);
    if (frame == nullptr) {
        printf("Failed to find frame '%s'\n", frame_type);
        exit(1);
    }

    if (strstr(frame_str, "cl84")) {
        // setup retract servos at front
        frame->motors[0].servo_type = Motor::SERVO_RETRACT;
        frame->motors[0].servo_rate = 7*60.0/90; // 7 seconds to change
        frame->motors[1].servo_type = Motor::SERVO_RETRACT;
        frame->motors[1].servo_rate = 7*60.0/90; // 7 seconds to change
    }
    
    // leave first 4 servos free for plane
    frame->motor_offset = motor_offset;

    // we use zero terminal velocity to let the plane model handle the drag
    frame->init(frame_str, &battery);

    // increase mass for plane components
    mass = frame->get_mass() * 1.5;
    frame->set_mass(mass);

    lock_step_scheduled = true;
}

/*
  update the quadplane simulation by one time step
 */
void QuadPlane::update(const struct sitl_input &input)
{
    // get wind vector setup
    update_wind(input);

    // first plane forces
    Vector3f rot_accel;
    calculate_forces(input, rot_accel);

    // now quad forces
    Vector3f quad_rot_accel;
    Vector3f quad_accel_body;

    motor_mask |= ((1U<<frame->num_motors)-1U) << frame->motor_offset;
    frame->calculate_forces(*this, input, quad_rot_accel, quad_accel_body, rpm, false);

    // rotate frames for copter tailsitters
    if (copter_tailsitter) {
        quad_rot_accel.rotate(ROTATION_PITCH_270);
        quad_accel_body.rotate(ROTATION_PITCH_270);
    }

    // estimate voltage and current
    frame->current_and_voltage(battery_voltage, battery_current);

    battery.set_current(battery_current);

    float throttle;
    if (reverse_thrust) {
        throttle = filtered_servo_angle(input, 2);
    } else {
        throttle = filtered_servo_range(input, 2);
    }
    // assume 20A at full fwd throttle
    throttle = fabsf(throttle);
    battery_current += 20 * throttle;
    
    // UNITS FIX: Plane::calculate_forces returns the aero TORQUE (Nm) in
    // rot_accel, but Frame::calculate_forces returns ANGULAR ACCEL (rad/s²,
    // already divided by moment_of_inertia). Upstream (incl. 4.6.3) sums them
    // directly — the aero moment is effectively amplified by the inertia
    // ratio. For most quadplanes the aero torque is small enough to hide it.
    // DF-200 exposed it (log 177): during tilt-30/AS>12 the real aero pitch
    // moment is ~-6.3 Nm, numerically equal to the railed rotor pitch accel
    // +2925/464.3 = +6.30 rad/s², so sumRA_y stayed ≈0 for 5 s while PDes was
    // railed at +40°/s — the pitch axis was annihilated, the nose fell, the
    // railed front/rear differential pinned avg throttle at 0.5, T/W at 0.88,
    // and the aircraft descended into the ground. Convert the plane torque to
    // angular accel with the frame inertia before summing.
    const Vector3f &inertia = frame->get_inertia();
    rot_accel.x /= inertia.x;
    rot_accel.y /= inertia.y;
    rot_accel.z /= inertia.z;

    rot_accel += quad_rot_accel;
    accel_body += quad_accel_body;

    // DF-200 dynamics bookkeeping: what rot_accel actually reaches
    // update_dynamics and the gyro response. 176 showed a steady rotor
    // Mtot_y=+2918 Nm (outputs frozen at 50Hz MOTT, not aliasing) with
    // Iyy=464.3, i.e. expected q̇=6.28 rad/s², but measured q̇≈0.02 —
    // the angular book does not close, so log both ends of update_dynamics.
    //   /tmp/df200_dyn.log
    // planeRA is the aero angular accel (Plane torque / frame inertia, rad/s²
    // after the units fix below); quadRA is the Frame output (rad/s²).
    const bool df200_dyn_log = (frame->name != nullptr && strcmp(frame->name, "df200") == 0);
    bool df200_dyn_sample = false;
    Vector3f df200_gyro_pre;
    if (df200_dyn_log) {
        static uint64_t df200_dyn_log_last_ms;
        const uint64_t now_ms = AP_HAL::millis64();
        if (now_ms - df200_dyn_log_last_ms >= 50) {
            df200_dyn_log_last_ms = now_ms;
            df200_dyn_sample = true;
            df200_gyro_pre = get_gyro();
        }
    }

    update_dynamics(rot_accel);

    if (df200_dyn_sample) {
        const Vector3f gyro_post = get_gyro();
        FILE *fp = fopen("/tmp/df200_dyn.log", "a");
        if (fp != nullptr) {
            fprintf(fp, "DF200DYN t=%.2f dt=%.6f gnd=%d "
                    "planeRA=%.4f,%.4f,%.4f quadRA=%.4f,%.4f,%.4f sumRA=%.4f,%.4f,%.4f "
                    "gPre=%.5f,%.5f,%.5f gPost=%.5f,%.5f,%.5f\n",
                    AP_HAL::millis64() * 1.0e-3,
                    frame_time_us * 1.0e-6,
                    (int)on_ground(),
                    (rot_accel - quad_rot_accel).x,
                    (rot_accel - quad_rot_accel).y,
                    (rot_accel - quad_rot_accel).z,
                    quad_rot_accel.x, quad_rot_accel.y, quad_rot_accel.z,
                    rot_accel.x, rot_accel.y, rot_accel.z,
                    df200_gyro_pre.x, df200_gyro_pre.y, df200_gyro_pre.z,
                    gyro_post.x, gyro_post.y, gyro_post.z);
            fclose(fp);
        }
    }
    update_external_payload(input);

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}
