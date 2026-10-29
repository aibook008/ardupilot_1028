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
  multicopter frame simulator class
*/

#include "SIM_Frame.h"
#include <AP_Motors/AP_Motors.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include "SIM_Aircraft.h"

#include <stdio.h>
#include <sys/stat.h>

using namespace SITL;

static Motor quad_plus_motors[] =
{
    Motor(AP_MOTORS_MOT_1,  90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2),
    Motor(AP_MOTORS_MOT_2, -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4),
    Motor(AP_MOTORS_MOT_3,   0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1),
    Motor(AP_MOTORS_MOT_4, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3),
};

static Motor quad_x_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_2, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_3,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4),
    Motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2),
};

// motor order to match betaflight conventions
// See: https://fpvfrenzy.com/betaflight-motor-order/
static Motor quad_bf_x_motors[] =
{
    Motor(AP_MOTORS_MOT_1,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 2),
    Motor(AP_MOTORS_MOT_2,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,1),
    Motor(AP_MOTORS_MOT_3, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,3),
    Motor(AP_MOTORS_MOT_4,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 4),
};

// motor order to match betaflight conventions, reversed direction
static Motor quad_bf_x_rev_motors[] =
{
    Motor(AP_MOTORS_MOT_1,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2),
    Motor(AP_MOTORS_MOT_2,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1),
    Motor(AP_MOTORS_MOT_3, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3),
    Motor(AP_MOTORS_MOT_4,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4),
};

// motor order to match DJI conventions
// See: https://forum44.djicdn.com/data/attachment/forum/201711/26/172348bppvtt1ot1nrtp5j.jpg
static Motor quad_dji_x_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_2,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4),
    Motor(AP_MOTORS_MOT_3, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2),
};

// motor order so that test order matches motor order ("clockwise X")
static Motor quad_cw_x_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_2,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2),
    Motor(AP_MOTORS_MOT_3, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_4,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4),
};

static Motor tiltquad_h_vectored_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1, -1, 0, 0, 7, 10, -90),
    Motor(AP_MOTORS_MOT_2, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3, -1, 0, 0, 8, 10, -90),
    Motor(AP_MOTORS_MOT_3,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4, -1, 0, 0, 8, 10, -90),
    Motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2, -1, 0, 0, 7, 10, -90),
};

static Motor tiltquad[] =
{
    Motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1, -1, 0, 0, 7, 10, -90),
    Motor(AP_MOTORS_MOT_2, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  3),
    Motor(AP_MOTORS_MOT_3,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   4, -1, 0, 0, 8, 10, -90),
    Motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2),
};

/*
  JT-20 style: inner 4 tilting + outer 4 fixed vertical.
  Angles from jt_20.sdf Gazebo (X fwd, Y left, Z up) → APM (X fwd, Y right, Z down):
    (x,y,z)_apm = (x, -y, -z); angle = atan2(Y_apm, X_apm).
  With motor_offset=4, motors use servos[4..11]; pitch_servo 8/9 → servos[12]/[13]
  (SERVO13/14 = k_tiltMotorLeft/Right). See docs/jt20_coordinate_mapping.md.
 */
static Motor jt20_motors[] =
{
    // Inner tilting (GZ motor_0..3)
    Motor(AP_MOTORS_MOT_1,   47.3f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1, -1, 0, 0, 9, 10, -90), // front-right → Right tilt
    Motor(AP_MOTORS_MOT_2, -145.3f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3, -1, 0, 0, 8, 10, -90), // rear-left  → Left tilt
    Motor(AP_MOTORS_MOT_3,  -47.7f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4, -1, 0, 0, 8, 10, -90), // front-left → Left tilt
    Motor(AP_MOTORS_MOT_4,  145.3f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2, -1, 0, 0, 9, 10, -90), // rear-right → Right tilt
    // Outer fixed vertical (GZ rotor_4..7)
    Motor(AP_MOTORS_MOT_5,   74.8f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5),
    Motor(AP_MOTORS_MOT_6, -117.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6),
    Motor(AP_MOTORS_MOT_7,  -74.8f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7),
    Motor(AP_MOTORS_MOT_8,  117.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  8),
};

/*
  DF-200: same topology as JT-20 (inner 4 tilt + outer 4 fixed vertical),
  geometry from docs/数据库_v2.xlsx 旋翼位置 about hover CG (1.900,0,0.483).
  Angles = atan2(Y_apm, X_apm). pitch_servo 8/9 → SERVO13/14 with motor_offset=4.
  Named df200 (not jt200) to avoid Frame::find_frame() / strstr prefix clash with jt20.
  See docs/vtol_df200.md.
 */
static Motor df200_motors[] =
{
    // Inner tilting (①⑤②⑥ → FR/RL/FL/RR in JT-20 motor order)
    // pitch_min/max = 0,-90: PWM MIN → true vertical (0°), PWM MAX → forward (-90°).
    // Do NOT copy jt20's (10,-90): that makes PWM=1000 a +10° forward bias in SITL.
    Motor(AP_MOTORS_MOT_1,   37.32f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1, -1, 0, 0, 9, 0, -90), // ① FR tilt → Right
    Motor(AP_MOTORS_MOT_2, -142.52f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3, -1, 0, 0, 8, 0, -90), // ⑥ RL tilt → Left
    Motor(AP_MOTORS_MOT_3,  -37.32f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4, -1, 0, 0, 8, 0, -90), // ⑤ FL tilt → Left
    Motor(AP_MOTORS_MOT_4,  142.52f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2, -1, 0, 0, 9, 0, -90), // ② RR tilt → Right
    // Outer fixed vertical (③⑧⑦④)
    Motor(AP_MOTORS_MOT_5,   67.08f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5), // ③ FR vert
    Motor(AP_MOTORS_MOT_6, -112.74f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6), // ⑧ RL vert
    Motor(AP_MOTORS_MOT_7,  -67.08f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7), // ⑦ FL vert
    Motor(AP_MOTORS_MOT_8,  112.74f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  8), // ④ RR vert
};

static Motor hexa_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1),
    Motor(AP_MOTORS_MOT_2, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4),
    Motor(AP_MOTORS_MOT_3,-120, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5),
    Motor(AP_MOTORS_MOT_4,  60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2),
    Motor(AP_MOTORS_MOT_5, -60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6),
    Motor(AP_MOTORS_MOT_6, 120, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3)
};

static Motor hexax_motors[] =
{
    Motor(AP_MOTORS_MOT_1,  90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2),
    Motor(AP_MOTORS_MOT_2, -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5),
    Motor(AP_MOTORS_MOT_3, -30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6),
    Motor(AP_MOTORS_MOT_4, 150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_5,  30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_6,-150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4)
};

static Motor hexa_dji_x_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_2,  -30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6),
    Motor(AP_MOTORS_MOT_3,  -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5),
    Motor(AP_MOTORS_MOT_4, -150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4),
    Motor(AP_MOTORS_MOT_5,  150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_6,   90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2)
};

static Motor hexa_cw_x_motors[] = 
{
    Motor(AP_MOTORS_MOT_1,   30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_2,   90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2),
    Motor(AP_MOTORS_MOT_3,  150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_4, -150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4),
    Motor(AP_MOTORS_MOT_5,  -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5),
    Motor(AP_MOTORS_MOT_6,  -30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6)
};

static Motor octa_motors[] =
{
    Motor(AP_MOTORS_MOT_1,    0,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1),
    Motor(AP_MOTORS_MOT_2,  180,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5),
    Motor(AP_MOTORS_MOT_3,   45,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2),
    Motor(AP_MOTORS_MOT_4,  135,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4),
    Motor(AP_MOTORS_MOT_5,  -45,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8),
    Motor(AP_MOTORS_MOT_6, -135,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6),
    Motor(AP_MOTORS_MOT_7,  -90,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7),
    Motor(AP_MOTORS_MOT_8,   90,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3)
};

static Motor octa_dji_x_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   22.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_2,  -22.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  8),
    Motor(AP_MOTORS_MOT_3,  -67.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 7),
    Motor(AP_MOTORS_MOT_4, -112.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6),
    Motor(AP_MOTORS_MOT_5, -157.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5),
    Motor(AP_MOTORS_MOT_6,  157.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4),
    Motor(AP_MOTORS_MOT_7,  112.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_8,   67.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2)
};

static Motor octa_cw_x_motors[] = 
{
    Motor(AP_MOTORS_MOT_1,   22.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_2,   67.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2),
    Motor(AP_MOTORS_MOT_3,  112.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_4,  157.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4),
    Motor(AP_MOTORS_MOT_5, -157.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5),
    Motor(AP_MOTORS_MOT_6, -112.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6),
    Motor(AP_MOTORS_MOT_7,  -67.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 7),
    Motor(AP_MOTORS_MOT_8,  -22.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  8)
};

static Motor octa_quad_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_2,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7),
    Motor(AP_MOTORS_MOT_3, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5),
    Motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3),
    Motor(AP_MOTORS_MOT_5,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8),
    Motor(AP_MOTORS_MOT_6,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2),
    Motor(AP_MOTORS_MOT_7,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4),
    Motor(AP_MOTORS_MOT_8, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6)
};

static Motor octa_quad_cw_x_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_2,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2),
    Motor(AP_MOTORS_MOT_3,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4),
    Motor(AP_MOTORS_MOT_5, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5),
    Motor(AP_MOTORS_MOT_6, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6),
    Motor(AP_MOTORS_MOT_7,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 7),
    Motor(AP_MOTORS_MOT_8,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  8)
};

static Motor dodeca_hexa_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1),
    Motor(AP_MOTORS_MOT_2,   30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2),
    Motor(AP_MOTORS_MOT_3,   90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   3),
    Motor(AP_MOTORS_MOT_4,   90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  4),
    Motor(AP_MOTORS_MOT_5,  150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  5),
    Motor(AP_MOTORS_MOT_6,  150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   6),
    Motor(AP_MOTORS_MOT_7, -150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   7),
    Motor(AP_MOTORS_MOT_8, -150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  8),
    Motor(AP_MOTORS_MOT_9,  -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  9),
    Motor(AP_MOTORS_MOT_10, -90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   10),
    Motor(AP_MOTORS_MOT_11, -30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   11),
    Motor(AP_MOTORS_MOT_12, -30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  12)
};

static Motor deca_motors[] =
{
    Motor(AP_MOTORS_MOT_1,     0, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1),
    Motor(AP_MOTORS_MOT_2,    36, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2),
    Motor(AP_MOTORS_MOT_3,    72, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  3),
    Motor(AP_MOTORS_MOT_4,   108, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   4),
    Motor(AP_MOTORS_MOT_5,   144, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  5),
    Motor(AP_MOTORS_MOT_6,   180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   6),
    Motor(AP_MOTORS_MOT_7,  -144, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  7),
    Motor(AP_MOTORS_MOT_8,  -108, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   8),
    Motor(AP_MOTORS_MOT_9,   -72, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  9),
    Motor(AP_MOTORS_MOT_10,  -36, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  10)
};

static Motor deca_cw_x_motors[] =
{
    Motor(AP_MOTORS_MOT_1,    18, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1),
    Motor(AP_MOTORS_MOT_2,    54, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2),
    Motor(AP_MOTORS_MOT_3,    90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  3),
    Motor(AP_MOTORS_MOT_4,   126, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   4),
    Motor(AP_MOTORS_MOT_5,   162, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  5),
    Motor(AP_MOTORS_MOT_6,  -162, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   6),
    Motor(AP_MOTORS_MOT_7,  -126, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  7),
    Motor(AP_MOTORS_MOT_8,   -90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   8),
    Motor(AP_MOTORS_MOT_9,   -54, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  9),
    Motor(AP_MOTORS_MOT_10,  -18, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  10)
};

static Motor tri_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_2,  -60, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 3),
    Motor(AP_MOTORS_MOT_4,  180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2, AP_MOTORS_MOT_7, 60, -60, -1, 0, 0),
};

static Motor tilttri_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1, -1, 0, 0, AP_MOTORS_MOT_8, 0, -90),
    Motor(AP_MOTORS_MOT_2,  -60, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3, -1, 0, 0, AP_MOTORS_MOT_8, 0, -90),
    Motor(AP_MOTORS_MOT_4,  180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2, AP_MOTORS_MOT_7, 60, -60, -1, 0, 0),
};

static Motor tilttri_vectored_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1, -1, 0, 0, 7, 10, -90),
    Motor(AP_MOTORS_MOT_2,  -60, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3, -1, 0, 0, 8, 10, -90),
    Motor(AP_MOTORS_MOT_4,  180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2)
};

static Motor y6_motors[] =
{
    Motor(AP_MOTORS_MOT_1,  60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2),
    Motor(AP_MOTORS_MOT_2, -60, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5),
    Motor(AP_MOTORS_MOT_3, -60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6),
    Motor(AP_MOTORS_MOT_4, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4),
    Motor(AP_MOTORS_MOT_5,  60, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1),
    Motor(AP_MOTORS_MOT_6, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3)
};

/*
  FireflyY6 is a Y6 with front motors tiltable using servo on channel 9 (output 8)
 */
static Motor firefly_motors[] =
{
    Motor(AP_MOTORS_MOT_1, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_2,  60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1, -1, 0, 0, 6, 0, -90),
    Motor(AP_MOTORS_MOT_3, -60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5, -1, 0, 0, 6, 0, -90),
    Motor(AP_MOTORS_MOT_4, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4),
    Motor(AP_MOTORS_MOT_5,  60, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2, -1, 0, 0, 6, 0, -90),
    Motor(AP_MOTORS_MOT_6, -60, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6, -1, 0, 0, 6, 0, -90)
};

/*
  table of supported frame types. String order is important for
  partial name matching
 */
static Frame supported_frames[] =
{
    Frame("+",         4, quad_plus_motors),
    Frame("quad",      4, quad_plus_motors),
    Frame("copter",    4, quad_plus_motors),
    Frame("x",         4, quad_x_motors),
    Frame("bfxrev",    4, quad_bf_x_rev_motors),
    Frame("bfx",       4, quad_bf_x_motors),
    Frame("djix",      4, quad_dji_x_motors),
    Frame("cwx",       4, quad_cw_x_motors),
    Frame("tilthvec",  4, tiltquad_h_vectored_motors),
    Frame("hexax",     6, hexax_motors),
    Frame("hexa-cwx",  6, hexa_cw_x_motors),
    Frame("hexa-dji",  6, hexa_dji_x_motors),
    Frame("hexa",      6, hexa_motors),
    Frame("octa-cwx",  8, octa_cw_x_motors),
    Frame("octa-dji",  8, octa_dji_x_motors),
    Frame("octa-quad-cwx",8, octa_quad_cw_x_motors),
    Frame("octa-quad", 8, octa_quad_motors),
    Frame("octa",      8, octa_motors),
    Frame("deca",     10, deca_motors),
    Frame("deca-cwx", 10, deca_cw_x_motors),
    Frame("dodeca-hexa", 12, dodeca_hexa_motors),
    Frame("tri",       3, tri_motors),
    Frame("tilttrivec",3, tilttri_vectored_motors),
    Frame("tilttri",   3, tilttri_motors),
    Frame("y6",        6, y6_motors),
    Frame("firefly",   6, firefly_motors),
    Frame("tilt",      4, tiltquad),
    Frame("df200",     8, df200_motors),
    Frame("jt20",      8, jt20_motors),
};

// get air density in kg/m^3
float Frame::get_air_density(float alt_amsl) const
{
    return AP_Baro::get_air_density_for_alt_amsl(alt_amsl);
}

/*
  load frame specific parameters from a json file if available
 */
void Frame::load_frame_params(const char *model_json)
{
    char *fname = nullptr;
    struct stat st;
    if (AP::FS().stat(model_json, &st) == 0) {
        fname = strdup(model_json);
    } else {
        IGNORE_RETURN(asprintf(&fname, "@ROMFS/models/%s", model_json));
        if (AP::FS().stat(model_json, &st) != 0) {
            AP_HAL::panic("%s failed to load\n", model_json);
        }
    }
    if (fname == nullptr) {
        AP_HAL::panic("%s failed to load\n", model_json);
    }
    AP_JSON::value *obj = AP_JSON::load_json(model_json);
    if (obj == nullptr) {
        AP_HAL::panic("%s failed to load\n", model_json);
    }

    enum class VarType {
        FLOAT,
        VECTOR3F,
    };

    struct json_search {
        const char *label;
        void *ptr;
        VarType t;
    };
    
    json_search vars[] = {
#define FRAME_VAR(s) { #s, &model.s, VarType::FLOAT }
        FRAME_VAR(mass),
        FRAME_VAR(diagonal_size),
        FRAME_VAR(refSpd),
        FRAME_VAR(refAngle),
        FRAME_VAR(refVoltage),
        FRAME_VAR(refCurrent),
        FRAME_VAR(refAlt),
        FRAME_VAR(refTempC),
        FRAME_VAR(maxVoltage),
        FRAME_VAR(battCapacityAh),
        FRAME_VAR(refBatRes),
        FRAME_VAR(propExpo),
        FRAME_VAR(refRotRate),
        FRAME_VAR(hoverThrOut),
        FRAME_VAR(pwmMin),
        FRAME_VAR(pwmMax),
        FRAME_VAR(spin_min),
        FRAME_VAR(spin_max),
        FRAME_VAR(slew_max),
        FRAME_VAR(disc_area),
        FRAME_VAR(mdrag_coef),
        {"moment_inertia", &model.moment_of_inertia, VarType::VECTOR3F},
        FRAME_VAR(num_motors),
    };

    for (uint8_t i=0; i<ARRAY_SIZE(vars); i++) {
        auto v = obj->get(vars[i].label);
        if (v.is<AP_JSON::null>()) {
            // use default value
            continue;
        }
        if (vars[i].t == VarType::FLOAT) {
            parse_float(v, vars[i].label, *((float *)vars[i].ptr));

        } else if (vars[i].t == VarType::VECTOR3F) {
            parse_vector3(v, vars[i].label, *(Vector3f *)vars[i].ptr);

        }
    }

    json_search per_motor_vars[] = {
        {"position", &model.motor_pos, VarType::VECTOR3F},
        {"vector", &model.motor_thrust_vec, VarType::VECTOR3F},
        {"yaw", &model.yaw_factor, VarType::FLOAT},
    };
    char label_name[20];
    for (uint8_t i=0; i<ARRAY_SIZE(per_motor_vars); i++) {
        for (uint8_t j=0; j<12; j++) {
            snprintf(label_name, 20, "motor%i_%s", j+1, per_motor_vars[i].label);
            auto v = obj->get(label_name);
            if (v.is<AP_JSON::null>()) {
                // use default value
                continue;
            }
            if (per_motor_vars[i].t == VarType::FLOAT) {
                parse_float(v, label_name, *(((float *)per_motor_vars[i].ptr) + j));

            } else if (per_motor_vars[i].t == VarType::VECTOR3F) {
                parse_vector3(v, label_name, *(((Vector3f *)per_motor_vars[i].ptr) + j));
            }
        }
    }

    delete obj;

    ::printf("Loaded model params from %s\n", model_json);
}

void Frame::parse_float(AP_JSON::value val, const char* label, float &param) {
    if (!val.is<double>()) {
        AP_HAL::panic("Bad json type for %s: %s", label, val.to_str().c_str());
    }
    param = val.get<double>();
}

void Frame::parse_vector3(AP_JSON::value val, const char* label, Vector3f &param) {
    if (!val.is<AP_JSON::value::array>() || !val.contains(2) || val.contains(3)) {
        AP_HAL::panic("Bad json type for %s: %s", label, val.to_str().c_str());
    }
    for (uint8_t j=0; j<3; j++) {
        parse_float(val.get(j), label, param[j]);
    }
}

#if AP_SIM_ENABLED

/*
  initialise the frame
 */
void Frame::init(const char *frame_str, Battery *_battery)
{
    model = default_model;
    battery = _battery;

    const char *colon = strchr(frame_str, ':');
    size_t slen = strlen(frame_str);
    if (colon != nullptr && slen > 5 && strcmp(&frame_str[slen-5], ".json") == 0) {
        load_frame_params(colon+1);
    }
    mass = model.mass;

    const float drag_force = model.mass * GRAVITY_MSS * tanf(radians(model.refAngle));

    const float cos_tilt = cosf(radians(model.refAngle));
    const float airspeed_bf = model.refSpd * cos_tilt;
    const float ref_thrust = model.mass * GRAVITY_MSS / cos_tilt;
    float ref_air_density = get_air_density(model.refAlt);

    const float momentum_drag = cos_tilt * model.mdrag_coef * airspeed_bf * sqrtf(ref_thrust * ref_air_density * model.disc_area);

    if (momentum_drag > drag_force) {
        model.mdrag_coef *= drag_force / momentum_drag;
        areaCd = 0.0;
        ::printf("Suggested EK3_DRAG_BCOEF_* = 0, EK3_DRAG_MCOEF = %.3f\n", (momentum_drag / (model.mass * airspeed_bf)) * sqrtf(1.225f / ref_air_density));
    } else {
        areaCd = (drag_force - momentum_drag) / (0.5f * ref_air_density * sq(model.refSpd));
        ::printf("Suggested EK3_DRAG_BCOEF_* = %.3f, EK3_DRAG_MCOEF = %.3f\n", model.mass / areaCd, (momentum_drag / (model.mass * airspeed_bf)) * sqrtf(1.225f / ref_air_density));
    }

    terminal_rotation_rate = model.refRotRate;

    float hover_thrust = mass * GRAVITY_MSS;
    float hover_power = model.refCurrent * model.refVoltage;
    float hover_velocity_out = 2 * hover_power / hover_thrust;
    float effective_disc_area = hover_thrust / (0.5 * ref_air_density * sq(hover_velocity_out));
    float velocity_max = hover_velocity_out / sqrtf(model.hoverThrOut);
    float effective_prop_area = effective_disc_area / num_motors;
    float true_prop_area = model.disc_area / num_motors;

    // power_factor is ratio of power consumed per newton of thrust
    float power_factor = hover_power / hover_thrust;

    battery->setup(model.battCapacityAh, model.refBatRes, model.maxVoltage);

    if (uint8_t(model.num_motors) != num_motors) {
        ::printf("Warning model expected %u motors and got %u\n", uint8_t(model.num_motors), num_motors);
    }

    for (uint8_t i=0; i<num_motors; i++) {
        motors[i].setup_params(model.pwmMin, model.pwmMax, model.spin_min, model.spin_max, model.propExpo, model.slew_max,
                               model.diagonal_size, power_factor, model.maxVoltage, effective_prop_area, velocity_max,
                               model.motor_pos[i], model.motor_thrust_vec[i], model.yaw_factor[i], true_prop_area,
                               model.mdrag_coef);
    }

    // DF-200 test path: replace momentum-disc thrust with measured prop tables.
    // Inner MOT_1..4 (tilt) = 40in cruise prop; outer MOT_5..8 (fixed) = 60in lift prop.
    // See SIM_MotorTable.h / docs/电机电池.docx. No JSON keys - hard-coded for this frame only.
    if (name != nullptr && strcmp(name, "df200") == 0 && num_motors >= 8) {
        for (uint8_t i=0; i<4; i++) {
            motors[i].set_thrust_model(Motor::TABLE_DF200_INNER_40IN);
        }
        for (uint8_t i=4; i<8; i++) {
            motors[i].set_thrust_model(Motor::TABLE_DF200_OUTER_60IN);
        }
        ::printf("DF-200: thrust model = prop tables (inner 40in, outer 60in)\n");
    }

    if (is_zero(model.moment_of_inertia.x) || is_zero(model.moment_of_inertia.y) || is_zero(model.moment_of_inertia.z)) {
        // if no inertia provided, assume 50% of mass on ring around center
        model.moment_of_inertia.x = model.mass * 0.25 * sq(model.diagonal_size*0.5);
        model.moment_of_inertia.y = model.moment_of_inertia.x;
        model.moment_of_inertia.z = model.mass * 0.5 * sq(model.diagonal_size*0.5);
    }

    // setup reasonable defaults for battery
    AP_Param::set_default_by_name("SIM_BATT_VOLTAGE", model.maxVoltage);
    AP_Param::set_default_by_name("SIM_BATT_CAP_AH", model.battCapacityAh);
    if (model.battCapacityAh > 0) {
        AP_Param::set_default_by_name("BATT_CAPACITY", model.battCapacityAh*1000);
    }
}

/*
  find a frame by name
 */
Frame *Frame::find_frame(const char *name)
{
    for (uint8_t i=0; i < ARRAY_SIZE(supported_frames); i++) {
        // do partial name matching to allow for frame variants
        if (strncasecmp(name, supported_frames[i].name, strlen(supported_frames[i].name)) == 0) {
            return &supported_frames[i];
        }
    }
    return nullptr;
}

// calculate rotational and linear accelerations
void Frame::calculate_forces(const Aircraft &aircraft,
                             const struct sitl_input &input,
                             Vector3f &rot_accel,
                             Vector3f &body_accel,
                             float* rpm,
                             bool use_drag)
{
    Vector3f thrust; // newtons
    Vector3f torque;

    const float air_density = get_air_density(aircraft.get_location().alt*0.01);
    Vector3f gyro = aircraft.get_gyro();

    Vector3f vel_air_bf = aircraft.get_dcm().transposed() * aircraft.get_velocity_air_ef();

    const auto *_sitl = AP::sitl();
    const bool df200_frame = (name != nullptr && strcmp(name, "df200") == 0);
    const bool df200_rotor_log = df200_frame;
    // ISOLATION TEST (161+): the TIMER roll/yaw blow-up is a positive-feedback
    // loop p/r -> position%gyro -> per-motor mvelz(L-R split) -> velin split
    // -> calc_thrust(velin^2) -> asymmetric thrust -> geom Mx(same sign as p)
    // -> p grows -> repeat, saturates in ~0.2s. Data traced every step.
    // Test A: above AS_HI cut the body-rate contribution to motor airspeed
    // (zero the gyro passed to Motor::calculate_forces). Physically the
    // position%gyro term is real, but calc_thrust squares velin and amplifies
    // the L-R asymmetry into a same-sign Mx (gain>1). If Test A stops the
    // blow-up -> root cause confirmed. NOT a final fix - diagnosis only.
    // Run#1 leave true (baseline with table model); Run#2 flip to false to
    // verify the table model alone kills the TIMER positive-feedback blow-up.
    // Run#2 (2026-09-21): flipped to false. Rationale: the table model broke
    // the amplifier — outer prop60_thrust(rpm) has no velin at all, inner
    // prop40 dT/dv ≈ -7..-10 N/(m/s) (wind-tunnel advance effect) vs the
    // momentum disc's 50..112 N/(m/s) at AS≈50, so loop gain ≈0.1 (was >1
    // at 161). Physical position%gyro inflow is restored. If the TIMER
    // blow-up returns, the table model did NOT kill the loop and §11.10's
    // real fixes (velin clamp / collective-only tilt>30°) are needed.
    static bool df200_zero_gyro_enable = false;
    const bool df200_zero_gyro_for_motor =
        df200_frame && df200_zero_gyro_enable &&
        aircraft.get_velocity_air_ef().length() >= 25.0f;  // AS>=25 only
    const Vector3f gyro_for_motor = df200_zero_gyro_for_motor ? Vector3f{} : gyro;
    Vector3f sum_geom, sum_rotor, sum_mdrag;
    static uint64_t df200_rotor_log_last_ms;
    // per-motor breakdown for the 154 roll-snap investigation:
    // which motor's r×F makes the sudden Mx, and are outputs rail-clipped.
    Vector3f pm_geom[8];
    Vector3f pm_thrust[8];
    float pm_cmd[8];
    float pm_velin[8];
    float pm_thrustmag[8];
    Vector3f pm_mvel[8];

    // test-only motor failure injection (SIM_MOT_FAIL): bitmask of motors whose
    // physical output is forced to zero (prop-out failure). The mixer keeps
    // commanding the failed motor, so thrust-loss detection and control
    // reallocation see a genuine failure signature
    const uint16_t motor_fail_mask = _sitl->motor_fail_mask.get();

    for (uint8_t i=0; i<num_motors; i++) {
        Vector3f mtorque, mthrust;
        if (df200_rotor_log) {
            Vector3f geom_tq, rotor_tq, mdrag_f;
            motors[i].calculate_forces(input, motor_offset, mtorque, mthrust, vel_air_bf, gyro_for_motor, air_density, battery->get_voltage(), use_drag,
                                       geom_tq, rotor_tq, mdrag_f,
                                       &pm_velin[i], &pm_thrustmag[i], &pm_mvel[i]);
            sum_geom += geom_tq;
            sum_rotor += rotor_tq;
            sum_mdrag += mdrag_f;
            if (i < 8) {
                pm_geom[i] = geom_tq;
                pm_thrust[i] = mthrust;
                pm_cmd[i] = motors[i].get_command();
            }
        } else {
            motors[i].calculate_forces(input, motor_offset, mtorque, mthrust, vel_air_bf, gyro_for_motor, air_density, battery->get_voltage(), use_drag);
        }
        if (motor_fail_mask & (1U << i)) {
            // declared failed (bit position = motor index, bit0 = motor 1):
            // no thrust, no torque, rpm zero
            mtorque.zero();
            mthrust.zero();
            rpm[motor_offset+i] = 0;
            continue;
        }
        torque += mtorque;
        thrust += mthrust;
        // simulate motor rpm
        if (!is_zero(_sitl->vibe_motor)) {
            rpm[motor_offset+i] = motors[i].get_command() * AP::sitl()->vibe_motor * 60.0f;
        }
    }

    // DF-200: rotor force/moment budget ~20 Hz, no AS gate (need hover force budget too).
    // QuadPlane calls this with use_drag=false, so mdrag terms should be ~0 here;
    // geometry (r×F) and rotor reaction are the useful split.
    //   /tmp/df200_rotor.log
    if (df200_rotor_log) {
        // Aircraft::airspeed is protected; use body-frame air-relative speed.
        const float as = aircraft.get_velocity_air_ef().length();
        const uint64_t now_ms = AP_HAL::millis64();
        if (now_ms - df200_rotor_log_last_ms >= 50) {
            df200_rotor_log_last_ms = now_ms;
            FILE *fp = fopen("/tmp/df200_rotor.log", "a");
            if (fp != nullptr) {
                fprintf(fp,
                        "DF200ROTOR t=%.2f AS=%.1f use_drag=%d "
                        "F=%.1f,%.1f,%.1f "
                        "Mtot=%.1f,%.1f,%.1f "
                        "Mgeom=%.1f,%.1f,%.1f "
                        "Mrotor=%.1f,%.1f,%.1f "
                        "Fmdrag=%.1f,%.1f,%.1f "
                        "pqr=%.3f,%.3f,%.3f "
                        "I=%.1f,%.1f,%.1f "
                        "cmd=%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f "
                        "Mxi=%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f "
                        "Tzi=%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f "
                        "Txi=%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f "
                        "velin=%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f "
                        "thrmag=%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f "
                        "rpm=%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f "
                        "mvelx=%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f "
                        "mvely=%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f "
                        "mvelz=%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n",
                        now_ms * 1.0e-3,
                        as,
                        (int)use_drag,
                        thrust.x, thrust.y, thrust.z,
                        torque.x, torque.y, torque.z,
                        sum_geom.x, sum_geom.y, sum_geom.z,
                        sum_rotor.x, sum_rotor.y, sum_rotor.z,
                        sum_mdrag.x, sum_mdrag.y, sum_mdrag.z,
                        gyro.x, gyro.y, gyro.z,
                        model.moment_of_inertia.x,
                        model.moment_of_inertia.y,
                        model.moment_of_inertia.z,
                        pm_cmd[0], pm_cmd[1], pm_cmd[2], pm_cmd[3],
                        pm_cmd[4], pm_cmd[5], pm_cmd[6], pm_cmd[7],
                        pm_geom[0].x, pm_geom[1].x, pm_geom[2].x, pm_geom[3].x,
                        pm_geom[4].x, pm_geom[5].x, pm_geom[6].x, pm_geom[7].x,
                        pm_thrust[0].z, pm_thrust[1].z, pm_thrust[2].z, pm_thrust[3].z,
                        pm_thrust[4].z, pm_thrust[5].z, pm_thrust[6].z, pm_thrust[7].z,
                        pm_thrust[0].x, pm_thrust[1].x, pm_thrust[2].x, pm_thrust[3].x,
                        pm_thrust[4].x, pm_thrust[5].x, pm_thrust[6].x, pm_thrust[7].x,
                        pm_velin[0], pm_velin[1], pm_velin[2], pm_velin[3],
                        pm_velin[4], pm_velin[5], pm_velin[6], pm_velin[7],
                        pm_thrustmag[0], pm_thrustmag[1], pm_thrustmag[2], pm_thrustmag[3],
                        pm_thrustmag[4], pm_thrustmag[5], pm_thrustmag[6], pm_thrustmag[7],
                        motors[0].get_rpm(), motors[1].get_rpm(), motors[2].get_rpm(), motors[3].get_rpm(),
                        motors[4].get_rpm(), motors[5].get_rpm(), motors[6].get_rpm(), motors[7].get_rpm(),
                        pm_mvel[0].x, pm_mvel[1].x, pm_mvel[2].x, pm_mvel[3].x,
                        pm_mvel[4].x, pm_mvel[5].x, pm_mvel[6].x, pm_mvel[7].x,
                        pm_mvel[0].y, pm_mvel[1].y, pm_mvel[2].y, pm_mvel[3].y,
                        pm_mvel[4].y, pm_mvel[5].y, pm_mvel[6].y, pm_mvel[7].y,
                        pm_mvel[0].z, pm_mvel[1].z, pm_mvel[2].z, pm_mvel[3].z,
                        pm_mvel[4].z, pm_mvel[5].z, pm_mvel[6].z, pm_mvel[7].z);
                fclose(fp);
            }
        }
    }

    // calculate total rotational acceleration
    // inertia perturbation (SIM_INERTIA_SCL, docs/df200_bias_sim.md §2.5):
    // divides all three axes; >1 means higher inertia / slower response.
    // (_sitl already declared at :614 in this function.)
    const float inertia_scale = _sitl ? MAX(_sitl->inertia_scale, 0.05f) : 1.0f;
    rot_accel.x = torque.x / (model.moment_of_inertia.x * inertia_scale);
    rot_accel.y = torque.y / (model.moment_of_inertia.y * inertia_scale);
    rot_accel.z = torque.z / (model.moment_of_inertia.z * inertia_scale);

    if (terminal_rotation_rate > 0) {
        // rotational air resistance
        rot_accel.x -= gyro.x * radians(400.0) / terminal_rotation_rate;
        rot_accel.y -= gyro.y * radians(400.0) / terminal_rotation_rate;
        rot_accel.z -= gyro.z * radians(400.0) / terminal_rotation_rate;
    }

    if (use_drag) {
        // use the model params to calculate drag
        Vector3f drag_bf;
        drag_bf.x = areaCd * 0.5f * air_density * sq(vel_air_bf.x);
        if (is_negative(vel_air_bf.x)) {
            drag_bf.x = -drag_bf.x;
        }

        drag_bf.y = areaCd * 0.5f * air_density * sq(vel_air_bf.y);
        if (is_negative(vel_air_bf.y)) {
            drag_bf.y = -drag_bf.y;
        }

        drag_bf.z = areaCd * 0.5f * air_density * sq(vel_air_bf.z);
        if (is_negative(vel_air_bf.z)) {
            drag_bf.z = -drag_bf.z;
        }

        thrust -= drag_bf;
    }

    // mass perturbation (SIM_MASS_SCL, docs/df200_bias_sim.md §2.5): paired
    // with the same scale at Plane::calculate_forces accel_body /= mass, or
    // the plane/quad parts see inconsistent effective gravity.
    const float mass_scale = _sitl ? _sitl->mass_scale : 1.0f;
    body_accel = thrust/(aircraft.gross_mass() * mass_scale);
}


// calculate current and voltage
void Frame::current_and_voltage(float &voltage, float &current)
{
    float param_voltage = AP::sitl()->batt_voltage;
    if (!is_equal(last_param_voltage,param_voltage)) {
        battery->init_voltage(param_voltage);
        last_param_voltage = param_voltage;
    }
    voltage = battery->get_voltage();
    current = 0;
    for (uint8_t i=0; i<num_motors; i++) {
        current += motors[i].get_current();
    }
}
#endif // AP_SIM_ENABLED
