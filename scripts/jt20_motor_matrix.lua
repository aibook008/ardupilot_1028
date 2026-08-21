-- JT-20 custom motor matrix for ArduPilot
-- Inner 4 tilting + outer 4 fixed vertical (jt_20.sdf).
-- Must match libraries/SITL/SIM_Frame.cpp jt20_motors[]:
--   same motor indices, same angles, same CW/CCW signs.
--
-- add_motor_raw(index, roll_factor, pitch_factor, yaw_factor, testing_order)
-- APM convention (nominal):
--   roll  = cos(angle_deg + 90)
--   pitch = cos(angle_deg)
--   yaw: CW=+1, CCW=-1
--   throttle = 1
--
-- 2026-08-21: yaw signs FLIPPED vs SITL jt20_motors / nominal CW/CCW.
-- log 00000010 showed sustained ~-440 deg/s yaw with equal spool then
-- controller fighting but not stopping — likely mixer yaw vs SIM torque
-- sign mismatch. roll/pitch unchanged. Revert if spin direction worsens.

local M = {}

M[0] = { roll = -0.7349, pitch =  0.6782, yaw =  1, order = 1 } -- M1  47.3  was CCW(-1) → +1
M[1] = { roll =  0.5693, pitch = -0.8221, yaw =  1, order = 2 } -- M2 -145.3 was CCW(-1) → +1
M[2] = { roll =  0.7396, pitch =  0.6730, yaw = -1, order = 3 } -- M3  -47.7 was CW(+1)  → -1
M[3] = { roll = -0.5693, pitch = -0.8221, yaw = -1, order = 4 } -- M4 145.3  was CW(+1)  → -1
M[4] = { roll = -0.9650, pitch =  0.2622, yaw =  1, order = 5 } -- M5  74.8  was CCW(-1) → +1
M[5] = { roll =  0.8910, pitch = -0.4540, yaw =  1, order = 6 } -- M6 -117.0 was CCW(-1) → +1
M[6] = { roll =  0.9650, pitch =  0.2622, yaw = -1, order = 7 } -- M7  -74.8 was CW(+1)  → -1
M[7] = { roll = -0.8910, pitch = -0.4540, yaw = -1, order = 8 } -- M8 117.0  was CW(+1)  → -1

for i = 0, 7 do
    MotorsMatrix:add_motor_raw(i, M[i].roll, M[i].pitch, M[i].yaw, M[i].order)
end

assert(MotorsMatrix:init(8), "Failed to init JT-20 MotorsMatrix")
motors:set_frame_string("JT-20 custom 8x tiltquad yawflip")

gcs:send_text(6, "JT-20 motor matrix loaded (yaw flipped)")
