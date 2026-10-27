-- DF-200 dynamic motor matrix (Q_FRAME_CLASS=17 / Motors_dynamic)
-- Inner 4 tilting (40in) + outer 4 fixed vertical (60in).
-- Geometry from docs/数据库_v2.xlsx 旋翼位置 → APM body about hover CG.
-- Must match libraries/SITL/SIM_Frame.cpp df200_motors[] angles and
-- Tools/autotest/models/df200.json motor order / yaw signs.
--
-- Why dynamic (17) instead of static MotorsMatrix (15):
--   add_motor_raw() is ignored after init. Need runtime factor updates.
--
-- Airspeed fade (re-enabled for transition pitch/roll study):
--   AS ≤ AS_LO (15) → full mix
--   AS ≥ AS_HI (25) → attenuated mix
--   between         → linear
--
-- Channel floors at high AS (pull-back trial vs 118 wall):
--   roll → 0.0   (same isolation idea)
--   pitch → PITCH_FLOOR (keep some VTOL pitch authority)
--   yaw   → YAW_FLOOR   (keep some yaw damping against dutch-roll seed)
-- Rationale: 116/118 with all R/P/Y→0 still blew at AS≈27; first axis was
-- yaw/roll. Keeping pitch/yaw residual asks: can VTOL actions still arrest
-- the divergence while Mot roll differential is gone.
-- No latch (119–124 latch zeroed mix from takeoff).
--
-- Motor map (APM Motor1..8 / Lua index 0..7):
--   M1 ① FR tilt  +37.32°
--   M2 ⑥ RL tilt -142.52°
--   M3 ⑤ FL tilt  -37.32°
--   M4 ② RR tilt +142.52°
--   M5 ③ FR vert  +67.08°
--   M6 ⑧ RL vert -112.74°
--   M7 ⑦ FL vert  -67.08°
--   M8 ④ RR vert +112.74°
--
-- roll  = cos(angle_deg + 90)
-- pitch = cos(angle_deg)
-- yaw: flipped vs nominal CW/CCW (same JT-20 convention)

local MAV_SEVERITY_INFO = 6
local UPDATE_MS = 100

local BASE = {
    --          roll      pitch     yaw    order
    { -0.6063,  0.7953,  1, 1 }, -- M1
    {  0.6085, -0.7936,  1, 2 }, -- M2
    {  0.6063,  0.7953, -1, 3 }, -- M3
    { -0.6085, -0.7936, -1, 4 }, -- M4
    { -0.9210,  0.3894,  1, 5 }, -- M5
    {  0.9223, -0.3866,  1, 6 }, -- M6
    {  0.9210,  0.3894,  -1, 7 }, -- M7
    { -0.9223, -0.3866,  -1, 8 }, -- M8
}

local AS_LO = 15.0
local AS_HI = 25.0
-- 137: P/Y floor=0.3 still blew at AS≈27 with lead axis yaw→roll and YOut≈0.
-- Keep roll faded (isolation), keep some pitch, but leave yaw FULL so VTOL
-- yaw damping stays available against the dutch-roll seed.
-- 155 (all floors to 0): 153/154 TIMER roll snap came from rotor differential
-- authority that should not exist at high speed: P=0.3 kept fighting
-- Mgeom_y≈+2100 Nm front-high/rear-low, rear tilt motors pinned at the 0
-- rail, breaking differential cancellation -> sudden net Mx. AS≥25 now =
-- pure collective rotor; FW surfaces (ail 0.091 / elev 0.42 / rudder, 153)
-- are the ONLY attitude actuators. Test: can FW control alone reach FW done?
local ROLL_FLOOR = 0.0
local PITCH_FLOOR = 0.0
local YAW_FLOOR = 0.0
local SCALE_EPS = 0.05

local last_roll = -1.0
local last_pitch = -1.0
local last_yaw = -1.0
local last_fail_mask = 0
local inited = false

local function lerp_scale(as, floor)
    if as == nil or as < 0 then
        as = 0
    end
    if as <= AS_LO then
        return 1.0
    end
    if as >= AS_HI then
        return floor
    end
    local x = (as - AS_LO) / (AS_HI - AS_LO)
    return 1.0 + (floor - 1.0) * x
end

local function apply_factors(roll_s, pitch_s, yaw_s)
    -- motor failure injection: SCR_USER1=1 zeroes the motors selected by
    -- MOT_STOP_BITMASK (added by motor_failure_test_param.lua, KEY 75).
    -- Zeroed factors make the mixer treat those motors as failed, so
    -- check_for_failed_motor detects the imbalance and thrust boost
    -- reallocation acts on the real allocation path.
    local fail_mask = 0
    if param:get("SCR_USER1") == 1 then
        local m = param:get("MOT_STOP_BITMASK")
        if m then
            fail_mask = m
        end
    end
    local factors = motor_factor_table()
    for i = 0, 7 do
        local b = BASE[i + 1]
        if (fail_mask & (1 << i)) ~= 0 then
            factors:roll(i, 0)
            factors:pitch(i, 0)
            factors:yaw(i, 0)
            factors:throttle(i, 0)
        else
            factors:roll(i, b[1] * roll_s)
            factors:pitch(i, b[2] * pitch_s)
            factors:yaw(i, b[3] * yaw_s)
            factors:throttle(i, 1.0)
        end
    end
    Motors_dynamic:load_factors(factors)
end

local function ensure_init()
    if inited then
        return true
    end
    for i = 0, 7 do
        Motors_dynamic:add_motor(i, BASE[i + 1][4])
    end
    apply_factors(1.0, 1.0, 1.0)
    if not Motors_dynamic:init(8) then
        gcs:send_text(3, "DF-200: Motors_dynamic:init failed")
        return false
    end
    motors:set_frame_string("DF-200 dyn 8x fade-PYkeep")
    gcs:send_text(MAV_SEVERITY_INFO,
        string.format("DF-200 dyn: AS fade; R→%.2f P→%.2f Y→%.2f",
            ROLL_FLOOR, PITCH_FLOOR, YAW_FLOOR))
    inited = true
    last_roll, last_pitch, last_yaw = 1.0, 1.0, 1.0
    return true
end

local function update()
    if not ensure_init() then
        return update, UPDATE_MS
    end

    -- track failure mask (SCR_USER1=1 injects per MOT_STOP_BITMASK);
    -- reload factors whenever it changes so injection is immediate
    local fail_mask = 0
    if param:get("SCR_USER1") == 1 then
        local m = param:get("MOT_STOP_BITMASK")
        if m then
            fail_mask = m
        end
    end
    if fail_mask ~= last_fail_mask then
        gcs:send_text(MAV_SEVERITY_INFO,
            string.format("DF-200 fail mask 0x%X -> 0x%X", last_fail_mask, fail_mask))
        last_fail_mask = fail_mask
        -- force reload this cycle (drop below threshold comparison)
        last_roll, last_pitch, last_yaw = -1, -1, -1
    end

    local as = ahrs:airspeed_estimate()
    if as == nil then
        as = 0.0
    end
    local roll_s = lerp_scale(as, ROLL_FLOOR)
    local pitch_s = lerp_scale(as, PITCH_FLOOR)
    local yaw_s = lerp_scale(as, YAW_FLOOR)

    if math.abs(roll_s - last_roll) >= SCALE_EPS
        or math.abs(pitch_s - last_pitch) >= SCALE_EPS
        or math.abs(yaw_s - last_yaw) >= SCALE_EPS then
        apply_factors(roll_s, pitch_s, yaw_s)
        last_roll, last_pitch, last_yaw = roll_s, pitch_s, yaw_s
        gcs:send_named_float("RSCALE", roll_s)
        gcs:send_named_float("PSCALE", pitch_s)
        gcs:send_named_float("YSCALE", yaw_s)
        gcs:send_named_float("AS_MIX", as)
        gcs:send_text(MAV_SEVERITY_INFO,
            string.format("DF-200 mix R=%.2f P=%.2f Y=%.2f AS=%.1f",
                roll_s, pitch_s, yaw_s, as))
    end
    return update, UPDATE_MS
end

return update, UPDATE_MS
