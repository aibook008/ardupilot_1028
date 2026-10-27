-- DF-200 电机失效注入脚本 v6（boost 开关 + 参数管理）
-- 分工（避免因子表双写冲突，见 log 32 分析）：
--   失效因子清零：df200_motor_matrix.lua（因子表唯一写者，读 SCR_USER1+MOT_STOP_BITMASK）
--   boost 开关  ：本脚本（SCR_USER1 变化时直接 set_thrust_boost）
--
-- 参数：
--   MOT_STOP_BITMASK : 要停的电机 bitmask（本脚本创建，KEY 75）
--   SCR_USER1        : 1=注入失效, 0=恢复
--
-- 与飞控侧 thrust_loss_check 分工：
--   注入测试：Q_TLOSS_ENABLE=0（boost 由本脚本直控，无逻辑打架）
--   真实故障：Q_TLOSS_ENABLE=1（thrust_loss_check 触发，本脚本 boost 部分闲置）

local INFO = 6

-- 创建 MOT_STOP_BITMASK 参数
local PARAM_TABLE_KEY = 75
assert(param:add_table(PARAM_TABLE_KEY, "MOT_", 1), "could not add param table")
assert(param:add_param(PARAM_TABLE_KEY, 1, "STOP_BITMASK", 0), "could not add param")

local stop_bitmask = Parameter()
if not stop_bitmask:init("MOT_STOP_BITMASK") then
  return update, 1000
end

local trigger = Parameter()
assert(trigger:init("SCR_USER1"), "no SCR_USER1 param")

local last_trigger = -1

function update()
  local t = trigger:get()
  local bm = stop_bitmask:get()

  if t ~= last_trigger then
    gcs:send_text(INFO, string.format("mft v6: trigger=%d bitmask=%d", t, bm))
    if t == 1 then
      -- 失效因子清零由 df200_motor_matrix.lua 完成（检测到 mask 变化即重载）
      -- 本脚本只负责开 boost：激活混控器重分配
      MotorsMatrix:set_thrust_boost(true)
      gcs:send_text(INFO, string.format("mft v6: injected mask=0x%X, boost ON", bm))
    else
      MotorsMatrix:set_thrust_boost(false)
      gcs:send_text(INFO, "mft v6: restored, boost OFF")
    end
    last_trigger = t
  end

  return update, 50 -- 20Hz
end

gcs:send_text(INFO, "mft v6 ready (boost switch; factors via df200 script)")
return update()
