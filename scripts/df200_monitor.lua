-- DF-200 电机失效检测监控脚本
-- 每 1s 上报 thrust boost 失效检测状态到 GCS Messages
--
-- 背景：AP_MotorsMatrix::check_for_failed_motor() 每混控周期在跑（检测阈值
--   thrust_balance>=1.5，仅 >=6 电机启用），但 Plane 无 set_thrust_boost(true)
--   调用方，_thrust_boost 恒 false——本脚本预期一直报 MotorOK。
--   作用：1) 验证检测代码路径存活；2) 移植 thrust_loss_check 后自动变成
--   失效上报通道；3) 与 motor_failure_test_param.lua 配合做注入-上报闭环。

local last = -1

function update()
  local idx
  if MotorsMatrix:get_thrust_boost() then
    idx = MotorsMatrix:get_lost_motor()
  else
    idx = -1
  end

  if idx ~= last then
    if idx == -1 then
      gcs:send_text(0, "DF200 monitor: MotorOK")
    else
      gcs:send_text(6, string.format("DF200 monitor: LostMotor %d", idx + 1))
    end
    last = idx
  end

  return update, 1000 -- 1Hz
end

return update, 1000
