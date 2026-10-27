#!/bin/bash
# DF-200 SITL restart for matrix runs (docs/df200_bias_sim.md).
# Kills any running SITL, wipes params (-w), boots fresh at KSFO, backgrounded.
pkill -9 -f "bin/arduplane" 2>/dev/null
pkill -9 -f "sim_vehicle" 2>/dev/null
sleep 2
cd "$(dirname "$0")/../.." || exit 1
nohup ./build/sitl/bin/arduplane \
  --model quadplane-df200:Tools/autotest/models/df200.json \
  -w --speedup 1 \
  --defaults Tools/autotest/default_params/quadplane.parm,Tools/autotest/default_params/quadplane-df200.parm \
  --home 37.619373,-122.376637,5.3,118.0 \
  > /tmp/df200_sv.log 2>&1 &
echo "SITL restarting, pid $!"
