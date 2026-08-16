#   bash run_eval.sh <task> <task_config> <run_name> <ckpt_file> [test_num] [seed] [port] [camera_map]
# 结果与日志落在 $NEW/eval_qz/<run_name>_<task_config>_s<seed>/
set -u
NEW=${ROBOTWIN_EVAL_ROOT:-/inspire/hdd2/project/liu-ming-huan/s26058}
TASK=${1:?}; TCFG=${2:?}; RUN=${3:?}; CKPT=${4:?}
TEST_NUM=${5:-50}; SEED=${6:-0}; PORT=${7:-9977}
CAMMAP=${8:-agentview=third_view_16,robot0_eye_in_hand=left_camera,robot1_eye_in_hand=right_camera}
OUT=$NEW/eval_qz/${RUN}_${TCFG}_s${SEED}; mkdir -p $OUT
cat > $OUT/cfg.yml <<Y
policy_name: SAPolicy
port: $PORT
sapolicy_cfg: $NEW/SpatialAlignVLA/configs/exps/robotwin/exp_${RUN}.yaml
ckpt_path: $NEW/outputs/$RUN/$RUN/checkpoints/$CKPT
workspace: $NEW
device: cuda
use_ema: true
n_action_steps: 8
task_name: $TASK
task_config: $TCFG
ckpt_setting: $(basename $CKPT .ckpt)
seed: $SEED
instruction_type: unseen
eval_video_log: false
Y
pkill -f "serve.py --config $OUT/cfg.yml" 2>/dev/null
workspace=$NEW HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1 PYTHONUNBUFFERED=1 \
PYTHONPATH=$NEW/RoboTwin/policy:$NEW/SpatialAlignVLA \
setsid nohup $NEW/venv/bin/python $NEW/RoboTwin/policy/SAPolicy/serve.py --config $OUT/cfg.yml --port $PORT \
  > $OUT/server.log 2>&1 < /dev/null &
until grep -q "listening on" $OUT/server.log 2>/dev/null; do
  grep -qE "Traceback" $OUT/server.log 2>/dev/null && { echo SERVER_FAIL > $OUT/state; exit 1; }
  sleep 5
done
VK=$NEW/.nvidia_vk_550.163.01
cd $NEW/RoboTwin
echo running > $OUT/state
SAPOLICY_CAMERA_MAP="$CAMMAP" \
ROBOTWIN_SHADER=rt ROBOTWIN_DENOISER=none PYTHONWARNINGS=ignore::UserWarning \
VK_ICD_FILENAMES=$VK/icd/nvidia_icd.json VK_DRIVER_FILES=$VK/icd/nvidia_icd.json \
__EGL_VENDOR_LIBRARY_FILENAMES=$VK/icd/10_nvidia.json \
LD_LIBRARY_PATH=$VK/lib:$NEW/syslibs \
workspace=$NEW PYTHONUNBUFFERED=1 \
$NEW/rtenv/bin/python script/eval_policy_client.py --config $OUT/cfg.yml --port $PORT \
  --overrides --task_name $TASK --task_config $TCFG --policy_name SAPolicy \
  --seed $SEED --test_num $TEST_NUM \
  > $OUT/client.log 2>&1
grep -a "Success rate" $OUT/client.log | tail -1 | sed 's/\x1b\[[0-9;]*m//g' > $OUT/final.txt
echo DONE >> $OUT/state
pkill -f "serve.py --config $OUT/cfg.yml" 2>/dev/null
