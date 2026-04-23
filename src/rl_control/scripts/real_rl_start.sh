#!/bin/bash
# RL 真机飞行启动脚本
# 1) VRPN 动捕  2) MAVROS + RL 控制器

SESSION="rl_real_flight"
SETUP_CMD="source /root/trajectory_project/devel/setup.bash"

# 清理旧会话
tmux kill-session -t $SESSION 2>/dev/null

# ---- Pane 0: VRPN ----
tmux new-session -d -s $SESSION -n "RL_Flight"
tmux send-keys -t $SESSION:0.0 "source /root/realaeraflight_ws/devel/setup.bash && roslaunch vrpn_client_ros sample.launch" C-m

sleep 4s
echo "[1/2] VRPN started, waiting 4s..."

# ---- Pane 1: MAVROS + RL 控制器 ----
tmux split-window -v -t $SESSION:0.0
tmux send-keys -t $SESSION:0.1 "$SETUP_CMD && roslaunch rl_control real_rl.launch" C-m

tmux select-layout -t $SESSION:0 tiled

echo "[2/2] RL controller started."
echo "---------------------------------------------------"
echo "Pane 0: VRPN Client"
echo "Pane 1: MAVROS + RL Controller"
tmux attach-session -t $SESSION
