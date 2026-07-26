#!/bin/bash
# BlueROV 用 byobu セッション。
# パッケージ:
#   driver/blue_rov     … mavlink_driver, dvl75_driver (自動起動)
#   localization        … localization_components (任意)
#   bluerov_control     … bluerov_control (任意)
# Kyubic 用 kyubic_byobu.sh / byobu エイリアスは変更しない。

SESSION="blue_ros2_ws"
WORKSPACE="$HOME/kyubic_ros/kyubic_ws"

MAVLINK_CMD="ros2 launch mavlink_driver mavlink_driver.launch.py"
DVL75_CMD="ros2 launch dvl75_driver dvl75_driver.launch.py"
LOC_CMD="ros2 launch localization localization_components.launch.py"
CTRL_CMD="ros2 launch bluerov_control bluerov_control.launch.py"

byobu-tmux has-session -t "$SESSION" 2>/dev/null

if [ $? != 0 ]; then
    # --- Window 0: blue_rov ドライバー (自動) ---
    byobu-tmux new-session -d -s "$SESSION" -n 'Drivers'
    byobu-tmux send-keys -t "$SESSION":0 "clear" C-m
    byobu-tmux send-keys -t "$SESSION":0 "cd $WORKSPACE" C-m
    byobu-tmux send-keys -t "$SESSION":0 "$MAVLINK_CMD" C-m

    byobu-tmux split-window -v -t "$SESSION":0
    byobu-tmux send-keys -t "$SESSION":0.1 "cd $WORKSPACE" C-m
    byobu-tmux send-keys -t "$SESSION":0.1 "$DVL75_CMD" C-m

    # --- Window 1: localization / bluerov_control (任意起動) ---
    byobu-tmux new-window -t "$SESSION" -n 'Stack'
    byobu-tmux send-keys -t "$SESSION":1 "clear" C-m
    byobu-tmux send-keys -t "$SESSION":1 "cd $WORKSPACE" C-m
    byobu-tmux send-keys -t "$SESSION":1 "echo '[任意] 自己位置推定 (localization) — Enter で起動'" C-m
    byobu-tmux send-keys -t "$SESSION":1 "$LOC_CMD"

    byobu-tmux split-window -v -t "$SESSION":1
    byobu-tmux send-keys -t "$SESSION":1.1 "cd $WORKSPACE" C-m
    byobu-tmux send-keys -t "$SESSION":1.1 "echo '[任意] ミッション制御 (bluerov_control) — Enter で起動 (拘束・安全確認後)'" C-m
    byobu-tmux send-keys -t "$SESSION":1.1 "$CTRL_CMD"

    # --- Window 2: 手動コマンド用 ---
    byobu-tmux new-window -t "$SESSION" -n 'Run1'
    byobu-tmux send-keys -t "$SESSION":2 "clear" C-m
    byobu-tmux send-keys -t "$SESSION":2 "cd $WORKSPACE" C-m
    byobu-tmux split-window -v -t "$SESSION":2

    # --- Window 3: センサ監視 ---
    byobu-tmux new-window -t "$SESSION" -n 'Sensors'
    byobu-tmux send-keys -t "$SESSION":3 "clear" C-m
    byobu-tmux send-keys -t "$SESSION":3 "cd $WORKSPACE" C-m
    byobu-tmux send-keys -t "$SESSION":3 "ros2 topic echo /driver/blue_rov/mavlink_driver/vehicle_state" C-m
    byobu-tmux split-window -h -t "$SESSION":3
    byobu-tmux send-keys -t "$SESSION":3.1 "ros2 topic hz /driver/dvl" C-m

    # --- Window 4: Node / Topic ---
    byobu-tmux new-window -t "$SESSION" -n 'Node-Topic'
    byobu-tmux send-keys -t "$SESSION":4 "clear" C-m
    byobu-tmux send-keys -t "$SESSION":4 "cd $WORKSPACE" C-m
    byobu-tmux send-keys -t "$SESSION":4 "sleep 2 && ros2 node list" C-m
    byobu-tmux split-window -h -t "$SESSION":4
    byobu-tmux send-keys -t "$SESSION":4.1 "sleep 2 && ros2 topic list" C-m

    byobu-tmux select-window -t "$SESSION":0
    byobu-tmux select-pane -t "$SESSION":0.0
fi

byobu-tmux attach -t "$SESSION"
