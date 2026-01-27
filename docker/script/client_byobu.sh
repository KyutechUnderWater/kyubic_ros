#!/bin/bash

SESSION="ros2_ws"                       # セッション名
WORKSPACE="$HOME/kyubic_ros/kyubic_ws/" # ワークスペースのパス

# 既存セッションの確認（多重起動防止）
byobu-tmux has-session -t $SESSION 2>/dev/null

if [ $? != 0 ]; then
    # 1. Create Session（background: -d）
    byobu-tmux new-session -d -s $SESSION -n 'Bringup'
    byobu-tmux send-keys -t $SESSION:0 "clear" C-m

    # --- Window 0: bringup ---
    byobu-tmux send-keys -t $SESSION:0 "cd $WORKSPACE" C-m
    byobu-tmux send-keys -t $SESSION:0 "ros2 launch kyubic_bringup web_visualizer.launch.py" C-m
    byobu-tmux split-window -v -t $SESSION:0
    byobu-tmux split-window -h -t $SESSION:0.1
    byobu-tmux split-window -v -t $SESSION:0.1 -l '100%'
    byobu-tmux send-keys -t $SESSION:0.1 "ros2 launch kyubic_bringup manual.launch.py"
    byobu-tmux send-keys -t $SESSION:0.2 "ros2 launch joy_common joy_common.launch.py"
    byobu-tmux send-keys -t $SESSION:0.3 "ros2 topic pub /driver/actuator_rp2040_driver/heartbeat std_msgs/msg/Bool 'data: true'"

    # --- Window 1: 実行・監視用 ---
    byobu-tmux new-window -t $SESSION -n 'Run1'
    byobu-tmux send-keys -t $SESSION:1 "clear" C-m
    byobu-tmux send-keys -t $SESSION:1 "cd $WORKSPACE" C-m
    byobu-tmux split-window -v -t $SESSION:1

    # --- Window 3: Vim ---
    byobu-tmux new-window -t $SESSION -n 'Nvim'
    byobu-tmux send-keys -t $SESSION:2 "clear" C-m
    byobu-tmux send-keys -t $SESSION:2 "cd $WORKSPACE" C-m
    byobu-tmux send-keys -t $SESSION:2 "nvim ." C-m

    # --- Window 4: Node Topic ---
    byobu-tmux new-window -t $SESSION -n 'Node-Topic'
    byobu-tmux send-keys -t $SESSION:3 "clear" C-m
    byobu-tmux send-keys -t $SESSION:3 "cd $WORKSPACE" C-m
    byobu-tmux send-keys -t $SESSION:3 "ros2 node list" C-m
    byobu-tmux split-window -h -t $SESSION:3
    byobu-tmux send-keys -t $SESSION:3.1 "ros2 topic list" C-m

    # 最初のウィンドウを選択状態にする
    byobu-tmux select-window -t $SESSION:0
    byobu-tmux select-pan -t $SESSION:0.1
fi

# プロセスリストを確認し、monitor_ros_status.sh が動いていなければ起動
if ! pgrep -f "monitor_battery.sh $SESSION" >/dev/null; then
    $HOME/kyubic_ros/docker/script/monitor_battery.sh "$SESSION" &
fi

# セッションにアタッチ
byobu-tmux attach -t $SESSION
