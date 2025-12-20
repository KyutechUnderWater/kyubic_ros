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
    byobu-tmux send-keys -t $SESSION:0 "ros2 launch kyubic_bringup kyubic.launch.py" C-m
    byobu-tmux split-window -v -t $SESSION:0 # 画面を左右に分割 (-h)
    byobu-tmux send-keys -t $SESSION:0.1 "ros2 launch kyubic_bringup kyubic_post.launch.py"

    # --- Window 1: 実行・監視用 ---
    byobu-tmux new-window -t $SESSION -n 'Run1'
    byobu-tmux send-keys -t $SESSION:1 "clear" C-m
    byobu-tmux send-keys -t $SESSION:1 "cd $WORKSPACE" C-m
    byobu-tmux split-window -v -t $SESSION:1

    # --- Window 2: 実行・監視用 ---
    byobu-tmux new-window -t $SESSION -n 'Run2'
    byobu-tmux send-keys -t $SESSION:2 "clear" C-m
    byobu-tmux send-keys -t $SESSION:2 "cd $WORKSPACE" C-m

    # --- Window 3: Vim ---
    byobu-tmux new-window -t $SESSION -n 'Nvim'
    byobu-tmux send-keys -t $SESSION:3 "clear" C-m
    byobu-tmux send-keys -t $SESSION:3 "cd $WORKSPACE" C-m
    byobu-tmux send-keys -t $SESSION:3 "nvim ." C-m

    # --- Window 4: Node Topic ---
    byobu-tmux new-window -t $SESSION -n 'Node-Topic'
    byobu-tmux send-keys -t $SESSION:4 "clear" C-m
    byobu-tmux send-keys -t $SESSION:4 "cd $WORKSPACE" C-m
    byobu-tmux send-keys -t $SESSION:4 "sleep 1 && ros2 node list" C-m
    byobu-tmux split-window -h -t $SESSION:4
    byobu-tmux send-keys -t $SESSION:4.1 "sleep 1 && ros2 topic list" C-m

    # 最初のウィンドウを選択状態にする
    byobu-tmux select-window -t $SESSION:0
fi

# プロセスリストを確認し、monitor_ros_status.sh が動いていなければ起動
if ! pgrep -f "monitor_battery.sh $SESSION" >/dev/null; then
    $HOME/kyubic_ros/docker/script/monitor_battery.sh "$SESSION" &
fi

# セッションにアタッチ
byobu-tmux attach -t $SESSION
