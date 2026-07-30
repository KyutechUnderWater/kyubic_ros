#!/bin/bash
# BlueROV 用 byobu セッション。
# 全ウィンドウ、コマンドは「入力」のみで自動実行はしない。
# オペレーターが内容を確認してから各ペインでEnterを押すこと。
#
# ウィンドウ構成:
#   0 Drivers       … driver_launcher (mavlink_driver + dvl75_driver + camera_driver)
#   1 Control       … bluerov_control か bluerov_control_bt_nodes のどちらか一方
#                     (heartbeat・emergency_surfacingは両方に内蔵済み。二重起動しないこと)
#   2 Nav           … localization, buoy_detector (perception)
#   3 Dashboard     … bluerov_dashboard (Web UI)
#   4 Monitor       … vehicle_state / dvl / node / topic 監視、system_health_check(要調整)
#   5 Run           … 手動コマンド用の空きペイン
#
# Kyubic 用 kyubic_byobu.sh / byobu エイリアスは変更しない。

SESSION="blue_ros2_ws"
WORKSPACE="$HOME/kyubic_ros/kyubic_ws"

DRIVER_CMD="ros2 launch driver_launcher blue_rov_driver.launch.py"

CTRL_CMD="ros2 launch bluerov_control bluerov_control.launch.py"
CTRL_BT_CMD="ros2 launch bluerov_control_bt_nodes bluerov_bt.launch.py"

LOC_CMD="ros2 launch localization localization_components.launch.py"
PERCEPTION_CMD="ros2 launch buoy_detector buoy_detector.launch.py"

DASHBOARD_CMD="ros2 launch bluerov_dashboard bluerov_dashboard.launch.py"

HEALTH_CMD="ros2 launch system_health_check system_health_check.launch.py"

byobu-tmux has-session -t "$SESSION" 2>/dev/null

if [ $? != 0 ]; then
    # --- Window 0: blue_rov ドライバー一式 ---
    byobu-tmux new-session -d -s "$SESSION" -n 'Drivers'
    byobu-tmux send-keys -t "$SESSION":0 "clear" C-m
    byobu-tmux send-keys -t "$SESSION":0 "cd $WORKSPACE" C-m
    byobu-tmux send-keys -t "$SESSION":0 "echo '[必須] カメラ/MAVLink/DVL ドライバー一式 — Enterで起動'" C-m
    byobu-tmux send-keys -t "$SESSION":0 "$DRIVER_CMD"

    # --- Window 1: 制御 (排他選択) ---
    byobu-tmux new-window -t "$SESSION" -n 'Control'
    byobu-tmux send-keys -t "$SESSION":1 "clear" C-m
    byobu-tmux send-keys -t "$SESSION":1 "cd $WORKSPACE" C-m
    byobu-tmux send-keys -t "$SESSION":1 "echo '[必須・排他A] 通常ミッション制御 (heartbeat/emergency内蔵) — Enterで起動'" C-m
    byobu-tmux send-keys -t "$SESSION":1 "$CTRL_CMD"

    byobu-tmux split-window -v -t "$SESSION":1
    byobu-tmux send-keys -t "$SESSION":1.1 "cd $WORKSPACE" C-m
    byobu-tmux send-keys -t "$SESSION":1.1 "echo '[必須・排他B] ビヘイビアツリー制御 (heartbeat/emergency内蔵) — 上と同時に起動しないこと'" C-m
    byobu-tmux send-keys -t "$SESSION":1.1 "$CTRL_BT_CMD"

    # --- Window 2: 自己位置推定・検知 ---
    byobu-tmux new-window -t "$SESSION" -n 'Nav'
    byobu-tmux send-keys -t "$SESSION":2 "clear" C-m
    byobu-tmux send-keys -t "$SESSION":2 "cd $WORKSPACE" C-m
    byobu-tmux send-keys -t "$SESSION":2 "echo '[必須] 自己位置推定 (localization) — Enterで起動'" C-m
    byobu-tmux send-keys -t "$SESSION":2 "$LOC_CMD"

    byobu-tmux split-window -v -t "$SESSION":2
    byobu-tmux send-keys -t "$SESSION":2.1 "cd $WORKSPACE" C-m
    byobu-tmux send-keys -t "$SESSION":2.1 "echo '[任意] ブイ検出 (YOLO) — Enterで起動'" C-m
    byobu-tmux send-keys -t "$SESSION":2.1 "$PERCEPTION_CMD"

    # --- Window 3: ダッシュボード ---
    byobu-tmux new-window -t "$SESSION" -n 'Dashboard'
    byobu-tmux send-keys -t "$SESSION":3 "clear" C-m
    byobu-tmux send-keys -t "$SESSION":3 "cd $WORKSPACE" C-m
    byobu-tmux send-keys -t "$SESSION":3 "echo '[任意] Web ダッシュボード (:8090) — Enterで起動'" C-m
    byobu-tmux send-keys -t "$SESSION":3 "$DASHBOARD_CMD"

    # --- Window 4: 監視 ---
    byobu-tmux new-window -t "$SESSION" -n 'Monitor'
    byobu-tmux send-keys -t "$SESSION":4 "clear" C-m
    byobu-tmux send-keys -t "$SESSION":4 "cd $WORKSPACE" C-m
    byobu-tmux send-keys -t "$SESSION":4 "echo '[監視] vehicle_state'" C-m
    byobu-tmux send-keys -t "$SESSION":4 "ros2 topic echo /driver/blue_rov/mavlink_driver/vehicle_state"

    byobu-tmux split-window -h -t "$SESSION":4
    byobu-tmux send-keys -t "$SESSION":4.1 "cd $WORKSPACE" C-m
    byobu-tmux send-keys -t "$SESSION":4.1 "echo '[監視] DVLレート'" C-m
    byobu-tmux send-keys -t "$SESSION":4.1 "ros2 topic hz /driver/dvl"

    byobu-tmux split-window -v -t "$SESSION":4.0
    byobu-tmux send-keys -t "$SESSION":4.2 "cd $WORKSPACE" C-m
    byobu-tmux send-keys -t "$SESSION":4.2 "echo '[任意・要BlueROV向け設定確認] system_health_check'" C-m
    byobu-tmux send-keys -t "$SESSION":4.2 "$HEALTH_CMD"

    byobu-tmux split-window -v -t "$SESSION":4.1
    byobu-tmux send-keys -t "$SESSION":4.3 "cd $WORKSPACE" C-m
    byobu-tmux send-keys -t "$SESSION":4.3 "echo '[確認用] node/topic 一覧'" C-m
    byobu-tmux send-keys -t "$SESSION":4.3 "ros2 node list; ros2 topic list"

    # --- Window 5: 手動コマンド用 ---
    byobu-tmux new-window -t "$SESSION" -n 'Run'
    byobu-tmux send-keys -t "$SESSION":5 "clear" C-m
    byobu-tmux send-keys -t "$SESSION":5 "cd $WORKSPACE" C-m
    byobu-tmux split-window -v -t "$SESSION":5

    byobu-tmux select-window -t "$SESSION":0
    byobu-tmux select-pane -t "$SESSION":0.0
fi

byobu-tmux attach -t "$SESSION"
