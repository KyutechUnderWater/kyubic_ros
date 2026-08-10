#!/bin/bash
# BlueROV ピンガー探査ミッション (pinger_search_control) 用 byobu セッション。
# 全ウィンドウ、コマンドは「入力」のみで自動実行はしない。
# オペレーターが内容を確認してから各ペインでEnterを押すこと。
#
# ウィンドウ構成(役割ごとに1タブ):
#   0 Core    … 常時起動系: driver_launcher / bluerov_dashboard / 手動heartbeat
#                (heartbeatはpinger_search_nodeにも内蔵。ここのは手動テスト・保険用で、
#                 二重送信になっても無害)
#   1 Mission … ミッション起動と開始操作: 左列=pinger_search_control(本番/0.7mプール 排他)、
#                右列=スタートトリガ / 緊急用の強制DISARM
#   2 Vision  … 画処理系: buoy_detector (enable_vision:true時のみ) + buoy_logger
#   3 Monitor … 監視系: ミッションstate / ピンガー / 画処理検出 / vehicle_state / robot_force
#   4 Run     … 手動コマンド用の空きペイン
#
# ハイドロフォン(/pinger_direction)は別コンピュータで起動するため、この機体側
# セッションには含めない。届いているかは Monitor のピンガーペインで確認する。
#
# ペイン指定は番号ではなくID(split-window -P で取得)で行う。tmuxはペイン番号を
# 位置順に振り直すため、番号指定だと分割の途中で別ペインへ送ってしまう。
#
# Kyubic 用 kyubic_byobu.sh / byobu エイリアスは変更しない。

SESSION="blue_ros2_ws"
WORKSPACE="$HOME/kyubic_ros/kyubic_ws"

DRIVER_CMD="ros2 launch driver_launcher blue_rov_driver.launch.py"
DASHBOARD_CMD="ros2 launch bluerov_dashboard bluerov_dashboard.launch.py"
HEARTBEAT_CMD="ros2 topic pub /driver/blue_rov/mavlink_driver/heartbeat std_msgs/msg/Bool 'data: true'"

MISSION_CMD="ros2 launch pinger_search_control pinger_search.launch.py"
MISSION_POOL_CMD="ros2 launch pinger_search_control pinger_search.launch.py config_file:=pinger_search_pool_0_7m.param.yaml"
START_CMD="ros2 topic pub --once /driver/blue_rov/mission_start_trigger driver_msgs/msg/BoolStamped '{data: true}'"
DISARM_CMD="ros2 service call /driver/blue_rov/mavlink_driver/set_armed std_srvs/srv/SetBool '{data: false}'"

PERCEPTION_CMD="ros2 launch buoy_detector buoy_detector.launch.py"
BUOY_LOGGER_CMD="ros2 launch buoy_detector buoy_logger.launch.py input_image_topic:=/driver/blue_rov/camera/image_raw"

STATE_ECHO_CMD="ros2 topic echo /control/automatic/pinger_search/pinger_search_node/state"
PINGER_ECHO_CMD="ros2 topic echo /pinger_direction"
BUOY_ECHO_CMD="ros2 topic echo /buoy/relative_position"
VEHICLE_ECHO_CMD="ros2 topic echo /driver/blue_rov/mavlink_driver/vehicle_state"
FORCE_ECHO_CMD="ros2 topic echo /driver/blue_rov/mavlink_driver/robot_force"

# $1=ペインID $2=説明(echoで表示) $3=コマンド(入力のみ、Enterは押さない)
prep_pane() {
    byobu-tmux send-keys -t "$1" "clear" C-m
    byobu-tmux send-keys -t "$1" "cd $WORKSPACE" C-m
    if [ -n "$2" ]; then
        byobu-tmux send-keys -t "$1" "echo '$2'" C-m
    fi
    if [ -n "$3" ]; then
        byobu-tmux send-keys -t "$1" "$3"
    fi
}

byobu-tmux has-session -t "$SESSION" 2>/dev/null

if [ $? != 0 ]; then
    # --- Window 0: 常時起動系 (ドライバー / ダッシュボード / heartbeat) ---
    CORE0=$(byobu-tmux new-session -d -s "$SESSION" -n 'Core' -P -F '#{pane_id}')
    prep_pane "$CORE0" '[必須] カメラ/MAVLink/リードスイッチ ドライバー一式 — Enterで起動' "$DRIVER_CMD"

    CORE1=$(byobu-tmux split-window -v -t "$CORE0" -P -F '#{pane_id}')
    prep_pane "$CORE1" '[任意] Web ダッシュボード — Enterで起動' "$DASHBOARD_CMD"

    CORE2=$(byobu-tmux split-window -v -t "$CORE1" -P -F '#{pane_id}')
    prep_pane "$CORE2" '[任意] heartbeat手動送信 — 手動テスト・保険用(二重送信でも無害)' "$HEARTBEAT_CMD"

    # --- Window 1: ミッション起動 + 開始操作 (左列=起動、右列=操作) ---
    MIS0=$(byobu-tmux new-window -t "$SESSION" -n 'Mission' -P -F '#{pane_id}')
    prep_pane "$MIS0" '[必須・排他A] ピンガー探査 本番(4m) — トリガ後30秒で自動ARMするので注意' "$MISSION_CMD"

    MIS1=$(byobu-tmux split-window -h -t "$MIS0" -P -F '#{pane_id}')
    prep_pane "$MIS1" '[手動] スタートトリガ (リードスイッチの代わり) — Enterでミッション開始(30秒後にARM)' "$START_CMD"

    MIS2=$(byobu-tmux split-window -v -t "$MIS0" -P -F '#{pane_id}')
    prep_pane "$MIS2" '[必須・排他B] ピンガー探査 0.7m練習プール — 排他Aと同時に起動しないこと' "$MISSION_POOL_CMD"

    MIS3=$(byobu-tmux split-window -v -t "$MIS1" -P -F '#{pane_id}')
    prep_pane "$MIS3" '[緊急] 強制DISARM — ミッションを止めた後は必ずこれでDISARM確認' "$DISARM_CMD"

    # --- Window 2: 画処理 + 画処理ロガー ---
    VIS0=$(byobu-tmux new-window -t "$SESSION" -n 'Vision' -P -F '#{pane_id}')
    prep_pane "$VIS0" '[任意] ブイ検出 (YOLO) — enable_vision:true のときだけ必要' "$PERCEPTION_CMD"

    VIS1=$(byobu-tmux split-window -v -t "$VIS0" -P -F '#{pane_id}')
    prep_pane "$VIS1" '[任意] ブイロガー — カメラ画像+検出結果を ~/buoy_logs へ保存' "$BUOY_LOGGER_CMD"

    # --- Window 3: 監視系をまとめて表示 (左列=state/armed、右列=pinger/buoy/force) ---
    MON0=$(byobu-tmux new-window -t "$SESSION" -n 'Monitor' -P -F '#{pane_id}')
    prep_pane "$MON0" '[監視] ミッションstate' "$STATE_ECHO_CMD"

    MON1=$(byobu-tmux split-window -h -t "$MON0" -P -F '#{pane_id}')
    prep_pane "$MON1" '[監視] ピンガー方位 (yaw/pitch/score) — 別PCから届いているかの確認も兼ねる' "$PINGER_ECHO_CMD"

    MON2=$(byobu-tmux split-window -v -t "$MON0" -P -F '#{pane_id}')
    prep_pane "$MON2" '[監視] vehicle_state (armed確認)' "$VEHICLE_ECHO_CMD"

    MON3=$(byobu-tmux split-window -v -t "$MON1" -P -F '#{pane_id}')
    prep_pane "$MON3" '[監視] 画処理検出 (/buoy/relative_position)' "$BUOY_ECHO_CMD"

    MON4=$(byobu-tmux split-window -v -t "$MON3" -P -F '#{pane_id}')
    prep_pane "$MON4" '[監視] robot_force (推力指令)' "$FORCE_ECHO_CMD"

    # --- Window 4: 手動コマンド用 ---
    RUN0=$(byobu-tmux new-window -t "$SESSION" -n 'Run' -P -F '#{pane_id}')
    prep_pane "$RUN0" '' ''
    byobu-tmux split-window -v -t "$RUN0"

    byobu-tmux select-window -t "$SESSION":0
    byobu-tmux select-pane -t "$CORE0"
fi

byobu-tmux attach -t "$SESSION"
