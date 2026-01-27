#!/bin/bash

SESSION_NAME="${1:-ros2_ws}"

# 設定
TOPIC="/driver/logic_distro_rp2040_driver/power_state"
FILE_BAT="/dev/shm/battery_status"

HAS_CONNECTED=0

while true; do
    CLIENT_COUNT=$(byobu-tmux list-clients -t "$SESSION_NAME" 2>/dev/null | wc -l)

    if [ "$CLIENT_COUNT" -gt 0 ]; then
        HAS_CONNECTED=1
    fi

    # 「一度接続があった」かつ「現在誰もいない」なら終了
    if [ "$HAS_CONNECTED" -eq 1 ] && [ "$CLIENT_COUNT" -eq 0 ]; then
        rm "$FILE_BAT"
        exit 0
    fi

    # 1. データの取得 (全体を取得して変数に格納)
    DATA=$(timeout 5 ros2 topic echo $TOPIC --once 2>/dev/null)

    # 2. 値の抽出 (grepとawkでパース)
    LOG_V=$(echo "$DATA" | grep "log_voltage:" | awk '{print $2}')
    ACT_V=$(echo "$DATA" | grep "act_voltage:" | awk '{print $2}')

    # データが空の場合はエラー表示
    if [ -z "$LOG_V" ] || [ -z "$ACT_V" ]; then
        echo -n "#[fg=colour240]BAT:NoTopic #[default]" >"$FILE_BAT"
    else
        # --- INF判定と表示用変数の作成 ---

        # LOG_V の処理
        if echo "$LOG_V" | grep -iq "inf"; then
            LOG_DISP="INF"
        else
            LOG_DISP=$(printf "%.2fV" "$LOG_V")
        fi

        # ACT_V の処理
        if echo "$ACT_V" | grep -iq "inf"; then
            ACT_DISP="INF"
        else
            ACT_DISP=$(printf "%.2fV" "$ACT_V")
        fi

        # --- 色判定ロジック ---
        if [ $(echo "$LOG_V <= 13.0 || ($LOG_V > 17.0 && $LOG_V < 19.0)" | bc) -eq 1 ]; then
            COLOR="#[fg=red]"
        else
            COLOR="#[fg=green]"
        fi

        # 4. 表示フォーマット
        printf "${COLOR}L:%s A:%s #[default]" "$LOG_DISP" "$ACT_DISP" >"$FILE_BAT"
    fi

    sleep 2
done
