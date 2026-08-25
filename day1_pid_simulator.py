import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


# ==========================================================
# ステップ0: 超シンプル制御 (オン・オフ制御 / バンバン制御)
# 目標より手前にいれば前進、奥にいれば後退するだけの単純な制御です。
# ==========================================================
def calculate_simple_control(target, current, force_power):
    if current < target:
        return force_power  # 手前なら一定の力で進む
    elif current > target:
        return -force_power  # 行き過ぎたら一定の力で戻る
    else:
        return 0.0


# ==========================================================
# ステップ1: P制御 (比例制御)
# 現在の誤差に比例した力を出力します。
# ==========================================================
def calculate_p_control(target, current, kp):
    error = target - current
    p_term = kp * error

    return p_term


# ==========================================================
# ステップ2: PD制御 (比例＋微分制御)
# P制御に加え、誤差の変化量（微分）からブレーキをかけます。
# 行き過ぎ（オーバーシュート）を防ぐ効果があります。
# ==========================================================
def calculate_pd_control(target, current, kp, kd, prev_error, dt):
    error = target - current
    p_term = kp * error

    # 誤差の変化量 (微分) を計算
    derivative = (error - prev_error) / dt if dt > 0 else 0.0
    d_term = kd * derivative

    return p_term + d_term


# ==========================================================
# ステップ3: PID制御 (比例＋積分＋微分制御)
# PD制御に加え、過去の誤差の蓄積（積分）を使って、
# 目標に届ききらないズレ（定常偏差）を無くします。
# ==========================================================
def calculate_pid_control(target, current, kp, ki, kd, prev_error, integral, dt):
    error = target - current
    p_term = kp * error

    # 誤差の蓄積 (積分) を計算
    integral += error * dt
    i_term = ki * integral

    # 誤差の変化量 (微分) を計算
    derivative = (error - prev_error) / dt if dt > 0 else 0.0
    d_term = kd * derivative

    output = p_term + i_term + d_term
    return output, integral


def run_simulation():
    TARGET = 10.0  # 目標の深さ (m)

    # 'SIMPLE', 'P', 'PD', 'PID' のいずれかに書き換える
    MODE = "SIMPLE"

    SIMPLE_FORCE = 15.0  # 超シンプル制御用の一定の力
    KP = 2.0  # 比例ゲイン: 目的地に向かう力
    KI = 0.5  # 積分ゲイン: 足りない分をジワジワ押し込む力
    KD = 2.0  # 微分ゲイン: ブレーキをかける力

    DISTURBANCE = -5.0  # 外乱

    print(f"--- 1D PID Simulation ---")
    print(f"Mode: {MODE} Control")

    mass = 5.0
    drag = 2.0  # 水の抵抗

    dt = 0.05  # 1ステップの時間 (秒)
    time_max = 20.0

    current_pos = 0.0  # ゲイン設定 (数値を色々変えてグラフの変化を見てみよう)
    current_vel = 0.0
    prev_error = TARGET - current_pos
    integral = 0.0

    # グラフ描画用リスト
    times = []
    positions = []
    targets = []
    forces = []

    t = 0.0
    while t <= time_max:
        # 1. 選択されたモードで操作量(推力)を計算
        mode_upper = MODE.upper()
        if mode_upper == "SIMPLE":
            force = calculate_simple_control(TARGET, current_pos, SIMPLE_FORCE)
        elif mode_upper == "P":
            force = calculate_p_control(TARGET, current_pos, KP)
        elif mode_upper == "PD":
            force = calculate_pd_control(TARGET, current_pos, KP, KD, prev_error, dt)
        elif mode_upper == "PID":
            force, integral = calculate_pid_control(
                TARGET, current_pos, KP, KI, KD, prev_error, integral, dt
            )
        else:
            force = 0.0

        # 2. 物理演算: 推力と水の抵抗、外乱から加速度を計算 (F = ma)
        net_force = force - (drag * current_vel) + DISTURBANCE
        acceleration = net_force / mass

        # 3. 状態の更新 (速度と位置の計算)
        # 次のステップの微分計算のために、現在の誤差を「前回の誤差」として保存
        prev_error = TARGET - current_pos

        current_vel += acceleration * dt
        current_pos += current_vel * dt

        # 4. データを記録
        times.append(t)
        positions.append(current_pos)
        targets.append(TARGET)
        forces.append(force)
        t += dt

    # ---------------------------------------------------------
    # アニメーションによるグラフの描画設定 (位置と入力値の2画面)
    # ---------------------------------------------------------
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), gridspec_kw={"height_ratios": [2, 1]})

    # 上のグラフ：位置（Depth）
    ax1.set_xlim(0, time_max)
    # 動的に位置の最大値に合わせて縦幅を調整
    max_pos = max(max(positions), TARGET * 1.2)
    ax1.set_ylim(0, max_pos * 1.1)
    ax1.set_title(f"1D Robot Simulation - {MODE} Control")
    ax1.set_ylabel("Depth [meters]")
    ax1.grid(True, linestyle=":", alpha=0.7)

    (line_pos,) = ax1.plot([], [], label="Robot Depth (Current)", linewidth=2.5, color="blue")
    (line_target,) = ax1.plot([], [], "r--", label="Target Depth", linewidth=2)
    ax1.legend()

    # 下のグラフ：入力値（Force）
    ax2.set_xlim(0, time_max)
    # グラフの縦軸幅を入力値の最大値に合わせて調整
    max_force = max(max(forces), 20.0)
    ax2.set_ylim(-max_force * 1.2, max_force * 1.2)
    ax2.set_xlabel("Time [seconds]")
    ax2.set_ylabel("Input Force [N]")
    ax2.grid(True, linestyle=":", alpha=0.7)

    (line_force,) = ax2.plot([], [], label="Control Input (Force)", linewidth=2.0, color="orange")
    ax2.legend()

    def init():
        line_pos.set_data([], [])
        line_target.set_data([], [])
        line_force.set_data([], [])
        return line_pos, line_target, line_force

    def update(frame):
        line_pos.set_data(times[:frame], positions[:frame])
        line_target.set_data(times[:frame], targets[:frame])
        line_force.set_data(times[:frame], forces[:frame])
        return line_pos, line_target, line_force

    ani = FuncAnimation(
        fig,
        update,
        frames=len(times),
        init_func=init,
        blit=True,
        interval=int(dt * 1000),
        repeat=False,
    )

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    run_simulation()
