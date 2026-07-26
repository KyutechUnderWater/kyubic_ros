"""
ミッション全体の統合ループ。flow.mission_tick()(Decide)とcontrolfunction.control_tick()
(Act、flow内部から呼ばれる)を一定周期で回す。
"""
import time

import params
from function import initialize
from mavlink_io import ManualControl
import flow


def run_mission(ctx=None):
    """
    ミッションを実行し、最終Stateを返す。
    ctx を引数で受け取れるようにしているのはテスト用(外から差し込めるように)。
    通常の実行では ctx=None のまま呼べばよい。
    """
    initialize()

    if ctx is None:
        ctx = flow.MissionContext()
    state = flow.State.INIT

    mission_start = time.time()
    tick_period = 1.0 / params.MAIN_LOOP_HZ

    print(f"[main] ミッション開始: {state}")

    try:
        while state not in (flow.State.DONE, flow.State.EMERGENCY):
            tick_start = time.time()
            ctx.elapsed_time = tick_start - mission_start

            new_state = flow.mission_tick(ctx, state)
            if new_state != state:
                print(f"[main] {state} -> {new_state}  (t={ctx.elapsed_time:.1f}s)")
            state = new_state

            sleep_time = tick_period - (time.time() - tick_start)
            if sleep_time > 0:
                time.sleep(sleep_time)
    except KeyboardInterrupt:
        print("[main] KeyboardInterruptを検知。ミッションを中断します")
    finally:
        # 何が起きても(正常終了・緊急停止・例外・Ctrl+C)、
        # 最後は安全のためホバリング指令で締める。
        ManualControl(0, 0, 500, 0)

    print(f"[main] ミッション終了: {state}")
    return state


if __name__ == "__main__":
    run_mission()
