"""Unit tests for observable PID state and teaching Services."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from educational_pid.pid_node import EducationalPidNode


def test_student_interfaces_include_state_topics_and_target_service():
    controller = SimpleNamespace(
        create_publisher=Mock(side_effect=lambda *args: args),
        create_subscription=Mock(side_effect=lambda *args: args),
        create_service=Mock(side_effect=lambda *args: args),
        joy_callback=Mock(),
        depth_callback=Mock(),
        imu_callback=Mock(),
        enable_callback=Mock(),
        reset_callback=Mock(),
        set_targets_callback=Mock(),
    )

    EducationalPidNode.create_interfaces(controller)

    publisher_names = {call.args[1] for call in controller.create_publisher.call_args_list}
    assert publisher_names == {
        "robot_force",
        "targets",
        "integral_depth",
        "integral_yaw",
        "error_depth",
        "error_yaw",
    }
    service_names = {call.args[1] for call in controller.create_service.call_args_list}
    assert service_names == {
        "/educational_pid/enable",
        "/educational_pid/reset_pid",
        "/educational_pid/set_targets",
    }


def test_errors_are_updated_while_output_can_be_disabled():
    parameters = {
        "target_depth": SimpleNamespace(value=2.5),
        "target_yaw": SimpleNamespace(value=-179.0),
    }
    controller = SimpleNamespace(
        current_depth=1.0,
        current_yaw=179.0,
        get_parameter=lambda name: parameters[name],
    )

    EducationalPidNode._update_control_errors(controller)

    assert controller.depth_error == pytest.approx(1.5)
    assert controller.yaw_error == pytest.approx(2.0)


def test_reset_clears_integral_and_derivative_history():
    controller = SimpleNamespace(
        prev_error_depth=4.0,
        integral_depth=3.0,
        depth_pid_initialized=True,
        prev_error_yaw=-5.0,
        integral_yaw=6.0,
        yaw_pid_initialized=True,
        last_control_time=10.0,
    )

    EducationalPidNode._reset_pid_state(controller)

    assert controller.prev_error_depth == 0.0
    assert controller.integral_depth == 0.0
    assert controller.depth_pid_initialized is False
    assert controller.prev_error_yaw == 0.0
    assert controller.integral_yaw == 0.0
    assert controller.yaw_pid_initialized is False
    assert controller.last_control_time is None


def test_pid_state_topics_publish_errors_and_integrals():
    publishers = [Mock(), Mock(), Mock(), Mock()]
    controller = SimpleNamespace(
        depth_error=1.5,
        yaw_error=-2.0,
        integral_depth=0.25,
        integral_yaw=-0.5,
        pub_error_depth=publishers[0],
        pub_error_yaw=publishers[1],
        pub_integral_depth=publishers[2],
        pub_integral_yaw=publishers[3],
    )

    EducationalPidNode.publish_pid_state(controller)

    published_values = [publisher.publish.call_args.args[0].data for publisher in publishers]
    assert published_values == pytest.approx([1.5, -2.0, 0.25, -0.5])


def test_set_targets_updates_both_parameters_and_resets_pid():
    controller = SimpleNamespace(
        set_parameters=Mock(
            return_value=[
                SimpleNamespace(successful=True),
                SimpleNamespace(successful=True),
            ]
        ),
        _reset_pid_state=Mock(),
        _update_control_errors=Mock(),
        publish_targets=Mock(),
        publish_pid_state=Mock(),
    )
    request = SimpleNamespace(target_depth=1.25, target_yaw=181.0)
    response = SimpleNamespace(success=False, message="")

    result = EducationalPidNode.set_targets_callback(
        controller,
        request,
        response,
    )

    assert result.success is True
    parameters = controller.set_parameters.call_args.args[0]
    assert parameters[0].name == "target_depth"
    assert parameters[0].value == pytest.approx(1.25)
    assert parameters[1].name == "target_yaw"
    assert parameters[1].value == pytest.approx(-179.0)
    controller._reset_pid_state.assert_called_once_with()
    controller.publish_pid_state.assert_called_once_with()


@pytest.mark.parametrize(
    "depth,yaw",
    [(-0.1, 0.0), (float("nan"), 0.0), (1.0, float("inf"))],
)
def test_set_targets_rejects_invalid_values(depth, yaw):
    controller = SimpleNamespace(set_parameters=Mock())
    request = SimpleNamespace(target_depth=depth, target_yaw=yaw)
    response = SimpleNamespace(success=True, message="")

    result = EducationalPidNode.set_targets_callback(
        controller,
        request,
        response,
    )

    assert result.success is False
    controller.set_parameters.assert_not_called()
