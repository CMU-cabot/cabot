# -*- coding: utf-8 -*-
import launch
from launch_ros.actions import Node
import launch_testing
import unittest


def generate_test_description():

    basic_test = Node(
        executable=launch.substitutions.PathJoinSubstitution(
            [
                launch.substitutions.LaunchConfiguration("test_binary_dir"),
                "test_util",
            ]
        ),
        output="screen",
    )

    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="test_binary_dir",
                description="Binary directory of package "
                "containing test executables",
            ),
            basic_test,
            # TimerAction(period=2.0, actions=[basic_test]),
            launch_testing.actions.ReadyToTest(),
        ]
    ), {
        "basic_test": basic_test,
    }


class TestGTestWaitForCompletion(unittest.TestCase):
    # Waits for test to complete, then waits a bit to make sure result files are generated
    def test_gtest_run_complete(self, basic_test):
        self.proc_info.assertWaitForShutdown(basic_test, timeout=4000.0)


@launch_testing.post_shutdown_test()
class TestGTestProcessPostShutdown(unittest.TestCase):
    # Checks if the test has been completed with acceptable exit codes
    def test_gtest_pass(self, proc_info, basic_test):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=basic_test
        )
