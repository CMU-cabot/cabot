from dataclasses import dataclass
import math
import numpy
import rospy
from std_msgs.msg import Float64


class AltitudeManager():
    def __init__(self, verbose=False):
        self.queue = []
        self.verbose = verbose
        self.queue_limit = 60
        self.timestamp_interval_limit = 3.0
        self.window_size = 20
        self.threshold = 0.3 * 12

        self.initial_pressure = None
        self.pressure_std_pub = rospy.Publisher("pressure_std", Float64, latch=True, queue_size=10)

    def put_pressure(self, pressure):
        if not self.initial_pressure:
            self.initial_pressure = pressure

        if self.queue_limit <= len(self.queue):
            self.queue.pop(0)

        if len(self.queue) == 0:
            self.queue.append(pressure)
            return

        if (self.queue[-1].header.stamp - pressure.header.stamp).to_sec() > self.timestamp_interval_limit:
            if self.verbose:
                rospy.logerr("timestamp interval between two altimters is too large ({} sec)."+
                             "AltitudeManager was reset.".format(self.timestamp_interval_limit))
            self.queue.clear()

        self.queue.append(pressure)

    def is_height_changed(self):
        if len(self.queue) < self.window_size:
            return False

        relative = []
        for i in range(1, self.window_size+1):
            relative.append(self.queue[-i].fluid_pressure - self.initial_pressure.fluid_pressure)

        stdev = numpy.std(relative)

        msg = Float64()
        msg.data = stdev
        self.pressure_std_pub.publish(msg)

        if self.verbose:
            rospy.loginfo("Altimeter changed: {}".format(stdev))

        if self.threshold < stdev:
            return True

        return False


@dataclass
class AltitudeFloorEstimatorParameters:
    enable: bool = True
    height_per_floor: float = 4.0  # [m]
    floor_threshold_height_coeff: float = 0.5
    height_diff_threshold_coeff: float = 0.2
    current_window: int = 5
    history_window: int = 10
    p0: float = 101325.0  # [Pa]
    c0: float = 44300.0
    c1: float = 5.255


@dataclass
class AltitudeFloorEstimatorResult:
    ref_height: float
    floor_est: int
    height_est: float
    floor_vel_est: float
    height_vel_est: float
    current_state: int
    floor_change_event: int


class AltitudeFloorEstimator:
    def __init__(self, parameters: AltitudeFloorEstimatorParameters = AltitudeFloorEstimatorParameters()):
        self._enabled = parameters.enable
        # set constants
        self._p0 = parameters.p0
        self._c0 = parameters.c0
        self._c1 = parameters.c1
        # set parameters
        self._CW = parameters.current_window
        self._HW = parameters.history_window
        self.height_per_floor = parameters.height_per_floor
        self.floor_threshold_height = parameters.floor_threshold_height_coeff * self.height_per_floor
        self.height_diff_threshold = parameters.height_diff_threshold_coeff * self.height_per_floor

        # initialize variables
        self.reset()

    def reset(self, floor_est=0, height_est=0.0):
        self.time_queue = []
        self.height_queue = []

        self.height_est_memory = height_est

        self.floor_est_memory = floor_est
        self.last_floor_est = floor_est

        self.last_state = 0  # no motion

        return

    def put_pressure(self, pressure_msg) -> AltitudeFloorEstimatorResult:
        # convert pressure to reference height
        pressure = pressure_msg.fluid_pressure
        timestamp = pressure_msg.header.stamp.to_sec()
        ref_height = self._c0 * (1.0 - (pressure / self._p0)**(1.0 / self._c1))

        # temporal estimated values
        height_est = self.height_est_memory
        floor_est = self.floor_est_memory
        height_vel_est = 0.0
        floor_vel_est = 0.0
        current_state = self.last_state
        floor_change_event = None

        # prepare height windows
        self.time_queue.append(timestamp)
        self.height_queue.append(ref_height)
        if len(self.height_queue) > self._CW + self._HW:
            self.time_queue.pop(0)
            self.height_queue.pop(0)
        else:  # return if not ready
            return AltitudeFloorEstimatorResult(ref_height, floor_est, height_est, floor_vel_est, height_vel_est, current_state, floor_change_event)

        time_CW_window = self.time_queue[self._HW:]
        height_CW_window = self.height_queue[self._HW:]
        height_HW_window = self.height_queue[:-self._CW]

        height_CW = numpy.average(height_CW_window)
        height_HW = numpy.average(height_HW_window)

        diff_CW_HW = height_CW - height_HW
        if diff_CW_HW > self.height_diff_threshold:
            current_state = 1
        elif diff_CW_HW < -self.height_diff_threshold:
            current_state = -1
        else:
            current_state = 0

        # velocity estimation
        time_CW_window = numpy.array(time_CW_window)
        height_CW_window = numpy.array(height_CW_window)
        A = numpy.vstack([time_CW_window - time_CW_window[0], numpy.ones(len(time_CW_window))]).T  # subtract t[0] for stable computation
        height_vel_est, _ = numpy.linalg.lstsq(A, height_CW_window, rcond=None)[0]
        floor_vel_est = height_vel_est / self.height_per_floor

        # no motion
        if self.last_state == 0:
            if current_state != 0:
                self.last_stable_height = height_HW
        # up / down
        elif self.last_state == 1 or self.last_state == -1:
            height_est = self.height_est_memory + (ref_height - self.last_stable_height)

            # estimate floor change by thresholding
            floor_diff_float, floor_diff_int = math.modf((ref_height - self.last_stable_height) / self.height_per_floor)
            height_diff_float = floor_diff_float * self.height_per_floor
            floor_diff_final = floor_diff_int + numpy.sign(height_diff_float) * (self.floor_threshold_height < numpy.abs(height_diff_float))
            floor_est = self.floor_est_memory + floor_diff_final

            # emit floor up/down change event
            if self.last_floor_est != floor_est:
                floor_change_event = self.last_state

            # end of moving up/down
            if current_state == 0:
                self.height_est_memory = height_est
                self.floor_est_memory = floor_est

                # up/down -> stable event
                floor_change_event = 0

        self.last_state = current_state
        self.last_floor_est = floor_est

        return AltitudeFloorEstimatorResult(ref_height, floor_est, height_est, floor_vel_est, height_vel_est, current_state, floor_change_event)

    def is_height_changed(self):
        if self.last_state == 0:  # no motion
            return False
        else:
            return True

    def enabled(self):
        return self._enabled


def main():
    import argparse
    import rosbag
    import matplotlib.pyplot as plt

    parser = argparse.ArgumentParser("Sample program to run altitude floor estimator")
    parser.add_argument("-i", "--input_bag", required=True)
    parser.add_argument("-p", "--pressure_topic", default="/pressure")

    args = parser.parse_args()

    input_bag = args.input_bag
    pressure_topic = args.pressure_topic

    altitude_floor_estimator = AltitudeFloorEstimator()
    altitude_floor_estimator.reset(floor_est=0)

    X = []
    with rosbag.Bag(input_bag) as bag:
        for topic, msg, t in bag.read_messages(topics=[pressure_topic]):
            if pressure_topic == topic:

                fluid_pressure = msg.fluid_pressure

                result = altitude_floor_estimator.put_pressure(msg)
                X.append([t.to_sec(), fluid_pressure, result.ref_height, result.floor_est, result.height_est, result.floor_vel_est, result.height_vel_est, result.current_state])

    X = numpy.array(X)

    fig, ax1 = plt.subplots()
    ax2 = ax1.twinx()
    ax1.plot(X[:, 0], X[:, 2], marker="o", label="height", color="red")
    ax2.plot(X[:, 0], X[:, 3], marker="x", label="floor", color="blue")
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    plt.legend(lines1 + lines2, labels1 + labels2)
    plt.show()


if __name__ == "__main__":
    main()
