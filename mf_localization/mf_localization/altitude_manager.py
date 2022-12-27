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
