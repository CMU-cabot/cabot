########################################################################
#
# Copyright (c) 2017, STEREOLABS.
#
# All rights reserved.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
########################################################################

import pyzed.sl as sl
import numpy as np
import os
import sys
import time

def main():

    # Create a Camera object
    zed = sl.Camera()

    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    #init_params.camera_buffer_count_linux = 2
    init_params.camera_fps = 30
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # Use PERFORMANCE depth mode
    init_params.coordinate_units = sl.UNIT.MILLIMETER  # Use milliliter units (for depth measurements)

    print('Camera resolution', init_params.camera_resolution)
    print('Camera frame rate', init_params.camera_fps)
    print('Camera depth mode', init_params.depth_mode)
    log_name = os.environ['CABOT_LOG_NAME']
    print('Log name', log_name)

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print('Cannot open the camera', err)
        exit(1)

    # Create and set RuntimeParameters after opening the camera
    runtime_parameters = sl.RuntimeParameters()
    runtime_parameters.sensing_mode = sl.SENSING_MODE.STANDARD  # Use STANDARD sensing mode

    # Capture 50 images and depth, then stop
    i = 0
    image_l = sl.Mat()
    image_r = sl.Mat()
    depth = sl.Mat()
    point_cloud = sl.Mat()
    max_record_frames = 1000

    while not(zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS):
        continue
    zed.retrieve_image(image_l, sl.VIEW.LEFT)
    frame_width = int(image_l.get_width())
    frame_height = int(image_l.get_height())

    path_output = "recordings/" + log_name + "_data3d.svo"
    record_parameters = sl.RecordingParameters(path_output, sl.SVO_COMPRESSION_MODE.LOSSLESS)
    err = zed.enable_recording(record_parameters)
    if err != sl.ERROR_CODE.SUCCESS:
        print(repr(err))
        exit(1)

    path_time = "recordings/" + log_name + "time.txt"
    file_time = open(path_time, "w")

    # Code that wait for signal from server

    while i < max_record_frames:
        # A new image is available if grab() returns SUCCESS
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:

            time_raw = zed.get_timestamp(sl.TIME_REFERENCE.IMAGE)
            time_raw = time_raw.data_ns // 1000
            print("Time when image is taken: {0}\n".format(time_raw))
            time_str = "{0}, {1}\n".format(i, time_raw)
            file_time.write(time_str)
            i += 1

    # Close the camera
    zed.disable_recording()
    zed.close()
    file_time.close()

if __name__ == "__main__":
    main()
    
