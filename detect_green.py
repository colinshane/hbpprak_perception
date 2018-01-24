# Imported Python Transfer Function
import numpy as np
import sensor_msgs.msg
from cv_bridge import CvBridge
import os
@nrp.MapRobotSubscriber("camera", Topic("/icub_model/left_eye_camera/image_raw", sensor_msgs.msg.Image))
@nrp.MapSpikeSource("upper_half", nrp.map_neurons(range(0, nrp.config.brain_root.resolution * (nrp.config.brain_root.resolution // 2)), lambda i: nrp.brain.upper_half[i]), nrp.dc_source)
@nrp.MapSpikeSource("lower_half", nrp.map_neurons(range(0, nrp.config.brain_root.resolution * (nrp.config.brain_root.resolution // 2)), lambda i: nrp.brain.lower_half[i]), nrp.dc_source)
@nrp.MapSpikeSource("left_half", nrp.map_neurons(range(0, nrp.config.brain_root.resolution * (nrp.config.brain_root.resolution // 2)), lambda i: nrp.brain.left_half[i]), nrp.dc_source)
@nrp.MapSpikeSource("right_half", nrp.map_neurons(range(0, nrp.config.brain_root.resolution * (nrp.config.brain_root.resolution // 2)), lambda i: nrp.brain.right_half[i]), nrp.dc_source)
@nrp.MapVariable("counter", initial_value=0, scope=nrp.GLOBAL)
@nrp.MapRobotPublisher('debug_sensors_left', Topic('/debug_sensors_left', std_msgs.msg.Float64))
@nrp.MapRobotPublisher('debug_sensors_right', Topic('/debug_sensors_right', std_msgs.msg.Float64))
@nrp.Robot2Neuron()
def grab_image(t, camera, upper_half, lower_half, left_half, right_half, counter, debug_sensors_left, debug_sensors_right):
    resolution = nrp.config.brain_root.resolution
    # Take the image from the robot's left eye
    image_msg = camera.value
    if image_msg is not None:
        img = CvBridge().imgmsg_to_cv2(image_msg, "rgb8")
        img_height, img_width, color_dim = img.shape

        col_width = img_width // resolution
        row_height = img_height // resolution

        average_right = 0
        average_left = 0
        count_right = 0
        count_left = 0

        green_threshold = 0.5
        amp_scaling_factor = 32.

        # Split the image into equidistant regions
        # Sensor neurons are addressed in row_major order, top left to bottom right
        for row_idx in range(resolution):
            for col_idx in range(resolution):
                x_start = col_idx * col_width
                x_end = x_start + col_width
                y_start = row_idx * row_height
                y_end = y_start + row_height
                mean_red = np.mean(img[y_start:y_end,x_start:x_end,0])
                mean_green = np.mean(img[y_start:y_end,x_start:x_end,1])
                mean_blue = np.mean(img[y_start:y_end,x_start:x_end,2])

                green_proportion = mean_green / float(mean_red + mean_green + mean_blue)

                idx = row_idx * resolution + col_idx


                amp = amp_scaling_factor * green_proportion if green_proportion > green_threshold else 0

                # Find relevant neurons
                if row_idx < resolution // 2:
                    upper_half[row_idx * resolution + col_idx].amplitude = amp
                elif row_idx > (resolution // 2) + 1:
                    population_row_idx = row_idx - (resolution // 2) - 1
                    lower_half[population_row_idx * resolution + col_idx].amplitude = amp
                if col_idx < resolution // 2:
                    left_half[row_idx * (resolution // 2) + col_idx].amplitude = amp
                    count_left += 1
                    average_left += amp
                elif col_idx > (resolution // 2) + 1:
                    population_col_idx = col_idx - (resolution // 2) - 1
                    right_half[row_idx * (resolution // 2) + population_col_idx].amplitude = amp
                    count_right += 1
                    average_right += amp

        counter.value = counter.value + 1
        average_left /= float(count_left)
        average_right /= float(count_right)
        debug_sensors_left.send_message(std_msgs.msg.Float64(average_left))
        debug_sensors_right.send_message(std_msgs.msg.Float64(average_right))
