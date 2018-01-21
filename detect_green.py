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
@nrp.MapVariable("last_mean_greens", initial_value=[], scope=nrp.GLOBAL)
@nrp.MapVariable("counter", initial_value=0, scope=nrp.GLOBAL)
@nrp.Robot2Neuron()
def grab_image(t, camera, upper_half, lower_half, left_half, right_half, last_mean_greens, counter):
    # Take the image from the robot's left eye
    image_msg = camera.value
    if image_msg is not None:
        # Read the image into an array, mean over 3 colors, resize it for the network and flatten the result
        img = CvBridge().imgmsg_to_cv2(image_msg, "rgb8")
        img_height, img_width, color_dim = img.shape

        col_width = img_width // nrp.config.brain_root.resolution
        row_height = img_height // nrp.config.brain_root.resolution
        delta_mean_greens = [0 for i in range(nrp.config.brain_root.resolution ** 2)]

        # Split the image into equidistant regions
        # Sensor neurons are addressed in row_major order, top left to bottom right
        for row_idx in range(nrp.config.brain_root.resolution):
            for col_idx in range(nrp.config.brain_root.resolution):
                x_start = col_idx * col_width
                x_end = x_start + col_width
                y_start = row_idx * row_height
                y_end = y_start + row_height
                green_channel = img[y_start:y_end,x_start:x_end,1]
                mean_green = np.mean(green_channel)
                if len(last_mean_greens.value) < (nrp.config.brain_root.resolution ** 2):
                    last_mean_greens.value.append(mean_green)

                idx = row_idx * nrp.config.brain_root.resolution + col_idx
                delta_mean_green = mean_green - last_mean_greens.value[idx]
                delta_mean_greens[idx] = delta_mean_green

                amp = 4. * max(0., delta_mean_green)

                # Find relevant neurons
                if row_idx < nrp.config.brain_root.resolution // 2:
                    upper_half[row_idx * nrp.config.brain_root.resolution + col_idx].amplitude = 4. * max(0., delta_mean_green)
                elif row_idx > (nrp.config.brain_root.resolution // 2) + 1:
                    population_row_idx = row_idx - (nrp.config.brain_root.resolution // 2) - 1
                    lower_half[population_row_idx * nrp.config.brain_root.resolution + col_idx].amplitude = 4. * max(0., delta_mean_green)
                if col_idx < nrp.config.brain_root.resolution // 2:
                    left_half[row_idx * (nrp.config.brain_root.resolution // 2) + col_idx].amplitude = 4. * max(0., delta_mean_green)
                elif col_idx > (nrp.config.brain_root.resolution // 2) + 1:
                    population_col_idx = col_idx - (nrp.config.brain_root.resolution // 2) - 1
                    right_half[population_col_idx * (nrp.config.brain_root.resolution // 2) + col_idx].amplitude = 4. * max(0., delta_mean_green)

                last_mean_greens.value[idx] = mean_green

            """
            if len(np.nonzero(last_mean_greens.value)) > 0:
                debug_dir = "/home/bal/.opt/nrpStorage/Experiment_0/debug"
                mat_filepath = debug_dir + "/debug_{}.csv".format(counter.value)
                img_filepath = debug_dir + "/debug_{}.png".format(counter.value)
                cv2.imwrite(img_filepath, img)
                delta_mean_greens_reshaped = np.reshape(delta_mean_greens, (nrp.config.brain_root.resolution, nrp.config.brain_root.resolution))
                np.savetxt(mat_filepath, delta_mean_greens_reshaped, fmt="%10.5f")
            """
        counter.value = counter.value + 1
