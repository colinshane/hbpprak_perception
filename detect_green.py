# Imported Python Transfer Function
import numpy as np
import sensor_msgs.msg
from cv_bridge import CvBridge
import os
@nrp.MapRobotSubscriber("camera", Topic("/icub_model/left_eye_camera/image_raw", sensor_msgs.msg.Image))
@nrp.MapSpikeSource("sensors", nrp.map_neurons(range(0, nrp.config.brain_root.resolution ** 2), lambda i: nrp.brain.sensors[i]), nrp.dc_source)
@nrp.MapVariable("last_mean_greens", initial_value=[], scope=nrp.GLOBAL)
@nrp.MapVariable("counter", initial_value=0, scope=nrp.GLOBAL)
@nrp.Robot2Neuron()
def grab_image(t, camera, sensors, last_mean_greens, counter):
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
        for idx, neuron in enumerate(sensors):
            row = idx // nrp.config.brain_root.resolution
            col = idx % nrp.config.brain_root.resolution
            x_start = col * col_width
            x_end = x_start + col_width
            y_start = row * row_height
            y_end = y_start + row_height
            green_channel = img[y_start:y_end,x_start:x_end,1]
            mean_green = np.mean(green_channel)
            if len(last_mean_greens.value) < (nrp.config.brain_root.resolution ** 2):
                last_mean_greens.value.append(mean_green)
            delta_mean_green = mean_green - last_mean_greens.value[idx]
            delta_mean_greens[idx] = delta_mean_green
            #neuron.amplitude = 3. * abs(delta_mean_green)
            neuron.amplitude = 4 * max(0., delta_mean_green)
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
