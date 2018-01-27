# Imported Python Transfer Function
import numpy as np
import sensor_msgs.msg
from cv_bridge import CvBridge
import os
@nrp.MapRobotSubscriber("camera", Topic("/icub_model/left_eye_camera/image_raw", sensor_msgs.msg.Image))
@nrp.MapSpikeSource("sensors", nrp.map_neurons(range(0, nrp.config.brain_root.resolution ** 2), lambda i: nrp.brain.sensors[i]), nrp.dc_source)
@nrp.Robot2Neuron()
def grab_image(t, camera, sensors):
    resolution = nrp.config.brain_root.resolution
    # Take the image from the robot's left eye
    image_msg = camera.value
    if image_msg is not None:
        img = CvBridge().imgmsg_to_cv2(image_msg, "rgb8")
        img_height, img_width, color_dim = img.shape

        col_width = img_width // resolution
        row_height = img_height // resolution

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

                sensors[row_idx * resolution + col_idx].amplitude = amp
