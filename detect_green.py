# Imported Python Transfer Function
import numpy as np
import sensor_msgs.msg
from cv_bridge import CvBridge

@nrp.MapRobotSubscriber("camera", Topic("/icub_model/left_eye_camera/image_raw", sensor_msgs.msg.Image))
@nrp.MapSpikeSource("sensors", nrp.map_neurons(range(0, nrp.config.brain_root.resolution ** 2), lambda i: nrp.brain.sensors[i]), nrp.dc_source)
@nrp.MapVariable("last_mean_greens", initial_value=[], scope=nrp.GLOBAL)
@nrp.Robot2Neuron()
def grab_image(t, camera, sensors, last_mean_greens):
    # Take the image from the robot's left eye
    image_msg = camera.value
    if image_msg is not None:
        # Read the image into an array, mean over 3 colors, resize it for the network and flatten the result
        img = CvBridge().imgmsg_to_cv2(image_msg, "rgb8")
        img_height, img_width, color_dim = img.shape
        sensor_rows = nrp.config.brain_root.resolution  # Sensors is a square matrix

        col_width = int(img_width / sensor_rows)
        row_height = int(img_height / sensor_rows)

        # Split the image into equidistant regions
        # Sensor neurons are addressed in row_major order
        for idx, neuron in enumerate(sensors):
            col = idx % sensor_rows
            row = idx // sensor_rows
            x_start = col * col_width
            x_end = (col + 1) * col_width
            y_start = row * row_height
            y_end = (row + 1) * row_height
            green_channel = img[y_start:y_end,x_start:x_end,1]
            mean_green = np.mean(green_channel)
            if len(last_mean_greens.value) < nrp.config.brain_root.resolution ** 2:
                last_mean_greens.value.append(mean_green)
            delta_mean_green = mean_green - last_mean_greens.value[idx]
            neuron.amplitude = 3. * max(0., delta_mean_green)
            last_mean_greens.value[idx] = mean_green
