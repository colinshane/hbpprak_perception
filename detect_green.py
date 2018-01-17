# Imported Python Transfer Function
import numpy as np
import sensor_msgs.msg
import math
from cv_bridge import CvBridge
from hbp_nrp_excontrol.logs import clientLogger
import time

num_sensors = nrp.config.brain_root.n_sensors
sensor_rows = int(math.sqrt(num_sensors))

@nrp.MapRobotSubscriber("camera", Topic("/icub_model/left_eye_camera/image_raw", sensor_msgs.msg.Image))
@nrp.MapSpikeSource("sensors", nrp.map_neurons(range(0, num_sensors), lambda i: nrp.brain.sensors[i]), nrp.dc_source)
@nrp.MapVariable("last_mean_green", initial_value=None, scope=nrp.GLOBAL)
@nrp.Robot2Neuron()
def grab_image(t, camera, sensors, last_mean_green):
    # Take the image from the robot's left eye
    image_msg = camera.value
    if image_msg is not None:
        # Read the image into an array, mean over 3 colors, resize it for the network and flatten the result
        img = CvBridge().imgmsg_to_cv2(image_msg, "rgb8")
        height, width, color_dim = img.shape
        # Split the image into equidistant regions
        # Sensor neurons are addressed in row_major order
        for idx, neuron in enumerate(sensors):
            col = idx % sensor_rows
            row = idx // sensor_rows
            clientLogger.info("Row: {}, col: {}".format(row, col))
            time.sleep(1)    
            continue
            green_channel = img[:,:,1]
            mean_green = np.mean(green_channel)
            if last_mean_green.value is None:
                last_mean_green.value = mean_green
            delta_mean_green = mean_green - last_mean_green.value
            neuron.amplitude = 3. * max(0., delta_mean_green)
        last_mean_green.value = mean_green
