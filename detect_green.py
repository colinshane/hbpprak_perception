# Imported Python Transfer Function
import numpy as np
import sensor_msgs.msg
import std_msgs.msg
from cv_bridge import CvBridge
@nrp.MapRobotSubscriber("camera", Topic("/icub_model/left_eye_camera/image_raw", sensor_msgs.msg.Image))
@nrp.MapRobotSubscriber("shuffle_status_sub", Topic("/group_3/shuffling", std_msgs.msg.Bool))
@nrp.MapSpikeSource("sensors", nrp.map_neurons(range(0, nrp.config.brain_root.resolution ** 2), lambda i: nrp.brain.sensors[i]), nrp.dc_source)
@nrp.MapRobotPublisher('debug_sensors_left', Topic('/group_3/debug_sensors_left', std_msgs.msg.Float64))
@nrp.MapRobotPublisher('debug_sensors_right', Topic('/group_3/debug_sensors_right', std_msgs.msg.Float64))
@nrp.MapRobotPublisher("shuffle_window_debug_pub", Topic("/group_3/windowed_image", sensor_msgs.msg.Image))
@nrp.MapRobotPublisher("retina_activity_debug_pub", Topic("/group_3/retina_activity", sensor_msgs.msg.Image))
@nrp.Robot2Neuron()
def grab_image(t, camera, shuffle_status_sub, sensors, debug_sensors_left, debug_sensors_right, shuffle_window_debug_pub, retina_activity_debug_pub):
    resolution = nrp.config.brain_root.resolution

    # Take the image from the robot's left eye
    image_msg = camera.value
    if image_msg is not None:
        cvBridge = CvBridge()

        img = cvBridge.imgmsg_to_cv2(image_msg, "rgb8")
        img_height, img_width, color_dim = img.shape

        detect_red = shuffle_status_sub.value.data if shuffle_status_sub.value is not None else False

        if not detect_red:
            # Detect green in the whole image
            col_width = img_width // resolution
            row_height = img_height // resolution
            green_threshold = 0.5
            amp_scaling_factor = 32.
            #retina_heatmap = np.zeros(shape=(img_height, img_width, 1), dtype=np.int8)
            
            # Split entire image into regions of same size
            # Sensor neurons are addressed in row_major order, top left to bottom right
            # Loop over neurons in retina...
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

                    sensors[idx].amplitude = amp
                    #retina_heatmap[y_start:y_end,x_start:x_end, 0] = int(amp)

            shuffle_window_debug_pub.send_message(cvBridge.cv2_to_imgmsg(img, encoding="rgb8"))
            #retina_activity_debug_pub.send_message(cvBridge.cv2_to_imgmsg(cv2.cvtColor(retina_heatmap, cv2.COLOR_GRAY2RGB), encoding="rgb8"))

        else:
            # Detect red in the center of the image
            window_factor = 5
            window_width = img_width // window_factor
            window_height = img_height // window_factor
            start_px_x = (window_factor // 2) * window_width
            start_px_y = (window_factor // 2) * window_height
            col_width = window_width // resolution
            row_height = window_height // resolution
            red_threshold = 0.6
            amp_scaling_factor = 1.3

            average_left = 0
            average_right = 0
            count_left = 0
            count_right = 0

            #retina_heatmap = np.zeros(shape=(window_height, window_width, 3), dtype=np.int8)

            # Split central area of image into regions of same size
            # Loop over neurons in retina...
            for row_idx in range(resolution):
                for col_idx in range(resolution):
                    x_start = start_px_x + col_idx * col_width
                    x_end = x_start + col_width
                    y_start = start_px_y + row_idx * row_height
                    y_end = y_start + row_height
                    mean_red = np.mean(img[y_start:y_end,x_start:x_end,0])
                    mean_green = np.mean(img[y_start:y_end,x_start:x_end,1])
                    mean_blue = np.mean(img[y_start:y_end,x_start:x_end,2])

                    red_proportion = mean_red / float(mean_red + mean_green + mean_blue)
                    
                    idx = row_idx * resolution + col_idx
                    amp = amp_scaling_factor * red_proportion if red_proportion > red_threshold else 0

                    sensors[idx].amplitude = amp

                    if col_idx < nrp.config.brain_root.resolution // 2:
                        count_left += 1
                        average_left += amp
                    elif col_idx > (nrp.config.brain_root.resolution // 2) + 1:
                        count_right += 1
                        average_right += amp

                    #retina_heatmap[y_start:y_end,x_start:x_end, :] = amp

            average_left /= float(count_left)
            average_right /= float(count_right)
            debug_sensors_left.send_message(std_msgs.msg.Float64(average_left))
            debug_sensors_right.send_message(std_msgs.msg.Float64(average_right))
            
            shuffle_window_debug_pub.send_message(cvBridge.cv2_to_imgmsg(img[start_px_y:start_px_y + window_height, start_px_x:start_px_x + window_width,:], encoding="rgb8"))
            #retina_activity_debug_pub.send_message(cvBridge.cv2_to_imgmsg(retina_heatmap, encoding="rgb8"))
