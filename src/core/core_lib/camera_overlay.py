import cv2
import numpy as np

class HUD():
    def __init__(self, resolution = (1920, 1080)):
        
        # Appearance Settings
        self.font = cv2.FONT_HERSHEY_DUPLEX
        self.color = (255, 255, 255) # White
        self.background_color = (32, 32, 32) # Gray
        self.thickness = 1
        self.display_width = resolution[0]
        self.display_height = resolution[1]
        self.line_type = cv2.LINE_AA
        self.left_align = int(self.display_width / 40)
        self.vertical_increment = int(self.display_height / 20)

    # Overlays the current camera's index and nickname
    def add_camera_details(self, frame, index, nickname):
        text = "{}: {}".format(index, nickname)
        font_size = 0.8
        position = (self.left_align, self.vertical_increment)

        frame = self.add_text(frame, text, position, font_size)
        return frame

    def add_second_camera(self, big_frame, small_frame):
        big_dimensions = big_frame.shape 
        big_width = big_dimensions[1]

        small_dimensions = (480, 270) # 1920x1080 scaled down by a factor of 0.25
        small_frame = cv2.resize(small_frame, small_dimensions, interpolation=cv2.INTER_AREA)

        small_dimensions = small_frame.shape
        small_height = dimensions[0]
        small_width = dimensions[1]

        big_frame[0:small_height, big_width-small_width:big_width] = small_frame_resized

        return big_frame



    # Overlays the current thruster status onto the frame
    def add_thruster_status(self, frame, status_bool):
        if status_bool: status_text = "Enabled"
        else: status_text = "Disabled" 
        text = "Thrusters: {}".format(status_text)

        font_size = 0.6
        position = (self.left_align, 2 * self.vertical_increment)

        frame = self.add_text(frame, text, position, font_size)
        return frame


    # Accepts a dictionary of sensitivity values and overlays them onto the frame
    def add_gripper(self, frame, gripper):
        
        # Add the header
        font_size = 0.6
        position = (self.left_align, 3 * self.vertical_increment)
        text = "Gripper: {}".format(gripper)
        frame = self.add_text(frame, text, position, font_size)
        
        return frame


    # Accepts a dictionary of sensitivity values and overlays them onto the frame
    def add_sensitivity(self, frame, sensitivities):
        
        # Add the header
        font_size = 0.6
        position = (self.left_align, 4 * self.vertical_increment)
        frame = self.add_text(frame, "Sensitivity:", position, font_size)

        # Add the individual sensitivies:
        font_size = 0.5
        counter = 4.66
        for key in sensitivities:
            position = (self.left_align + 20, int(counter * self.vertical_increment))
            text = "{}: {}".format(key, sensitivities[key])
            frame = self.add_text(frame, text, position, font_size)
            counter += 0.66
        
        return frame
    

    # Accepts a dictionary of sensitivity values and overlays them onto the frame
    def add_publish_status(self, frame, publish_status):
        
        # Add the header
        font_size = 0.6
        position = (self.left_align, 4 * self.vertical_increment)
        text = "Recording Feed: {}".format(publish_status)
        frame = self.add_text(frame, text, position, font_size)
        
        return frame
        
    def add_orientation_info(self, frame, yaw, pitch, roll):

        # Endpoints for the roll/pitch line
        startX = int(self.display_width / 2 - 100 + abs(roll / 90 * 100))
        startY = int(self.display_height / 2 + roll + pitch)
        endX = int(self.display_width / 2 + 100 - abs(roll / 90 * 100))
        endY = int(self.display_height / 2 - roll + pitch)

        # Draw roll/pitch line
        frame = cv2.line(frame, (startX, startY), (endX, endY), (0, 255, 0), 5)

        return frame


    def add_photogrammetry_box(self, frame):
        y1, y2, x1, x2 = 48, 810, 0, 1380
        sub_img = frame[y1:y2, x1:x2]
        darkened_rect = np.ones(frame.shape, dtype=np.uint8) * 40

        frame = cv2.addWeighted(frame, 0.5, darkened_rect, 0.5, 1.0)
        frame[y1:y2, x1:x2] = sub_img

        return frame


    def leak_notification(self, frame):
        text = "LEAK DETECTED"
        font_size = 1
        
        # Find how large the text will be so we know how to center it
        textSize = cv2.getTextSize(text, self.font, font_size, self.thickness)[0]
        position = (int(self.display_width / 2 - textSize[0] / 2), int(self.display_height / 2))

        frame = self.add_text(frame, text, position, font_size)
        return frame
    

    def add_text(self, frame, text, position, font_size = 0.6):
        frame = cv2.putText(frame, text, (position[0] + 1, position[1] + 1), self.font, font_size, self.background_color, self.thickness, self.line_type)
        frame = cv2.putText(frame, text, position, self.font, font_size, self.color, self.thickness, self.line_type)
        return frame
