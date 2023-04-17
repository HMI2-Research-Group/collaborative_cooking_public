"""
All the angle and coordinate axes are WRTO to the base_link of the ridgeback robot, as it can 
be visualized easily in RViz
"""
from pyrealsense2 import pyrealsense2
import numpy as np
from numpy import unravel_index
from math import atan2, pi
from pyzbar import pyzbar
import cv2
from dataclasses import dataclass
import threading
import rospy


@dataclass
class QR:
    pixel_x: int
    pixel_y: int
    x: float
    y: float
    z: float
    angle: float
    qr_edge_coordinates: list


class RealsenseQR:
    def __init__(self, device_id: str) -> None:
        self.device_id = device_id
        self.pipeline = pyrealsense2.pipeline()
        # Create a config and configure the pipeline to stream
        # different resolutions of color and depth streams
        config = pyrealsense2.config()
        config.enable_device(device_id)
        config.enable_stream(pyrealsense2.stream.depth, 640, 480, pyrealsense2.format.z16, 30)
        config.enable_stream(pyrealsense2.stream.color, 640, 480, pyrealsense2.format.rgb8, 30)
        # Align color and depth streams
        align_to = pyrealsense2.stream.color
        self.align = pyrealsense2.align(align_to)
        self.pipeline.start(config)
        # Initialize some class variables
        self.aligned_depth_image = None
        self.color_image = None
        self.depth_intrin = None
        # Run self.cam in a seperate thread
        self.cam_thread = threading.Thread(target=self.cam)
        self.cam_thread.start()

    def qr_code_data(self, interested_barcode) -> QR:
        try:
            barcodes = pyzbar.decode(self.color_image)
            for barcode in barcodes:
                (x, y, w, h) = barcode.rect
                cv2.circle(self.color_image, (x, y + h), 20, (0, 255, 0), -1)
                x_mid = int(x + w / 2)
                y_mid = int(y + h / 2)
                depth = self.aligned_depth_image.get_distance(x_mid, y_mid)
                depth_point = pyrealsense2.rs2_deproject_pixel_to_point(self.depth_intrin, [x_mid, y_mid], depth)
                depth_lr = self.aligned_depth_image.get_distance(x+h, y)
                depth_point_lr = pyrealsense2.rs2_deproject_pixel_to_point(self.depth_intrin, [x+h, y], depth_lr)
                barcodeData = barcode.data.decode("utf-8")
                theta = self.get_qr_angle(barcode)
                depth_point = [float("{:.2f}".format(x)) for x in depth_point]
                if barcodeData == interested_barcode:
                    return QR(
                        x_mid,
                        y_mid,
                        depth_point[0], depth_point[1], depth_point[2],
                        theta,
                        [depth_point_lr[0], depth_point_lr[1], depth_point_lr[2]],
                    )
            return None
        except Exception as e:
            rospy.logerr(e)
            return None

    def get_qr_angle(self, qrcode):
        try:
            poly = qrcode.polygon
            x1, y1 = poly[0].x, poly[0].y
            x2, y2 = poly[1].x, poly[1].y
            x3, y3 = poly[2].x, poly[2].y
            # Plot the points as green dots on the image
            # cv2.circle(self.color_image, (x1, y1), 3, (0, 255, 0), -1)
            # cv2.circle(self.color_image, (x2, y2), 3, (0, 255, 0), -1)
            # cv2.circle(self.color_image, (x3, y3), 3, (0, 255, 0), -1)
            # get 3d point
            depth_point1 = pyrealsense2.rs2_deproject_pixel_to_point(
                self.depth_intrin, [x1, y1], self.aligned_depth_image.get_distance(x1, y1)
            )
            depth_point1 = (
                float("{:.2f}".format(depth_point1[0])),
                float("{:.2f}".format(depth_point1[1])),
                float("{:.2f}".format(depth_point1[2])),
            )
            # cv2.putText(
            #     self.color_image,
            #     f"1, {(depth_point1[0], depth_point1[1], depth_point1[2])}",
            #     (x1, y1),
            #     cv2.FONT_HERSHEY_SIMPLEX,
            #     0.5,
            #     (0, 255, 0),
            #     2,
            # )
            depth_point2 = pyrealsense2.rs2_deproject_pixel_to_point(
                self.depth_intrin, [x2, y2], self.aligned_depth_image.get_distance(x2, y2)
            )
            # trim the floats to 2 digits after the decimal point
            depth_point2 = (
                float("{:.2f}".format(depth_point2[0])),
                float("{:.2f}".format(depth_point2[1])),
                float("{:.2f}".format(depth_point2[2])),
            )
            depth_point3 = pyrealsense2.rs2_deproject_pixel_to_point(
                self.depth_intrin, [x3, y3], self.aligned_depth_image.get_distance(x3, y3)
            )
            v1 = np.array(
                [
                    depth_point1[1] - depth_point2[0],
                    depth_point1[1] - depth_point2[0],
                    depth_point1[2] - depth_point2[2],
                ]
            )
            v2 = np.array(
                [
                    depth_point1[1] - depth_point3[0],
                    depth_point1[0] - depth_point3[1],
                    depth_point1[2] - depth_point3[2],
                ]
            )
            normal = np.cross(v2, v1)
            # Get unit vector pointing towards the camera
            camera_vector = np.array([1, 1, 0])
            # Get angle between the two vectors
            angle = atan2(np.linalg.norm(np.cross(camera_vector, normal)), np.dot(camera_vector, normal))
            # Convert to degrees
            angle = angle * 180 / pi
            return angle
        except:
            return None

    def cam(self):
        while 1:
            # Get frameset of color and depth
            frames = self.pipeline.wait_for_frames()
            # Align the depth frame to color frame
            aligned_frames = self.align.process(frames)
            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
            color_frame = aligned_frames.get_color_frame()
            # get intrinsics
            depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
            # Assign to class variables
            self.depth_image = aligned_depth_frame
            self.color_image = np.asanyarray(color_frame.get_data())
            self.depth_intrin = depth_intrin

    def is_qr_visible(self, interested_barcode) -> QR:
        if self.aligned_depth_image is None or self.color_image is None or self.depth_intrin is None:
            rospy.logwarn(f"{self.device_id} No Camera frames received yet")
            return None
        qr = self.qr_code_data(interested_barcode)
        cv2.imshow(f"Cam {self.device_id}", self.color_image)
        cv2.waitKey(1)
        return qr

    def reached_highest_depth(self, ideal_depth) -> bool:
        depth_image = np.asanyarray(self.aligned_depth_image.get_data())
        # get highest value in the depth image
        # TODO: Change to disparity map
        # https://github.com/IntelRealSense/librealsense/issues/7431
        # Get 2D indices of highest value
        highest_depth_index = np.unravel_index(np.argmax(depth_image), depth_image.shape)
        # Get highest value
        highest_depth = self.aligned_depth_image.get_distance(highest_depth_index[0], highest_depth_index[1])
        if highest_depth < ideal_depth:
            return True
        return False
