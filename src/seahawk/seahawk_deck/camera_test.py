from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node 
import rclpy
import cv2
from PyQt5 import QtGui as qtg
import sys
from os import environ

from sensor_msgs.msg import Image


class ImageView(Node):
    def __init__(self):
        super().__init__("image_view")
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, "repub_raw", self.callback_img, 10)

    def callback_img(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv2.imshow("Image window", cv_image)
            cv2.waitKey(3)
        except CvBridgeError as error:
            print(f"Image_View.callback_img() failed while trying to convert image from {msg.encoding} to 'bgr8'.\n{error}")
    
        # height, width, channel = cv_image.shape
        # bytesPerLine = 3 * width
        # frame = qtg.QImage(cv_image.data, width, height, bytesPerLine, qtg.QImage.Format_RGB888).rgbSwapped()
        # self.label.setPixmap(qtg.QPixmap(frame))


def fix_term():
    """
    If VS Code was installed with snap, the 'GTK_PATH' variable must be unset.
    This is automated in this function
    """
    if "GTK_PATH" in environ and "snap" in environ["GTK_PATH"]:
        environ.pop("GTK_PATH")


def main(args=None):
    # Setup dashboards
    fix_term()
    rclpy.init(args=args)
    rclpy.spin(ImageView())
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main(sys.argv)
    



######################################
# Sources:
# cv_bridge: http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
# desired_encoding: http://docs.ros.org/en/jade/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html#ab6e2cec975df0a802ca02ce56674b279
    

# class ImageView(Node):
#     def __init__(self):
#         super().__init__("image_view")
#         self.bridge = CvBridge()
#         self.subscription = self.create_subscription(Image, "repub_raw", self.callback_img, 10)

#     def callback_img(self, msg: Image):
#         try:
#             cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
#             cv2.imshow("Image window", cv_image)
#             cv2.waitKey(3)
#         except CvBridgeError as error:
#             print(f"Image_View.callback_img() failed while trying to convert image from {msg.encoding} to 'bgr8'.\n{error}")