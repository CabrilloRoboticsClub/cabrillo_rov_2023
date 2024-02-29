import sys
from os import environ, path
from threading import Thread
import cv2

from PyQt5 import QtWidgets as qtw
from PyQt5.QtGui import QKeyEvent
from cv_bridge import CvBridge, CvBridgeError
from PyQt5 import QtGui as qtg
from PyQt5 import QtCore as qtc
import rclpy
from rclpy.node import Node 
from rclpy.publisher import Publisher
from std_msgs.msg import String
from sensor_msgs.msg import Image

from seahawk_deck.dash_styling.color_palette import DARK_MODE
from seahawk_deck.dash_widgets.countdown_widget import CountdownWidget
from seahawk_deck.dash_widgets.numeric_data_widget import NumericDataWidget
from seahawk_deck.dash_widgets.state_widget import StateWidget
from seahawk_deck.dash_widgets.throttle_curve_widget import ThrtCrvWidget
from seahawk_deck.dash_widgets.turn_bank_indicator_widget import TurnBankIndicator
from seahawk_deck.set_remote_params import SetRemoteParams
from seahawk_msgs.msg import InputStates, DebugInfo


COLOR_CONSTS = DARK_MODE
PATH = path.dirname(__file__)


class RosQtBridge(qtw.QWidget):
    new_input_state_msg_sgl = qtc.pyqtSignal()
    new_cam_msg_sgl = qtc.pyqtSignal()
    new_publisher_sgl = qtc.pyqtSignal()
    new_set_params = qtc.pyqtSignal()

    def __init__(self):
        super().__init__()
        self.cam_msg = None 
        self.input_state_msg = None
        self.keystroke_pub = None
        self.pilot_input_set_params = None

    def callback_input_states(self, msg: InputStates):
        self.input_state_msg = msg
        self.new_input_state_msg_sgl.emit()

    def callback_img(self, msg: Image):
        self.cam_msg = msg
        self.new_cam_msg_sgl.emit()
    
    def add_publisher(self, pub: Publisher):
        self.keystroke_pub = pub
        self.new_publisher_sgl.emit()
    
    def add_set_params(self, set_param_obj: SetRemoteParams):
        self.pilot_input_set_params = set_param_obj
        self.new_set_params.emit()


class MainWindow(qtw.QMainWindow):
    """
    Creates a 'MainWindow' which inherits from the 'qtw.QMainWindow' class. 'MainWindow'
    provides the main dash window space to overlay widgets
    """

    def __init__(self, ros_qt_bridge):
        """
        Set up the 'MainWindow', overlay 'TabWidget's for multiple dash views, and display window
        """
        super().__init__()

        self.ros_qt_bridge = ros_qt_bridge
        self.ros_qt_bridge.new_publisher_sgl.connect(self.init_publisher)
        self.ros_qt_bridge.new_set_params.connect(self.init_set_params)
        self.keystroke_pub = None
        self.pilot_input_set_params = None

        # Set up main window
        self.setWindowTitle("SeaHawk II Dashboard")
        self.setStyleSheet(f"background-color: {COLOR_CONSTS['MAIN_WIN_BKG']};")

        # Create tabs
        self.tab_widget = TabWidget(self, PATH + "/dash_styling/tab_widget.txt", self.ros_qt_bridge)
        self.setCentralWidget(self.tab_widget)


        # Display window
        self.showMaximized()
    
    @qtc.pyqtSlot()
    def init_publisher(self):
        """
        Adds the keystroke publisher to 'MainWindow'
        """
        self.keystroke_pub = self.ros_qt_bridge.keystroke_pub

    @qtc.pyqtSlot()
    def init_set_params(self):
        """
        Adds the pilot input set params object to 'MainWindow'
        """
        self.pilot_input_set_params = self.ros_qt_bridge.pilot_input_set_params

    def keyPressEvent(self, a0: QKeyEvent) -> None:
        """
        Called for each time there is a keystroke. Publishes the code of the key that was
        pressed or released to the ROS topic 'keystroke' and sets parameters of other nodes
        dependant on keystrokes
        """
        try:
            data = chr(a0.key())
        except ValueError:
            data = "Invalid key"
        
        # Publish key to 'keystroke' topic
        msg = String()
        msg.data = data
        self.keystroke_pub.publish(msg)

        # Update throttle curve parameter
        if data in ["1", "2", "3"]:
            self.pilot_input_set_params.update_params("throttle_curve_choice", data)
            self.pilot_input_set_params.send_params()
            self.tab_widget.thrt_crv_widget.update(data)


class TabWidget(qtw.QWidget):
    """
    Creates a 'TabWidget' which inherits from the 'qtw.QWidget' class. A 'TabWidget' provides 
    a tab bar and a page area that is used to display pages related to each tab. The tab bar is 
    shown above the page area. Each tab is associated with a different widget called a page. 
    Only the current page is shown in the page area; all the other pages are hidden. The user can 
    show a different page by clicking on its tab
    """

    def __init__(self, parent: MainWindow, style_sheet_file: str, ros_qt_bridge):
        """
        Initialize tab widget

        Args:
            parent: Window where to place tabs
            style_sheet_file: Style sheet text file formatted as a CSS f-string
        """
        super().__init__(parent)
        
        # Bridge between ros and the rt dashboard
        self.ros_qt_bridge = ros_qt_bridge
        self.ros_qt_bridge.new_input_state_msg_sgl.connect(self.update_pilot_tab_input_states)
        self.ros_qt_bridge.new_cam_msg_sgl.connect(self.update_cam_img)

        # Define layout of tabs
        layout = qtw.QVBoxLayout(self)
        self.setLayout(layout)

        # Initialize tabs
        tabs = qtw.QTabWidget()

        # Create a dict in which the key is the provided name of the tab, and the value is a qtw.QWidget() object
        tab_names = ["Pilot", "Co-Pilot", "VPF", "Debug", "Cameras", "Control Mapping"]
        self.tab_dict = {name: qtw.QWidget() for name in tab_names}

        # Add tabs
        for name, tab in self.tab_dict.items():
            tabs.addTab(tab, name)
        
        # Apply css styling
        with open(style_sheet_file) as style_sheet:
            self.setStyleSheet(style_sheet.read().format(**COLOR_CONSTS))
        
        # Add tabs to widget
        layout.addWidget(tabs)

        # Create specific tabs
        self.create_pilot_tab(self.tab_dict["Pilot"])

    def create_pilot_tab(self, tab):
        """
        Creates pilot dash tab with the following widgets:
            - Feature states:   Displays the states of Bambi Mode (on/off), the claw (closed/open), CoM shift (engaged/not)
            - Throttle curve:   Displays the activated throttle curve
            - Temperature:      Displays the temperature reading
            - Depth:            Displays the depth reading
            - IMU:              Displays the IMU readings as a turn/bank indicator (graphic to help keep constant acceleration)
            - Countdown:        Displays a countdown
        """
        # Setup layouts
        home_window_layout = qtw.QHBoxLayout(tab)
        vert_widgets_layout = qtw.QVBoxLayout()
        vert_widgets_layout.setSpacing(0)
        cam_layout = qtw.QVBoxLayout()

        # Create widgets
        self.state_widget = StateWidget(tab, ["Bambi Mode", "Claw", "CoM Shift"], PATH + "/dash_styling/state_widget.txt")
        self.thrt_crv_widget = ThrtCrvWidget(tab)
        self.temp_widget = NumericDataWidget(tab, "Temperature", PATH + "/dash_styling/numeric_data_widget.txt")
        self.depth_widget = NumericDataWidget(tab, "Depth", PATH + "/dash_styling/numeric_data_widget.txt")
        self.turn_bank_indicator_widget = TurnBankIndicator(tab, PATH + "/dash_styling/numeric_data_widget.txt")
        self.countdown_widget = CountdownWidget(tab, PATH + "/dash_styling/countdown_widget.txt", minutes=15, seconds=0)

        # Add widgets to side vertical layout
        # Stretch modifies the ratios of the widgets (must add up to 100)
        vert_widgets_layout.addWidget(self.state_widget, stretch=16)
        vert_widgets_layout.addWidget(self.thrt_crv_widget, stretch=16)
        vert_widgets_layout.addWidget(self.temp_widget, stretch=16)
        vert_widgets_layout.addWidget(self.depth_widget, stretch=16)
        vert_widgets_layout.addWidget(self.turn_bank_indicator_widget, stretch=16)
        vert_widgets_layout.addWidget(self.countdown_widget, stretch=20)

        # Temp code for cameras
        self.label = qtw.QLabel()
        cam_layout.addWidget(self.label)

        home_window_layout.addLayout(vert_widgets_layout, stretch=1)
        home_window_layout.addLayout(cam_layout, stretch=9)

        # Track the camera state. Upon first camera frame, note the dimensions
        self.cam_init = True
        self.cam_height = None
        self.cam_width = None

    @qtc.pyqtSlot()
    def update_pilot_tab_input_states(self):
        """
        Update gui display of input states
        """
        input_state_dict = {
            "Bambi Mode":   self.ros_qt_bridge.input_state_msg.bambi_mode,
            "Claw":         self.ros_qt_bridge.input_state_msg.claw_state,
            "CoM Shift":    self.ros_qt_bridge.input_state_msg.com_shift
        }
        self.state_widget.update(input_state_dict)

    @qtc.pyqtSlot()
    def update_cam_img(self):
        """
        Update gui display of camera frame
        """
        # Collect camera geometry if it is the first time opening the camera
        if self.cam_init:
            self.cam_height = self.label.height()
            self.cam_width = self.label.width()
            self.cam_init = False

        bridge = CvBridge()
        try:
            cv_image = cv2.resize(bridge.imgmsg_to_cv2(self.ros_qt_bridge.cam_msg, desired_encoding="bgr8"), (self.cam_width, self.cam_height))
        except CvBridgeError as error:
            print(f"update_cam_img() failed while trying to convert image from {self.ros_qt_bridge.cam_msg.encoding} to 'bgr8'.\n{error}")
            sys.exit()

        height, width, channel = cv_image.shape
        bytesPerLine = 3 * width
        frame = qtg.QImage(cv_image.data, width, height, bytesPerLine, qtg.QImage.Format_RGB888).rgbSwapped()
        self.label.setPixmap(qtg.QPixmap(frame))


class Dash(Node):
    """
    Creates and runs a ROS node which updates the PyQt dashboard with data from ROS topics
    """

    def __init__(self, dash_window, ros_qt_bridge):
        """
        Initialize 'dash' node
        """
        super().__init__("dash")
        self.dash_window = dash_window

        self.create_subscription(InputStates, "input_states", ros_qt_bridge.callback_input_states, 10)
        # self.create_subscription(DebugInfo, "debug_info", bridge.callback_debug, 10)
        self.create_subscription(Image, "repub_raw", ros_qt_bridge.callback_img, 10)
        ros_qt_bridge.add_publisher(self.create_publisher(String, "keystroke", 10))

        # dash_window.add_publisher(self.create_publisher(String, "keystroke", 10))
        ros_qt_bridge.add_set_params(SetRemoteParams(self, "pilot_input"))


def fix_term():
    """
    If VS Code was installed with snap, the 'GTK_PATH' variable must be unset.
    This is automated in this function
    """
    if "GTK_PATH" in environ and "snap" in environ["GTK_PATH"]:
        environ.pop("GTK_PATH")


def main(args=None):
    rclpy.init(args=args)
    
    fix_term()

    app = qtw.QApplication([])

    bridge = RosQtBridge()

    pilot_dash = MainWindow(bridge)

    # Setup node
    dash_node = Dash(pilot_dash, bridge)

    # Threading allows the process to display the dash and run the node at the same time
    # Create and start a thread for rclpy.spin function so the node spins while the dash is running
    node_thread = Thread(target=rclpy.spin, args=(dash_node,))
    node_thread.start()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main(sys.argv)
