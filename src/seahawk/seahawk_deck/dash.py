import sys
from os import environ, path
from threading import Thread

from PyQt5 import QtCore as qtc
from PyQt5 import QtGui as qtg
from PyQt5 import QtWidgets as qtw
from PyQt5.QtGui import QKeyEvent
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node 
from rclpy.publisher import Publisher
from std_msgs.msg import String
from sensor_msgs.msg import Image

from seahawk_deck.dash_styling.color_palette import DARK_MODE, LIGHT_MODE
from seahawk_deck.dash_widgets.countdown_widget import CountdownWidget
from seahawk_deck.dash_widgets.numeric_data_widget import NumericDataWidget
from seahawk_deck.dash_widgets.state_widget import StateWidget
from seahawk_deck.dash_widgets.throttle_curve_widget import ThrtCrvWidget
from seahawk_deck.dash_widgets.turn_bank_indicator_widget import TurnBankIndicator
from seahawk_deck.dash_widgets.term_widget import TermWidget
from seahawk_deck.set_remote_params import SetRemoteParams
from seahawk_msgs.msg import InputStates

PATH = path.dirname(__file__)

DEFAULT_COLORS = DARK_MODE


class RosQtBridge(qtw.QWidget):
    """
    Thread safe bridge between ROS and Qt using signals and slots. `RosQtBridge`
    object instance must be passed to the node and main window class.
    """
    
    # Signals (must be class variables)
    new_input_state_msg_sgl = qtc.pyqtSignal()
    new_cam_front_msg_sgl = qtc.pyqtSignal()
    new_cam_claw_msg_sgl = qtc.pyqtSignal()
    new_cam_top_msg_sgl = qtc.pyqtSignal()
    new_publisher_sgl = qtc.pyqtSignal()
    new_set_params_sgl = qtc.pyqtSignal()

    def __init__(self):
        """
        Initialize `RosQtBridge` object.
        """
        super().__init__()
        # Variables to transfer data between ROS and Qt
        self.input_state_msg = None
        self.cam_front_msg = None 
        self.cam_claw_msg = None
        self.cam_top_msg = None
        self.keystroke_pub = None
        self.pilot_input_set_params = None

    def callback_input_states(self, msg: InputStates):
        """
        Called for each time a message is published to the `input_states` topic.
        Collects the contents of the message sent and emits a `new_input_state_msg_sgl`
        signal which is received by Qt.

        Args:
            msg: Message of type `InputStates` from the `input_states` topic.
        """
        self.input_state_msg = msg
        self.new_input_state_msg_sgl.emit()

    def callback_cam_front(self, msg: Image):
        """
        Called for each time a message is published to the `camera/front/h264` topic.
        Collects the contents of the message sent and emits a `new_cam_front_msg_sgl`
        signal which is received by Qt.

        Args:
            msg: Message of type `Image` from the `camera/front/h264` topic.
        """
        self.cam_front_msg = msg
        self.new_cam_front_msg_sgl.emit()

    def callback_cam_claw(self, msg: Image):
        """
        Called for each time a message is published to the `camera/claw/h264` topic.
        Collects the contents of the message sent and emits a `new_cam_claw_msg_sgl`
        signal which is received by Qt.

        Args:
            msg: Message of type `Image` from the `camera/claw/h264` topic.
        """
        self.cam_claw_msg = msg
        self.new_cam_claw_msg_sgl.emit()

    def callback_cam_top(self, msg: Image):
        """
        Called for each time a message is published to the `camera/top/h264` topic.
        Collects the contents of the message sent and emits a `new_cam_top_msg_sgl`
        signal which is received by Qt.

        Args:
            msg: Message of type `Image` from the `camera/top/h264` topic
        """
        self.cam_top_msg = msg
        self.new_cam_top_msg_sgl.emit()

    def add_publisher(self, pub: Publisher):
        """
        Gives Qt access to a ROS publisher and emits a `new_publisher_sgl`.

        Args:
            pub: Publisher to add, publishes to `keystroke` topic.
        """
        self.keystroke_pub = pub
        self.new_publisher_sgl.emit()
    
    def add_set_params(self, set_param_obj: SetRemoteParams):
        """
        Gives Qt access to a `SetRemoteParams` and emits a `pilot_input_set_params`.

        Args:
            set_param_obj: SetRemoteParams instance to give Qt access to.
        """
        self.pilot_input_set_params = set_param_obj
        self.new_set_params_sgl.emit()


class VideoFrame():
    """
    Data needed to display a video frame.
    """

    def __init__(self):
        """
        Set up the 'VideoFrame' attributes initial values.
        """
        self.image = None
    
        self.label = qtw.QLabel()
        # Fit video frame to size of label, no need to resize it later
        self.label.setScaledContents(True)
        self.label.setSizePolicy(qtw.QSizePolicy.Ignored, qtw.QSizePolicy.Ignored)


class MainWindow(qtw.QMainWindow):
    """
    Creates a 'MainWindow' which inherits from the 'qtw.QMainWindow' class. 'MainWindow'
    provides the main dash window space to overlay widgets.
    """

    def __init__(self, ros_qt_bridge: RosQtBridge):
        """
        Set up the 'MainWindow', overlay 'TabWidget's for multiple dash views, and display window.

        Args:
            ros_qt_bridge: Bridge object for functionality between ROS and Qt.
        """
        super().__init__()

        self.ros_qt_bridge = ros_qt_bridge
        self.ros_qt_bridge.new_publisher_sgl.connect(self.init_publisher)
        self.ros_qt_bridge.new_set_params_sgl.connect(self.add_set_params)
        self.keystroke_pub = None
        self.pilot_input_set_params = None

        # Set up main window
        self.colors = DEFAULT_COLORS
        self.setWindowTitle("SeaHawk II Dashboard")
        self.setStyleSheet(f"background-color: {self.colors['MAIN_WIN_BKG']};")

        # Create tabs
        self.tab_widget = TabWidget(self, self.ros_qt_bridge, PATH + "/dash_styling/tab_widget.txt", self.colors)
        self.setCentralWidget(self.tab_widget)

        # Display window
        self.showMaximized()

    @qtc.pyqtSlot()
    def init_publisher(self):
        """
        Adds the keystroke publisher to `MainWindow`.
        """
        self.keystroke_pub = self.ros_qt_bridge.keystroke_pub

    @qtc.pyqtSlot()
    def add_set_params(self):
        """
        Adds the pilot input set params object to `MainWindow`.
        """
        self.pilot_input_set_params = self.ros_qt_bridge.pilot_input_set_params

    def keyPressEvent(self, a0: QKeyEvent) -> None:
        """
        Called for each time there is a keystroke. Publishes the code of the key that was
        pressed or released to the ROS topic `keystroke` and sets parameters of other nodes
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
            self.tab_widget.thrt_crv_widget.update(int(data))
        
        # Change colors mode between light and dark mode
        if data == "0":
            if self.colors == DARK_MODE: self.update_colors(LIGHT_MODE)
            else: self.update_colors(DARK_MODE)

    def update_colors(self, new_colors: dict):
        self.colors = new_colors
        self.setStyleSheet(f"background-color: {self.colors['MAIN_WIN_BKG']};")
        self.tab_widget.set_colors(self.colors)
        self.tab_widget.state_widget.set_colors(self.colors)
        self.tab_widget.thrt_crv_widget.set_colors(self.colors)
        self.tab_widget.temp_widget.set_colors(self.colors)
        self.tab_widget.depth_widget.set_colors(self.colors)
        self.tab_widget.turn_bank_indicator_widget.set_colors(self.colors)
        self.tab_widget.countdown_widget.set_colors(self.colors)
        self.tab_widget.term_widget.set_colors(self.colors)
        

class TabWidget(qtw.QWidget):
    """
    Creates a `TabWidget` which inherits from the `qtw.QWidget` class. A `TabWidget` provides 
    a tab bar and a page area that is used to display pages related to each tab. The tab bar is 
    shown above the page area. Each tab is associated with a different widget called a page. 
    Only the current page is shown in the page area; all the other pages are hidden. The user can 
    show a different page by clicking on its tab
    """

    def __init__(self, parent: MainWindow, ros_qt_bridge: RosQtBridge, style_sheet_file: str, colors: dict):
        """
        Initialize tab widget.

        Args:
            parent: Window where to place tabs.
            ros_qt_bridge: Bridge object for functionality between ROS and Qt.
            style_sheet_file: Style sheet text file formatted as a CSS f-string.
            colors: Hex codes to color widget with.
        """
        super().__init__(parent)

        self.colors = colors
        with open(style_sheet_file) as style_sheet:
            self.style_sheet = style_sheet.read()
        
        # Bridge between ros and the rt dashboard
        self.ros_qt_bridge = ros_qt_bridge
        # Connect signals to slots for thread safe communication between ROS and Qt
        self.ros_qt_bridge.new_input_state_msg_sgl.connect(self.update_pilot_tab_input_states)
        self.ros_qt_bridge.new_cam_front_msg_sgl.connect(self.update_cam_front)
        self.ros_qt_bridge.new_cam_claw_msg_sgl.connect(self.update_cam_claw)
        self.ros_qt_bridge.new_cam_top_msg_sgl.connect(self.update_cam_top)
    
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

        self.setStyleSheet(self.style_sheet.format(**self.colors))
        
        # Add tabs to widget
        layout.addWidget(tabs)

        # Create specific tabs
        self.create_pilot_tab(self.tab_dict["Pilot"])
        self.create_debug_tab(self.tab_dict["Debug"])

        # Apply css styling
        self.set_colors(self.colors)
    
    def set_colors(self, new_colors: dict):
        """
        Sets widget colors given a dictionary of hex color codes.

        Args:
            new_colors: Hex codes to color widget with.
        """
        self.setStyleSheet(self.style_sheet.format(**new_colors))
        self.demo_map.setPixmap(qtg.QPixmap(new_colors["MAP_IMG"]))

    def create_pilot_tab(self, tab: qtw.QWidget):
        """
        Creates pilot dash tab with the following widgets:
            - Feature states:   Displays the states of Bambi Mode (on/off), the claw (closed/open), CoM shift (engaged/not)
            - Throttle curve:   Displays the activated throttle curve
            - Temperature:      Displays the temperature reading
            - Depth:            Displays the depth reading
            - IMU:              Displays the IMU readings as a turn/bank indicator (graphic to help keep constant acceleration)
            - Countdown:        Displays a countdown
            - Front camera      Displays video feed from front camera
            - Claw camera       Displays video feed from claw camera
            - Top camera        Displays video feed from top camera
            - Product demo map  Displays a static image of the product demo area map
        
        Args: 
            tab: Tab object instance of pilot tab.
        """
        # Setup layouts
        home_window_layout = qtw.QHBoxLayout(tab)
        vert_widgets_layout = qtw.QVBoxLayout()
        vert_widgets_layout.setSpacing(0)
        cam_layout = qtw.QGridLayout()

        # Create widgets
        self.state_widget = StateWidget(tab, ["Bambi Mode", "CoM Shift"], PATH + "/dash_styling/state_widget.txt", self.colors)
        self.thrt_crv_widget = ThrtCrvWidget(tab, self.colors)
        self.temp_widget = NumericDataWidget(tab, "Temperature", PATH + "/dash_styling/numeric_data_widget.txt", self.colors)
        self.depth_widget = NumericDataWidget(tab, "Depth", PATH + "/dash_styling/numeric_data_widget.txt", self.colors)
        self.turn_bank_indicator_widget = TurnBankIndicator(tab, PATH + "/dash_styling/numeric_data_widget.txt", self.colors)
        self.countdown_widget = CountdownWidget(tab, PATH + "/dash_styling/countdown_widget.txt", self.colors, minutes=15, seconds=0)

        # Add widgets to side vertical layout
        # Stretch modifies the ratios of the widgets (must add up to 100)
        vert_widgets_layout.addWidget(self.state_widget, stretch=12)
        vert_widgets_layout.addWidget(self.thrt_crv_widget, stretch=16)
        vert_widgets_layout.addWidget(self.temp_widget, stretch=16)
        vert_widgets_layout.addWidget(self.depth_widget, stretch=16)
        vert_widgets_layout.addWidget(self.turn_bank_indicator_widget, stretch=16)
        vert_widgets_layout.addWidget(self.countdown_widget, stretch=24)

        # Setup cameras
        self.cam_front = VideoFrame()
        self.cam_claw = VideoFrame()
        self.cam_top = VideoFrame()
        
        # Product demo map image
        self.demo_map = qtw.QLabel()
        # Dynamically sized, endures the image fits any aspect ratio. Will resize image to fit
        self.demo_map.setScaledContents(True)
        self.demo_map.setSizePolicy(qtw.QSizePolicy.Ignored, qtw.QSizePolicy.Ignored)

        # (0, 0)    (0, 1)
        # (1, 0)    (1, 1)
        cam_layout.addWidget(self.cam_front.label, 0, 0)
        cam_layout.addWidget(self.cam_claw.label, 0, 1)
        cam_layout.addWidget(self.cam_top.label, 1, 0)
        cam_layout.addWidget(self.demo_map, 1, 1)

        home_window_layout.addLayout(vert_widgets_layout, stretch=1)
        home_window_layout.addLayout(cam_layout, stretch=9)

    @staticmethod
    def update_cam_img(data: Image, video_frame: VideoFrame):
        """
        Updates the display of the video on the dashboard.

        Args:
            data: ROS message of type `Image` containing the new image to display.
            video_frame: `VideoFrame` object to update.
        """
        bridge = CvBridge()
        try:
            # Create cv image from ros image then resize it to fit dashboard
            cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as error:
            print(f"update_cam_img() failed while trying to convert image from {data.cam_msg.encoding} to 'bgr8'.\n{error}")
            sys.exit()

        height, width, _ = cv_image.shape
        bytesPerLine = 3 * width
        # Cv image must be converted to a QImage to be displayed
        frame = qtg.QImage(cv_image.data, width, height, bytesPerLine, qtg.QImage.Format_RGB888).rgbSwapped()
        video_frame.label.setPixmap(qtg.QPixmap(frame))

    @qtc.pyqtSlot()
    def update_cam_front(self):
        """
        Slot which updates front camera image on the dashboard.
        """
        TabWidget.update_cam_img(self.ros_qt_bridge.cam_front_msg, self.cam_front)
    
    @qtc.pyqtSlot()
    def update_cam_claw(self):
        """
        Slot which updates claw camera image on the dashboard.
        """     
        TabWidget.update_cam_img(self.ros_qt_bridge.cam_claw_msg, self.cam_claw)
    
    @qtc.pyqtSlot()
    def update_cam_top(self):
        """
        Slot which updates front top image on the dashboard.
        """
        TabWidget.update_cam_img(self.ros_qt_bridge.cam_top_msg, self.cam_top)

    @qtc.pyqtSlot()
    def update_pilot_tab_input_states(self):
        """
        Slot which updates gui display of input states.
        """
        input_state_dict = {
            "Bambi Mode":   self.ros_qt_bridge.input_state_msg.bambi_mode,
            "CoM Shift":    self.ros_qt_bridge.input_state_msg.com_shift
        }
        self.state_widget.update(input_state_dict)
    
    def create_debug_tab(self, tab: qtw.QWidget):
        # Setup layouts
        debug_layout = qtw.QHBoxLayout(tab)
        graph_layout = qtw.QGridLayout()
        term_layout = qtw.QVBoxLayout()

        # Create debug graph instances
        temp_graph_1 = qtw.QFrame()
        temp_graph_2 = qtw.QFrame()
        temp_graph_3 = qtw.QFrame()
        temp_graph_4 = qtw.QFrame()

        # (0, 0)    (0, 1)
        # (1, 0)    (1, 1)
        graph_layout.addWidget(temp_graph_1, 0, 0)
        graph_layout.addWidget(temp_graph_2, 0, 1)
        graph_layout.addWidget(temp_graph_3, 1, 0)
        graph_layout.addWidget(temp_graph_4, 1, 1)


        self.term_widget = TermWidget(tab, PATH + "/dash_styling/term_widget.txt", self.colors)

        term_layout.addWidget(self.term_widget)

        debug_layout.addLayout(graph_layout, stretch=7)
        debug_layout.addLayout(term_layout, stretch=3)


class Dash(Node):
    """
    Creates and runs a ROS node which updates the PyQt dashboard with data from ROS topics.
    """

    def __init__(self, ros_qt_bridge: RosQtBridge):
        """
        Initialize 'dash' node

        Args:
            ros_qt_bridge: Bridge object for functionality between ROS and Qt.
        """
        super().__init__("dash")

        self.create_subscription(InputStates, "input_states", ros_qt_bridge.callback_input_states, 10)
        # self.create_subscription(DebugInfo, "debug_info", bridge.callback_debug, 10)

        # Camera subscriptions
        self.create_subscription(Image, "camera/front/image", ros_qt_bridge.callback_cam_front, 10)
        self.create_subscription(Image, "camera/claw/image", ros_qt_bridge.callback_cam_claw, 10)
        self.create_subscription(Image, "camera/top/image", ros_qt_bridge.callback_cam_top, 10)

        ros_qt_bridge.add_publisher(self.create_publisher(String, "keystroke", 10))
        
        # Comment this out if we want to test the dashboard without parameters (will crash if pilot_input is inactive)
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
    dash_node = Dash(bridge)

    # Threading allows the process to display the dash and run the node at the same time
    # Create and start a thread for rclpy.spin function so the node spins while the dash is running
    node_thread = Thread(target=rclpy.spin, args=(dash_node,))
    node_thread.start()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main(sys.argv)
