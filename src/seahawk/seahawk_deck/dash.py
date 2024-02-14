import sys
from os import environ, path
from threading import Thread

from PyQt5 import QtWidgets as qtw
from PyQt5.QtGui import QKeyEvent
# from PyQt5 import QtGui as qtg
# from PyQt5 import QtCore as qtc
import rclpy
from rclpy.node import Node 
from rclpy.publisher import Publisher
from std_msgs.msg import Int32

from seahawk_deck.dash_styling.color_palette import DARK_MODE
from seahawk_deck.dash_widgets.countdown_widget import CountdownWidget
from seahawk_deck.dash_widgets.numeric_data_widget import NumericDataWidget
from seahawk_deck.dash_widgets.state_widget import StateWidget
from seahawk_deck.dash_widgets.throttle_curve_widget import ThrtCrvWidget
from seahawk_deck.dash_widgets.turn_bank_indicator_widget import TurnBankIndicator
from seahawk_msgs.msg import InputStates, DebugInfo
# from h264_image_transport.h264_msgs import Packet

# Constants
# MAX_WIDTH   = 1862
# MAX_HEIGHT  = 1053
MAX_WIDTH   = 1000  # Temp for debugging
MAX_HEIGHT  = 600   # Temp for debugging
COLOR_CONSTS = DARK_MODE
PATH = path.dirname(__file__)


class MainWindow(qtw.QMainWindow):
    """
    Creates a 'MainWindow' which inherits from the 'qtw.QMainWindow' class. 'MainWindow'
    provides the main dash window space to overlay widgets
    """

    def __init__(self):
        """
        Set up the 'MainWindow', overlay 'TabWidget's for multiple dash views, and display window
        """
        super().__init__()

        # Set up main window
        self.setWindowTitle("SeaHawk II Dashboard")
        self.setStyleSheet(f"background-color: {COLOR_CONSTS['MAIN_WIN_BKG']};")
        # self.setGeometry(0, 0, MAX_WIDTH, MAX_HEIGHT)

        # Create tabs
        self.tab_widget = TabWidget(self, PATH + "/dash_styling/tab_widget.txt")
        self.setCentralWidget(self.tab_widget)

        self.keystroke_pub = None

        # Display window
        # self.show()
        self.showMaximized()
    
    def keyPressEvent(self, a0: QKeyEvent) -> None:
        """
        Called for each time there is a keystroke. Publishes the code of the key that was
        pressed or released to the ROS topic 'keystroke'
        """
        msg = Int32()
        msg.data = a0.key()
        self.keystroke_pub.publish(msg)

    def add_keystroke_publisher(self, pub: Publisher):
        """
        Adds the keystroke publisher to 'MainWindow'
        """
        self.keystroke_pub = pub


class TabWidget(qtw.QWidget):
    """
    Creates a 'TabWidget' which inherits from the 'qtw.QWidget' class. A 'TabWidget' provides 
    a tab bar and a page area that is used to display pages related to each tab. The tab bar is 
    shown above the page area. Each tab is associated with a different widget called a page. 
    Only the current page is shown in the page area; all the other pages are hidden. The user can 
    show a different page by clicking on its tab
    """

    def __init__(self, parent: MainWindow, style_sheet_file: str):
        """
        Initialize tab widget

        Args:
            parent: Window where to place tabs
            style_sheet_file: Style sheet text file formatted as a CSS f-string
        """
        super().__init__(parent)
        
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
        home_window_layout = qtw.QHBoxLayout(tab)
        vert_widgets_layout = qtw.QVBoxLayout()
        vert_widgets_layout.setSpacing(0)
        cam_layout = qtw.QVBoxLayout()

        # Create widgetss
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
        frame = qtw.QFrame()
        frame.setStyleSheet("background-color: red;")
        cam_layout.addWidget(frame)

        home_window_layout.addLayout(vert_widgets_layout, stretch=1)
        home_window_layout.addLayout(cam_layout, stretch=9)

    def update_pilot_tab_input_states(self, state_to_update: str):
        """
        Update gui display of input states

        Args:
            feat_state_update: List of values to update for the feature widget
            thrt_crv_update: Updated value for throttle curve
        """
        self.state_widget.update_state(state_to_update["state_widget"])
        self.thrt_crv_widget.update_thrt_crv(state_to_update["throttle_curve"])


def fix_term():
    """
    If VS Code was installed with snap, the 'GTK_PATH' variable must be unset.
    This is automated in this function
    """
    if "GTK_PATH" in environ and "snap" in environ["GTK_PATH"]:
        environ.pop("GTK_PATH")


class Dash(Node):
    """
    Creates and runs a ROS node which updates the PyQt dashboard with data from ROS topics
    """

    def __init__(self, dash_window):
        """
        Initialize 'dash' node
        """
        super().__init__("dash")
        self.dash_window = dash_window

        self.create_subscription(InputStates, "input_states", self.callback_input_states, 10)
        self.create_subscription(DebugInfo, "debug_info", self.callback_debug, 10)
        # self.create_subscription(Packet, "republish_claw_camera", self.callback_camera, 10)
        
        # Add keystroke publisher to the dash so it can capture keystrokes and publish them to the ROS network
        dash_window.add_keystroke_publisher(self.create_publisher(Int32, "keystroke", 10))

    def callback_input_states(self, input_state_msg: InputStates): 
        """
        For every message published to the 'input_states' topic, update the relevant values on the gui 

        Updates dash representation of:
            - Bambi mode
            - Claw state 
            - CoM shift
            - Throttle curve option
        Based on values from 'input_states' topic

        Args:
            input_state_msg: Message from the type 'InputStates' from the 'input_states' topic
        """
        # Map the values sent from 'input_states' to feature names
        input_state_dict = {
            "state_widget": {
                "Bambi Mode":   input_state_msg.bambi_mode,
                "Claw":         input_state_msg.claw_state,
                "CoM Shift":    input_state_msg.com_shift,
            },
            "throttle_curve":   int(input_state_msg.throttle_curve),
        }
        self.dash_window.tab_widget.update_pilot_tab_input_states(input_state_dict)

    def callback_camera(self, camera_msg):
        # self.dash_window.tab_widget.write_framw(camera_msg)
        pass

    def callback_debug(self):
        pass


def main(args=None):
    # Setup dashboards
    fix_term()
    app = qtw.QApplication([])
    pilot_dash = MainWindow()
    
    # Setup node
    rclpy.init(args=args)
    dash_node = Dash(pilot_dash)

    # Threading allows the process to display the dash and run the node at the same time
    # Create and start a thread for rclpy.spin function so the node spins while the dash is running
    node_thread = Thread(target=rclpy.spin, args=(dash_node,))
    node_thread.start()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main(sys.argv)