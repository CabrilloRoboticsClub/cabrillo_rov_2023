from os import environ, path

from PyQt5 import QtWidgets as qtw
from PyQt5.QtGui import QKeyEvent
from PyQt5.QtCore import QEvent, QObject

from qt_gui.plugin import Plugin

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

# Constants
# MAX_WIDTH   = 1862
# MAX_HEIGHT  = 1053
MAX_WIDTH   = 1000  # Temp for debugging
MAX_HEIGHT  = 600   # Temp for debugging
COLOR_CONSTS = DARK_MODE
PATH = path.dirname(__file__)


class TabWidget(qtw.QWidget):
    """
    Creates a 'TabWidget' which inherits from the 'qtw.QWidget' class. A 'TabWidget' provides 
    a tab bar and a page area that is used to display pages related to each tab. The tab bar is 
    shown above the page area. Each tab is associated with a different widget called a page. 
    Only the current page is shown in the page area; all the other pages are hidden. The user can 
    show a different page by clicking on its tab
    """

    def __init__(self, style_sheet_file: str):
        """
        Initialize tab widget

        Args:
            parent: Window where to place tabs
            style_sheet_file: Style sheet text file formatted as a CSS f-string
        """
        super().__init__()
        
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

        # Keystroke publisher
        self.keystroke_pub = None

    def add_keystroke_publisher(self, pub: Publisher):
        """
        Adds the keystroke publisher to 'MainWindow'
        """
        self.keystroke_pub = pub

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

        # FIXME: Need a better way to get keys. 
        self.countdown_widget.keyPressEvent = self.keyPressEvent

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

    def keyPressEvent(self, a0: QKeyEvent) -> None:
        """
        Called for each time there is a keystroke. Publishes the code of the key that was
        pressed or released to the ROS topic 'keystroke'
        """
        if self.keystroke_pub is not None:
            msg = Int32()
            msg.data = a0.key()
            self.keystroke_pub.publish(msg)

def fix_term():
    """
    If VS Code was installed with snap, the 'GTK_PATH' variable must be unset.
    This is automated in this function
    """
    if "GTK_PATH" in environ and "snap" in environ["GTK_PATH"]:
        environ.pop("GTK_PATH")


class DashPlugin(Plugin):

    """
    rqt_console plugin's main class. Handles communication with ros_gui and contains
    callbacks to handle incoming message
    """

    def __init__(self, context):
        """
        :param context: plugin context hook to enable adding widgets as a ROS_GUI pane,
                        ''PluginContext''
        """
        super().__init__(context)
        self.setObjectName('Seahawk')

        self._context = context
        self._widget = TabWidget(style_sheet_file=PATH + "/dash_styling/tab_widget.txt")
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

        self._context.node.create_subscription(
            InputStates, "input_states", self.callback_input_states, 10
        )
        self._context.node.create_subscription(
            DebugInfo, "debug_info", self.callback_debug, 10
        )
        self._widget.add_keystroke_publisher(
            self._context.node.create_publisher(Int32, "keystroke", 10)
        )

    def shutdown_plugin(self):
        pass

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
        self._widget.update_pilot_tab_input_states(input_state_dict)

    def callback_debug(self):
        pass

