import sys
import os

from PyQt5 import QtWidgets as qtw
# from PyQt5 import QtGui as qtg
# from PyQt5 import QtCore as qtc

# from seahawk_msgs.msg import InputStates
from dash_styling.color_palette import COLOR_CONSTS
from dash_widgets.numeric_data_widget import NumericDataWidget
from dash_widgets.state_widget import StateWidget
from dash_widgets.throttle_curve_widget import ThrtCrvWidget


# Size constants
# MAX_WIDTH   = 1862
# MAX_HEIGHT  = 1053
MAX_WIDTH   = 1000  # Temp for debugging
MAX_HEIGHT  = 600  # Temp for debugging


# Run 'unset GTK_PATH' in new terminals if installed vs code with snap
if "GTK_PATH" in os.environ:
    if "snap" in os.environ["GTK_PATH"]:
        os.environ.pop("GTK_PATH")

PATH = os.path.dirname(__file__)

class MainWindow(qtw.QMainWindow):
    """
    Creates a 'MainWindow' which inherits from the 'qtw.QMainWindow' class. 'MainWindow'
    provides the main application window space to overlay widgets
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
        tab_widget = TabWidget(self, ["Home", "Debug", "Cameras", "Control Mapping"], "dash_styling/tab_widget.txt")
        self.setCentralWidget(tab_widget)

        # Display window
        # self.show()
        self.showMaximized() # Uncomment in final


class TabWidget(qtw.QWidget):
    """
    Creates a 'TabWidget' which inherits from the 'qtw.QWidget' class. A 'TabWidget' provides 
    a tab bar and a page area that is used to display pages related to each tab. The tab bar is 
    shown above the page area. Each tab is associated with a different widget called a page. 
    Only the current page is shown in the page area; all the other pages are hidden. The user can 
    show a different page by clicking on its tab
    """

    def __init__(self, parent: MainWindow, tab_names: list[str], style_sheet_file: str):
        """
        Initialize tabs
        Args:
            parent: Window where to place tabs
            tab_names: List of tab names
            style_sheet_file: Style sheet text file formatted as a CSS f-string
        """
        super().__init__(parent)
        
        # Define layout of tabs
        layout = qtw.QVBoxLayout(self)
        self.setLayout(layout)

        # Initialize tabs
        tabs = qtw.QTabWidget()

        # Create a dict in which the key is the provided name of the tab, and the value is a qtw.QWidget() object
        tab_dict = {name: qtw.QWidget() for name in tab_names}

        # Add tabs
        for name, tab in tab_dict.items():
            tabs.addTab(tab, name)
        
        # Apply css styling
        with open(style_sheet_file) as style_sheet:
            self.setStyleSheet(style_sheet.read().format(**COLOR_CONSTS))
        
        # Add tabs to widget
        layout.addWidget(tabs)
        
        # Display feature state widget
        feat_state_widget = StateWidget(tab_dict["Home"], ["Bambi Mode", "Claw", "CoM Shift"], PATH + "/dash_styling/state_widget.txt")
        feat_state_widget.resize(180, 150) # FIXME: This should probably not be a fixed value
        # feat_state_widget.update_state("Claw")

        # Display throttle curve widget
        thrt_crv_widget = ThrtCrvWidget(tab_dict["Home"])
        thrt_crv_widget.move(0, 140)
        thrt_crv_widget.resize(180, 150)

        sensor_widget = NumericDataWidget(tab_dict["Home"], "Temperature", PATH + "/dash_styling/numeric_data_widget.txt")
        sensor_widget.move(0, 280)
        sensor_widget.resize(180, 150)

        sensor_widget = NumericDataWidget(tab_dict["Home"], "Depth", PATH + "/dash_styling/numeric_data_widget.txt")
        sensor_widget.move(0, 420)
        sensor_widget.resize(180, 150)

        # What to do when a tab is clicked
        # self.__tabs.currentChanged.connect(self.__on_click)

    # @qtc.pyqtSlot()
    # def __on_click(self):

def main():
    app = qtw.QApplication([])
    mv = MainWindow()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()