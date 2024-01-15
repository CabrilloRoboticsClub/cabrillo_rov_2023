# Run 'unset GTK_PATH' in new terminals if installed vs code with snap

from PyQt5 import QtWidgets as qtw
from PyQt5 import QtGui as qtg
from PyQt5 import QtCore as qtc
import sys

# Size constants
# MAX_WIDTH   = 1862
# MAX_HEIGHT  = 1053
MAX_WIDTH   = 1000  # Temp for debugging
MAX_HEIGHT  = 600  # Temp for debugging

# Color constants
COLOR_CONSTS = {
    "MAIN_WIN_BKG"      : "#0c0c0f",
    "INACTIVE_TAB_BKG"  : "#141416",
    "INACTIVE_TAB_BD"   : "#2d2938",
    "ACTIVE_TAB_BKG"    : "#111113",
    "SURFACE_DRK"       : "#18181b",
    "SURFACE_BRIGHT"    : "#34343c",
    "ACCENT_LIGHT"      : "#9172f8",
    "ACCENT_DRK"        : "#5c4ba3",
    "PRIMARY_TEXT"      : "#fff0f5",
    "SECONDARY_TEXT"    : "#8f8c95",
}


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
        self.setGeometry(0, 0, MAX_WIDTH, MAX_HEIGHT)
        
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
        
        # Display feature state node
        feat_state_widget = FeatStateWidget(tab_dict["Home"], ["Bambi Mode", "Claw", "CoM Shift"], "dash_styling/feat_state_widget.txt")
        feat_state_widget.resize(180, 150) # FIXME: This should probably not be a fixed value

        # What to do when a tab is clicked
        # self.__tabs.currentChanged.connect(self.__on_click)

    # @qtc.pyqtSlot()
    # def __on_click(self):

class FeatStateWidget(qtw.QWidget):
    """
    Creates a 'FeatStateWidget' which inherits from the 'qtw.QWidget' class. A 'FeatStateWidget' provides 
    a visual representation of if a feature is engaged or not
    """

    def __init__(self, parent: qtw.QWidget, feature_names: list[str], style_sheet_file: str):
        """
        Initialize feature state widget
        Args:
            parent: Widget to overlay 'FeatStateWidget' on
            feature_names: List of feature names
            style_sheet_file: Style sheet text file formatted as a CSS f-string
        """
        super().__init__(parent)

        # Import state images
        self.__on_img   = qtg.QPixmap("dash_styling/on_img.svg")
        self.__off_img  = qtg.QPixmap("dash_styling/off_img.svg")
        
        # Track if feature is engaged
        self.__feat_engaged = {name: False for name in feature_names}

        # Create a dictionary of label objects
        self.__label_dict = {name: {"feat": qtw.QLabel(), "state": qtw.QLabel()} for name in feature_names}

        # Define layout of frame on parent
        layout_outer = qtw.QGridLayout(self)
        self.setLayout(layout_outer)

        # Create frame widget
        frame = qtw.QFrame()
        layout_outer.addWidget(frame)

        # Set layout of labels on frame to grid
        layout_inner = qtw.QGridLayout(frame)
        frame.setLayout(layout_inner)

        # Set text of each label and add to grid
        for i, (name, labels) in enumerate(self.__label_dict.items()):
            labels["feat"].setText(name)
            labels["state"].setPixmap(self.__off_img)
            # Grid layout:
            # (0, 0)    (0, 1)
            # (1, 0)    (1, 1)
            layout_inner.addWidget(labels["feat"], i, 0)
            layout_inner.addWidget(labels["state"], i, 1)

        # Apply css styling
        with open(style_sheet_file) as style_sheet:
            self.setStyleSheet(style_sheet.read().format(**COLOR_CONSTS))

    def update_state(self, feature: str):
        """
        Update graphical representation of the feature state if the state has changed

        Args:
            The name of the feature to update
        """
        if self.__feat_engaged[feature]:
            self.__label_dict[feature]["state"].setPixmap(self.__off_img)
            self.__feat_engaged[feature] = False
        else:
            self.__label_dict[feature]["state"].setPixmap(self.__on_img)
            self.__feat_engaged[feature] = True


def main():
    app = qtw.QApplication([])
    mv = MainWindow()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()