from PyQt5 import QtWidgets as qtw
# from PyQt5 import QtGui as qtg
# from PyQt5 import QtCore as qtc
import sys

# Size constants
# MAX_WIDTH   = 1862
# MAX_HEIGHT  = 1053
MAX_WIDTH   = 1000  # Temp for debugging
MAX_HEIGHT  = 600  # Temp for debugging

# Color constants
COLOR_CONSTS = {
    "MAIN_WIN_BKG"        : "#0f0f0f",
    "INACTIVE_TAB_BKG"    : "#141414",
    "INACTIVE_TAB_BD"     : "#2d2938",
    "ACTIVE_TAB_BKG"      : "#181818",
    "ACCENT_LIGHT"        : "#9171f8",
    "PRIMARY_TEXT"        : "#fff0f5",
    "SECONDARY_TEXT"      : "#8f8c95",
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
        self.__tab_widget = TabWidget(self, ["Home", "Debug", "Cameras", "Control Mapping"], "dash_styling/tab_widget.txt")
        self.setCentralWidget(self.__tab_widget)

        # Display window
        self.show()
        # self.showMaximized() # Uncomment in final


class TabWidget(qtw.QWidget):
    """
    Creates a 'TabWidget' which inherits from the 'qtw.QWidget' class. A 'TabWidget' provides 
    a tab bar and a page area that is used to display pages related to each tab. The tab bar is 
    shown above the page area. Each tab is associated with a different widget called a page. 
    Only the current page is shown in the page area; all the other pages are hidden. The user can 
    show a different page by clicking on its tab
    """

    def __init__(self, parent: MainWindow, tab_list: list[str], style_sheet_file: str):
        """
        Initialize tabs
        Args:
            parent: Window where to place tabs
            tab_list: List of tab names
            style_sheet_file: Style sheet text file formatted as a CSS f-string
        """
        super().__init__(parent)
        
        # Define layout of tabs
        self.__layout = qtw.QVBoxLayout(self)

        # Initialize tabs
        self.__tabs = qtw.QTabWidget()

        # Create a dict in which the key is the provided name of the tab, and the value is a qtw.QWidget() object
        self.__tab_dict = {name: qtw.QWidget() for name in tab_list}

        # Add tabs
        for name, tab in self.__tab_dict.items():
            self.__tabs.addTab(tab, name)
        
        # Apply css styling
        with open(style_sheet_file) as style_sheet:
            self.setStyleSheet(style_sheet.read().format(**COLOR_CONSTS))
        
        # Add tabs to widget
        self.__layout.addWidget(self.__tabs)
        self.setLayout(self.__layout)

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