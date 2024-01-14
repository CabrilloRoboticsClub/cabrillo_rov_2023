from PyQt5 import QtWidgets as qtw
from PyQt5 import QtGui as qtg
from PyQt5 import QtCore as qtc
# from PyQt5.QtCore import pyqtSlot as pyQtSlot
import sys

# Size constants
# MAX_WIDTH   = 1862
# MAX_HEIGHT  = 1053
MAX_WIDTH   = 500
MAX_HEIGHT  = 500

# Color constants
COLOR_CONSTS = {
    "MAIN_WIN_BKG"        : "rgb(15, 15, 15)",
    "INACTIVE_TAB_BKG"    : "rgb(20, 20, 20)",
    "INACTIVE_TAB_BD"     : "rgb(30, 32, 37)",
    "ACTIVE_TAB_BKG"      : "rgb(24, 24, 24)",
    "ACCENT_LIGHT"        : "rgb(87, 134, 252)",
    "PRIMARY_TEXT"        : "rgb(255, 240, 245)",
    "SECONDARY_TEXT"      : "rgb(132,148,164)",
}


class MainWindow(qtw.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SeaHawk II Dashboard")
        self.setStyleSheet(f"background-color: {COLOR_CONSTS['MAIN_WIN_BKG']};")
        self.setGeometry(0, 0, MAX_WIDTH, MAX_HEIGHT)
        
        self.table_widget = TabWidget(self)
        self.setCentralWidget(self.table_widget)

        self.show()

class TabWidget(qtw.QWidget):
    def __init__(self, parent):
        super().__init__(parent)
        
        # Define layout of tabs
        layout = qtw.QVBoxLayout(self)

        # Initialize tabs
        self.__tabs = qtw.QTabWidget()
        self.__tab_dict =  {
            "Home":             qtw.QWidget(),  # Index 0
            "Debug":            qtw.QWidget(),  # Index 1
            "Cameras":          qtw.QWidget(),  # Index 2
            "User Control Map": qtw.QWidget()   # Index 3
        }

        # Track index of the current opened tab
        # self.__prev_tab_index = 0

        # Add tabs
        for name, tab in self.__tab_dict.items():
            self.__tabs.addTab(tab, name)
        
        # Apply css styling
        with open("dash_css/tab_widget.txt") as style_sheet:
            self.setStyleSheet(style_sheet.read().format(**COLOR_CONSTS))
        
        # Add tabs to widget
        layout.addWidget(self.__tabs)
        self.setLayout(layout)

        # What to do when a tab is clicked
        # self.__tabs.currentChanged.connect(self.__on_click)

    # @qtc.pyqtSlot()
    # def __on_click(self):

    
class HomeDash():
    pass


class DebugDash():
    pass


def main():
    app = qtw.QApplication([])
    mv = MainWindow()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()