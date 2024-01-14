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
MAIN_WIN_BKG        = "rgb(15, 15, 15)" 
INACTIVE_TAB_BKG    = "rgb(20, 20, 20)"
INACTIVE_TAB_BD     = "rgb(30, 32, 37)"
ACTIVE_TAB_BKG      = "rgb(24, 24, 24)"
ACCENT_LIGHT        = "rgb(87, 134, 252)"
PRIMARY_TEXT        = "rgb(255, 240, 245)"
SECONDARY_TEXT      = "rgb(132,148,164)"


class MainWindow(qtw.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SeaHawk II Dashboard")
        self.setStyleSheet(f"background-color: {MAIN_WIN_BKG};")
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
        self.__tab_dict =  {"Home":             qtw.QWidget(),  # Index 0
                            "Debug":            qtw.QWidget(),  # Index 1
                            "Cameras":          qtw.QWidget(),  # Index 2
                            "User Control Map": qtw.QWidget()   # Index 3
                            }

        # Track index of the current opened tab
        # self.__prev_tab_index = 0

        # Add tabs
        for name, tab in self.__tab_dict.items():
            self.__tabs.addTab(tab, name)

        stylesheet = f"""
        /* Modifies the open tab body style*/
        QTabWidget>QWidget>QWidget{{
            background: {ACTIVE_TAB_BKG};
            border: 2px solid {ACCENT_LIGHT};
            border-top-right-radius:    10px;
            border-bottom-left-radius:  10px;
            border-bottom-right-radius: 10px;
        }}

        /* Modifies all tab chips */
        QTabBar::tab {{
            border: 2px solid {INACTIVE_TAB_BD};
            color:  {SECONDARY_TEXT};
            border-top-left-radius:     8px;
            border-top-right-radius:    8px;
        }}

        /* Modifies selected tab chip*/
        QTabBar::tab:selected {{
            background: {ACCENT_LIGHT};
            border: 2px solid {ACCENT_LIGHT};
            color:  {PRIMARY_TEXT}
        }}

        /* Modifies unselected selected tabs  */
        QTabBar::tab:!selected {{
            background: {INACTIVE_TAB_BKG};  
        }}

        /* Modifies tab when mouse is hovered over it */
        QTabBar::tab:!selected:hover {{
            color: {PRIMARY_TEXT}
        }}

        /* Makes tab "jump up" when pressed */
        QTabBar::tab:top:!selected {{
            margin-top: 3px;
        }}

        /* Makes tab "jump up" when pressed */
        QTabBar::tab:bottom:!selected {{
            margin-bottom: 3px;
        }}

        /* Padding around text */
        QTabBar::tab:top, QTabBar::tab:bottom {{
            min-width: 8ex;
            margin-right: -1px;
            padding: 5px 10px 5px 10px;
        }}

        /* Prevents "border overflow" */
        QTabBar::tab:left:selected {{
            border-left-color: none;
        }}

        /* Prevents "border overflow" */
        QTabBar::tab:right:selected {{
            border-right-color: none;
        }}

        """

        self.setStyleSheet(stylesheet)
        
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