# test.py
from os import environ, path
import sys

from PyQt5 import QtWidgets as qtw
from PyQt5 import QtGui as qtg
from PyQt5 import QtCore as qtc


DARK_MODE = {
    "MAIN_WIN_BKG"      : "#171717",
    "INACTIVE_TAB_BKG"  : "#1a1a1a",
    "INACTIVE_TAB_BD"   : "#2f2f2f",
    "ACTIVE_TAB_BKG"    : "#1a1a1a",
    "SURFACE_DRK"       : "#212121",
    "SURFACE_BRIGHT"    : "#2f2f2f",
    "ACCENT_LIGHT"      : "#ff5900",
    "PRIMARY_TEXT"      : "#fffbf0",
    "SECONDARY_TEXT"    : "#ded3b6",
    "WARNING"           : "#fc3019",
}

PATH = path.dirname(__file__)

# It may be either of these depending on how you run it
# from seahawk_deck.dash_widgets.check_list import CheckList
from term_widget import TermWidget

class MainWindow(qtw.QMainWindow):
    """
    Creates a 'MainWindow' which inherits from the 'qtw.QMainWindow' class. 'MainWindow'
    provides the main dash window space to overlay widgets
    """

    def __init__(self):
        """
        Set up the 'MainWindow'
        """
        super().__init__()

        # Set up main window
        self.setWindowTitle("Terminal")

        term = TermWidget(self, PATH + "/../dash_styling/term_widget.txt", DARK_MODE)
        self.setCentralWidget(term)

        # Display window
        self.show()


def fix_term():
    """
    If VS Code was installed with snap, the 'GTK_PATH' variable must be unset.
    This is automated in this function
    """
    if "GTK_PATH" in environ and "snap" in environ["GTK_PATH"]:
        environ.pop("GTK_PATH")


def main():
    # Setup dashboards
    fix_term()
    app = qtw.QApplication([])
    pilot_dash = MainWindow()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()