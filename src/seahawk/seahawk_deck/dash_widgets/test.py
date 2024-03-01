from os import environ
import sys

from PyQt5 import QtWidgets as qtw
from PyQt5 import QtGui as qtg
from PyQt5 import QtCore as qtc
# from PyQt5.QtMultiMedia import Qsound

# It may be either of these depending on how you run it
# from seahawk_deck.dash_widgets.check_list import CheckList
from check_list import CheckList

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
        self.setWindowTitle("Check List Test")

        task_list = CheckList(self, "tasks.json", "dummy_file.txt")
        self.setCentralWidget(task_list)

        # Display window
        self.showMaximized()


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