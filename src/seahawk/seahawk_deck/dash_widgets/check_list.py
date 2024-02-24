# dash_widgets/check_list.py
from PyQt5 import QtWidgets as qtw
from PyQt5 import QtGui as qtg
from PyQt5 import QtCore as qtc

# from seahawk_deck.dash_styling.color_palette import DARK_MODE
# COLOR_CONSTS = DARK_MODE

import re
import json


class CheckList(qtw.QWidget):
    """
    Creates a 'CheckList' which inherits from the 'qtw.QWidget' class. A 'CheckList'
    functions as a task checklist. Tasks and their point values are read from a
    file then listed as a check-able list. Once a task is checked off, the points for
    that task are added to a sum and graphically displayed on a progress bar
    """

    def __init__(self, parent: str, task_list_file: str, style_sheet_file: str):
        """
        Initialize checklist widget
        
        Args:
            parent: Widget to overlay 'CheckList' on
            task_list_file: File containing a list of tasks to be displayed
            style_sheet_file: Style sheet text file formatted as a CSS f-string
        """
        super().__init__(parent)

        # Create layout where widget will be mounted on parent
        outer_layout = qtw.QVBoxLayout(self)
        self.setLayout(outer_layout)

        # Add a widget to mount to the outer layout
        frame = qtw.QFrame()
        outer_layout.addWidget(frame)

        # Add a grid to the widget to add text & features
        inner_layout = qtw.QVBoxLayout(frame)
        frame.setLayout(inner_layout)

        # Creating labels for text to be displayed on
        self.title = qtw.QLabel()
        self.task_titles = qtw.QLabel()

        # Giving string values to the created labels
        self.title.setText("COMPETITION TASKS:")

        # Add variable for checkBox
        self.checkBox = qtw.QCheckBox()

        # Checks if we're in a sub category
        in_sub_cat = False

        # Has the task as a key, and a its # of points as value
        self.task_dict = {}

        # Variable to store points of each task
        points = 0

        # Used later to grab point value out of string task
        point_search = r"(\d+)pts"

        with open('tasks.json', 'r') as file:
            data_list = json.load(file)

        for part, tasks_dict in data_list.items():
            for task_title, tasks_list in tasks_dict.items():
                self.task_titles.setText(task_title)
                for task in tasks_list:
                    self.checkBox.setText(task)
                    match = re.search(point_search, task)
                    points = int(match.group(1))
                    self.task_dict[task] = points

       
        # Uncomment later when we add the CSS
        # with open(style_sheet_file) as style_sheet:
        #     self.setStyleSheet(style_sheet.read().format(**COLOR_CONSTS))
        # test.py

from os import environ
import sys

from PyQt5 import QtWidgets as qtw
from PyQt5 import QtGui as qtg
from PyQt5 import QtCore as qtc

# It may be either of these depending on how you run it
# from seahawk_deck.dash_widgets.check_list import CheckList
# from dash_widgets.check_list import CheckList

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

        # task_list = CheckList(self, "dummy_file.csv", "dummy_file.txt")
        # self.setCentralWidget(task_list)

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