# dash_widgets/check_list.py
from PyQt5 import QtWidgets as qtw
from PyQt5 import QtGui as qtg
from PyQt5 import QtCore as qtc

from seahawk_deck.dash_styling.color_palette import DARK_MODE
COLOR_CONSTS = DARK_MODE

import re


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

        # Variable for title of widget
        category_title = "ALL TASKS: "

        # Variable for storing sub_cat titles during txt parsing
        sub_category_title = ""

        # Variable for storing tasks during txt parsing
        task = ""

        # Creating labels for text to be displayed on
        self.category = qtw.QLabel()
        self.sub_category = qtw.QLabel()

        # Giving string values to the created labels
        self.category.setText(category_title)

        # Add variable for checkBox
        self.checkBox = qtw.QCheckBox()

        # Checks if we're in a sub category
        in_sub_cat = False
        
        # Has the task as a key, and a its # of points as value
        self.task_dict = {}

        # Variable to store points of each task
        points = 0

        # Moves through text file line by line and parses important data
        with open(task_list_file, 'r') as file:
            for line in file:
                if line == "[SUB]":
                    next(file)  # Move one line in the file
                    sub_category_title = line  # This should give the title of subcategory
                    self.sub_category.setText(sub_category_title)  # Add this as text to widget
                    in_sub_cat = True  # We are in a sub category now, so this is true
                elif line == "[END_SUB]" or line == "":  # maybe redundant?
                    in_sub_cat = False  # At the end of a sub category, so this will be false
                    # Add a couple of white spaces between chunks such as [1.1] & [2.1]
                else:
                    task = line
                    self.checkBox.setText(task)  # is this legal? and is it making and publishing a checkbox?
                    match = re.search(r'(\d+)pts', line)  # grabs the number before 'pts' in txt file
                    points = int(match.group(1))  # sets points variable equal to the value grabbed by match
                    self.task_dict[task] = points  # adds the task with associated pts to dict

       
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