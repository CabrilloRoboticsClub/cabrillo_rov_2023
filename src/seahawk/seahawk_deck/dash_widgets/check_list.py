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

        inner_layout.addStretch()

        # Creating labels for text to be displayed on
        self.title = qtw.QLabel(parent)
        self.title.setAlignment(qtc.Qt.AlignCenter)


        # Creating a label for points earned
        self.points_earned = qtw.QLabel(parent)
        self.points_earned.setAlignment(qtc.Qt.AlignCenter)
        #self.points_earned.setWordWrap(True)

        # Giving string values to the created labels
        self.title.setText("TASKS:")

        # Create a scroll area template
        scroll_area = qtw.QScrollArea()

        # Make the scroll_area resizeable with window
        scroll_area.setWidgetResizable(True)

        # Create the scroll area on the main frame
        scroll_area.setWidget(frame)

        # Add the scroll area to the outer layout
        outer_layout.addWidget(scroll_area)

        # Creating the progress bar
        self.progress_bar = qtw.QProgressBar(parent)

        # Setting dimensions of the progress bar (change this l8r)
        # (x, y, l, ?)
        self.progress_bar.setGeometry(25, 60, 200, 60)

        # Changing the dimensions, color, border, and text algnment of progress bar
        self.progress_bar.setStyleSheet("QProgressBar {border: 5px solid grey; border-radius: 10px; text-align: center;} "
                                        "QProgressBar::chunk {background-color: #9757f5; width: 30px;}")

        # Has the task as a key, and a its # of points as value
        self.task_dict = {}

        # Variable to store points of each task
        self.points = 0

        # Variable for the total points achievable for comp
        self.total_points = 0

        # Variable for keeping track of current points earned
        self.current_points = 0

        # Variable for keeping track of progress bar percentage
        self.prog_par_percent = 0

        # Used later to grab point value out of string task
        point_search = r"(\d+)pts"

        # Int variable that stores total tasks
        self.total_tasks = 0

        # Int variable that keeps track of how many tasks have been done
        self.current_tasks = 0

        # Checks if the function check_box_state was called
        self.was_check_box_called = False

        # Open json file and store it as a dictionary of dictionaries of strings and lists
        with open(task_list_file, 'r') as file:
            data_list = json.load(file)

        #inner_layout.addSpacing(100)

        outer_layout.addWidget(self.title)
        outer_layout.addWidget(self.points_earned)
        outer_layout.addWidget(self.progress_bar)
        

        for part, tasks_dict in data_list.items():
            for task_title, tasks_list in tasks_dict.items():
                if self.total_tasks != 0:
                    inner_layout.addSpacing(20)  # Space things out by 20 pixels
                self.task_titles = qtw.QLabel(parent)  # Create new task title for each task_title
                self.task_titles.setText(task_title)  # Set the task_title text
                inner_layout.addWidget(self.task_titles)  # Add the task title as a widget to the inner layout
                for task in tasks_list:
                    self.total_tasks += 1
                    spliced_task = task.split('\t')
                    task = f"{spliced_task[0]:.<70}{spliced_task[1]}"
                    self.checkBox = qtw.QCheckBox(parent)  # Create a new check box for each task
                    self.checkBox.setStyleSheet("QCheckBox::indicator {width: 20px; height: 20px;}")
                    self.checkBox.setText(task)  # Add the task text
                    inner_layout.addWidget(self.checkBox)  # Add the checkbox widget onto the inner_layout
                    match = re.search(point_search, task)  # Parse the task for its point value
                    self.points = int(match.group(1))  # Add the point value to the points variable
                    self.total_points += self.points
                    self.task_dict[task] = self.points  # Store the task and its point value in a dictionary
                    inner_layout.addSpacing(10)  # Space the check boxes out
                    self.checkBox.stateChanged.connect(self.check_box_state)  # Keep track of each checkbox state
                    self.show()  # Idk what this is but it helped format things good
                
        if self.was_check_box_called == False:
            self.points_earned.setText(f"POINTS EARNED: {self.current_points} / {self.total_points}                  TASKS COMPLETED: {self.current_tasks} / {self.total_tasks}")



    def check_box_state (self, state):
        sender = self.sender()  # Find the checkbox that was pressed

        was_check_box_called = True
        
        if state == qtc.Qt.Checked:  # If the button has been pressed
            self.current_tasks += 1
            self.current_points += self.task_dict[sender.text()]  # Add to the current score

        else:  # If the button has been de-selected
            self.current_tasks -= 1
            self.current_points -= self.task_dict[sender.text()]  # Remove from the current score

        self.points_earned.setText(f"POINTS EARNED: {self.current_points} / {self.total_points}                  TASKS COMPLETED: {self.current_tasks} / {self.total_tasks}")

        self.prog_par_percent = int(100 * (self.current_points/self.total_points))  # Get % of how many points earned

        self.progress_bar.setValue(self.prog_par_percent)  # Update progress bar to this new percentage
        
        # Uncomment later when we add the CSS
        # with open(style_sheet_file) as style_sheet:
        #     self.setStyleSheet(style_sheet.read().format(**COLOR_CONSTS))
        # test.py
