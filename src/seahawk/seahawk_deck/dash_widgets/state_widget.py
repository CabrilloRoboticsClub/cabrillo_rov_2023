from os import path

from PyQt5 import QtWidgets as qtw
from PyQt5 import QtGui as qtg

from dash_styling.color_palette import COLOR_CONSTS

PATH = path.dirname(__file__)

class StateWidget(qtw.QWidget):
    """
    Creates a 'StateWidget' which inherits from the 'qtw.QWidget' class. A 'StateWidget' provides 
    a visual representation of if a feature is engaged or not
    """

    def __init__(self, parent: qtw.QWidget, feature_names: list[str], style_sheet_file: str):
        """
        Initialize feature state widget
        Args:
            parent: Widget to overlay 'StateWidget' on
            feature_names: List of feature names
            style_sheet_file: Style sheet text file formatted as a CSS f-string
        """
        super().__init__(parent)

        # Import state images
        self.__on_img   = qtg.QPixmap(PATH + "/../dash_styling/on_img.svg")
        self.__off_img  = qtg.QPixmap(PATH + "/../dash_styling/off_img.svg")
        
        # Track if feature is engaged
        self.__prev_state = {name: False for name in feature_names}

        # Create a dictionary of label objects
        self.__label_dict = {name: {"feat": qtw.QLabel(), "state": qtw.QLabel()} for name in feature_names}

        # Define layout of frame on parent
        layout_outer = qtw.QVBoxLayout(self)
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
        # If last recorded feature state was on, and the function was called because of an updated state,
        # then the graphic is updated to the off state
        if self.__prev_state[feature]: 
            self.__label_dict[feature]["state"].setPixmap(self.__off_img)
            self.__prev_state[feature] = False
        
        # If last recorded feature state was off, and the function was called because of an updated state,
        # then the graphic is updated to the on state
        else:
            self.__label_dict[feature]["state"].setPixmap(self.__on_img)
            self.__prev_state[feature] = True
