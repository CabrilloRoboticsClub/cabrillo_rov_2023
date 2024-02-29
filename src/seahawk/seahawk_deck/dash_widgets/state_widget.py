from os import path

from PyQt5 import QtWidgets as qtw
from PyQt5 import QtGui as qtg

from seahawk_deck.dash_styling.color_palette import DARK_MODE

COLOR_CONSTS = DARK_MODE
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
        self.on_img   = qtg.QPixmap(PATH + "/../dash_styling/on_img.svg")
        self.off_img  = qtg.QPixmap(PATH + "/../dash_styling/off_img.svg")

        # Create a dictionary of label objects
        self.label_dict = {name: {"feat": qtw.QLabel(), "state": qtw.QLabel()} for name in feature_names}

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
        for i, (name, labels) in enumerate(self.label_dict.items()):
            labels["feat"].setText(name)
            labels["state"].setPixmap(self.off_img)
            # Grid layout:
            # (0, 0)    (0, 1)
            # (1, 0)    (1, 1)
            layout_inner.addWidget(labels["feat"], i, 0)
            layout_inner.addWidget(labels["state"], i, 1)

        # Apply css styling
        with open(style_sheet_file) as style_sheet:
            self.setStyleSheet(style_sheet.read().format(**COLOR_CONSTS))

    def update(self, msg):
        """
        Update graphical representation of the feature state

        Args:
            Dictionary in which the name of the feature is the key and the state
            to update it to is the value
        """
        for feature, state_of_feature in msg.items():
            if state_of_feature:
                self.label_dict[feature]["state"].setPixmap(self.on_img)
            else:
                self.label_dict[feature]["state"].setPixmap(self.off_img)       
