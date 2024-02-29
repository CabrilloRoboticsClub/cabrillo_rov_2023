from PyQt5 import QtWidgets as qtw

from seahawk_deck.dash_styling.color_palette import DARK_MODE

COLOR_CONSTS = DARK_MODE

class NumericDataWidget(qtw.QWidget):
    """
    Creates a 'NumericDataWidget' which inherits from the 'qtw.QWidget' class. A 'NumericDataWidget'
    displays numeric data
    """

    def __init__(self, parent: qtw.QWidget, sensor_name: str, style_sheet_file: str):
        """
        Initialize numeric display widget
        
        Args:
            parent: Widget to overlay 'NumericDataWidget' on
            style_sheet_file: Style sheet text file formatted as a CSS f-string
        """
        super().__init__(parent)

        # Define layout of frame on parent
        layout_outer = qtw.QVBoxLayout(self)
        self.setLayout(layout_outer)

        # Create frame widget
        frame = qtw.QFrame()
        layout_outer.addWidget(frame)

        # Set layout of labels on frame to grid
        layout_inner = qtw.QVBoxLayout(frame)
        frame.setLayout(layout_inner)

        # Create label widgets
        self.header = qtw.QLabel()
        self.numeric_data = qtw.QLabel()

        # Set text on widget 
        self.header.setText(sensor_name)
        self.numeric_data.setText("n/a")

        # Set an accessible name for each 
        self.header.setAccessibleName("name")
        self.numeric_data.setAccessibleName("data")
        layout_inner.addWidget(self.header)
        layout_inner.addWidget(self.numeric_data)
       
        with open(style_sheet_file) as style_sheet:
            self.setStyleSheet(style_sheet.read().format(**COLOR_CONSTS))
        
    def update(self, data: str):
        """
        Update data displayed by widget

        Args:
            data: New data to display
        """
        self.numeric_data.setText(data)
