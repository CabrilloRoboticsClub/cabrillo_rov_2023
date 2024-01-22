from PyQt5 import QtWidgets as qtw

from dash_styling.color_palette import COLOR_CONSTS

class TurnBankIndicator(qtw.QWidget):
    """
    Creates a 'NumericDataWidget' which inherits from the 'qtw.QWidget' class. A 'NumericDataWidget'
    displays numeric data
    """

    def __init__(self, parent: qtw.QWidget, style_sheet_file: str=None):
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

        # Apply css styling
        with open(style_sheet_file) as style_sheet:
            self.setStyleSheet(style_sheet.read().format(**COLOR_CONSTS))