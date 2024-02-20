from os import path

from PyQt5 import QtWidgets as qtw
from PyQt5 import QtGui as qtg

from seahawk_deck.dash_styling.color_palette import DARK_MODE


COLOR_CONSTS = DARK_MODE
PATH = path.dirname(__file__)


class ThrtCrvWidget(qtw.QWidget):
    """
    Creates a 'ThrtCrvWidget' which inherits from the 'qtw.QWidget' class. A 'ThrtCrvWidget' provides 
    a visual representation of the current chosen throttle curve
    """

    def __init__(self, parent: qtw.QWidget):
        """
        Initialize throttle curve widget
        
        Args:
            parent: Widget to overlay 'ThrtCrvWidget' on
        """
        super().__init__(parent)

        NUM_CURVES = 3
        self.throttle_crv_imgs = {str(i) : qtg.QPixmap(f"{PATH}/../dash_styling/thrt_crv_img_{i}.svg") for i in range(1, NUM_CURVES + 1)}

        # Define layout of frame on parent
        layout_outer = qtw.QVBoxLayout(self)
        self.setLayout(layout_outer)

        # Create frame widget
        self.frame = qtw.QFrame()
        layout_outer.addWidget(self.frame)

        # Set layout of labels on frame
        layout_inner = qtw.QVBoxLayout(self.frame)
        self.frame.setLayout(layout_inner)

        # Qlabel allows text and images to be displayed as widget
        self.label = qtw.QLabel()
        # Set default curve as one
        self.label.setPixmap(self.throttle_crv_imgs["1"])

        # Add widget to layout
        layout_inner.addWidget(self.label)

        self.setStyleSheet(
            f"""
            QFrame {{
                background-color: {COLOR_CONSTS['SURFACE_DRK']};
                border-radius: 8px;
            }}
            """
        )

    def update_thrt_crv(self, thrt_crv: str):
        """
        Update graphical representation of the throttle curves

        Args:
            Key of throttle curve to update
        """
        # Grab the specific throttle curve img needed to be displayed on widget
        self.label.setPixmap(self.throttle_crv_imgs[thrt_crv])
