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
        Initialize feature state widget
        
        Args:
            parent: Widget to overlay 'StateWidget' on
        """
        super().__init__(parent)

        NUM_CURVES = 3
        self.__throttle_crv_imgs = {i : qtg.QPixmap(f"{PATH}/../dash_styling/thrt_crv_img_{i}.svg") for i in range(NUM_CURVES)}
        # ^ variable for added details to throttle curve widget

        # Define layout of frame on parent
        layout_outer = qtw.QVBoxLayout(self)
        self.setLayout(layout_outer)

        # Create frame widget
        self.frame = qtw.QFrame()
        layout_outer.addWidget(self.frame)

        # Set layout of labels on frame
        layout_inner = qtw.QVBoxLayout(self.frame)
        self.frame.setLayout(layout_inner)

        self.__label = qtw.QLabel()
        self.__label.setPixmap(self.__throttle_crv_imgs[0])

        layout_inner.addWidget(self.__label)

        self.setStyleSheet(
            f"""
            QFrame {{
                background-color: {COLOR_CONSTS['SURFACE_DRK']};
                border-radius: 8px;
            }}
            """
        )

    def update_thrt_crv(self, thrt_crv: int):
        """
        Update graphical representation of the throttle curves

        Args:
            Index of throttle curve to update (also the key you press to change it)
        """

        self.__label.setPixmap(self.throttle_crv_imgs[thrt_crv])






        
        
