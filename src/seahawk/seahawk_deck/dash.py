from PyQt5 import QtWidgets as qtw
from PyQt5 import QtGui as qtg
import sys

class MainWindow(qtw.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SeaHawk II Dashboard")
        self.setStyleSheet("background-color: #333333;")
        self.showMaximized()
    
    def setStyle(self):
        self.setStyleSheet("background-color: #333333;")

def main():
    app = qtw.QApplication([])
    mw = MainWindow()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
