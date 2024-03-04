import os
import sys
import shlex

from PyQt5 import QtWidgets as qtw
from PyQt5 import QtCore as qtc
from PyQt5 import QtGui as qtg

class TermWidget(qtw.QWidget):
    """
    Creates a `TermWidget` which inherits from the `qtw.QWidget` class. A `TermWidget`
    provides an embedded terminal.
    """

    def __init__(self, parent: qtw.QWidget, style_sheet_file: str, colors: dict):
        """
        Initialize numeric display widget.
        
        Args:
            parent: Widget to overlay 'NumericDataWidget' on.
            style_sheet_file: Style sheet text file formatted as a CSS f-string.
            colors: Hex codes to color widget with.
        """
        super().__init__(parent)

        # with open(style_sheet_file) as style_sheet:
        #     self.style_sheet = style_sheet.read()

        # The QProcess class is used to start external programs and to communicate with them. 
        self.proc = qtc.QProcess(self)
        self.proc.setProcessChannelMode(qtc.QProcess.MergedChannels)
        self.proc.readyRead.connect(self.read_and_display_cmd_feedback)
        # self.proc.finished.connect(self.del_cmd)
        self.proc.setWorkingDirectory(os.getcwd())

        # Define layout of frame on parent
        layout_outer = qtw.QVBoxLayout(self)
        self.setLayout(layout_outer)

        # Create frame widget
        frame = qtw.QFrame()
        layout_outer.addWidget(frame)

        # Set layout of labels on frame to grid
        layout_inner = qtw.QVBoxLayout(frame)
        frame.setLayout(layout_inner)

        self.cmd_history = []
        self.cmd_history_tracker = 0

        self.feedback = qtw.QTextEdit()
        self.feedback.setReadOnly(True)
        
        self.prompt = qtw.QLabel()
        self.get_prompt()

        self.cmd_line = qtw.QPlainTextEdit()
        self.cmd_line.setLineWrapMode(qtw.QPlainTextEdit.NoWrap)
        self.cmd_line.setFocus()
        self.cmd_line.installEventFilter(self)
        self.curs = self.cmd_line.textCursor()

        self.del_cmd()

        layout_inner.addWidget(self.feedback, stretch=90)
        layout_inner.addWidget(self.prompt, stretch=5)
        layout_inner.addWidget(self.cmd_line, stretch=5)

        # self.set_colors(colors)

    def get_prompt(self):
        self.prompt.setText(str(os.getcwd()))
    
    def del_cmd(self):
        self.cmd_line.setPlainText("")
        self.curs.movePosition(qtg.QTextCursor.End, 0)
        self.cmd_line.setFocus()
    
    def read_and_display_cmd_feedback(self):
        try:
            text = str(self.proc.readAll(), encoding = "utf8").strip()
        except TypeError:
            text = str(self.proc.readAll()).strip()
        self.feedback.append(text)
    
    def run_cmd(self, command=None):
        self.feedback.setFocus()
        cmd_txt = command if command else self.cmd_line.toPlainText()
        cmd_list = shlex.split(cmd_txt, posix=False)
        cmd = str(cmd_list[0]).casefold()
        cmd_args = " ".join(cmd_list[1:])

        WARNING     = '<span style="color:#fc3019;">{}</span>'
        SUCCESS     = '<span style="color:#2e933c;">{}</span>'
        CMD_NAME    = '<span style="color:#afc97e; font-weight:bold;">{}</span>'

        match (cmd):
            case "exit":
                sys.exit()
            case "clear":
                self.feedback.setText("")
            case "cd":
                text = SUCCESS.format(u"\n\u276F ") + CMD_NAME.format(cmd) + " " + cmd_txt.partition(" ")[2]
                self.feedback.append(text)
                os.chdir(os.path.abspath(cmd_args))
                self.proc.setWorkingDirectory(os.getcwd())
                self.get_prompt()
            case _:
                if qtc.QStandardPaths.findExecutable(cmd):
                    text = SUCCESS.format(u"\n\u276F ") + CMD_NAME.format(cmd) + " " + cmd_txt.partition(" ")[2]
                    self.feedback.append(text)
                    if self.proc.state() != self.proc.Running:
                        self.proc.start(cmd + " " + cmd_args)
                else:
                    self.feedback.append(WARNING.format(u"\n\u276F ") + f"Command not found: {cmd_txt}")
        self.cmd_history.append(cmd_txt)
        self.del_cmd()

    def eventFilter(self, a0: qtc.QObject, a1: qtc.QEvent) -> bool:
        if a0 == self.cmd_line:
            if (a1.type() == qtc.QEvent.KeyPress):
                if a1.key() == qtc.Qt.Key_Return:
                    self.run_cmd()
                    return True
                elif a1.key() == qtc.Qt.Key_Up:
                    # Scroll up in cmd history
                    self.cmd_line.setPlainText(self.cmd_history[self.cmd_history_tracker])
                    return True
                elif a1.key() == qtc.Qt.Key_Down:
                    # Scroll down in cmd history
                    return True
                # elif a1.key() == qtc.Qt.Key_up:
                    # Do something with ctrl c
        return False


    def set_colors(self, new_colors: dict):
        """
        Sets widget colors given a dictionary of hex color codes.

        Args:
            new_colors: Hex codes to color widget with.
        """
        self.setStyleSheet(self.style_sheet.format(**new_colors)) 
