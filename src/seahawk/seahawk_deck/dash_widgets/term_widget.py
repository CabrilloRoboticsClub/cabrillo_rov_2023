import os
import sys
import shlex

from PyQt5 import QtWidgets as qtw
from PyQt5 import QtCore as qtc
from PyQt5 import QtGui as qtg


class TermWidget(qtw.QWidget):
    """
    Creates a `TermWidget` which inherits from the `qtw.QWidget` class. A `TermWidget`
    provides an embedded terminal. Must be put on a thread.
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

        with open(style_sheet_file) as style_sheet:
            self.style_sheet = style_sheet.read()

        # The QProcess class is used to start external programs and to communicate with them. 
        self.proc = qtc.QProcess(self)
        self.proc.setProcessChannelMode(qtc.QProcess.MergedChannels)
        self.proc.readyRead.connect(self.read_and_display_cmd_feedback)
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

        # For tracking command history
        self.cmd_history = [] # List of previously ran commands
        self.cmd_history_tracker = 0

        # Feedback window for displaying command results
        self.feedback = qtw.QTextEdit()
        self.feedback.setReadOnly(True)

        # Displays the path of working directory
        self.prompt = qtw.QLabel()
        self.get_prompt()

        # Command line window for the user to enter commands
        self.cmd_line = qtw.QPlainTextEdit()
        self.cmd_line.setLineWrapMode(qtw.QPlainTextEdit.NoWrap)
        self.cmd_line.setFocus()
        self.cmd_line.installEventFilter(self)  # Filter command line events (see eventFilter())
        self.curs = self.cmd_line.textCursor()

        # Add widgets to layout
        layout_inner.addWidget(self.feedback, stretch=95)
        layout_inner.addWidget(self.prompt, stretch=1)
        layout_inner.addWidget(self.cmd_line, stretch=4)
        
        # Apply colors
        self.set_colors(colors)

    def get_prompt(self):
        """
        Get current directory path and display it.
        """
        # HTML makes the text highlighted and bold
        self.prompt.setText(f'<span style="background-color:#ff5900; font-weight:bold">{str(os.getcwd()) + " "}</span>')
    
    def del_cmd(self):
        """
        Deletes all text present in the `cmd_line` textbox.
        """
        self.cmd_line.setPlainText("")
        self.curs.movePosition(qtg.QTextCursor.End, 0)
        self.cmd_line.setFocus()
    
    def read_and_display_cmd_feedback(self):
        """
        Reads feedback from command entered in process and appends it to feedback window.
        """
        try:
            text = str(self.proc.readAll(), encoding = "utf8").strip()
        except TypeError:
            text = str(self.proc.readAll()).strip()
        self.feedback.append(text)
    
    def run_cmd(self, command=None):
        """
        Runs the command provided by the argument or present in the command line. The command
        ran and its results are appended to the feedback window, with formatting. If the
        command is not found, the user is notified with a message. Command is appended
        to the command history list.

        Args:
            command: Command to run (optional).
        """
        self.feedback.setFocus()
        # If provided an argument, use that, otherwise grab the text from the command line
        cmd_txt = command if command else self.cmd_line.toPlainText()
        # Split `cmd_txt` into commands with shell-like syntax
        cmd_list = shlex.split(cmd_txt, posix=False)
        cmd = str(cmd_list[0]).casefold()   # First index holds the actual command
        cmd_args = " ".join(cmd_list[1:])   # The rest of the list contains the arguments

        # HTML formatting strings for colored text
        WARNING     = '<span style="color:#fc3019;">{}</span>'
        SUCCESS     = '<span style="color:#2e933c;">{}</span>'
        CMD_NAME    = '<span style="color:#afc97e; font-weight:bold;">{}</span>'

        match (cmd):
            case "exit":
                sys.exit()
            case "clear":  # Clear feedback window contents
                self.feedback.setText("")
            case "cd":  # Change directories
                # Format command to string as "❯ cmd cmd-args" with colored text
                text = SUCCESS.format(u"\n\u276F ") + CMD_NAME.format(cmd) + " " + cmd_txt.partition(" ")[2]
                self.feedback.append(text)
                # Change directory
                os.chdir(os.path.abspath(cmd_args))
                self.proc.setWorkingDirectory(os.getcwd())
                # Update prompt representation of path
                self.get_prompt()
            case "history":  # Display previous commands ran in this terminal session with indexes
                self.feedback.append(SUCCESS.format(u"\n\u276F ") + CMD_NAME.format("history"))
                for index, token in enumerate(self.cmd_history):
                    self.feedback.append(f"{index:<4}{token}")
            case _:
                if qtc.QStandardPaths.findExecutable(cmd):  # If command is one of the executable commands
                    text = SUCCESS.format(u"\n\u276F ") + CMD_NAME.format(cmd) + " " + cmd_txt.partition(" ")[2]
                    self.feedback.append(text)
                    if self.proc.state() != qtc.QProcess.Running:
                        self.proc.start(cmd + " " + cmd_args)
                else:  # Otherwise command is not executable, display error
                    self.feedback.append(WARNING.format(u"\n\u276F ") + f"Command not found: {cmd_txt}")
        # Add command to history
        self.cmd_history.append(cmd_txt)
        self.cmd_history_tracker = len(self.cmd_history)
        # Delete what is currently in the command line text box to get ready for new command
        self.del_cmd()

    def eventFilter(self, a0: qtc.QObject, a1: qtc.QEvent) -> bool:
        """
        Filters and handles events. Makes terminal behave as expected when certain keys are pressed.

        Key press:          Functionality:           
        - return/enter:     Run command
        - up-arrow:         Scroll up through command history
        - down-arrow:       Scroll down through command history
        """
        if a0 == self.cmd_line: # If the object the event is from is the command line
            if (a1.type() == qtc.QEvent.KeyPress):  # Use pressed return, run command.
                if a1.key() == qtc.Qt.Key_Return:
                    self.run_cmd()
                    return True
                elif a1.key() == qtc.Qt.Key_Up:     # Scroll up in cmd history
                    self.cmd_history_tracker = self.cmd_history_tracker - 1 if self.cmd_history_tracker >= 0 else len(self.cmd_history) - 1
                    self.cmd_line.setPlainText(self.cmd_history[self.cmd_history_tracker])
                    # TODO: Move cursor to end of line
                    return True
                elif a1.key() == qtc.Qt.Key_Down:   # Scroll down in cmd history
                    # TODO: Invert logic
                    self.cmd_history_tracker = self.cmd_history_tracker - 1 if self.cmd_history_tracker >= 0 else len(self.cmd_history) - 1
                    self.cmd_line.setPlainText(self.cmd_history[self.cmd_history_tracker])
                    # Scroll down in cmd history
                    return True

                # TODO: Quit process with ctr-c
                # TODO: Copy with ctrl-shift-c
                # TODO: Paste with ctrl-shift-p
                # TODO: Terminal shortcuts (ctrl-u, ctrl-a, ctrl-e...)

                # elif a1.key() == qtc.Qt.Key_C and qtc.Qt.ControlModifier and qtc.Qt.ShiftModifier:  # Copy
                #     # Copy selected text
                #     return True
                # elif qtc.Qt.ControlModifier and a1.key() == qtc.Qt.Key_C:   # Quit process
                #     print(qtc.Qt.ControlModifier)
                #     print("quits")
                #     # Quit process
                #     if self.proc.state() != qtc.QProcess.Running:
                #         self.proc.terminate()
                #     return True 
                
        return False


    def set_colors(self, new_colors: dict):
        """
        Sets widget colors given a dictionary of hex color codes.

        Args:
            new_colors: Hex codes to color widget with.
        """
        self.setStyleSheet(self.style_sheet.format(**new_colors)) 
