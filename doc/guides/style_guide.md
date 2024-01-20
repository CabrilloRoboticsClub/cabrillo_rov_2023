# Style Guidelines

Adhering to a standard style guide is important for code constancy across the project. Consistency in style makes code easier to read and understand by all collaborators on the project, and makes the final result more cohesive.
The [PEP 8 – Style Guide for Python Code](https://peps.python.org/pep-0008/#function-annotations) is a good place to start for information about correct style practices. Below are some specific guidelines to follow

## 1 Comments
### 1.1 File docstring
All files should begin with the following comment where `<file_name.py>` is the name of the file and `<description>` is a short summary of its purpose.
```py
"""
<file_name.py>

<Description>

Copyright (C) 2023-2024 Cabrillo Robotics Club

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

Cabrillo Robotics Club
6500 Soquel Drive Aptos, CA 95003
cabrillorobotics@gmail.com
"""
``` 
### 1.2 Class docstring
The beginning of a user defined class should contain a comment describing its purpose

**Example:**
```py
class TabWidget(qtw.QWidget):
    """
    Creates a 'TabWidget' which inherits from the 'qtw.QWidget' class. 
    A 'TabWidget' provides a tab bar and a page area that is used to display pages 
    related to each tab. The tab bar is shown above the page area. Each tab is 
    associated with a different widget called a page. Only the current page is shown 
    in the page area; all the other pages are hidden. The user can show a different 
    page by clicking on its tab
    """
    # Class definition here
```

### 1.3 Function docstring
Docstrings provide information about a user defined function. Every function should contain a doc string which adheres to the following format:
```py
"""
A short summary of the function's purpose

Args:
    arg_1: Description
    arg_2: Description

Returns:
    Description of what the function returns
"""
```
**Example:**
```py
def get_minimum_current_scalar(self, mv: list) -> float:
    """
    Returns a scalar which shows the maximum amount of thrust the robot can 
    produce for the given direction without exceeding total current (A) limits, 
    or the current (A) limit of either ESC

    Args:
        mv: The motor values in newtons that when produced will result in
            our desired twist

    Returns:
        The largest scalar we can scale those motor values by without exceeding the 
        total current (A) limit and the current limit of each ESC
    """
    # Function definition here
```
Note, the `self` param does not need to be described in the list of arguments

### 1.4 Single line comments 
Single line comments should especially be added to parts of code who's purpose is not clear upon first glance.

**Example good use of comments:**
```py
# Enable terminal raw mode. In raw mode characters are directly 
# read from and written to the device without any translation or
# interpretation by the operating system
tty.setraw(sys.stdin.fileno())
```
Explains code which may not be easily understandable without further research. 

**Example unnessicary use of comments:**
```py
# Using the addition operator, compute the sum of two numbers: 'a' and 'b'.
# Store the result of the computation in a variable called 'sum'
sum = a + b
```
Code which is self explanatory and does not need a comment

## 2 Naming conventions

### 2.1 File names
Files should be named using `snake_case`. `snake_case` is a naming convention in which spaces are replaced with underscores and all letters are lowercase.

**Example:**
```sh
pilot_input.py
```

### 2.2 Class names
Classes should be named using `PascalCase`. `PascalCase` is a naming convention in which the first letter of each word is capitalized. Note that this differs from camelCase because the first letter is also capitalized.

**Example:**
```py
class SetRemoteParams():
    # Class definition here
```

### 2.3 Function names
Functions should be named using `snake_case`. `snake_case` is a naming convention in which spaces are replaced with underscores and all letters are lowercase.

**Example:**
```py
def get_current_scalar_value(self, mv: list, limit: float) -> float:
    # Function definition here
```

### 2.4 Variable names
Variables should generally be named using `snake_case`. `snake_case` is a naming convention in which spaces are replaced with underscores and all letters are lowercase.

**Example:**
```py
coef_list = self.get_polynomial_coef(mv, limit)
```

### 2.5 Constant variable names
Constants are variables which once set, do not change. Python does not enforce constants however constants can be denoted by the naming conventions. Constant variables should be named using `SCREAMING_SNAKE_CASE`. `SCREAMING_SNAKE_CASE` is a naming convention in which spaces are replaced with underscores and all letters are uppercase.

**Example:**
```py
COLOR_CONSTS = {
    "MAIN_WIN_BKG":         "#0c0c0f",
    "INACTIVE_TAB_BKG":     "#141416",
    "INACTIVE_TAB_BD":      "#2d2938",
    "ACTIVE_TAB_BKG":       "#111113",
    "SURFACE_DRK":          "#18181b",
    "SURFACE_BRIGHT":       "#34343c",
    "ACCENT_LIGHT":         "#9172f8",
    "ACCENT_DRK":           "#5c4ba3",
    "PRIMARY_TEXT":         "#fff0f5",
    "SECONDARY_TEXT":       "#8f8c95"
}
```

### 2.6 Private class attributes
Private attributes are attributes of an object which can only be accessed and modified in the class itself. Python does not enforce private class attributes, however they can be denoted by the naming conventions. Python style standards advise using two leading underscores before a attribute name to denote it should be considered private.

**Example:**
```py
self.__track_state = 0b0000  
```
When creating a class for a ROS2 node, technically all member variables and functions should be private. However, since all attributes should be private to make the code cleaner for this project we have decided to **NOT** use leading double underscores within node classes.

**Example:**
```py
self.claw_pub = self.create_publisher(Bool, "claw_state", 10)
```

## 3 Whitespace
### 3.1 Spaces per indentation
Use 4 spaces per indentation level. See [Basic Editing: Indentation](https://code.visualstudio.com/docs/editor/codebasics#_indentation) for configuring indentation with VS Code.

### 3.2 Blank lines 
- Classes and functions should have a padding of two blank lines after their definition.
- Within a class, methods should be separated by one blank line.
- Lines with distinctly different function should be separated with a blank line

### 3.3 Continuation lines
Continuation lines are single statements, broken up onto multiple lines. Lines may be broken without disrupting syntax when the content is enclosed within `()`, `{}`, or `[]`. Note: is recommended to enclose a statement which needs to be broken with parentheses than using the continuation operator.
> Do not use a backslash for [explicit line continuation](https://docs.python.org/3/reference/lexical_analysis.html#explicit-line-joining).
>
> Instead, make use of Python’s [implicit line joining](https://docs.python.org/3/reference/lexical_analysis.html#implicit-line-joining) inside parentheses, brackets and braces. If necessary, you can add an extra pair of parentheses around an expression.

**Example:**
```py
return min([(self.MAX_FWD_THRUST / thrust) if thrust > 0
    else ((self.MAX_REV_THRUST / thrust) if thrust < 0
    else float("inf"))
    for thrust in motor_values])
```
Continuation lines should be indented another four spaces from normal indentation. Assignments of literal `set()`, `dict()` or `list()` should be broken with one element per line (see example)

**Example:**
```py
self.buttons = {
    "claw":         StickyButton(),     # a
    "bambi_mode":   StickyButton()      # b
}
```

### 3.4 Spacing of operators
There should should one space before and after operators.

**Example of good operator spacing**
```py
self.track_state = (self.track_state << 1 | cur_button_state) & 0b1111
```

**Example of poor operator spacing**
```py
self.track_state=(self.track_state<<1    | cur_button_state)&  0b1111
```

### 3.5 Spacing within collection literals
*See Section 3.3 for breaking literals onto multiple lines*

In a dictionary, put no whitespace between each key and colon, and one space before the value. If the dictionary is separated on multiple lines, add enough tabs such that values align. 

**Example:**
```py
self.buttons = {
    "claw":         StickyButton(),     # a
    "bambi_mode":   StickyButton()      # b
}
```
For sets, tuples, and lists put no whitespace between the element and following comma, and one space between the comma and following element.

**Example:**
```py
self.motor_positions = [    # [X, Y, Z] positions for each motors
    [0.19, 0.12, 0.047],    # Motor 0
    [0.19, -0.12, 0.047],   # Motor 1
    [-0.19, 0.12, 0.047],   # Motor 2
    [-0.19, -0.12, 0.047],  # Motor 3
    [0.105, 0.15, -0.038],  # Motor 4
    [0.105, -0.15, -0.038], # Motor 5
    [-0.105, 0.15, -0.038], # Motor 6
    [-0.105, -0.15, -0.038] # Motor 7
]
```

### 3.6 Type hints spacing 
For type annotations, there should be no whitespace between the variable name and colon, and one space before the type information. 

**Example:**
```py
def __init__(self, parent: MainWindow, tab_names: list[str], style_sheet_file: str):
```

### 3.7 Additional whitespace guidelines
See [PEP 8 – Style Guide for Python Code: White space in expressions and statements](https://peps.python.org/pep-0008/#whitespace-in-expressions-and-statements) for some additional guidelines regarding whitespace

## 4 Strings
### 4.1 Quote type
Python treats double and single quote the same. For consistency, opt for double quotes when working on this project. Single quotes should only be used if necessary to nest strings within f-strings or to reference variable/type names within comments.

**Examples:**
```py
# Basic usage of double quotes 
TEAM_NUM = "PN03"
```
```py
# Exception #1: Nested string within an f-string
self.setStyleSheet(
    f"""
    QFrame {{
        background-color: {COLOR_CONSTS['SURFACE_DRK']};
        border-radius: 8px;
    }}
    """
)
```
```py
# Exception #2: Reference names within a comment
"""
Creates a 'MainWindow' which inherits from the 'qtw.QMainWindow' class. 'MainWindow'
provides the main application window space to overlay widgets
"""
```

### 4.2 Use f-strings
Use [f-strings](https://docs.python.org/3/tutorial/inputoutput.html#tut-f-strings) over [C-style](https://docs.python.org/3/tutorial/inputoutput.html#old-string-formatting) strings and [`str.format()`](https://docs.python.org/3/tutorial/inputoutput.html#the-string-format-method). Formatting strings is the process of combining predefined text with variable values into a human-readable sting message. Python has multiple methods of formatting strings, but it is recommended to use f-strings.
> The combination of expressiveness, tenseness, and clarity provided by f-strings makes them the best built-in option for Python programmers. Anytime you need to format values into strings, choose f-stings over the alternatives.
When possible, in this project opt for f-strings

## 5 Imports
### 5.1 Placement
Imports should always be placed at the top of the file, just below the file docstring. 

### 5.2 Order
Imports should be ordered in the following order:
1. Standard library imports 
2. Third-party imports 
3. Modules created for the project
Each subsection should be ordered in alphabetical order and there should be a blank line between each group of imports.

### 5.3 Avoid importing more than you need
To import the `foo` package from `bar`, opt for
```py
from foo import bar
```
Rather than 
```py
import bar
```

### 5.4 Multiple imports
Imports from different modules should be placed on different lines
```py
# Do not do this
import foo, baz

# Do this instead
import foo
import baz
```
However, if importing multiple packages from the same module, the following syntax is accepted
```py
from foo import bar, qux
``` 

## 6 Miscellaneous

### 6.1 Type hints 

### 6.2 Opening files

### 6.3 Prefer enumerate over range

### 6.4 Prefer comprehensions over map() and filter()

### 6.5 Handling exceptions

# References:
1. [PEP 8 – Style Guide for Python Code](https://peps.python.org/pep-0008/)
2. [Google Python Style Guide](https://google.github.io/styleguide/pyguide.html)
3. [Effective Python, The Book: Second Edition](https://effectivepython.com/)
4. [Python Documentation: The Python Tutorial](https://docs.python.org/3/tutorial/index.html)