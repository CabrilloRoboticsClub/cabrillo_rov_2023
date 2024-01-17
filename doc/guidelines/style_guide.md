# Style Guidelines

Adhering to a standard style guideline is important for code constancy across the project. Consistency in style makes code easier to read and understand by all collaborators on the project and makes the final result more cohesive.
The [Google Python Style Guide](https://google.github.io/styleguide/pyguide.html) is a good place to start for information about correct style practices. Below are some notable guidelines to follow

## 1 Comments
### 1.1 File doc string
All files should begin with the following comment where `<file_name.py>` is the name of the file and `<description>` is a short summary of its purpose.
```py
"""
<file_name.py>

<Description>

Copyright (C) 2022-2023 Cabrillo Robotics Club

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
### 1.2 Class doc string
The beginning of a user defined class should contain a comment describing its purpose
Example:
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
```

### 1.3 Function doc strings
Doc strings provide information about a user defined function. Every function should contain a doc string which adheres to the following format:
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
Example:
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
    # Function body here
```
Note, the `self` param does not need to be described in the list of arguments

### 1.4 Single line comments 
Comments should be used frequently to describe the purpose of your code. Comments should especially be added to parts of code who's purpose is not clear on first glance

Good use of comments:

Unideal use of comments:

