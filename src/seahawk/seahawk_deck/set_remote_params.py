"""
set_remote_params.py

Abstracts requesting to set another node's parameters via service to class
'SetRemoteParams'. To import this class to a program on the deck use 
'from .set_remote_params import SetRemoteParams'

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
# ROS client library
from rclpy.node import Node 

# Type hints
from typing import Any

# Parameters
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

class RemoteParamsClient():
    """
    Abstracts requesting to set another node's parameters via service to a class. 
    """
    def __init__(self, this_node: Node, other_node_name: str):
        """
        Set up request to set another node's parameters via service named '/other_node_name/set_parameters'
    
        Args:
            this_node: Node which makes the request to set another node's parameters. Client node
            other_node_name: Name of the node whose parameters should be set. Service node
        """
        # Create service name. Services to set parameters must be named as '/other_node_name/set_parameters'
        srv_name = "/" + other_node_name + "/set_parameters"

        # Set up client to remotely set parameters of another node using a service
        self.__cli = this_node.create_client(SetParameters, srv_name)

        # Try to connect to the service (wait one second between attempts)
        while not self.__cli.wait_for_service(timeout_sec=1.0):
            this_node.get_logger().info(f"{srv_name} service not available, waiting again...")
        
        # Create a set parameter request object
        self.__req = SetParameters.Request()

        # List to store parameters before sending them
        self.__param_list = []

    def update_params(self, param_name: str, param_value: Any):
        """
        Appends a single new parameter given its name and new value to the list of parameters to set

        Args: 
            param_name: Name of the parameter
            param_value: Value of the parameter (note must of a be a valid parameter type)
        """
        self.__param_list.append(Parameter(name=param_name, value=param_value).to_parameter_msg())
	
    def send_params(self) -> (Any | None):
        """
        Sends updated parameter list to the service

        Returns:
            The result set by the task, None if no result was set
        """
        self.__req.parameters = self.__param_list   # Create updated param list
        future = self.__cli.call_async(self.__req)  # Send param to service
        self.__param_list.clear()                   # Reset list
        return future.result()                      # Return if the parameter is set correctly


class RemoteParamsServiceWrapper():
    """
    Abstracts requesting setting node's parameters from another node via service to a class.
    """
    def __init__(self, this_node: Node):
        """
        Set up request to set this node's parameters from another node

        Args:
            this_node: Name of the node whose parameters should be set. Service node
        """
        self.__this_node = this_node
        # Call parameter callback
        self.__this_node.add_on_set_parameters_callback(self.__set_params)
    
    def __set_params(self, params: list[Parameter]) -> SetParametersResult:
        """
        Callback for parameter update. Updates all local parameters to have values set by the service

        Args:
            params: List of updated parameters

        Returns:
            SetParametersResult() which lets ROS2 know if the parameters were set correctly or not
        """
        try:
            # Try to set parameters to updated values
            self.__this_node.set_parameters(params)
        except:
            return SetParametersResult(successful=False)
        return SetParametersResult(successful=True)