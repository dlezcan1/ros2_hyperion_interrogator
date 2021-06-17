import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException

from std_msgs.msg import Float64MultiArray, MultiArrayDimension

import asyncio
import numpy as np
from hyperion import AsyncHyperion


class HyperionPublisher(Node):
    def __init__(self):
        super().__init__('HyperionPublisher')
        self.signal_pub      = self.create_publisher(Float64MultiArray, 'sensor/raw', 10)
        self.get_logger().info('Publishing signals.')
        
        self.proc_signal_pub = self.create_publisher(Float64MultiArray, 'sensor/processed', 10)
        self.get_logger().info('Publishing processed signals.')
    
        # Hyperion parameters
        self.ip_address = '10.0.0.5' # default value
        self.get_params()
        
        self.connect()
        
    # __init__
    
    def connect(self):
        ''' (Re)Instantiate the Hyperion interrogator'''
        self.interrogator = AsyncHyperion(self.ip_address)
        
    # connect
    
    def get_params(self):
        ''' Read in parameters for the Hyperion interrogator '''
        # Hyperion parameter names to get
        ip_param_name = 'interrogator/ip_address'
        
        # Hyperion IP address
        if self.has_parameter(ip_param_name):
            self.ip_address = self.get_paramater(ip_param_name).get_paramater_value()
        
        # if
        
        else:
        self.get_logger().warning(("Interrogators IP address parameter,"
                                    "'{}/interrogator/ip_address' not set."
                                    " Using default IP address: {}").format(self.get_name(),self.ip_address))
                                        
        # else
        
    # get_params
    
# class: HyperionPublisher