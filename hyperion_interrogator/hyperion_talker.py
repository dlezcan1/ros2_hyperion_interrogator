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
        
        # setup the publishers
        self.instantiate_publishers()
    
        # Hyperion parameters
        self.ip_address = '10.0.0.5' # default value
        self.get_params()
        
        # connect to Hyperion Interrogator
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
        num_ch_param_name = 'num_channels'
        num_aa_param_name = 'num_active_areas'
        
        # Hyperion IP address
        if self.has_parameter(ip_param_name):
            self.ip_address = self.get_paramater(ip_param_name).get_paramater_value()
        
        # if
        
        else:
            self.get_logger().warning(("Interrogators IP address parameter, "
                                       "'{}/{}' not set. Using default IP address: "
                                       "{}").format(self.get_name(), ip_param_name, self.ip_address))
                                        
        # else
        
        # Number of channels
        try:
            self.num_ch = self.get_paramater(num_ch_param_name).get_paramater_value()

        # try
        except:
            self.get_logger().error(("Number of channels param not set, "
                                     "'{}/{}'.").format(self.get_name(), num_ch_param_name))

        # except
        
        # Number of active areas
        try:
            self.num_aa = self.get_paramater(num_aa_param_name).get_paramater_value()

        # try
        except:
            self.get_logger().error(("Number of a param not set, "
                                     "'{}/{}'.").format(self.get_name(), num_aa_param_name))

        # except
        
        
    # get_params
    
    def instantiate_publishers(self):
        ''' Method to instantiate the publishers for the class'''
        # raw sensor data
        self.signal_pub      = self.create_publisher(Float64MultiArray, 'sensor/raw', 10)
        self.get_logger().info('Publishing signals.')
        
        # processed sensor data
        self.proc_signal_pub = self.create_publisher(Float64MultiArray, 'sensor/processed', 10)
        self.get_logger().info('Publishing processed signals.')
        
    # instantiate_publishers
    
# class: HyperionPublisher