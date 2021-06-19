import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException

from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from std_srvs.srv import Trigger

import asyncio
import numpy as np
from .hyperionlib.hyperion import Hyperion, AsyncHyperion


class HyperionPublisher(Node):
    def __init__(self):
        super().__init__('HyperionPublisher')
        
        # Hyperion parameters
        self.get_params()
        
        # connect to Hyperion Interrogator
        self.connect()
        
        # setup the publishers
        self.instantiate_publishers()
        
        # setup the publisher callback
        timer_period = 0.001
        self.timer = self.create_timer(timer_period, self.publish_peaks)
        
        
    # __init__
    
    def connect(self):
        ''' (Re)Instantiate the Hyperion interrogator'''
        # self.interrogator = AsyncHyperion(self.ip_address)
        self.interrogator = Hyperion(self.ip_address)
        
        try:
            self.interrogator.is_ready
            self.is_connected = True
            
        # try
        except OSError:
            self.get_logger.warning("Interrogator is not configured to a proper IP address")
            self.is_connected = False
            
        # except
        
    # connect
    
    def get_params(self):
        ''' Read in parameters for the Hyperion interrogator '''
        # Hyperion parameter names to get
        ip_param_name = 'interrogator/ip_address'
        
        self.declare_parameter(ip_param_name, '10.0.0.5')
        
        # Hyperion IP address
        
        self.ip_address = self.get_paramater(ip_param_name).get_paramater_value()
        self.get_logger().log("Connecting to IP: {}".format(self.ip_address))        
        
    # get_params
    
    def instantiate_publishers(self):
        ''' Method to instantiate the publishers for the class'''
        
        self.signal_pubs = {}
        topic_raw = 'sensor/CH{:d}/raw'
        topic_proc = 'sensor/CH{:d}/processd'
        for idx in range(1, self.interrogator.channel_count+1):
            ch_pub = {}
            ch_pub['raw'] = self.create_publisher(Float64MultiArray, topic_raw.format(idx), 10)
            ch_pub['processed'] = self.create_publisher(Float64MultiArray, topic_proc.format(idx), 10)
            self.signal_pubs[idx] = ch_pub
            
        # idx
        
        self.get_logger().info('Publishing raw and processed signals.')
        
    # instantiate_publishers
    
    def parse_peaks(self, peak_data):
        ''' Parse the peak data into a dict'''
        
        data = {}
        for idx in range(1, self.interrogator.channel_count+1):
            data[idx] = peak_data[idx].astype(np.float64)
            
        # for
        
        return data
        
    # parse_peaks
    
    def process_signals(self, raw_signals):
        ''' Method to perform the signal processing
        
            This includes:
                - Base wavelength shifting
                - Temperature compensation (if set)
        '''
        pass
        
    # process_signals
    
    def publish_peaks(self):
        ''' Publish the peaks on an timer '''
        
        if self.is_connected and self.interrogator.is_ready:
            peaks = parse_peaks(self.interrogator.peaks)
            
            for ch_num, data in peaks.items():
                # grab the channel publishers
                raw_pub = self.signal_pubs[ch_num]['raw']
                proc_pub = self.signal_pubs[ch_num]['processed']
                
                # prepare the message
                raw_msg = Float64MultiArray()
                proc_msg = Float64MultiArray()
                
                raw_msg.stride = data.dtype.itemsize
                raw_msg.size = data.size
                raw_msg.data = data.flatten().tolist()
                
                proc_msg.stride = data.dtype.itemsize
                proc_msg.size = data.size
                proc_msg.data = data.flatten().tolist() # TODO: NEED TO DO ACTUAL PROCESSING
            
                # publish the messages
                raw_pub.publish(raw_msg)
                proc_pub.publish(proc_msg)
                
            # for
        # if
        
        else:
            pass
            
        # else
        
    # publish_peaks
    
# class: HyperionPublisher

def main(args = None):
    rclpy.init(args=args)
    
    hyperion_talker = HyperionPublisher()
    
    try:    
        rclpy.spin(hyperion_talker)
    
    except KeyboardInterrupt: 
        pass
        
        
    # clean up
    hyperion_talker.get_logger().info('{} shutting down...'.format(hyperion_talker.get_name()))
    hyperion_talker.destroy_node()
    rclpy.shutdown()
    
# main

if __name__ == "__main__":
    main()
    
# if: main