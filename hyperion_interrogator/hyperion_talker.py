import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException

from std_msgs.msg import Bool, Float64MultiArray, MultiArrayDimension
from std_srvs.srv import Trigger
from rcl_interfaces.msg import SetParametersResult

import asyncio
import numpy as np
from .hyperionlib.hyperion import Hyperion, AsyncHyperion


class HyperionPublisher(Node):
    # PARAMETER NAMES
    param_names = {
                   'ip': 'interrogator/ip_address',
                   'ref_wl': 'sensor/reference'
                  }
        
    def __init__(self):
        super().__init__('Hyperion')
        
        # Hyperion parameters
        self.declare_parameter(HyperionPublisher.param_names['ip'], '10.0.0.5')
        self.declare_parameter(HyperionPublisher.param_names['ref_wl'])
        self.get_params()
        self.add_on_set_parameters_callback(self.parameter_callback) # update parameters
        
        # connect to Hyperion Interrogator
        self.connect()
        
        # setup the publishers
        self.start_publishers()
        
        # setup services
        self.start_services()
        
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
            self.num_chs = self.interrogator.channel_count
            
        # try
        except OSError:
            self.get_logger().warning("Interrogator is not configured to a proper IP address")
            self.is_connected = False
            
        # except
        
    # connect
    
    def get_params(self):
        ''' Read in parameters for the Hyperion interrogator '''
        # Hyperion IP address
        
        self.ip_address = self.get_parameter(HyperionPublisher.param_names['ip']).get_parameter_value().string_value
        self.get_logger().info("Connecting to IP: {}".format(self.ip_address))
        
        self.ref_wavelengths = self.get_parameter(HyperionPublisher.param_names['ref_wl']).get_parameter_value().double_array_value
        self.get_logger().info(f"Reference wavelengths detected: {self.ref_wavelengths}")
        
    # get_params
    
    def parameter_callback(self, params):
        ''' Parameter update call back function'''
        for param in params:
            if param.name == HyperionPublisher.param_names['ip']:
                self.ip_address = param.string_value
                
            # if
            
            elif param.name == HyperionPublisher.param_names['ref_wl']:
                self.ref_wavelengths = param.double_array_value
                
            # elif
            
        # for
        
        return SetParameterResult(successful=True)
        
    # parameter_callback
    
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
            
        # publish the connection status
        self.connected_pub.publish(Bool(data=self.is_connected))
        
        if self.is_connected and self.interrogator.is_ready:
            peaks = parse_peaks(self.interrogator.peaks)
            
            # prepare total message
            raw_tot_msg = Float64MultiArray()
            proc_tot_msg = Float64MultiArray()
            
            raw_tot_msg.layout.dim = []
            proc_tot_msg.layout.dim = []
            raw_tot_msg.data = []
            proc_tot_msg.data = []

            for ch_num, data in peaks.items():
                # grab the channel publishers
                raw_pub = self.signal_pubs[ch_num]['raw']
                proc_pub = self.signal_pubs[ch_num]['processed']
                
                # prepare the message
                raw_msg = Float64MultiArray()
                proc_msg = Float64MultiArray()
                
                raw_msg.layout.dim.stride = data.dtype.itemsize
                raw_msg.layout.dim.size = data.size
                raw_msg.data = data.flatten().tolist()
                
                proc_msg.layout.dim.stride = data.dtype.itemsize
                proc_msg.layout.dim.size = data.size
                proc_msg.data = data.flatten().tolist() # TODO: NEED TO DO ACTUAL PROCESSING
            
                # add the message to the total message
                raw_tot_msg.layout.dim.append(MultiArrayDimension(label=f"CH{ch_num}",
                                                                  size=raw_msg.size, stride=data.dtype.itemsize))
                proc_tot_msg.layout.dim.append(MultiArrayDimension(label=f"CH{ch_num}",
                                                                   size=proc_msg.size, stride=data.dtype.itemsize))
                
                raw_tot_msg.data.append(raw_msg.data)
                proc_tot_msg.data.append(proc_msg.data)
            
                # publish the messages
                raw_pub.publish(raw_msg)
                proc_pub.publish(proc_msg)
                
            # for
            
            # publish the entire signal
            self.signal_pubs['all']['raw'].publish(raw_tot_msg)
            self.signal_pubs['all']['processed'].publish(proc_tot_msg)
            
        # if
        
        else:
            pass
            
        # else
        
    # publish_peaks
    
    def reconnect_service(self, request, response):
        ''' reconnect to the IP address '''
        self.get_logger().info("Reconnecting to Hyperion interrogator...")
        self.connect()
        self.start_publishers()
        
        response.success = self.is_connected
        
        if not self.is_connected:
            response.message = "{} is not a valid IP for the Hyperion interrogator".format(self.ip_address)
            
        # if
        
        return response
        
    # reconnect_service
    
    def start_publishers(self):
        ''' Method to instantiate the publishers for the class'''
        
        # connected publisher
        self.connected_pub = self.create_publisher(Bool, 'interrogator/connected', 10)
        self.signal_pubs = {}
        self.signal_pubs['all'] = {'raw':       self.create_publisher(Float64MultiArray, 'sensor/raw', 10),
                                   'processed': self.create_publisher(Float64MultiArray, 'sensor/processed', 10)}
        if self.is_connected:
            topic_raw = '/sensor/CH{:d}/raw'
            topic_proc = '/sensor/CH{:d}/processed'
            for idx in range(1, self.num_chs + 1):
                ch_pub = {}
                ch_pub['raw'] = self.create_publisher(Float64MultiArray, topic_raw.format(idx), 10)
                ch_pub['processed'] = self.create_publisher(Float64MultiArray, topic_proc.format(idx), 10)
                self.signal_pubs[idx] = ch_pub
                
            # idx
            
            self.get_logger().info('Publishing raw and processed signals.')
            
        # if
        
        else:
            self.get_logger().warning("FBG Publishers not configured since interrogator not connected.")
            
        # else
        
    # start_publishers
    
    def start_services(self):
        ''' Method to instantiate the services for this node '''
        self.reconnect_srv = self.create_service(Trigger, '{}/interrogator/reconnect'.format(self.get_name()), 
                                self.reconnect_service)
        
    # start_services
    
    
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