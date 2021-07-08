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
                   'ip': 'interrogator.ip_address',
                   'ref_wl': 'sensor.CH{:d}.reference',
                   'num_samples': 'sensor.num_samples'
                  }
        
    def __init__(self):
        super().__init__('Hyperion')
        
        # Hyperion parameters
        self.declare_parameter(HyperionPublisher.param_names['ip'], '10.0.0.55')
        self.declare_parameter(HyperionPublisher.param_names['num_samples'], 200)
        self.ref_wavelengths = {}
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
        self.get_logger().info("Connecting to IP: {}".format(self.ip_address))
        self.interrogator = Hyperion(self.ip_address)
        
        try:
            self.interrogator.is_ready
            self.is_connected = True
            self.num_chs = self.interrogator.channel_count
            self.get_logger().info("Connected to IP: {}".format(self.ip_address))
            
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
        
        self.num_samples = self.get_parameter(HyperionPublisher.param_names['num_samples']).get_parameter_value().integer_value
        
    # get_params
    
    def parameter_callback(self, params):
        ''' Parameter update call back function'''
        for param in params:
            if param.name == HyperionPublisher.param_names['ip']:
                self.ip_address = param.get_parameter_value().string_value
                
            # if
            
            elif param.name == HyperionPublisher.param_names['num_samples']:
                self.ref_wavelengths = param.get_parameter_value().integer_value
                
            # elif
            
        # for
        
        return SetParametersResult(successful=True)
        
    # parameter_callback
    
    def parse_peaks(self, peak_data):
        ''' Parse the peak data into a dict'''
        
        data = {}
        for idx in range(1, self.interrogator.channel_count+1):
            data[idx] = peak_data[idx].astype(np.float64)
            
        # for
        
        return data
        
    # parse_peaks
    
    def process_signals(self, peak_signals, temp_comp: bool = False):
        ''' Method to perform the signal processing
        
            This includes:
                - Base wavelength shifting
                - NOTIMPLEMENTED: Temperature compensation (if set)
            
            Including 
        '''
        proc_signals = {}
        
        for ch_num, peaks in peak_signals.items():
            proc_signals[ch_num] = {}
            
            proc_signals[ch_num]['raw'] = peaks
            try:
                proc_signals[ch_num]['processed'] = peaks - self.ref_wavelengths[ch_num]
            
            # try
           
            except Exception as e:
                # self.get_logger().warning(No)
                continue
            # except
            
        # for
        
        # temperature compensation 
        if temp_comp:
            try:
                mean_shifts = np.vstack(peak_signals.values()).mean(axis=0)
                
                for ch_num in proc_signals.keys():
                    proc_signals[ch_num]['processed'] -= mean_shifts
                    
                # for
                
            except:
                self.get_logger().warning("Unable to perform temperature compensation for signal processing.")
                
            # except
            
        # if
        
        return proc_signals
        
    # process_signals
    
    def publish_peaks(self):
        ''' Publish the peaks on an timer '''
            
        # publish the connection status
        self.connected_pub.publish(Bool(data=self.is_connected))
        
        if self.is_connected and self.interrogator.is_ready:
            peaks = self.parse_peaks(self.interrogator.peaks)
            all_peaks = self.process_signals(peaks) # perform processing 
            
            # prepare total message
            raw_tot_msg = Float64MultiArray()
            proc_tot_msg = Float64MultiArray()
            
            raw_tot_msg.layout.dim = []
            proc_tot_msg.layout.dim = []
            raw_tot_msg.data = []
            proc_tot_msg.data = []

            for ch_num in all_peaks.keys():
                # grab the channel publishers
                raw_pub = self.signal_pubs[ch_num]['raw']
                proc_pub = self.signal_pubs[ch_num]['processed']
                
                # prepare the message
                raw_msg = Float64MultiArray()
                proc_msg = Float64MultiArray()
                
                # Raw data processing
                raw_data = all_peaks[ch_num]['raw']
                
                raw_dim = MultiArrayDimension(stride=raw_data.dtype.itemsize,
                                              size=raw_data.size*raw_data.dtype.itemsize)

                raw_msg.layout.dim.append(raw_dim)
                raw_msg.data = raw_data.flatten().tolist()
                
                # processed data
                if 'processed' in all_peaks[ch_num].keys():
                    proc_data = all_peaks[ch_num]['processed']
                    
                    proc_dim = MultiArrayDimension(stride=proc_data.dtype.itemsize,
                                                   size=proc_data.size*proc_data.dtype.itemsize)
                    proc_msg.layout.dim.append(proc_dim)
                    proc_msg.data = proc_data.flatten().tolist()
            
                # if 
            
                # add the message to the total message
                raw_tot_msg.layout.dim.append(MultiArrayDimension(label=f"CH{ch_num}",
                                                                  size=raw_msg.layout.dim[0].size, 
                                                                  stride=raw_msg.layout.dim[0].stride))
                if 'processed' in all_peaks[ch_num].keys():
                    proc_tot_msg.layout.dim.append(MultiArrayDimension(label=f"CH{ch_num}",
                                                                       size=proc_msg.layout.dim[0].size, 
                                                                       stride=proc_msg.layout.dim[0].stride))
                # if
                
                raw_tot_msg.data += raw_msg.data
                
                if 'processed' in all_peaks[ch_num].keys():
                    proc_tot_msg.data += proc_msg.data
                    
                # if
            
                # publish the channel messages
                raw_pub.publish(raw_msg)
                if 'processed' in all_peaks[ch_num].keys():
                    proc_pub.publish(proc_msg)
                    
                # if
                
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
    
    def ref_wl_service(self, request, response):
        ''' Service to get the reference wavelength '''
        if self.is_connected and self.interrogator.is_ready:
            self.get_logger().info(f"Starting to recalibrate the sensors wavelengths for {self.num_samples} samples.")
            data = {}   
            for i in range(self.num_samples):
                try:
                    signal = parse_peaks(self.interrogator.peaks)
                    
                    for ch_num, peaks in signal.items():
                        if ch_num not in data.keys():
                            data[ch_num] = peaks
                            
                        else:
                            data[ch_num] += peaks
                            
                    # for
                # try
                except:
                    pass
                    
                # except
                
            # for
            
            # divide out
            for ch_num, agg_peaks in data.items():
                self.ref_wavelengths[ch_num] = agg_peaks/self.num_samples
            # for
            
            response.success = True
        
        # if
        else:
            response.success = False
            response.message = "Interrogator was not able to gather peaks. Check connection and IP address."
            
        # else
            
        return response
        
    # ref_wl_service
    
    def start_publishers(self):
        ''' Method to instantiate the publishers for the class'''
        
        # connected publisher
        self.connected_pub = self.create_publisher(Bool, 'interrogator/connected', 10)
        self.signal_pubs = {}
        self.signal_pubs['all'] = {'raw':       self.create_publisher(Float64MultiArray, 'sensor/raw', 10),
                                   'processed': self.create_publisher(Float64MultiArray, 'sensor/processed', 10)}
        if self.is_connected:
            topic_raw = 'sensor/CH{:d}/raw'
            topic_proc = 'sensor/CH{:d}/processed'
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
        self.calibrate_srv = self.create_service(Trigger, 'sensor/calibrate', self.ref_wl_service)
        
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
