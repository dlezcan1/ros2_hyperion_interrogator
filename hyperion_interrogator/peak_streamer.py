import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException

from std_msgs.msg import Bool, Float64MultiArray, MultiArrayDimension
from std_srvs.srv import Trigger
from rcl_interfaces.msg import SetParametersResult

import asyncio
import numpy as np
from .hyperionlib.hyperion import Hyperion, HCommTCPPeaksStreamer

from hyperion_talker import HyperionPublisher

class PeakStreamer(HyperionPublisher):
    # PARAMETER NAMES
    
    def __init__(self, name="PeakStreamer"):
        super().__init__(name)
        
        # remove the timer
        self.timer.destroy() # remove the timed publishing 
        del(self.timer_period)

        
        
        
    # __init__
    
    def connect(self):
        ''' Connect to hyperion interrogator and initialize peak streamer OVERRIDE'''
        super().connect()
        
        # initialize the peak steramer
        self.streamer_loop = asyncio.get_event_loop()
        self.streamer_queue = asyncio.Queue(maxsize=5, loop=self.streamer_loop)
        self.streamer = HCommTCPPeaksStreamer(self.ip_address, self.streamer_loop, self.streamer_queue)
            
    # connect
    
    async def publish_peaks_stream(self):
        while rclpy.ok():
            # grab the peak data
            peak_data = await self.streamer_queue.get()
            self.streamer_queue.task_done()
            
            # parse and process the peaks
            peaks = self.parse_peaks(peak_data)
            all_peaks = self.process_peaks(peaks)
            
            # split the peaks into raw and processed signals
            raw_peaks = dict((ch_num, all_peaks[ch_num]['raw']) for ch_num in all_peaks.keys())
            
            proc_ch_nums = [ch for ch in all_peaks.keys() if 'processed' in all_peaks[ch].keys()]
            proc_peaks = dict((ch, all_peaks[ch]['processed']) for ch in proc_ch_nums)
            
            # prepare messages
            raw_tot_msg, raw_ch_msgs = self.parsed_peaks_to_msg(raw_peaks)
            proc_tot_msg, proc_ch_msgs = self.parsed_peaks_to_msg(proc_peaks)
            
            for ch_num in all_peaks.keys():
                # raw signals
                raw_pub = self.signal_pubs[ch_num]['raw']
                raw_msg = raw_ch_msgs[ch_num]
                raw_pub.publish(raw_msg)
                
                # processed signals
                if ch_num in proc_peaks.keys():
                    proc_msg = proc_ch_msgs[ch_num]
                    proc_pub = self.signal_pubs[ch_num]['processed']
                    proc_pub.publish(proc_msg)
                    
                # if
            
            # ch_num
            
            # publish the entire signal
            self.signal_pubs['all']['raw'].publish(raw_tot_msg)
            self.signal_pubs['all']['processed'].publish(proc_tot_msg)
            
            # publish that the interrogator is connected
            self.connected_pub.publish(Bool(data=self.is_connected))
            
        # while
        
    # publish_peaks_stream
    
    def ref_wl_service(self, request, response):
        ''' Service to get the reference wavelength OVERRIDE
            
            TODO:
                - include timeout (5 seconds)
        
        '''
        self.get_logger().info(f"Starting to recalibrate the sensors wavelengths for {self.num_samples} samples.")
        
        # initialize data container
        data = {} 
        counter = 0
        
        def update_data(msg):
            nonlocal data, counter
            
            # parse in FBG msgs
            peaks = self.unpack_fbg_msg(msg)
            
            for ch_num, ch_peaks in peaks.items():
                if ch_num not in data.keys():
                    data[ch_num] = ch_peaks
                    
                else:
                    data[ch_num] += ch_peaks
                    
                # increment counter
                counter += 1 
            # for
            
        # update_data
        
        # temporary subscriber node to raw data
        tmp_node = Node('tmp/signal_subscriber')
        tmp_sub = tmp_node.create_subscription(Float64MultiArray, self.signal_pubs['all']['raw'].topic_name, 
                                               update_data, 10)
        
        
        # Wait to gather 200 signals
        while counter < self.num_samples:
            rclpy.spin_once(tmp_node)
            
        # while
        
        # normalize the data
        for ch_num, agg_peaks in data.items():
            self.ref_wavelengths[ch_num] = agg_peaks/counter
            
        # for
        
        response.success = True
        self.get_logger().info("Recalibration successful")
        self.get_logger().info("Reference wavelengths: {}".format(list(self.ref_wavelengths.values())))
        
        # destroy the subscriber
        if tmp_node.handle:
            tmp_node.destroy_node()
            
        # if
        
    # ref_wl_service
    
# class: PeakStreamer

def main(args = None):
    rclpy.init(args=args)
    
    peak_streamer = PeakStreamer()
    
    try:    
        rclpy.spin(peak_streamer)
    
    except KeyboardInterrupt: 
        pass
        
        
    # clean up
    peak_streamer.get_logger().info('{} shutting down...'.format(peak_streamer.get_name()))
    peak_streamer.destroy_node()
    rclpy.shutdown()
    
# main

if __name__ == "__main__":
    main()
    
# if: main
