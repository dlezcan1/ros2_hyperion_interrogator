import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.executors import Executor

from std_msgs.msg import Bool, Float64MultiArray, MultiArrayDimension
from std_srvs.srv import Trigger
from rcl_interfaces.msg import SetParametersResult

import asyncio
import numpy as np
from .hyperionlib.hyperion import Hyperion, HCommTCPPeaksStreamer

from .hyperion_talker import HyperionPublisher

class PeakStreamer(HyperionPublisher):
    # PARAMETER NAMES
    
    def __init__(self, name="PeakStreamer"):
        super().__init__(name)
        
        self.connect_streamer()
        
        # remove the timer
        self.timer.destroy() # remove the timed publishing 
        del(self.timer_period)

    # __init__
    
    def connect_streamer(self) -> bool:
        ''' Connect to hyperion interrogator and initialize peak streamer OVERRIDE'''
        # initialize the peak steramer
        self.streamer_loop = asyncio.get_event_loop()
        self.streamer_queue = asyncio.Queue(maxsize=5)
        self.streamer = HCommTCPPeaksStreamer(self.ip_address, self.streamer_loop, self.streamer_queue)
        
        self.streamer_loop.create_task( self.publish_peaks_stream() )
        self.streamer_loop.run_until_complete( self.streamer.stream_data() )
        
        return True
    
    # connect_streamer
    
    async def publish_peaks_stream(self):
        while True: # rclpy.ok():
            # grab the peak data
            self.get_logger().debug("Streaming Peaks")
            peak_data = await self.streamer_queue.get()
            self.streamer_queue.task_done()
            
            # parse and process the peaks
            self.get_logger().debug("Parsing Peaks...")
            peaks = self.parse_peaks(peak_data['data'])
            self.get_logger().debug("Parsed peaks. Processing signals...")
            all_peaks = self.process_signals(peaks)
            self.get_logger().debug("Processed peaks.")
            
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
            
            # for
            
            # publish the entire signal
            self.signal_pubs['all']['raw'].publish(raw_tot_msg)
            self.signal_pubs['all']['processed'].publish(proc_tot_msg)
            
            # publish that the interrogator is connected
            self.connected_pub.publish(Bool(data=self.is_connected))
            self.get_logger().debug("Published peaks.")
            
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
        
        return response
        
    # ref_wl_service
    
    def reconnect_service(self, request, response):
        ''' Reconnect to the IP address '''
        response = super().reconnect_service(request, response)
        self.streamer.stop_streaming() # stop the current interrogator
        self.connect_streamer()
                
        return response
        
    # reconnect_service
    
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