import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException

from std_msgs.msg import Bool, Float64MultiArray, MultiArrayDimension
from std_srvs.srv import Trigger
from rcl_interfaces.msg import SetParametersResult

import sys
import asyncio
import numpy as np
import threading
import multiprocessing

from .hyperionlib.hyperion import Hyperion, HCommTCPPeaksStreamer
from .hyperion_talker import HyperionPublisher


class PeakStreamer( HyperionPublisher ):
    # PARAMETER NAMES

    def __init__( self, name="PeakStreamer" ):
        super().__init__( name )

        self.streamer_loop : asyncio.AbstractEventLoop = None
        self.streamer_queue: asyncio.Queue             = None
        self.streamer      : HCommTCPPeaksStreamer     = None

        self.task_spin        : asyncio.Task = None
        self.task_pub         : asyncio.Task = None
        self.task_stream_data : asyncio.Task = None

        self.connect_streamer()

        # reinstantiate a new timer for publishing connectivity
        self.timer.cancel()  # remove the timed publishing
        self.destroy_timer( self.timer )
        timer_period = 0.5
        self.timer = self.create_timer( timer_period, self.connection_publisher )

        # remove reconnect service (Not Implemented yet)
        self.destroy_service( self.reconnect_srv )

    # __init__

    def connect_streamer( self ) -> bool:
        ''' Connect to hyperion interrogator and initialize peak streamer OVERRIDE'''
        # initialize the peak steramer
        self.streamer_loop  = None
        self.streamer_queue = asyncio.Queue( maxsize=5 )
        self.streamer       = HCommTCPPeaksStreamer( 
            address=self.ip_address, 
            loop=self.streamer_loop, 
            queue=self.streamer_queue,
        )
        self.streamer.stream_active = True

        return True

    # connect_streamer

    def connection_publisher( self ):
        ''' Publish if the interrogator is connected or not '''
        self.connected_pub.publish( Bool( data=self.is_connected ) )

    # connection_publisher

    def destroy_node( self ):
        ''' Destroy the node '''
        self.get_logger().info( "Stopping streaming..." )
        self.streamer.stop_streaming()  # stop the current interrogator
        self.streamer.stream_active = False
        self.streamer_loop.stop()
        self.get_logger().info( "Waiting for the loop to close." )
        while self.streamer_loop.is_running():
            pass

        super().destroy_node()
        self.get_logger().info( "Streaming stopped. Node destroyed." )

    # destroy_node

    async def publish_peaks_stream( self ):
        self.get_logger().info( "Started publishing peaks." )
        while rclpy.ok() and self.streamer.stream_active:
            # grab the peak data
            self.get_logger().debug( "Streaming Peaks" )
            peak_data = await self.streamer_queue.get()
            self.streamer_queue.task_done()

            # parse and process the peaks
            self.get_logger().debug( "Parsing Peaks..." )
            peaks = self.parse_peaks( peak_data[ 'data' ] )
            self.get_logger().debug( "Parsed peaks. Processing signals..." )
            all_peaks = self.process_signals( peaks, temp_comp=False )
            self.get_logger().debug( "Processed peaks." )

            # split the peaks into raw and processed signals
            raw_peaks = dict( (ch_num, all_peaks[ ch_num ][ 'raw' ]) for ch_num in all_peaks.keys() )

            proc_ch_nums = [ ch for ch in all_peaks.keys() if 'processed' in all_peaks[ ch ].keys() ]
            proc_peaks   = dict( (ch, all_peaks[ ch ][ 'processed' ]) for ch in proc_ch_nums )

            # prepare messages
            raw_tot_msg,  raw_ch_msgs  = self.parsed_peaks_to_msg( raw_peaks )
            proc_tot_msg, proc_ch_msgs = self.parsed_peaks_to_msg( proc_peaks )

            for ch_num in all_peaks.keys():
                # raw signals
                raw_pub = self.signal_pubs[ ch_num ][ 'raw' ]
                raw_msg = raw_ch_msgs[ ch_num ]
                raw_pub.publish( raw_msg )

                # processed signals
                if ch_num in proc_peaks.keys():
                    proc_msg: Float64MultiArray = proc_ch_msgs[ ch_num ]
                    if len(proc_msg.data) > 0:
                        proc_pub = self.signal_pubs[ ch_num ][ 'processed' ]
                        proc_pub.publish( proc_msg )

                    # if
                # if

            # for

            # publish the entire signal
            self.signal_pubs[ 'all' ][ 'raw' ].publish( raw_tot_msg )
            self.get_logger().debug( "Published raw peak values: {}".format( raw_tot_msg.data ) )

            if len(proc_tot_msg.data) > 0:
                self.signal_pubs[ 'all' ][ 'processed' ].publish( proc_tot_msg )
                self.get_logger().debug( "Published processed peak values: {}".format( proc_tot_msg.data ) )
            # if

        # while

        self.get_logger().info( "Peak publishing terminated." )

    # publish_peaks_stream

    def ref_wl_service( self, request: Trigger.Request, response: Trigger.Response ):
        ''' Service to get the reference wavelength OVERRIDE '''
        TIMEOUT = 5 # (seconds)
        self.get_logger().info( f"Starting to recalibrate the sensors wavelengths for {self.num_samples} samples." )

        # initialize data container
        data = { }
        counter = 0

        def update_data( msg ):
            nonlocal data, counter

            # parse in FBG msgs
            peaks = self.unpack_fbg_msg( msg )
            self.get_logger().debug( f"Counter: {counter}, Recalibration Peaks: {peaks}" )
            self.get_logger().debug( f"Data: {data}" )
            for ch_num, ch_peaks in peaks.items():
                if ch_num not in data.keys():
                    data[ ch_num ] = ch_peaks

                else:
                    data[ ch_num ] += ch_peaks

            # for

            # increment counter
            counter += 1

        # update_data

        # temporary subscriber node to raw data
        tmp_node = Node( 'TempSignalSubscriber' )
        tmp_sub = tmp_node.create_subscription( 
            Float64MultiArray, 
            self.signal_pubs[ 'all' ][ 'raw' ].topic_name,
            update_data, 
            10,
        )

        # Wait to gather 200 signals
        t0 = self.get_clock().now().nanoseconds
        dt = 0
        while (
            (counter < self.num_samples)
            and (dt * 1e-9 < TIMEOUT)
        ):
            rclpy.spin_once( tmp_node )
            dt = t0 - self.get_clock().now().nanoseconds

        # while

        if counter < self.num_samples: # failure
            response.success = False
            response.message = f"Calibration timed out. Received {counter} / {self.num_samples} required."
            self.get_logger().error(f"Recalibration unsuccessful. Service timed out. Received {counter} / {self.num_samples} required.")

            return response
        
        # if

        # normalize the data
        for ch_num, agg_peaks in data.items():
            self.ref_wavelengths[ ch_num ] = agg_peaks / counter

        # for

        # update the FBG needle
        self.update_fbgneedle()

        response.success = True
        self.get_logger().info( "Recalibration successful" )
        self.get_logger().info( "Reference wavelengths: {}".format( list( self.ref_wavelengths.values() ) ) )

        # destroy the subscriber
        if tmp_node.handle:
            tmp_node.destroy_node()

        # if

        return response

    # ref_wl_service

    def reconnect_service( self, request, response ):
        ''' Reconnect to the IP address '''
        self.streamer.stop_streaming()  # stop the current interrogator
        self.streamer.stream_active = False
        self.streamer_loop.stop()
        self.get_logger().info( "Waiting for the loop to stop running." )
        while self.streamer_loop.is_running():
            pass

        response = super().reconnect_service( request, response )
        self.connect_streamer()

        return response

    # reconnect_service

    async def spin(self):
        cancel = self.create_guard_condition(lambda: None)
        def _spin(node: Node, future: asyncio.Future, event_loop: asyncio.AbstractEventLoop):
            while not future.cancelled():
                rclpy.spin_once(node)

            if not future.cancelled():
                event_loop.call_soon_threadsafe(future.set_result, None)

        # _spin

        event_loop  = asyncio.get_event_loop()
        spin_task   = event_loop.create_future()
        spin_thread = threading.Thread(
            target=_spin,
            args=(self, spin_task, event_loop)
        )
        spin_thread.start()

        try:
            await spin_task

        except asyncio.CancelledError:
            cancel.trigger()

        spin_thread.join()
        self.destroy_guard_condition(cancel)

    # spin

    async def run(self):
        self.task_spin        = asyncio.get_event_loop().create_task(self.spin())
        self.task_pub         = asyncio.get_event_loop().create_task(self.publish_peaks_stream())
        self.task_stream_data = asyncio.get_event_loop().create_task(self.streamer.stream_data())

        tasks = [
            self.task_spin,
            self.task_pub,
            self.task_stream_data,
        ]
        await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)

        # cancel the rest of the tasks
        for task in tasks:
            if task.cancel():
                await task

        # for

    # run


# class: PeakStreamer

def main( args=None ):
    rclpy.init( args=args )

    peak_streamer = PeakStreamer()

    try:
        asyncio.get_event_loop().run_until_complete(peak_streamer.run())

    except asyncio.CancelledError:
        pass

    finally: # clean up
        asyncio.get_event_loop().close()

    peak_streamer.get_logger().info( '{} shutting down...'.format( peak_streamer.get_name() ) )
    peak_streamer.destroy_node()

    rclpy.shutdown()


# main

if __name__ == "__main__":
    main()

# if: main
