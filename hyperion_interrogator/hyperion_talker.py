from socket import gaierror  # for connection error

import os
from collections import defaultdict

from typing import (
    Any,
    Dict,
)

import numpy as np

import rclpy
import rclpy.publisher
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, MultiArrayDimension
from std_srvs.srv import Trigger

from .hyperionlib.hyperion import Hyperion

from needle_shape_sensing.shape_sensing import ShapeSensingFBGNeedle


class HyperionPublisher( Node ):
    # PARAMETER NAMES
    param_names = {
            'ip'         : 'interrogator.ip_address',
            'ref_wl'     : 'sensor.CH{:d}.reference',
            'num_samples': 'sensor.num_samples',
            'fbg_needle' : 'fbg_needle.path',
    }

    CH_REMAPS = {
            1: 1,
            2: 2,
            3: 3,
            4: 4,
    }

    def __init__( self, name="Hyperion" ):
        super().__init__( name )

        # parameters
        self.is_connected    = False
        self.ip_address      = '10.0.0.55'
        self.num_chs         = 0
        self.ref_wavelengths = dict()
        self.num_samples     = 200
        self.signal_pubs: Dict[Any, Dict[str, rclpy.publisher.Publisher]] = dict()
        self.connected_pub   = None
        self.reconnect_srv   = None
        self.calibrate_srv   = None
        self.interrogator    = None

        self.fbgneedle_path  = None
        self.fbgneedle       = None

        # Hyperion parameters
        self.declare_parameter( HyperionPublisher.param_names[ 'ip' ], self.ip_address )
        self.declare_parameter( HyperionPublisher.param_names[ 'num_samples' ], self.num_samples )
        self.declare_parameter( HyperionPublisher.param_names[ 'fbg_needle' ], '' )
        self.get_params()
        self.add_on_set_parameters_callback( self.parameter_callback )  # update parameters

        # connect to Hyperion Interrogator
        self.connect()

        # setup the publishers
        self.start_publishers()

        # setup services
        self.start_services()

        # setup the publisher callback
        timer_period = 0.001
        self.timer = self.create_timer( timer_period, self.publish_peaks )

    # __init__

    def connect( self ) -> bool:
        """ (Re)Instantiate the Hyperion interrogator"""
        # self.interrogator = AsyncHyperion(self.ip_address)
        self.get_logger().info( "Connecting to IP: {}".format( self.ip_address ) )
        self.interrogator = Hyperion( self.ip_address )

        try:
            self.interrogator.is_ready
            self.is_connected = True
            self.num_chs = self.interrogator.channel_count
            self.get_logger().info( "Connected to IP: {}".format( self.ip_address ) )

        # try
        except OSError:
            self.get_logger().warning( "Interrogator is not configured to a proper IP address" )
            self.is_connected = False

        # except

        return self.is_connected

    # connect

    def get_params( self ):
        """ Read in parameters for the Hyperion interrogator """
        # Hyperion IP address

        self.ip_address = self.get_parameter( HyperionPublisher.param_names[ 'ip' ] ).get_parameter_value().string_value

        self.num_samples = self.get_parameter(
                HyperionPublisher.param_names[ 'num_samples' ]
        ).get_parameter_value().integer_value

        fbgneedle_path = self.get_parameter(
                HyperionPublisher.param_names[ 'fbg_needle' ]
        ).get_parameter_value().string_value

        self.load_fbgneedle( fbgneedle_path )

    # get_params

    def load_fbgneedle( self, fbgneedle_path=None ):
        """ Loads the FBG needle into the class """
        # handle initialization issues with None fbgneedle paths
        if fbgneedle_path is None:
            fbgneedle_path = self.fbgneedle_path

        try:
            if os.path.isfile( fbgneedle_path ):
                self.fbgneedle = ShapeSensingFBGNeedle.load_json( fbgneedle_path )
                self.fbgneedle_path = fbgneedle_path
            # if

            else:
                self.get_logger().warn(f"'{self.fbgneedle_path}' is not a valid needle parameter JSON file. File does not exist!")

        # try
        except Exception as e:
            self.get_logger().warn( str( e ) )

        # except

        if self.fbgneedle is None:
            return

        # if

        self.get_logger().info( f"Loaded FBG Needle: {str( self.fbgneedle )}" )
        if np.any( self.fbgneedle.ref_wavelengths < 0 ):
            return

        # if

        self.ref_wavelengths = defaultdict( list )
        ch_assignments = self.fbgneedle.assignments_ch()

        for i, ch_assmt in enumerate( ch_assignments ):
            self.ref_wavelengths[ self.CH_REMAPS[ ch_assmt ] ].append(
                    self.fbgneedle.ref_wavelengths[ i ]
            )

        # for

        self.ref_wavelengths = dict( self.ref_wavelengths )
        for ch, peaks in self.ref_wavelengths.items():
            self.get_logger().info( f"Reference wavelengths for CH{ch}: {peaks}" )

        # for

    # load_fbgneedle

    def parameter_callback( self, params ):
        """ Parameter update call back function"""
        for param in params:
            if param.name == HyperionPublisher.param_names[ 'ip' ]:
                self.ip_address = param.get_parameter_value().string_value

            # if

            elif param.name == HyperionPublisher.param_names[ 'num_samples' ]:
                self.num_samples = param.get_parameter_value().integer_value

            # elif

        # for

        return SetParametersResult( successful=True )

    # parameter_callback

    def parse_peaks( self, peak_data ) -> dict:
        """ Parse the peak data into a dict"""

        data = { }
        for idx in range( 1, self.num_chs + 1 ):
            data[ idx ] = peak_data[ idx ].astype( np.float64 )

        # for

        return data

    # parse_peaks

    @staticmethod
    def parsed_peaks_to_msg( parsed_peaks ) -> (np.ndarray, dict):
        """ Convert the parsed peaks to a Float64MultiArray msgs (total and per channel)"""
        # initialize the Total message
        total_msg = Float64MultiArray()
        total_msg.layout.dim = [ ]
        total_msg.data = [ ]

        channel_msgs = { }

        for ch_num, peaks in parsed_peaks.items():
            # prepare the individual channel msg
            ch_msg = Float64MultiArray()
            ch_msg.layout.dim.append(
                MultiArrayDimension(
                    label=f"CH{ch_num}",
                    stride=peaks.dtype.itemsize,
                    size=peaks.size * peaks.dtype.itemsize
                    )
                )
            ch_msg.data = peaks.flatten().tolist()

            channel_msgs[ ch_num ] = ch_msg

            # append to total msg
            total_msg.layout.dim.append( ch_msg.layout.dim[ 0 ] )
            total_msg.data += ch_msg.data

        # for 

        return total_msg, channel_msgs

    # parsed_peaks_to_msg

    def process_signals( self, peak_signals, temp_comp: bool = False ) -> dict:
        """ Method to perform the signal processing

            This includes:
                - Base wavelength shifting
                - NOTIMPLEMENTED: Temperature compensation (if set)

            Including
        """
        proc_signals = { }

        for ch_num, peaks in peak_signals.items():
            proc_signals[ ch_num ] = { }

            proc_signals[ ch_num ][ 'raw' ] = peaks
            try:
                proc_signals[ ch_num ][ 'processed' ] = peaks - self.ref_wavelengths[ ch_num ]

            # try

            except Exception as e:
                # self.get_logger().warning(No)
                continue
            # except

        # for

        # temperature compensation 
        if temp_comp:
            try:
                mean_shifts = np.vstack( peak_signals.values() ).mean( axis=0 )

                for ch_num in proc_signals.keys():
                    proc_signals[ ch_num ][ 'processed' ] -= mean_shifts

                # for

            except:
                self.get_logger().warning( "Unable to perform temperature compensation for signal processing." )

            # except

        # if

        return proc_signals

    # process_signals

    def publish_peaks( self ):
        """ Publish the peaks on an timer """

        # publish the connection status
        self.connected_pub.publish( Bool( data=self.is_connected ) )

        if self.is_connected and self.interrogator.is_ready:
            peaks = self.parse_peaks( self.interrogator.peaks )
            all_peaks = self.process_signals( peaks, temp_comp=False )  # perform processing

            # split the peaks into raw and processed signals
            raw_peaks = dict( (ch_num, all_peaks[ ch_num ][ 'raw' ]) for ch_num in all_peaks.keys() )

            proc_ch_nums = [ ch for ch in all_peaks.keys() if 'processed' in all_peaks[ ch ].keys() ]
            proc_peaks = dict( (ch, all_peaks[ ch ][ 'processed' ]) for ch in proc_ch_nums )

            # prepare messages
            raw_tot_msg, raw_ch_msgs = self.parsed_peaks_to_msg( raw_peaks )
            proc_tot_msg, proc_ch_msgs = self.parsed_peaks_to_msg( proc_peaks )

            for ch_num in all_peaks.keys():
                # raw signals
                raw_pub = self.signal_pubs[ ch_num ][ 'raw' ]
                raw_msg = raw_ch_msgs[ ch_num ]
                raw_pub.publish( raw_msg )

                # processed signals
                if ch_num in proc_peaks.keys():
                    proc_msg = proc_ch_msgs[ ch_num ]
                    proc_pub = self.signal_pubs[ ch_num ][ 'processed' ]
                    proc_pub.publish( proc_msg )

                # if

            # ch_num

            # publish the entire signal
            self.signal_pubs[ 'all' ][ 'raw' ].publish( raw_tot_msg )
            self.get_logger().debug( "Published raw peak values: {}".format( raw_tot_msg.data ) )

            if len( proc_tot_msg.data ) > 0:
                self.signal_pubs[ 'all' ][ 'processed' ].publish( proc_tot_msg )
                self.get_logger().debug( "Published processed peak values: {}".format( proc_tot_msg.data ) )

            # if

        # if

        else:
            pass

        # else

    # publish_peaks

    def reconnect_service( self, request, response ):
        """ reconnect to the IP address """
        self.get_logger().info( "Reconnecting to Hyperion interrogator..." )
        self.connect()
        self.start_publishers()

        response.success = self.is_connected

        if not self.is_connected:
            response.message = "{} is not a valid IP for the Hyperion interrogator".format( self.ip_address )

        # if

        return response

    # reconnect_service

    def ref_wl_service( self, request, response ):
        """ Service to get the reference wavelength """
        if self.is_connected and self.interrogator.is_ready:
            self.get_logger().info( f"Starting to recalibrate the sensors wavelengths for {self.num_samples} samples." )

            data = { }
            counter = 0
            error_counter = 0
            max_errors = 5
            while counter < self.num_samples:
                try:
                    signal = self.parse_peaks( self.interrogator.peaks )

                    for ch_num, peaks in signal.items():
                        if ch_num not in data.keys():
                            data[ ch_num ] = peaks

                        else:
                            data[ ch_num ] += peaks

                    # for

                    # increment counter
                    counter += 1
                    error_counter = 0


                # try
                except gaierror:  # no peaks
                    error_counter += 1

                    if error_counter > max_errors:  # make sure we don't go into an endless loop
                        response.success = False
                        response.message = "Timeout interrogation occurred."

                        return response

                    # if
                # except
            # while   

            # find the average
            for ch_num, agg_peaks in data.items():
                self.ref_wavelengths[ ch_num ] = agg_peaks / self.num_samples
            # for

            # update the FBG needle
            self.update_fbgneedle()

            response.success = True
            self.get_logger().info( "Recalibration successful" )
            self.get_logger().info( "Reference wavelengths: {}".format( list( self.ref_wavelengths.values() ) ) )

        # if
        else:
            response.success = False
            response.message = "Interrogator was not able to gather peaks. Check connection and IP address."

        # else

        return response

    # ref_wl_service

    def start_publishers( self ):
        """ Method to instantiate the publishers for the class"""

        # connected publisher
        self.connected_pub = self.create_publisher( Bool, 'interrogator/connected', 10 )
        self.signal_pubs[ 'all' ] = {
                'raw'      : self.create_publisher( Float64MultiArray, 'sensor/raw', 10 ),
                'processed': self.create_publisher( Float64MultiArray, 'sensor/processed', 10 )
        }
        if self.is_connected:
            topic_raw = 'sensor/CH{:d}/raw'
            topic_proc = 'sensor/CH{:d}/processed'
            for idx in range( 1, self.num_chs + 1 ):
                ch_pub = { }
                ch_pub[ 'raw' ] = self.create_publisher( Float64MultiArray, topic_raw.format( idx ), 10 )
                ch_pub[ 'processed' ] = self.create_publisher( Float64MultiArray, topic_proc.format( idx ), 10 )
                self.signal_pubs[ idx ] = ch_pub

            # idx

            self.get_logger().info( 'Publishing signals.' )

        # if

        else:
            self.get_logger().warning( "FBG Publishers not configured since interrogator not connected." )

        # else

    # start_publishers

    def start_services( self ):
        """ Method to instantiate the services for this node """
        self.reconnect_srv = self.create_service( Trigger, 'interrogator/reconnect', self.reconnect_service )
        self.calibrate_srv = self.create_service( Trigger, 'sensor/calibrate', self.ref_wl_service )

    # start_services

    @staticmethod
    def unpack_fbg_msg( msg ) -> dict:
        """ Unpack Float64MultiArray into dict of numpy arrays """
        ret_val = { }
        idx_i = 0

        for dim in msg.layout.dim:
            ch_num = int( dim.label.strip( 'CH' ) )
            size = int( dim.size / dim.stride )

            ret_val[ ch_num ] = np.float64( msg.data[ idx_i:idx_i + size ] )

            idx_i += size  # increment size to next counter

        # for

        return ret_val

    # unpack_fbg_msg

    def update_fbgneedle( self ):
        """ Update the FBG needle path """
        if self.fbgneedle is None:
            return

        # if

        self.fbgneedle.ref_wavelengths = np.hstack(
                peaks for ch, peaks in sorted( self.ref_wavelengths.items(), key=lambda x: x[ 0 ] )
        )

        fbgneedle_outpath = (
                self.fbgneedle_path
                if self.fbgneedle_path.endswith( "-ref-wavelength-latest.json" ) else
                self.fbgneedle_path.replace( ".json", "-ref-wavelength-latest.json" )
        )

        self.fbgneedle.save_json( fbgneedle_outpath )
        self.get_logger().info(
                f"Saved FBG needle param file w/ updated reference wavelengths to: {fbgneedle_outpath}"
        )

    # update_fbgneedle


# class: HyperionPublisher

def main( args=None ):
    rclpy.init( args=args )

    hyperion_talker = HyperionPublisher()

    try:
        rclpy.spin( hyperion_talker )

    except KeyboardInterrupt:
        pass

    # clean up
    hyperion_talker.get_logger().info( '{} shutting down...'.format( hyperion_talker.get_name() ) )
    hyperion_talker.destroy_node()
    rclpy.shutdown()


# main

if __name__ == "__main__":
    main()

# if: main
