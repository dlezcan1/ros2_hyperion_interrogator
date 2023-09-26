import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import SetParametersResult

import numpy as np
from collections import namedtuple

from .hyperion_talker import HyperionPublisher
from socket import gaierror

# interrogagtor named tuple
DemoHyperionInterrogator = namedtuple( 'Interrogator', [ 'is_ready', 'peaks', 'channel_count' ] )


class HyperionDemo( HyperionPublisher ):
    # PARAMETER NAMES
    param_names = HyperionPublisher.param_names
    param_names[ 'num_chs' ] = 'demo.num_channels'
    param_names[ 'num_aa' ] = 'demo.num_active_areas'

    def __init__( self, name='HyperionDemo', num_chs=3, num_aa=4 ):
        self.num_chs = num_chs
        self.num_aa = num_aa

        super().__init__( name=name )

    # __init__

    def connect( self ) -> bool:
        # FBG parameters
        self.declare_parameter( HyperionDemo.param_names[ 'num_chs' ], self.num_chs )
        self.declare_parameter( HyperionDemo.param_names[ 'num_aa' ], self.num_aa )
        self.num_chs = self.get_parameter( HyperionDemo.param_names[ 'num_chs' ] ).get_parameter_value().integer_value
        self.num_aa = self.get_parameter( HyperionDemo.param_names[ 'num_aa' ] ).get_parameter_value().integer_value

        self.get_logger().info(f"Demo Interrogator config: CHs = {self.num_chs} | AAs = {self.num_aa}")
        # initialize reference wavelengths
        self.base_wavelengths = { }
        for i in range( 1, self.num_chs + 1 ):
            self.base_wavelengths[ i ] = 1540 + 10 * np.arange( self.num_aa ) + i

        # for

        """ Connect demo override """
        if (self.num_chs > 0) and (self.num_aa > 0):
            # configure demo interrogator
            self.interrogator = DemoHyperionInterrogator( is_ready=True, peaks=self.base_wavelengths,
                                                          channel_count=self.num_chs )
            self.is_connected = self.interrogator.is_ready
            self.get_logger().info( "Connected to demo hyperion interrogator" )
            self.get_logger().info( "Connected to IP: {}".format( self.ip_address ) )

        # if

        else:
            self.interrogator = DemoHyperionInterrogator( is_ready=False, peaks=self.base_wavelengths,
                                                          channel_count=self.num_chs )
            self.is_connected = self.interrogator.is_ready

        # else

        return self.is_connected

    # connect

    def parameter_callback( self, params ):
        """ Extend parameter callback """
        # values that should not be changed
        fixed_params = [ HyperionDemo.param_names[ 'num_chs' ], HyperionDemo.param_names[ 'num_aa' ] ]
        if any( [ fix_p in params for fix_p in fixed_params ] ):
            success = SetParametersResult( successful=False )

        # if

        else:
            success = super().parameter_callback( params )

        # else

        return success

    # parameter_callback

    def ref_wl_service( self, request, response ):
        ''' Service to get the reference wavelength '''
        if self.is_connected:
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

                    # if
                    continue

                # except
            # while   

            # find the average
            for ch_num, agg_peaks in data.items():
                self.ref_wavelengths[ ch_num ] = agg_peaks / self.num_samples
            # for

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


# class: HyperionDemo

def main( args=None ):
    # Arguments
    num_chs = 3
    num_aa = 4

    rclpy.init( args=args )

    hyperion_demo = HyperionDemo( num_chs=num_chs, num_aa=num_aa )

    try:
        rclpy.spin( hyperion_demo )

    except KeyboardInterrupt:
        pass

    hyperion_demo.get_logger().info( 'Shutting down...' )
    hyperion_demo.destroy_node()
    rclpy.shutdown()


# main

if __name__ == "__main__":
    main()

# if: main
