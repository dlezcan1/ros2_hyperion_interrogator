import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger

def main(args=None):
    # initialize the node and client
    rclpy.init(args=args)
    
    node = rclpy.create_node('sensor_calibrate_client')
    srv_name = '{}/sensor/calibrate'.format(node.get_namespace())
    node.get_logger().info(f"Connecting to {srv_name}")
    client = node.create_client(Trigger, srv_name)
    
    req = Trigger.Request()
    
    if not client.wait_for_service(timeout_sec = 1.0):
        node.get_logger().info('{} not available. Please try again later.'.format(srv_name))
        
    # if
    
    else:
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        
        response = future.result()
        
        if response.success:
            node.get_logger().info("Calibration successful")
            
        else:
            node.get_logger().warning("Calibration unsuccessful")
            
    # else
    
    node.destroy_node()
    rclpy.shutdown()
    
# main

if __name__ == "__main__":
    main()
    
# if: main