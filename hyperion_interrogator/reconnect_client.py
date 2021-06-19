import argparse
import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger

parser = argparse.ArgumentParser(description='Trigger Hyperion interrogator reconnect service')
parser.add_argument('--node-name', type=str, default='Hyperion', required=False, help='Node name of interrogator')

def main(args=None):
    # parse arguments
    parsed_args = parser.parse_args()
    
    # service name
    srv_name = '/{}/interrogator/reconnect'.format(parsed_args.node_name)
    
    rclpy.init(args=args)
    
    node = rclpy.create_node('{}_interrogator_reconnect_client'.format(parsed_args.node_name.replace('/','_')))
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
            node.get_logger().info("Reconnect successful")
            
        else:
            node.get_logger().warning("Reconnect unsuccessful")
            
    # else
    
    node.destroy_node()
    rclpy.shutdown()
    
# main

if __name__ == "__main__":
    main()
    
# if: main