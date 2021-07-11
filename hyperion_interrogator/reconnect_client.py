import argparse
import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger

parser = argparse.ArgumentParser(description='Trigger Hyperion interrogator reconnect service')
parser.add_argument('--namespace', type=str, default='', required=False, help='Namespace of interrogator')

def main(args=None):
    # parse arguments
    parsed_args = parser.parse_args()
    
    namespace = parsed_args.namespace.rstrip('/')
    # service name
    srv_name = '{}/interrogator/reconnect'.format(namespace)
    
    rclpy.init(args=args)
    node_head = namespace if len(namespace) == 0 else namespace + "_"
    node = rclpy.create_node('{}interrogator_reconnect_client'.format(node_head))
    client = node.create_client(Trigger, srv_name)
    
    req = Trigger.Request()
    
    if not client.wait_for_service(timeout_sec = 1.0):
        node.get_logger().info('{} not available. Please try again later.'.format(srv_name))
        
    # if
    
    else:
        future = client.call_async(req)
        node.get_logger().info("Prompting for interrogator reconnect...")
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