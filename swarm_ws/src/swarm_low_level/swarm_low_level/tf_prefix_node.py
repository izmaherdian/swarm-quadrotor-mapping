import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

class TFPrefixNode(Node):
    def __init__(self):
        super().__init__('tf_prefix_node')
        
        self.declare_parameter('drone_id', 1)
        self.drone_id = self.get_parameter('drone_id').get_parameter_value().integer_value
        
        self.prefix = f'iris_{self.drone_id}/'
        
        self.tf_pub = self.create_publisher(TFMessage, '/tf', 100)
        self.tf_sub = self.create_subscription(TFMessage, f'/iris_{self.drone_id}/tf_raw', self.tf_callback, 100)
        
    def tf_callback(self, msg):
        new_msg = TFMessage()
        for transform in msg.transforms:
            # Jaga world/swarm_world, tambah prefix ke link lokal drone
            if transform.header.frame_id not in ['world', 'swarm_world']:
                transform.header.frame_id = self.prefix + transform.header.frame_id
            
            if transform.child_frame_id not in ['world', 'swarm_world']:
                transform.child_frame_id = self.prefix + transform.child_frame_id
            
            new_msg.transforms.append(transform)
            
        self.tf_pub.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TFPrefixNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
