import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener_node')
        self.counter = 0
        # Create a timer with a 1-second period
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Timer callback triggered.')
        self.counter += 1

        # If some condition is met (e.g., certain number of timer ticks)
        # create a new node
        if self.counter >= 10:
            self.spin_new_node()

    def spin_new_node(self):
        new_node = rclpy.create_node('new_node')
        # Perform necessary operations in the new node
        new_node.get_logger().info('Spinning new node...')
        new_node.spin()

def main(args=None):
    rclpy.init(args=args)
    listener = ListenerNode()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
