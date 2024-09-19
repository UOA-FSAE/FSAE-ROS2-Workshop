import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ChatNode(Node):
    def __init__(self):
        super().__init__('chat_node')

        # Publisher for /tron_game/chat_log topic
        self.publisher_ = self.create_publisher(String, '/tron_game/chat_log', 10)

        # Subscriber to the /tron_game/chat_log topic
        self.subscription = self.create_subscription(
            String,
            '/tron_game/chat_log',
            self.chat_callback,
            10
        )

        # Prompt user for input and publish it
        self.get_logger().info('Type your message and hit Enter to send. Type "exit" to quit.')

        # Spin in a separate thread to handle user input while the node is active
        self.user_input_thread()

    def user_input_thread(self):
        while rclpy.ok():
            user_input = input()  # Get user input from the console
            if user_input.lower() == "exit":
                self.get_logger().info('Exiting chat...')
                break

            # Publish the user input
            msg = String()
            msg.data = user_input
            self.publisher_.publish(msg)

    def chat_callback(self, msg):
        # Callback function to process messages received from the chat topic
        self.get_logger().info(f'Received: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)

    chat_node = ChatNode()

    # Keep the node alive and spinning
    try:
        rclpy.spin(chat_node)
    except KeyboardInterrupt:
        pass

    # Clean up and shutdown the ROS node
    chat_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
