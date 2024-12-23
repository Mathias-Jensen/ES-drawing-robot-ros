import rclpy
from rclpy.node import Node
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from dynamixel_sdk_custom_interfaces.srv import GetPosition
from std_msgs.msg import Int32
from math import acos, pow, sqrt, atan2, pi
import time


class MotionController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.publisher = self.create_publisher(SetPosition, 'set_position', 10)
        self.client = self.create_client(GetPosition, 'get_position')
        self.subscriber = self.create_subscription(Int32, 'control_number', self.number_callback, 10)
        #self.timer = self.create_timer(2.0, self.update_motor_angles)  # Run every 2 seconds
        self.motor_ids = [0, 1]  # Motor IDs
        self.sequences = {
            0: self.generate_points_between((-1, 7),(1, 7), 10) + self.generate_points_between((1,7), (1,6), 10) + self.generate_points_between((1,6), (-1,6), 10) + self.generate_points_between((-1,6), (-1,7), 10),
            1: self.generate_points_between((-1,7),(-1,6),10) + self.generate_points_between((-1,6),(-1,7),10),
            2: self.generate_points_between((-1,7),(1,7),10) + self.generate_points_between((1,7), (1,6.5),5) + self.generate_points_between((1,6.5),(-1,6.5),10) + self.generate_points_between((-1,6.5),(-1,6),5) + self.generate_points_between((-1,6), (1,6),10),
            3: self.generate_points_between((-1,7),(1,7),10) + self.generate_points_between((1,7),(1,6.5),5) + self.generate_points_between((1,6.5),(-1,6.5),10) + self.generate_points_between((-1,6.5),(1,6.5),10) + self.generate_points_between((1,6.5),(1,6),5) + self.generate_points_between((1,6),(-1,6),10),
            4: self.generate_points_between((-1,7),(-1,6.5),5) + self.generate_points_between((-1,6.5),(1,6.5),10) + self.generate_points_between((1,6.5),(1,7),5) + self.generate_points_between((1,7),(1,6),10),
            5: self.generate_points_between((1,7),(-1,7),10) + self.generate_points_between((-1,7),(-1,6.5),5) + self.generate_points_between((-1,6.5),(1,6.5),10) + self.generate_points_between((1,6.5),(1,6),5) + self.generate_points_between((1,6),(-1,6),10),
            6: self.generate_points_between((1,7),(-1,7),10) + self.generate_points_between((-1,7),(-1,6.5),5) + self.generate_points_between((-1,6.5),(1,6.5),10) + self.generate_points_between((1,6.5),(1,6),5) + self.generate_points_between((1,6),(-1,6),10) + self.generate_points_between((-1,6),(-1,7),10),
            7: self.generate_points_between((-1,7),(1,7),10) + self.generate_points_between((1,7),(1,6),10),
            8: self.generate_points_between((-1,7),(1,7),10) + self.generate_points_between((1,7),(1,6.5),5) + self.generate_points_between((1,6.5),(-1,6.5),10) + self.generate_points_between((-1,6.5),(1,6.5),10) + self.generate_points_between((1,6.5),(1,6),5) + self.generate_points_between((1,6),(-1,6),10) + self.generate_points_between((-1,6),(-1,7),10),
            9: self.generate_points_between((-1,7),(1,7),10) + self.generate_points_between((1,7),(1,6.5),5) + self.generate_points_between((1,6.5),(-1,6.5),10) + self.generate_points_between((-1,6.5),(-1,7),5) + self.generate_points_between((-1,7),(1,7),10) + self.generate_points_between((1,7),(1,6),10) + self.generate_points_between((1,6),(-1,6),10),
            # Add more sequences for numbers 3–9
        }
        self.current_number = None
        self.sequence_in_progress = False

    def number_callback(self, msg):
        self.get_logger().info(f'Received number: {msg.data}')
        if msg.data in self.sequences and not self.sequence_in_progress:
            self.current_number = msg.data
            self.execute_sequence()

    def execute_sequence(self):
        if self.current_number is None:
            return

        sequence = self.sequences.get(self.current_number, [])
        if not sequence:
            self.get_logger().warning(f'No sequence defined for number {self.current_number}')
            return

        self.sequence_in_progress = True
        for x, y in sequence:
            angles = self.inverse_kinematics(x, y)
            if angles:
                theta_left, theta_right = angles
                self.publish_position(self.motor_ids[0], int(theta_left))  # Motor 1: left
                self.publish_position(self.motor_ids[1], int(theta_right))    # Motor 2: right
                self.get_logger().info(f'Moving to (x: {x}, y: {y}) -> left: {theta_left/3.422}, right: {theta_right/3.422}')
            else:
                self.get_logger().error(f'Invalid target (x: {x}, y: {y}), skipping...')
            time.sleep(0.02)  # Simulate time for motion (adjust as needed)

        self.get_logger().info(f'Sequence for number {self.current_number} completed')
        self.current_number = None
        self.sequence_in_progress = False

    def publish_position(self, motor_id, position):
        msg = SetPosition()
        msg.id = motor_id
        msg.position = position
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: Motor ID {motor_id}, Position {position}')

    def inverse_kinematics(self, x, y):
        L = 5
        motor_pos = [[-1.9, -2],[1.9, -2]]
        scale = 3.422

        theta_left = atan2(y,x) + acos((pow(x - motor_pos[0][0],2) + pow(y - motor_pos[0][1],2))/(2*sqrt(pow(x - motor_pos[0][0],2) + pow(y - motor_pos[0][1],2))*L))
        theta_left = theta_left*(180/pi) + 60
        theta_right = atan2(y,x) - acos((pow(x - motor_pos[1][0],2) + pow(y - motor_pos[1][1],2))/(2*sqrt(pow(x - motor_pos[1][0],2) + pow(y - motor_pos[1][1],2))*L))
        theta_right = theta_right*(180/pi) + 60

        return theta_left*scale, theta_right*scale

    def generate_points_between(self, p1, p2, num_points):
        """
        Generate a list of evenly spaced points between two given points.
        
        Parameters:
            p1 (tuple): The starting point (x1, y1).
            p2 (tuple): The ending point (x2, y2).
            num_points (int): The number of points to generate, including the endpoints.
            
        Returns:
            list: A list of tuples representing the points.
        """
        if num_points < 2:
            raise ValueError("Number of points must be at least 2.")
        
        x1, y1 = p1
        x2, y2 = p2
        
        # Calculate the step size for x and y
        x_step = (x2 - x1) / (num_points - 1)
        y_step = (y2 - y1) / (num_points - 1)
        
        # Generate the points
        points = [(x1 + i * x_step, y1 + i * y_step) for i in range(num_points)]
        
        return points


def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotionController()
    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        pass
    finally:
        motor_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
