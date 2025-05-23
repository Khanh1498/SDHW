import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PointStamped
from visualization_msgs.msg import Marker
from tf_transformations import euler_from_quaternion
from trajectory_control.pid_controller import PIDController  # Import PID
import math

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')
        
        # Parameters
        self.lookahead_distance = 2.0
        self.target_speed = 1.5  # m/s (adjust as needed)
        self.waypoints = self.load_waypoints()
        
        # PID for speed control
        self.pid = PIDController(Kp=0.5, Ki=0.01, Kd=0.1, max_output=2.0)
        self.last_time = self.get_clock().now()
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Constant speed (for Part 1)
        self.constant_speed = 1.0  # m/s
        
        self.get_logger().info("Pure Pursuit Controller Initialized!")

    def load_waypoints(self):
        """Load waypoints from sonoma_waypoints.txt"""
        waypoints = []
        waypoint_file = "src/trajectory_control/resource/sonoma_waypoints.txt"
        with open(waypoint_file, 'r') as f:
            for line in f:
                x, y, _ = map(float, line.strip().split(','))
                waypoints.append((x, y))
        return waypoints

    def odom_callback(self, msg):
        # Get current state
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        current_speed = msg.twist.twist.linear.x
        
        # Time step for PID
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        
        # --- Steering Control (Pure Pursuit) ---
        nearest_idx = self.find_nearest_waypoint(x, y)
        lookahead_point = self.find_lookahead_point(nearest_idx, x, y)
        steering_angle = self.calculate_steering_angle(x, y, yaw, lookahead_point)
        
        # --- Speed Control (PID) ---
        speed_error = self.target_speed - current_speed
        throttle = self.pid.compute(speed_error, dt)
        
        # Publish combined command
        self.publish_cmd_vel(throttle, steering_angle)

    def find_nearest_waypoint(self, x, y):
        """Find the closest waypoint to the current position"""
        min_dist = float('inf')
        nearest_idx = 0
        for i, (wx, wy) in enumerate(self.waypoints):
            dist = math.sqrt((wx - x)**2 + (wy - y)**2)
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i
        return nearest_idx

    def find_lookahead_point(self, nearest_idx, x, y):
        """Find the first waypoint beyond the lookahead distance"""
        for i in range(nearest_idx, len(self.waypoints)):
            wx, wy = self.waypoints[i]
            dist = math.sqrt((wx - x)**2 + (wy - y)**2)
            if dist >= self.lookahead_distance:
                return (wx, wy)
        return None

    def calculate_steering_angle(self, x, y, yaw, lookahead_point):
        """Pure Pursuit steering angle calculation"""
        # Transform lookahead point to vehicle coordinates
        lx, ly = lookahead_point
        dx = lx - x
        dy = ly - y
        
        # Vehicle-relative coordinates
        rel_x = dx * math.cos(yaw) + dy * math.sin(yaw)
        rel_y = -dx * math.sin(yaw) + dy * math.cos(yaw)
        
        # Calculate curvature (1/r)
        curvature = 2.0 * rel_y / (rel_x**2 + rel_y**2)
        
        # Convert curvature to steering angle (simplified)
        steering_angle = math.atan(curvature * 0.5)  # Adjust gain as needed
        return steering_angle

    def publish_cmd_vel(self, steering_angle):
        """Publish Twist command to /cmd_vel"""
        cmd_vel = Twist()
        cmd_vel.linear.x = self.constant_speed  # Fixed speed for Part 1
        cmd_vel.angular.z = steering_angle
        self.cmd_vel_pub.publish(cmd_vel)

    def publish_lookahead_marker(self, point):
        """Visualize the lookahead point in RViz"""
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = 0.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.lookahead_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    pure_pursuit = PurePursuit()
    rclpy.spin(pure_pursuit)
    pure_pursuit.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()