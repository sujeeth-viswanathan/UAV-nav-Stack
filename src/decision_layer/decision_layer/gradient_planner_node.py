import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from scipy.interpolate import splprep, splev

class GradientPlannerNode(Node):
    def __init__(self):
        super().__init__('gradient_planner_node')
        self.cost_field = None
        self.wind_field = None 
        self.step_size =  1.0 
        self.drift_factor = 0.3
        self.grad_x = None 
        self.grad_y = None
        self.cost_field_sub = self.create_subscription(
            Float32MultiArray,
            '/environment/cost_field',
            self.cost_field_callback,
            10
        )
        self.path_pub = self.create_publisher(Path, '/uav/smoothed_path', 10)
        self.get_logger().info('[Planner] Initialized and waiting for cost field...')

    def cost_field_callback(self, msg):
        size = int(np.sqrt(len(msg.data)))  # Assumes square
        self.cost_field = np.array(msg.data).reshape((size, size))
        self.grad_x, self.grad_y = np.gradient(self.cost_field)
        self.get_logger().info(f'[Planner] Cost field received and gradient calculated.')

    def get_wind_vector(self, pos):
        if self.wind_field is None:
            return np.zeros(2)
        i = int(np.clip(pos[0], 0, self.wind_field.shape[0]-1))
        j = int(np.clip(pos[1], 0, self.wind_field.shape[1]-1))
        return self.wind_field[i, j]

    def plan_path(self, start, goal, max_steps=500, tolerance=2.0):
        if self.cost_field is None:
            self.get_logger().warn('[Planner] Cannot plan — no cost field received.')
            return []

        path = [np.array(start, dtype=float)]
        current_position = np.array(start, dtype=float)

        for _ in range(max_steps):
            if np.linalg.norm(current_position - goal) < tolerance:
                break

            i = int(np.clip(current_position[0], 0, self.cost_field.shape[0]-1))
            j = int(np.clip(current_position[1], 0, self.cost_field.shape[1]-1))

            grad = np.array([self.grad_x[i, j], self.grad_y[i, j]])
            grad_norm = np.linalg.norm(grad)
            if grad_norm < 1e-6:
                grad = np.random.uniform(-1, 1, size=2)
                grad_norm = np.linalg.norm(grad)

            grad /= grad_norm

            wind_vec = self.get_wind_vector(current_position)
            wind_norm = np.linalg.norm(wind_vec)
            wind_dir = wind_vec / (wind_norm + 1e-6) if wind_norm > 1e-6 else np.zeros_like(wind_vec)

            corrected_move = (1 - self.drift_factor) * (-grad) + self.drift_factor * wind_dir
            corrected_move /= (np.linalg.norm(corrected_move) + 1e-6)

            next_position = current_position + corrected_move * self.step_size

            next_position[0] = np.clip(next_position[0], 0, self.cost_field.shape[0]-1)
            next_position[1] = np.clip(next_position[1], 0, self.cost_field.shape[1]-1)

            path.append(next_position.copy())
            current_position = next_position

        return np.array(path)
    
    def smooth_path(self, path):
        if len(path) < 4:
            return path

        x = path[:, 0]
        y = path[:, 1]

        tck, _ = splprep([x, y], s=5.0)  # smoothing factor
        u_fine = np.linspace(0, 1, num=100)
        x_fine, y_fine = splev(u_fine, tck)

        return np.vstack((x_fine, y_fine)).T
 
    def publish_path(self, path_array):
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        for pt in path_array:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(pt[0])
            pose.pose.position.y = float(pt[1])
            pose.pose.position.z = 5.0  # assume constant flight height
            msg.poses.append(pose)

        self.path_pub.publish(msg)
        self.get_logger().info('[Planner] Smoothed path published.')

def main(args=None):
    rclpy.init(args=args)
    node = GradientPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
