import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import math
import numpy as np

class Costmap(Node):
    def __init__(self):
        super().__init__('costmap')

        self.declare_parameter('map_size', 200)
        self.declare_parameter('resolution', 0.05)

        self.map_size = self.get_parameter('map_size').get_parameter_value().integer_value
        self.resolution = self.get_parameter('resolution').get_parameter_value().double_value

        self.map = np.zeros((self.map_size, self.map_size), dtype=np.int8)
        self.center = self.map_size // 2

        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pub = self.create_publisher(OccupancyGrid, '/my_map', 10)

        self.get_logger().info('Costmap started waiting for data')

    def scan_callback(self, msg: LaserScan):
        self.map.fill(0)
        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r):
                continue

            angle = msg.angle_min + i * msg.angle_increment
            x = int(self.center + (r * math.cos(angle)) / self.resolution)
            y = int(self.center + (r * math.sin(angle)) / self.resolution)

            if 0 <= x < self.map_size and 0 <= y < self.map_size:
                self.map[y, x] = 100
            

        inflation_cells = int(self.inflation_radius / self.resolution)

        inflated_map = self.map.copy()

        for y in range(self.map_size):
            for x in range(self.map_size):
                if self.map[y, x] == 100:
                    for dy in range(-inflation_cells, inflation_cells + 1):
                        for dx in range(-inflation_cells, inflation_cells + 1):
                            ny = y + dy
                            nx = x + dx
                            if 0 <= nx < self.map_size and 0 <= ny < self.map_size:
                                distance = math.sqrt(dx*dx + dy*dy)
                                if distance <= inflation_cells:
                                    cost = int(100 * (1 - distance / inflation_cells))
                                    if cost > inflated_map[ny, nx]:
                                        inflated_map[ny, nx] = cost

        self.map = inflated_map


        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'base_link'
        grid.info.resolution = self.resolution
        grid.info.width = self.map_size
        grid.info.height = self.map_size
        grid.info.origin.position.x = 0.0
        grid.info.origin.position.y = 0.0
        grid.info.origin.position.z = 0.0
        grid.data = self.map.flatten().tolist()
        self.pub.publish(grid)

def main():
    rclpy.init()
    node = Costmap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
