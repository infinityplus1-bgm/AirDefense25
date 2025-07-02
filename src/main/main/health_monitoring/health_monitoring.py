import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HealthMonitoringNode(Node):
    def __init__(self):
        super().__init__('health_monitoring_node')
        self.node_health_status = {}
        self.timer_period = 1
        self.nodes_to_monitor = [
            'camera_node',
            'command_handler_node',
            'detection_node',
            'image_processing_node',
            'serial_node',
            'tracking_node'
        ]

        for node_name in self.nodes_to_monitor:
            topic_name = f'/health/{node_name}'
            self.node_health_status[node_name] = 'offline'
            self.create_subscription(
                String,
                topic_name,
                lambda msg, node=node_name: self.health_callback(msg, node),
                10
            )

        self.system_health_publisher = self.create_publisher(String, '/health/system', 10)
        self.timer = self.create_timer(self.timer_period, self.publish_system_health)

    def health_callback(self, msg, node_name):
        self.node_health_status[node_name] = msg.data

    def publish_system_health(self):
        system_status = "healthy"
        for node_name, status in self.node_health_status.items():
            if status != 'healthy':
                system_status = "unhealthy"
                self.get_logger().warn(f'{node_name} is not healthy: {status}')

        msg = String()
        msg.data = system_status
        self.system_health_publisher.publish(msg)
        self.get_logger().info(f'System health: {system_status}')

def main(args=None):
    rclpy.init(args=args)
    health_monitoring_node = HealthMonitoringNode()
    rclpy.spin(health_monitoring_node)
    health_monitoring_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
