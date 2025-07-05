import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from main import config as cfg

class HealthMonitoringNode(Node):
    def __init__(self):
        super().__init__(cfg.NODE_HEALTH_MONITORING)
        self.node_health_status = {}
        self.timer_period = 1
        self.nodes_to_monitor = {
            cfg.NODE_CAMERA: cfg.TOPIC_HEALTH_CAMERA_NODE,
            cfg.NODE_COMMAND_HANDLER: cfg.TOPIC_HEALTH_COMMAND_HANDLER_NODE,
            cfg.NODE_YOLO_DETECTION: cfg.TOPIC_HEALTH_DETECTION_NODE,
            cfg.NODE_IMAGE_PROCESSING: cfg.TOPIC_HEALTH_IMAGE_PROCESSING_NODE,
            cfg.NODE_SERIAL: cfg.TOPIC_HEALTH_SERIAL_NODE,
            cfg.NODE_TRACKING: cfg.TOPIC_HEALTH_TRACKING_NODE
        }

        for node_name, topic_name in self.nodes_to_monitor.items():
            self.node_health_status[node_name] = 'offline'
            self.create_subscription(
                String,
                topic_name,
                lambda msg, node=node_name: self.health_callback(msg, node),
                10
            )

        self.system_health_publisher = self.create_publisher(String, cfg.TOPIC_HEALTH_SYSTEM, 10)
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
