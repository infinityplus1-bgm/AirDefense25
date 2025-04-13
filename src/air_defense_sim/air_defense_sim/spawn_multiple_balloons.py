import rclpy
from gazebo_msgs.srv import SpawnEntity
import random
import time

BALLOON_TEMPLATE_PATH = "/home/juka/ros2_ws/src/air_defense_sim/models/balloon/model.sdf"

def generate_balloon_xml(radius, color):
    """Reads the balloon SDF file and replaces placeholders with actual values."""
    with open(BALLOON_TEMPLATE_PATH, "r") as f:
        sdf_data = f.read()
    
    sdf_data = sdf_data.replace("${radius}", str(radius))
    sdf_data = sdf_data.replace("${color}", color)
    
    return sdf_data

def spawn_balloon(node, name, x, y, z, xml):
    client = node.create_client(SpawnEntity, "/spawn_entity")
    req = SpawnEntity.Request()
    req.name = name
    req.xml = xml

    req.initial_pose.position.x = float(x)
    req.initial_pose.position.y = float(y)
    req.initial_pose.position.z = float(z)

    while not client.wait_for_service(timeout_sec=2.0):
        node.get_logger().info("Waiting for /spawn_entity service...")

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    return future.result()

def main():
    rclpy.init()
    node = rclpy.create_node("spawn_multiple_balloons")

    positions = [
        (0, 0.5, 1),
        (0, 2.5, 1),
        (0, -1, 2),
        (0, 2, 1)
    ]

    balloon_variants = [
        {"radius": 0.2, "color": "1 0 0 1"},  # Big Red
        {"radius": 0.1, "color": "0 0 1 1"}   # Small Blue
    ]

    for i, (x, y, z) in enumerate(positions):
        balloon_config = random.choice(balloon_variants)
        xml = generate_balloon_xml(
            balloon_config["radius"],
            balloon_config["color"]
        )

        node.get_logger().info(
            f"Spawning balloon{i+1} at ({x}, {y}, {z}) with radius={balloon_config['radius']} and color={balloon_config['color']}"
        )
        
        spawn_balloon(node, f"balloon{i+1}", x, y, z, xml)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()