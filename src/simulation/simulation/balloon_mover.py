import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetEntityState, SetEntityState
import math

class BalloonMover(Node):
    def __init__(self):
        super().__init__('balloon_mover')
        
        # Create clients for getting and setting entity states
        self.get_entity_client = self.create_client(GetEntityState, '/get_entity_state')
        self.set_entity_client = self.create_client(SetEntityState, '/set_entity_state')
        
        # Wait for services to be available
        self.get_logger().info('Waiting for entity state services...')
        self.get_entity_client.wait_for_service()
        self.set_entity_client.wait_for_service()
        self.get_logger().info('Services are available!')
        
        # Balloon parameters
        self.balloon_names = ['balloon1', 'balloon2', 'balloon3', 'balloon4']
        self.square_size = 1.7     # Size of the square path (meters)
        self.move_speed = 0.8      # Movement speed (meters/second)
        self.update_rate = 10      # Updates per second (Hz)
        
        # Movement tracking for each balloon
        self.spawn_positions = {}  # Original positions
        self.balloon_paths = {}    # Path points for each balloon
        self.balloon_states = {}   # Current state for each balloon (segment, progress)
        
        # Initialize spawn positions
        self.get_initial_positions()
        
        # Setup movement data for each balloon
        self.initialize_balloon_movements()
        
        # Create timer for movement updates
        self.timer_period = 1.0 / self.update_rate  # seconds
        self.movement_timer = self.create_timer(self.timer_period, self.update_balloon_positions)
    
    def get_initial_positions(self):
        """Get the initial positions of all balloons."""
        for name in self.balloon_names:
            try:
                req = GetEntityState.Request()
                req.name = name
                req.reference_frame = 'world'
                
                future = self.get_entity_client.call_async(req)
                rclpy.spin_until_future_complete(self, future)
                
                if future.result() is not None and future.result().success:
                    position = future.result().state.pose.position
                    self.spawn_positions[name] = (position.x, position.y, position.z)
                    self.get_logger().info(f'Initial position for {name}: ({position.x:.2f}, {position.y:.2f}, {position.z:.2f})')
                else:
                    self.get_logger().error(f'Failed to get position for {name}, entity may not exist')
            except Exception as e:
                self.get_logger().error(f'Exception getting position for {name}: {e}')
    
    def generate_square_path(self, spawn_pos):
        """Generate square path points based on spawn position."""
        x, y, z = spawn_pos
        half_size = self.square_size / 2
        
        # Create square path in Y-Z plane, keeping X constant
        return [
            (x, y + half_size, z + half_size),  # Top-right
            (x, y - half_size, z + half_size),  # Top-left 
            (x, y - half_size, z - half_size),  # Bottom-left
            (x, y + half_size, z - half_size),  # Bottom-right
            (x, y + half_size, z + half_size)   # Back to top-right (same as first, to complete the loop)
        ]
    
    def initialize_balloon_movements(self):
        """Set up the movement data for each balloon."""
        for name in self.balloon_names:
            if name in self.spawn_positions:
                # Generate path for this balloon
                path = self.generate_square_path(self.spawn_positions[name])
                self.balloon_paths[name] = path
                
                # Initialize state: current segment (0-3) and progress along segment (0.0-1.0)
                self.balloon_states[name] = {
                    'segment': 0,
                    'progress': 0.0
                }
            else:
                self.get_logger().warning(f'No spawn position for {name}, skipping')
    
    def update_balloon_positions(self):
        """Update the position of all balloons based on their current state."""
        try:
            # Distance to move this update
            distance_per_update = (self.move_speed * self.timer_period)
            
            for name in self.balloon_names:
                if name not in self.balloon_states or name not in self.balloon_paths:
                    continue
                
                # Get current state
                segment = self.balloon_states[name]['segment']
                progress = self.balloon_states[name]['progress']
                
                # Get points for current segment
                path = self.balloon_paths[name]
                start_point = path[segment]
                end_point = path[segment + 1]
                
                # Calculate segment length
                dx = end_point[0] - start_point[0]
                dy = end_point[1] - start_point[1]
                dz = end_point[2] - start_point[2]
                segment_length = math.sqrt(dx**2 + dy**2 + dz**2)
                
                # Calculate progress increment
                progress_increment = distance_per_update / segment_length
                new_progress = progress + progress_increment
                
                # Check if we've reached the end of this segment
                if new_progress >= 1.0:
                    # Move to next segment
                    segment = (segment + 1) % 4  # Wrap around at the end (4 segments total)
                    new_progress = new_progress - 1.0  # Carry over excess progress
                
                # Update balloon state
                self.balloon_states[name]['segment'] = segment
                self.balloon_states[name]['progress'] = new_progress
                
                # Calculate new position
                start_point = path[segment]
                end_point = path[segment + 1]
                
                # Interpolate between start and end
                x = start_point[0] + (end_point[0] - start_point[0]) * new_progress
                y = start_point[1] + (end_point[1] - start_point[1]) * new_progress
                z = start_point[2] + (end_point[2] - start_point[2]) * new_progress
                
                # Move the balloon
                self.set_balloon_position(name, x, y, z)
                
        except Exception as e:
            self.get_logger().error(f'Error in update_balloon_positions: {e}')
    
    def set_balloon_position(self, name, x, y, z):
        """Set the position of a specific balloon."""
        try:
            # Create request
            req = SetEntityState.Request()
            req.state.name = name
            req.state.pose.position.x = x
            req.state.pose.position.y = y
            req.state.pose.position.z = z
            req.state.pose.orientation.x = 0.0
            req.state.pose.orientation.y = 0.0
            req.state.pose.orientation.z = 0.0
            req.state.pose.orientation.w = 1.0
            req.state.reference_frame = 'world'
            
            # Call service non-blocking (we don't wait for the result)
            self.set_entity_client.call_async(req)
            
        except Exception as e:
            self.get_logger().error(f'Error setting position for {name}: {e}')

def main(args=None):
    rclpy.init(args=args)
    balloon_mover = BalloonMover()
    
    try:
        rclpy.spin(balloon_mover)
    except KeyboardInterrupt:
        pass
    finally:
        balloon_mover.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()