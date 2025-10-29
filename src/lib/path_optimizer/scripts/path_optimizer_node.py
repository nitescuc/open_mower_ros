#!/usr/bin/env python3
"""
Path Optimizer Node

This node provides advanced path optimization algorithms for autonomous mowing.
It subscribes to path messages and publishes optimized paths.

License: MIT
"""

import rospy
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from path_optimizer.srv import OptimizePaths, OptimizePathsResponse, GetAreaConfig, GetAreaConfigResponse
from path_optimizer.path_optimizer_core import PathOptimizerCore
from path_optimizer.config_loader import PathOptimizerConfig
from slic3r_coverage_planner.msg import Path as Slic3rPath

class PathOptimizerNode:
    """
    ROS node for path optimization
    """
    
    def __init__(self):
        """Initialize the path optimizer node"""
        rospy.init_node('path_optimizer_node', anonymous=False)
        
        # Load configuration from YAML file
        self.config_loader = PathOptimizerConfig()
        
        # Get global settings
        self.global_settings = self.config_loader.get_global_settings()
        
        # Initialize simple path processor (no complex optimization)
        default_config = self.config_loader.get_default_config()
        self.processor = PathOptimizerCore(config=default_config)
        
        # Log available areas
        available_areas = self.config_loader.get_available_areas()
        rospy.loginfo(f"Available area configurations: {available_areas}")
        
        # Publishers and Subscribers
        self.optimized_path_pub = rospy.Publisher(
            '~/optimized_path', Path, queue_size=1
        )
        
        self.path_sub = rospy.Subscriber(
            '~/input_path', Path, self.path_callback, queue_size=1
        )
        
        # Service Server
        self.optimize_service = rospy.Service(
            '~/optimize', OptimizePaths, self.optimize_service_callback
        )
        
        self.get_area_config_service = rospy.Service(
            '~/get_area_config', GetAreaConfig, self.get_area_config_callback
        )
        
        rospy.loginfo("Path Optimizer Node initialized (inner_first + reverse + distance ordering)")
        rospy.loginfo("Path optimization service available at ~/optimize")
        rospy.loginfo("Get area config service available at ~/get_area_config")
    
    def path_callback(self, path_msg):
        """
        Callback function for incoming path messages
        
        Args:
            path_msg (nav_msgs/Path): Input path to optimize
        """
        try:
            # Extract waypoints from path message
            waypoints = self.extract_waypoints(path_msg)
            
            if len(waypoints) < 2:
                rospy.logwarn("Path has fewer than 2 waypoints, cannot optimize")
                return
            
            # Process the path (apply reversal if configured)
            processed_waypoints = self.processor.optimize_path(waypoints)
            
            # Create processed path message
            optimized_path = self.create_path_message(
                processed_waypoints, 
                path_msg.header
            )
            
            # Publish optimized path
            self.optimized_path_pub.publish(optimized_path)
            
            rospy.logdebug(f"Processed path: {len(waypoints)} -> {len(processed_waypoints)} waypoints")
            
        except Exception as e:
            rospy.logerr(f"Error optimizing path: {str(e)}")
    
    def optimize_service_callback(self, request):
        """
        Service callback for path optimization requests
        
        Args:
            request (OptimizePathsRequest): Service request containing paths with is_outline field and area
            
        Returns:
            OptimizePathsResponse: Service response with optimized paths
        """
        try:
            rospy.loginfo(f"Received optimization request for {len(request.paths)} paths in area {request.area}")
            
            # Get area configuration for path ordering
            area_config = self.config_loader.get_area_config(request.area)
            inner_first = area_config.get('inner_first', False)
            
            # Create list of path info for processing
            path_info_list = []
            for i, slic3r_path_msg in enumerate(request.paths):
                path_info_list.append({
                    'original_slic3r_path': slic3r_path_msg,
                    'original_nav_path': slic3r_path_msg.path,  # Extract nav_msgs/Path
                    'is_outline': bool(slic3r_path_msg.is_outline),
                    'original_index': i
                })
            
            # Optimize each path individually
            optimized_path_info = []
            for path_info in path_info_list:
                nav_path_msg = path_info['original_nav_path']
                
                # Extract waypoints from nav_msgs/Path message
                waypoints = self.extract_waypoints(nav_path_msg)
                
                if len(waypoints) < 2:
                    rospy.logwarn(f"Path {path_info['original_index']} has fewer than 2 waypoints, skipping optimization")
                    optimized_nav_path = nav_path_msg  # Use original path
                else:
                    # Create area-specific processor
                    area_processor = self.get_area_specific_processor(request.area)
                    
                    # Check if this is an outline path - skip reversal for outline paths
                    if path_info['is_outline']:
                        # For outline paths, don't apply reversal even if configured for the area
                        processed_waypoints = waypoints  # Use original waypoints without reversal
                        rospy.logdebug(f"Path {path_info['original_index']}: Outline path - skipping reversal")
                    else:
                        # For inner paths, apply configured processing (including reversal if enabled)
                        processed_waypoints = area_processor.optimize_path(waypoints)
                        rospy.logdebug(f"Path {path_info['original_index']}: Inner path - applying area processing")
                    
                    # Create processed nav_msgs/Path message
                    optimized_nav_path = self.create_path_message(
                        processed_waypoints, 
                        nav_path_msg.header
                    )
                    
                    rospy.logdebug(f"Path {path_info['original_index']}: {len(waypoints)} -> {len(processed_waypoints)} waypoints")
                
                # Store optimized path with metadata
                path_info['optimized_nav_path'] = optimized_nav_path
                optimized_path_info.append(path_info)
            
            # Reorder paths based on area configuration
            final_slic3r_paths = self.reorder_paths_by_area_config(optimized_path_info, inner_first, area_config)
            
            # Create response
            response = OptimizePathsResponse()
            response.paths = final_slic3r_paths
            
            rospy.loginfo(f"Successfully optimized and reordered {len(final_slic3r_paths)} paths for area {request.area}")
            if inner_first:
                inner_count = sum(1 for info in optimized_path_info if not info['is_outline'])
                outline_count = sum(1 for info in optimized_path_info if info['is_outline'])
                rospy.loginfo(f"Reordered with inner_first=True: {inner_count} inner paths, {outline_count} outline paths")
            
            return response
            
        except Exception as e:
            rospy.logerr(f"Error in optimize service: {str(e)}")
            # Return original paths on error
            response = OptimizePathsResponse()
            response.paths = request.paths
            return response
    
    def get_area_config_callback(self, request):
        """
        Service callback for fetching area configuration
        
        Args:
            request (GetAreaConfigRequest): Service request containing area ID
            
        Returns:
            GetAreaConfigResponse: Service response with area configuration fields
        """
        try:
            area_id = request.area
            rospy.loginfo(f"Received get_area_config request for area {area_id}")
            
            # Get area configuration
            area_config = self.config_loader.get_area_config(area_id)
            
            # Create response with structured fields
            response = GetAreaConfigResponse()
            response.success = True
            response.message = f"Successfully retrieved configuration for area {area_id}"
            
            # Basic configuration fields
            response.name = area_config.get('name', f'Area {area_id}')
            response.inner_first = bool(area_config.get('inner_first', False))
            response.reverse = bool(area_config.get('reverse', False))
            response.outlines_count = int(area_config.get('outlines_count', 4))
            
            # Start point
            start_point = area_config.get('start_point', {'x': 0.0, 'y': 0.0})
            response.start_point_x = float(start_point.get('x', 0.0))
            response.start_point_y = float(start_point.get('y', 0.0))
            
            # Fix point (optional)
            fix_point = area_config.get('fix_point', None)
            if fix_point is not None and isinstance(fix_point, dict):
                response.has_fix_point = True
                response.fix_point_x = float(fix_point.get('x', 0.0))
                response.fix_point_y = float(fix_point.get('y', 0.0))
            else:
                response.has_fix_point = False
                response.fix_point_x = 0.0
                response.fix_point_y = 0.0
            
            rospy.loginfo(f"Successfully returned config for area {area_id}")
            return response
            
        except Exception as e:
            rospy.logerr(f"Error in get_area_config service: {str(e)}")
            response = GetAreaConfigResponse()
            response.success = False
            response.message = f"Error retrieving config for area {request.area}: {str(e)}"
            # Set default values for all fields
            response.name = ""
            response.inner_first = False
            response.reverse = False
            response.outlines_count = 4
            response.start_point_x = 0.0
            response.start_point_y = 0.0
            response.has_fix_point = False
            response.fix_point_x = 0.0
            response.fix_point_y = 0.0
            return response
    
    def get_area_specific_processor(self, area_id):
        """
        Get area-specific path processor configuration
        
        Args:
            area_id (int): Area identifier
            
        Returns:
            PathOptimizerCore: Configured processor for the specific area
        """
        # Get configuration from YAML file
        config = self.config_loader.get_area_config(area_id)
        
        # Log configuration details if enabled
        if self.global_settings.get('log_optimization_statistics', True):
            area_name = config.get('name', f'Area {area_id}')
            reverse_setting = config.get('reverse', False)
            rospy.logdebug(f"Creating processor for {area_name}: reverse={reverse_setting}")
        
        return PathOptimizerCore(config=config)
    
    def reorder_paths_by_area_config(self, path_info_list, inner_first, area_config=None):
        """
        Reorder paths based on area configuration with intelligent distance-based ordering
        
        Args:
            path_info_list (list): List of path info dictionaries with optimized_nav_path and is_outline
            inner_first (bool): Whether to put inner paths first
            area_config (dict): Area configuration containing start_point
            
        Returns:
            list: Reordered list of optimized slic3r_coverage_planner/Path messages
        """
        if not inner_first:
            # If inner_first is False, return paths in original order
            return [self.create_slic3r_path_message(path_info['optimized_nav_path'], path_info['is_outline']) 
                    for path_info in path_info_list]
        
        # Separate inner and outline paths
        inner_path_infos = []
        outline_paths = []
        
        for path_info in path_info_list:
            if path_info['is_outline']:
                slic3r_path_msg = self.create_slic3r_path_message(path_info['optimized_nav_path'], path_info['is_outline'])
                outline_paths.append(slic3r_path_msg)
            else:
                inner_path_infos.append(path_info)
        
        # Order inner paths based on distance optimization
        if inner_path_infos:
            ordered_inner_paths = self.optimize_inner_path_order(inner_path_infos, area_config)
        else:
            ordered_inner_paths = []
        
        # Return inner paths first, then outline paths
        reordered_paths = ordered_inner_paths + outline_paths
        
        rospy.logdebug(f"Reordered paths: {len(ordered_inner_paths)} inner + {len(outline_paths)} outline = {len(reordered_paths)} total")
        
        return reordered_paths
    
    def optimize_inner_path_order(self, inner_path_infos, area_config):
        """
        Optimize the order of inner paths to minimize travel distance
        
        Args:
            inner_path_infos (list): List of inner path info dictionaries
            area_config (dict): Area configuration containing start_point
            
        Returns:
            list: Optimally ordered slic3r_coverage_planner/Path messages
        """
        if not inner_path_infos:
            return []
        
        if len(inner_path_infos) == 1:
            # Only one path, return it as slic3r message
            return [self.create_slic3r_path_message(inner_path_infos[0]['optimized_nav_path'], False)]
        
        # Get area start point or determine optimal starting position
        if area_config and 'start_point' in area_config:
            start_point_config = area_config['start_point']
            # Convert path_infos to slic3r format for resolve_start_position
            inner_paths = [self.create_slic3r_path_message(path_info['optimized_nav_path'], False) 
                          for path_info in inner_path_infos]
            start_x, start_y = self.resolve_start_position(start_point_config, inner_paths)
            rospy.logdebug(f"Using configured start point: {start_point_config} -> ({start_x}, {start_y})")
        else:
            # No start point configured - find path with lowest right position (min x + min y)
            start_x, start_y = self.find_lowest_right_start_position(inner_path_infos)
            rospy.logdebug(f"No start point configured - using lowest right position: ({start_x}, {start_y})")
        
        rospy.logdebug(f"Optimizing order for {len(inner_path_infos)} inner paths from start point ({start_x}, {start_y})")
        
        # Extract path start and end points
        path_data = []
        for i, path_info in enumerate(inner_path_infos):
            nav_path = path_info['optimized_nav_path']
            if len(nav_path.poses) >= 2:
                start_pos = nav_path.poses[0].pose.position
                end_pos = nav_path.poses[-1].pose.position
                
                path_data.append({
                    'index': i,
                    'path_info': path_info,
                    'start': (start_pos.x, start_pos.y),
                    'end': (end_pos.x, end_pos.y)
                })
            else:
                rospy.logwarn(f"Inner path {i} has insufficient waypoints for ordering optimization")
                path_data.append({
                    'index': i,
                    'path_info': path_info,
                    'start': (start_x, start_y),  # Fallback to area start
                    'end': (start_x, start_y)
                })
        
        if not path_data:
            return []
        
        # First, find the path with start point closest to the configured start position
        first_path = None
        min_start_distance = float('inf')
        first_path_index = -1
        
        for i, path_info in enumerate(path_data):
            distance_to_start_pos = self.calculate_distance((start_x, start_y), path_info['start'])
            if distance_to_start_pos < min_start_distance:
                min_start_distance = distance_to_start_pos
                first_path = path_info
                first_path_index = i
        
        if first_path is None:
            rospy.logwarn("Could not find starting path")
            return []
        
        # Start with the closest path to start position
        ordered_paths = [first_path]
        remaining_paths = path_data.copy()
        remaining_paths.pop(first_path_index)
        current_position = first_path['end']
        
        rospy.logdebug(f"Starting with path {first_path['index']} (distance from start: {min_start_distance:.2f}m)")
        
        # Use greedy nearest neighbor for remaining paths
        while remaining_paths:
            # Find the nearest path start point to current position
            min_distance = float('inf')
            nearest_path = None
            nearest_index = -1
            
            for i, path_info in enumerate(remaining_paths):
                # Calculate distance to path start
                distance_to_start = self.calculate_distance(current_position, path_info['start'])
                
                if distance_to_start < min_distance:
                    min_distance = distance_to_start
                    nearest_path = path_info
                    nearest_index = i
            
            if nearest_path:
                # Add the nearest path to ordered list
                ordered_paths.append(nearest_path)
                
                # Update current position to the end of this path
                current_position = nearest_path['end']
                
                # Remove from remaining paths
                remaining_paths.pop(nearest_index)
                
                rospy.logdebug(f"Selected path {nearest_path['index']} (distance: {min_distance:.2f}m)")
            else:
                # Safety break
                break
        
        # Convert back to slic3r_coverage_planner/Path messages
        result_paths = []
        for path_data_item in ordered_paths:
            path_info = path_data_item['path_info']
            slic3r_path = self.create_slic3r_path_message(path_info['optimized_nav_path'], False)
            result_paths.append(slic3r_path)
        
        # Calculate total travel distance for logging
        total_distance = 0.0
        prev_end = (start_x, start_y)
        for path_data_item in ordered_paths:
            total_distance += self.calculate_distance(prev_end, path_data_item['start'])
            prev_end = path_data_item['end']
        
        rospy.loginfo(f"Inner path ordering optimized: total inter-path travel distance = {total_distance:.2f}m")
        
        return result_paths
    
    def resolve_start_position(self, start_point_config, inner_path_infos):
        """
        Resolve start position based on configuration type (enum or coordinates)
        
        Args:
            start_point_config (dict): Start point configuration from YAML
            inner_path_infos (list): List of inner path info dictionaries
            
        Returns:
            tuple: (x, y) coordinates of the resolved start position
        """
        start_type = start_point_config.get('type', 'bottom_left')
        
        if start_type == 'coordinates':
            # Use custom coordinates
            x = float(start_point_config.get('x', 0.0))
            y = float(start_point_config.get('y', 0.0))
            rospy.logdebug(f"Using custom coordinates: ({x}, {y})")
            return (x, y)
        
        # Calculate bounding box of all inner path start points
        if not inner_path_infos:
            rospy.logwarn("No inner paths available for start position calculation")
            return (0.0, 0.0)
        
        # Get all start positions
        start_positions = []
        for path_info in inner_path_infos:
            nav_path = path_info['optimized_nav_path']
            if len(nav_path.poses) > 0:
                start_pos = nav_path.poses[0].pose.position
                start_positions.append((start_pos.x, start_pos.y))
        
        if not start_positions:
            rospy.logwarn("No valid start positions found")
            return (0.0, 0.0)
        
        # Calculate bounding box
        min_x = min(pos[0] for pos in start_positions)
        max_x = max(pos[0] for pos in start_positions)
        min_y = min(pos[1] for pos in start_positions)
        max_y = max(pos[1] for pos in start_positions)
        
        # Select position based on enum
        if start_type == 'top_left':
            position = (min_x, max_y)
        elif start_type == 'top_right':
            position = (max_x, max_y)
        elif start_type == 'bottom_left':
            position = (min_x, min_y)
        elif start_type == 'bottom_right':
            position = (max_x, min_y)
        else:
            rospy.logwarn(f"Unknown start_point type '{start_type}', using bottom_left")
            position = (min_x, min_y)
        
        rospy.logdebug(f"Calculated {start_type} position: ({position[0]:.2f}, {position[1]:.2f}) from bounding box")
        rospy.logdebug(f"  Bounding box: x=[{min_x:.2f}, {max_x:.2f}], y=[{min_y:.2f}, {max_y:.2f}]")
        
        return position
    
    def find_lowest_right_start_position(self, inner_path_infos):
        """
        Find the start position of the path with the lowest right position (minimum x + y)
        
        Args:
            inner_path_infos (list): List of inner path info dictionaries
            
        Returns:
            tuple: (x, y) coordinates of the lowest right start position
        """
        if not inner_path_infos:
            return (0.0, 0.0)
        
        min_sum = float('inf')
        lowest_right_position = (0.0, 0.0)
        
        for path_info in inner_path_infos:
            nav_path = path_info['optimized_nav_path']
            if len(nav_path.poses) > 0:
                start_pos = nav_path.poses[0].pose.position
                position_sum = start_pos.x + start_pos.y
                
                rospy.logdebug(f"Path start at ({start_pos.x:.2f}, {start_pos.y:.2f}), sum = {position_sum:.2f}")
                
                if position_sum < min_sum:
                    min_sum = position_sum
                    lowest_right_position = (start_pos.x, start_pos.y)
        
        rospy.loginfo(f"Selected lowest right start position: ({lowest_right_position[0]:.2f}, {lowest_right_position[1]:.2f}) with sum = {min_sum:.2f}")
        return lowest_right_position
    
    def calculate_distance(self, point1, point2):
        """
        Calculate Euclidean distance between two points
        
        Args:
            point1 (tuple): (x, y) coordinates
            point2 (tuple): (x, y) coordinates
            
        Returns:
            float: Distance between points
        """
        import math
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
    
    def extract_waypoints(self, path_msg):
        """
        Extract waypoints from Path message
        
        Args:
            path_msg (nav_msgs/Path): Input path message
            
        Returns:
            numpy.ndarray: Array of waypoints [[x1, y1], [x2, y2], ...]
        """
        waypoints = []
        for pose_stamped in path_msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            waypoints.append([x, y])
        
        return np.array(waypoints)
    
    def create_path_message(self, waypoints, header):
        """
        Create Path message from waypoints
        
        Args:
            waypoints (numpy.ndarray): Optimized waypoints
            header (std_msgs/Header): Header from original path
            
        Returns:
            nav_msgs/Path: Optimized path message
        """
        path_msg = Path()
        path_msg.header = header
        path_msg.header.stamp = rospy.Time.now()
        
        for waypoint in waypoints:
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            pose_stamped.pose.position.x = float(waypoint[0])
            pose_stamped.pose.position.y = float(waypoint[1])
            pose_stamped.pose.position.z = 0.0
            
            # Keep orientation as identity quaternion for simplicity
            pose_stamped.pose.orientation.x = 0.0
            pose_stamped.pose.orientation.y = 0.0
            pose_stamped.pose.orientation.z = 0.0
            pose_stamped.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose_stamped)
        
        return path_msg
    
    def create_slic3r_path_message(self, nav_path, is_outline: bool):
        """
        Create slic3r_coverage_planner/Path message from nav_msgs/Path and is_outline flag
        
        Args:
            nav_path (nav_msgs/Path): The nav_msgs/Path message
            is_outline (bool): Whether this path is an outline/boundary path
            
        Returns:
            slic3r_coverage_planner/Path: The slic3r_coverage_planner path message
        """
        try:
            # Import the message type dynamically to avoid import issues
            from slic3r_coverage_planner.msg import Path as Slic3rPath
            
            slic3r_path_msg = Slic3rPath()
            slic3r_path_msg.path = nav_path
            slic3r_path_msg.is_outline = 1 if is_outline else 0  # Convert bool to uint8
            
            return slic3r_path_msg
            
        except ImportError:
            rospy.logerr("Failed to import slic3r_coverage_planner.msg.Path - make sure the package is built")
            # Fallback: create a simple object with the required fields
            class FallbackPath:
                def __init__(self):
                    self.path = nav_path
                    self.is_outline = 1 if is_outline else 0
            return FallbackPath()
    
    def run(self):
        """Run the node"""
        rospy.loginfo("Path Optimizer Node running...")
        rospy.spin()


if __name__ == '__main__':
    try:
        node = PathOptimizerNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Path Optimizer Node interrupted")
    except Exception as e:
        rospy.logerr(f"Path Optimizer Node failed: {str(e)}")