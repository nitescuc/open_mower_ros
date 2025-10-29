#!/usr/bin/env python3
"""
Path Optimizer Service Client Example

This script demonstrates how to use the path optimization service.
It creates sample paths and calls the optimization service.

Author: Clemens Elflein
License: MIT
"""

import rospy
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from path_optimizer.srv import OptimizePaths
from std_msgs.msg import Header


def create_sample_path(waypoints, frame_id="map"):
    """
    Create a Path message from waypoints
    
    Args:
        waypoints (list): List of [x, y] coordinates
        frame_id (str): Frame ID for the path
        
    Returns:
        nav_msgs/Path: Path message
    """
    path_msg = Path()
    path_msg.header = Header()
    path_msg.header.stamp = rospy.Time.now()
    path_msg.header.frame_id = frame_id
    
    for waypoint in waypoints:
        pose_stamped = PoseStamped()
        pose_stamped.header = path_msg.header
        pose_stamped.pose.position.x = float(waypoint[0])
        pose_stamped.pose.position.y = float(waypoint[1])
        pose_stamped.pose.position.z = 0.0
        
        # Identity quaternion
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = 0.0
        pose_stamped.pose.orientation.w = 1.0
        
        path_msg.poses.append(pose_stamped)
    
    return path_msg


def test_optimization_service():
    """Test the path optimization service"""
    rospy.init_node('path_optimizer_client_test', anonymous=True)
    
    # Wait for service to be available
    rospy.loginfo("Waiting for path optimization service...")
    rospy.wait_for_service('/path_optimizer/optimize')
    
    try:
        # Create service proxy
        optimize_service = rospy.ServiceProxy('/path_optimizer/optimize', OptimizePaths)
        
        # Create sample paths for testing
        sample_paths = []
        
        # Path 1: Zigzag pattern (good for Douglas-Peucker)
        zigzag_waypoints = [
            [0, 0], [1, 0.1], [2, 0], [3, 0.1], [4, 0], [5, 0.1], [6, 0]
        ]
        sample_paths.append(create_sample_path(zigzag_waypoints))
        
        # Path 2: Noisy straight line (good for smoothing)
        noisy_line = []
        for i in range(20):
            noise_x = np.random.normal(0, 0.05)
            noise_y = np.random.normal(0, 0.02)
            noisy_line.append([i * 0.5 + noise_x, 2.0 + noise_y])
        sample_paths.append(create_sample_path(noisy_line))
        
        # Path 3: Curved path with many points
        curve_waypoints = []
        for i in range(50):
            t = i / 49.0 * 2 * np.pi
            x = 5 * np.cos(t)
            y = 5 * np.sin(t)
            curve_waypoints.append([x, y + 5])
        sample_paths.append(create_sample_path(curve_waypoints))
        
        # Test different area configurations
        test_areas = [1, 2, 3, 4, 99]  # Including unknown area
        
        for area_id in test_areas:
            rospy.loginfo(f"\n=== Testing Area {area_id} ===")
            
            # Create is_outline flags for testing
            # Let's make the first path an inner path and the others outline paths
            is_outline_flags = [False, True, True]  # First path is inner, others are outline
            
            # Call the optimization service
            response = optimize_service(
                paths=sample_paths, 
                is_outline=is_outline_flags,
                area=area_id
            )
            
            # Display results
            rospy.loginfo(f"Area {area_id} Results:")
            rospy.loginfo(f"  Input: {len(sample_paths)} paths (inner_first affects path order)")
            
            for i, (original, optimized) in enumerate(zip(sample_paths, response.optimized_paths)):
                original_count = len(original.poses)
                optimized_count = len(optimized.poses)
                reduction = (1.0 - optimized_count / original_count) * 100 if original_count > 0 else 0
                path_type = "outline" if (i < len(is_outline_flags) and is_outline_flags[i]) else "inner"
                
                rospy.loginfo(f"  Path {i+1} ({path_type}): {original_count} -> {optimized_count} waypoints ({reduction:.1f}% reduction)")
            
            # Show path reordering effect
            rospy.loginfo(f"  Note: Path ordering may differ from input based on area's inner_first setting")
        
        rospy.loginfo("\n=== Service Test Completed Successfully ===")
        
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
    except Exception as e:
        rospy.logerr(f"Test failed: {e}")


if __name__ == '__main__':
    try:
        test_optimization_service()
    except rospy.ROSInterruptException:
        rospy.loginfo("Test interrupted")
    except Exception as e:
        rospy.logerr(f"Test failed: {str(e)}")