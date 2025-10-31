#!/usr/bin/env python3
"""
Path Reordering Test Script

This script specifically tests the inner_fir            rospy.loginfo(f"Input path order:")
            for i, (path_name, is_outline) in enumerate(zip(path_names, is_outline_flags)):
                path_type = "OUTLINE" if is_outline else "INNER  "
                first_point = test_paths[i].poses[0].pose.position
                last_point = test_paths[i].poses[-1].pose.position
                rospy.loginfo(f"  {i+1}. [{path_type}] {path_name}")
                rospy.loginfo(f"      Start: ({first_point.x:.1f}, {first_point.y:.1f}) -> End: ({last_point.x:.1f}, {last_point.y:.1f})")
            
            # Call optimization service
            response = optimize_service(
                paths=test_paths,
                is_outline=is_outline_flags,
                area=area_id
            )rdering functionality
of the path optimizer service.

Author: Clemens Elflein
License: MIT
"""

import rospy
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from path_optimizer.srv import OptimizePaths
from std_msgs.msg import Header


def create_test_path(waypoints, frame_id="map", path_name=""):
    """Create a Path message from waypoints with optional name in header"""
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


def test_path_reordering():
    """Test path reordering functionality"""
    rospy.init_node('path_reordering_test', anonymous=True)
    
    # Wait for service
    rospy.loginfo("Waiting for path optimization service...")
    rospy.wait_for_service('/path_optimizer/optimize')
    
    try:
        optimize_service = rospy.ServiceProxy('/path_optimizer/optimize', OptimizePaths)
        
        # Create test paths
        rospy.loginfo("Creating test paths...")
        
        # Inner path 1 (small inner area)
        inner_path_1 = create_test_path([
            [2, 2], [3, 2], [3, 3], [2, 3], [2, 2]
        ], path_name="Inner Area 1")
        
        # Inner path 2 (another small inner area)  
        inner_path_2 = create_test_path([
            [5, 5], [6, 5], [6, 6], [5, 6], [5, 5]
        ], path_name="Inner Area 2")
        
        # Outline path 1 (boundary)
        outline_path_1 = create_test_path([
            [0, 0], [10, 0], [10, 10], [0, 10], [0, 0]
        ], path_name="Outer Boundary")
        
        # Outline path 2 (another boundary)
        outline_path_2 = create_test_path([
            [1, 1], [9, 1], [9, 9], [1, 9], [1, 1]  
        ], path_name="Inner Boundary")
        
        # Test paths and their outline flags
        test_paths = [inner_path_1, outline_path_1, inner_path_2, outline_path_2]
        is_outline_flags = [False, True, False, True]  # inner, outline, inner, outline
        path_names = ["Inner Area 1", "Outer Boundary", "Inner Area 2", "Inner Boundary"]
        
        # Test different areas with different inner_first and reverse settings
        test_scenarios = [
            {"area": 1, "name": "High Precision (inner_first=True, reverse=False)"},
            {"area": 2, "name": "Standard Mowing (inner_first=False, reverse=False)"},
            {"area": 3, "name": "Large Open Areas (inner_first=False, reverse=True)"},
            {"area": 4, "name": "Smooth Path Required (inner_first=True, reverse=True)"},
            {"area": 5, "name": "Narrow Passages (inner_first=True, reverse=False)"}
        ]
        
        for scenario in test_scenarios:
            area_id = scenario["area"]
            scenario_name = scenario["name"]
            
            rospy.loginfo(f"\n{'='*60}")
            rospy.loginfo(f"Testing Scenario: {scenario_name}")
            rospy.loginfo(f"{'='*60}")
            
            rospy.loginfo("Input path order:")
            for i, (path_name, is_outline) in enumerate(zip(path_names, is_outline_flags)):
                path_type = "OUTLINE" if is_outline else "INNER  "
                rospy.loginfo(f"  {i+1}. [{path_type}] {path_name}")
            
            # Call optimization service
            response = optimize_service(
                paths=test_paths,
                is_outline=is_outline_flags,
                area=area_id
            )
            
            rospy.loginfo(f"\nOutput path order (Area {area_id}):")
            
            # To determine the output order, we need to identify which path is which
            # We'll use the number of waypoints as a simple identifier
            input_waypoint_counts = [len(path.poses) for path in test_paths]
            output_waypoint_counts = [len(path.poses) for path in response.optimized_paths]
            
            for i, output_count in enumerate(output_waypoint_counts):
                # Find which input path this corresponds to
                matching_indices = [j for j, input_count in enumerate(input_waypoint_counts) 
                                  if input_count == output_count]
                
                if matching_indices:
                    # Take the first match (assuming unique waypoint counts for this test)
                    original_idx = matching_indices[0]
                    path_name = path_names[original_idx]
                    is_outline = is_outline_flags[original_idx]
                    path_type = "OUTLINE" if is_outline else "INNER  "
                    
                    # Check if path content has been reversed
                    original_path = test_paths[original_idx]
                    output_path = response.optimized_paths[i]
                    
                    if len(original_path.poses) > 1 and len(output_path.poses) > 1:
                        orig_start = original_path.poses[0].pose.position
                        orig_end = original_path.poses[-1].pose.position
                        out_start = output_path.poses[0].pose.position
                        out_end = output_path.poses[-1].pose.position
                        
                        # Simple reversal detection: check if start/end are swapped
                        start_distance = ((orig_start.x - out_start.x)**2 + (orig_start.y - out_start.y)**2)**0.5
                        end_distance = ((orig_end.x - out_end.x)**2 + (orig_end.y - out_end.y)**2)**0.5
                        cross_start_distance = ((orig_start.x - out_end.x)**2 + (orig_start.y - out_end.y)**2)**0.5
                        cross_end_distance = ((orig_end.x - out_start.x)**2 + (orig_end.y - out_start.y)**2)**0.5
                        
                        # If cross distances are smaller, path was likely reversed
                        is_reversed = (cross_start_distance + cross_end_distance) < (start_distance + end_distance)
                        reverse_indicator = " (REVERSED)" if is_reversed else ""
                    else:
                        reverse_indicator = ""
                    
                    rospy.loginfo(f"  {i+1}. [{path_type}] {path_name} (was position {original_idx+1}){reverse_indicator}")
                else:
                    rospy.loginfo(f"  {i+1}. [UNKNOWN] Path with {output_count} waypoints")
            
            # Analysis
            rospy.loginfo(f"\nAnalysis:")
            inner_positions = []
            outline_positions = []
            
            for i, output_count in enumerate(output_waypoint_counts):
                matching_indices = [j for j, input_count in enumerate(input_waypoint_counts) 
                                  if input_count == output_count]
                if matching_indices:
                    original_idx = matching_indices[0]
                    if is_outline_flags[original_idx]:
                        outline_positions.append(i+1)
                    else:
                        inner_positions.append(i+1)
            
            rospy.loginfo(f"  Inner paths at positions: {inner_positions}")
            rospy.loginfo(f"  Outline paths at positions: {outline_positions}")
            
            if inner_positions and outline_positions:
                inner_first_applied = max(inner_positions) < min(outline_positions)
                rospy.loginfo(f"  Inner-first ordering: {'✓ APPLIED' if inner_first_applied else '✗ NOT APPLIED'}")
            else:
                rospy.loginfo(f"  Mixed path types not available for comparison")
        
        rospy.loginfo(f"\n{'='*60}")
        rospy.loginfo("Path reordering test completed!")
        rospy.loginfo("Expected behavior:")
        rospy.loginfo("  - Areas with inner_first=True should have inner paths before outline paths")
        rospy.loginfo("  - Areas with inner_first=False should maintain original order")
        rospy.loginfo(f"{'='*60}")
        
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
    except Exception as e:
        rospy.logerr(f"Test failed: {e}")


if __name__ == '__main__':
    try:
        test_path_reordering()
    except rospy.ROSInterruptException:
        rospy.loginfo("Test interrupted")
    except Exception as e:
        rospy.logerr(f"Test failed: {str(e)}")