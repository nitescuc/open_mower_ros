#!/usr/bin/env python3
"""
Path Content Reversal Test Script

This script specifically tests the path content reversal functionality
when the reverse parameter is set to true in area configurations.

Author: Clemens Elflein
License: MIT
"""

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from path_optimizer.srv import OptimizePaths
from std_msgs.msg import Header


def create_simple_path(waypoints, frame_id="map"):
    """Create a Path message from waypoints"""
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


def print_path_waypoints(path, path_name="Path"):
    """Print waypoints of a path for debugging"""
    rospy.loginfo(f"{path_name} waypoints:")
    for i, pose in enumerate(path.poses):
        pos = pose.pose.position
        rospy.loginfo(f"  {i+1}: ({pos.x:.1f}, {pos.y:.1f})")


def test_path_content_reversal():
    """Test path content reversal functionality"""
    rospy.init_node('path_content_reversal_test', anonymous=True)
    
    # Wait for service
    rospy.loginfo("Waiting for path optimization service...")
    rospy.wait_for_service('/path_optimizer/optimize')
    
    try:
        optimize_service = rospy.ServiceProxy('/path_optimizer/optimize', OptimizePaths)
        
        # Create test paths with distinct waypoint sequences
        rospy.loginfo("Creating test paths with distinct waypoint sequences...")
        
        # Straight line path
        straight_path = create_simple_path([
            [0, 0], [1, 0], [2, 0], [3, 0], [4, 0]
        ])
        
        # L-shaped path
        l_shaped_path = create_simple_path([
            [0, 0], [0, 1], [0, 2], [1, 2], [2, 2]
        ])
        
        # Zigzag path
        zigzag_path = create_simple_path([
            [0, 0], [1, 1], [2, 0], [3, 1], [4, 0]
        ])
        
        test_paths = [straight_path, l_shaped_path, zigzag_path]
        path_names = ["Straight Line", "L-Shape", "Zigzag"]
        is_outline = [False, False, True]  # First two are inner, last is outline
        
        # Test areas with different reverse settings
        test_areas = [
            {"id": 2, "name": "Standard Mowing", "reverse": False},
            {"id": 3, "name": "Large Open Areas", "reverse": True}, 
            {"id": 4, "name": "Smooth Path Required", "reverse": True}
        ]
        
        for area_info in test_areas:
            area_id = area_info["id"]
            area_name = area_info["name"]
            expected_reverse = area_info["reverse"]
            
            rospy.loginfo(f"\n{'='*60}")
            rospy.loginfo(f"Testing Area {area_id}: {area_name}")
            rospy.loginfo(f"Expected reverse behavior: {expected_reverse}")
            rospy.loginfo(f"{'='*60}")
            
            # Show input paths
            rospy.loginfo("INPUT PATHS:")
            for i, (path, name) in enumerate(zip(test_paths, path_names)):
                rospy.loginfo(f"\n{i+1}. {name}:")
                print_path_waypoints(path, "  Input")
            
            # Call optimization service
            response = optimize_service(
                paths=test_paths,
                is_outline=is_outline,
                area=area_id
            )
            
            # Show output paths and analyze reversal
            rospy.loginfo(f"\nOUTPUT PATHS (Area {area_id}):")
            
            for i, (input_path, output_path, name) in enumerate(zip(test_paths, response.optimized_paths, path_names)):
                rospy.loginfo(f"\n{i+1}. {name}:")
                print_path_waypoints(output_path, "  Output")
                
                # Check if path was reversed by comparing first and last waypoints
                if len(input_path.poses) > 1 and len(output_path.poses) > 1:
                    input_start = input_path.poses[0].pose.position
                    input_end = input_path.poses[-1].pose.position
                    output_start = output_path.poses[0].pose.position
                    output_end = output_path.poses[-1].pose.position
                    
                    # Calculate distances to determine if reversed
                    same_direction = (
                        abs(input_start.x - output_start.x) + abs(input_start.y - output_start.y) +
                        abs(input_end.x - output_end.x) + abs(input_end.y - output_end.y)
                    )
                    
                    reversed_direction = (
                        abs(input_start.x - output_end.x) + abs(input_start.y - output_end.y) +
                        abs(input_end.x - output_start.x) + abs(input_end.y - output_start.y)
                    )
                    
                    is_actually_reversed = reversed_direction < same_direction
                    
                    rospy.loginfo(f"  Analysis: Path direction {'REVERSED' if is_actually_reversed else 'UNCHANGED'}")
                    
                    # Check if behavior matches expectation
                    if is_actually_reversed == expected_reverse:
                        rospy.loginfo(f"  Result: ✓ CORRECT (expected reverse={expected_reverse})")
                    else:
                        rospy.logwarn(f"  Result: ✗ UNEXPECTED (expected reverse={expected_reverse}, got reverse={is_actually_reversed})")
                else:
                    rospy.loginfo("  Analysis: Cannot determine reversal (insufficient waypoints)")
        
        rospy.loginfo(f"\n{'='*60}")
        rospy.loginfo("Path content reversal test completed!")
        rospy.loginfo("Summary of expected behaviors:")
        rospy.loginfo("  - Area 2 (Standard): reverse=False → paths should be unchanged")
        rospy.loginfo("  - Area 3 (Large Open): reverse=True → paths should be reversed")
        rospy.loginfo("  - Area 4 (Smooth Path): reverse=True → paths should be reversed")
        rospy.loginfo(f"{'='*60}")
        
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
    except Exception as e:
        rospy.logerr(f"Test failed: {e}")


if __name__ == '__main__':
    try:
        test_path_content_reversal()
    except rospy.ROSInterruptException:
        rospy.loginfo("Test interrupted")
    except Exception as e:
        rospy.logerr(f"Test failed: {str(e)}")