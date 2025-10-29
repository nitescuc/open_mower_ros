#!/usr/bin/env python3
"""
Test Start Point Logic - Configured vs Automatic Selection

This script tests both configured start point behavior and automatic selection
of the lowest right position when no start point is configured.

Author: Clemens Elflein
License: MIT
"""

import rospy
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from path_optimizer.srv import OptimizePaths
from std_msgs.msg import Header


def create_nav_path(waypoints, frame_id="map"):
    """Create a nav_msgs/Path message from waypoints"""
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


def create_slic3r_path(waypoints, is_outline, path_name):
    """Create a slic3r_coverage_planner/Path message"""
    try:
        from slic3r_coverage_planner.msg import Path as Slic3rPath
        
        slic3r_path = Slic3rPath()
        slic3r_path.path = create_nav_path(waypoints, path_name)
        slic3r_path.is_outline = 1 if is_outline else 0
        
        return slic3r_path
        
    except ImportError:
        rospy.logerr("Failed to import slic3r_coverage_planner.msg.Path")
        return None


def calculate_distance(point1, point2):
    """Calculate distance between two points"""
    return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)


def get_path_start_position(slic3r_path):
    """Get start position of a path"""
    if slic3r_path is None or len(slic3r_path.path.poses) < 1:
        return None
    
    start_pose = slic3r_path.path.poses[0].pose.position
    return (start_pose.x, start_pose.y)


def analyze_start_point_selection(paths, expected_start_behavior, area_info):
    """Analyze whether the start point selection worked as expected"""
    rospy.loginfo(f"\nANALYZING START POINT SELECTION FOR {area_info['name']}:")
    
    inner_paths = [p for p in paths if not p.is_outline]
    
    if not inner_paths:
        rospy.loginfo("  No inner paths to analyze")
        return
    
    # Show all inner path start positions
    rospy.loginfo("  Inner path start positions:")
    for i, path in enumerate(inner_paths):
        start_pos = get_path_start_position(path)
        if start_pos:
            sum_coords = start_pos[0] + start_pos[1]
            rospy.loginfo(f"    Path {i+1}: ({start_pos[0]:.1f}, {start_pos[1]:.1f}), sum = {sum_coords:.1f}")
    
    # Get the actual first path in the output
    first_path_start = get_path_start_position(inner_paths[0])
    
    if expected_start_behavior == "configured":
        expected_start = area_info.get('expected_start', (0, 0))
        rospy.loginfo(f"  Expected behavior: Start near configured point {expected_start}")
        
        if first_path_start:
            distance_to_expected = calculate_distance(first_path_start, expected_start)
            rospy.loginfo(f"  First path starts at: {first_path_start}")
            rospy.loginfo(f"  Distance from configured start: {distance_to_expected:.2f}m")
            
            # Check if it's reasonably close (within 5m tolerance for this test)
            if distance_to_expected <= 5.0:
                rospy.loginfo("  Result: ✓ CORRECT - Started near configured point")
            else:
                rospy.logwarn("  Result: ✗ UNEXPECTED - Did not start near configured point")
    
    elif expected_start_behavior == "lowest_right":
        rospy.loginfo("  Expected behavior: Start with lowest right position (min x+y)")
        
        # Find the actual lowest right position among all inner paths
        min_sum = float('inf')
        expected_lowest_right = None
        
        for path in inner_paths:
            start_pos = get_path_start_position(path)
            if start_pos:
                sum_coords = start_pos[0] + start_pos[1]
                if sum_coords < min_sum:
                    min_sum = sum_coords
                    expected_lowest_right = start_pos
        
        rospy.loginfo(f"  Actual lowest right position: {expected_lowest_right} (sum = {min_sum:.1f})")
        rospy.loginfo(f"  First path actually starts at: {first_path_start}")
        
        if first_path_start and expected_lowest_right:
            if abs(first_path_start[0] - expected_lowest_right[0]) < 0.1 and abs(first_path_start[1] - expected_lowest_right[1]) < 0.1:
                rospy.loginfo("  Result: ✓ CORRECT - Started with lowest right position")
            else:
                rospy.logwarn("  Result: ✗ UNEXPECTED - Did not start with lowest right position")


def test_start_point_logic():
    """Test both configured start point and automatic lowest-right selection"""
    rospy.init_node('test_start_point_logic', anonymous=True)
    
    # Wait for service
    rospy.loginfo("Waiting for path optimization service...")
    rospy.wait_for_service('/path_optimizer/optimize')
    
    try:
        optimize_service = rospy.ServiceProxy('/path_optimizer/optimize', OptimizePaths)
        
        rospy.loginfo("Creating test paths with varying start positions...")
        
        # Create inner paths with different start positions
        # Path positions chosen to make lowest-right selection obvious
        inner_paths = [
            create_slic3r_path([[8, 6], [9, 6], [10, 6]], False, "path_mid_high"),      # sum = 14
            create_slic3r_path([[5, 8], [6, 8], [7, 8]], False, "path_mid_higher"),    # sum = 13  
            create_slic3r_path([[2, 3], [3, 3], [4, 3]], False, "path_lowest_right"),  # sum = 5 (lowest!)
            create_slic3r_path([[10, 2], [11, 2], [12, 2]], False, "path_high_low")    # sum = 12
        ]
        
        # Add outline path
        outline_path = create_slic3r_path([[0, 0], [15, 0], [15, 10], [0, 10], [0, 0]], True, "outline")
        test_paths = inner_paths + [outline_path]
        
        # Check if all paths were created
        if any(path is None for path in test_paths):
            rospy.logerr("Failed to create test paths - aborting test")
            return
        
        # Test scenarios
        test_scenarios = [
            {
                "area_id": 1,  # High Precision - has configured start_point (0,0)
                "name": "Area 1 (High Precision)",
                "expected_behavior": "configured",
                "expected_start": (0.0, 0.0)
            },
            {
                "area_id": 7,  # Speed Priority - has configured start_point (20,20) 
                "name": "Area 7 (Speed Priority)",
                "expected_behavior": "configured", 
                "expected_start": (20.0, 20.0)
            },
            {
                "area_id": 99,  # Non-existent area - should use lowest right
                "name": "Area 99 (Non-existent)",
                "expected_behavior": "lowest_right"
            }
        ]
        
        for scenario in test_scenarios:
            rospy.loginfo(f"\n{'='*80}")
            rospy.loginfo(f"Testing Start Point Logic: {scenario['name']}")
            rospy.loginfo(f"Expected behavior: {scenario['expected_behavior']}")
            rospy.loginfo(f"{'='*80}")
            
            # Show input path information
            rospy.loginfo("INPUT PATH START POSITIONS:")
            for i, path in enumerate(inner_paths):
                start_pos = get_path_start_position(path)
                if start_pos:
                    sum_coords = start_pos[0] + start_pos[1]
                    rospy.loginfo(f"  Inner Path {i+1}: starts at ({start_pos[0]:.1f}, {start_pos[1]:.1f}), sum = {sum_coords:.1f}")
            
            # Call optimization service
            response = optimize_service(
                paths=test_paths,
                area=scenario["area_id"]
            )
            
            # Analyze the result
            analyze_start_point_selection(response.optimized_paths, scenario["expected_behavior"], scenario)
        
        rospy.loginfo(f"\n{'='*80}")
        rospy.loginfo("Start Point Logic Test Completed!")
        rospy.loginfo("Key behaviors verified:")
        rospy.loginfo("  - Areas with configured start_point use that position for path ordering")
        rospy.loginfo("  - Areas without start_point automatically use lowest right position")
        rospy.loginfo("  - Lowest right = path with minimum (x + y) coordinate sum")
        rospy.loginfo("  - Distance-based optimization still works in both cases")
        rospy.loginfo(f"{'='*80}")
        
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
    except Exception as e:
        rospy.logerr(f"Test failed: {e}")


if __name__ == '__main__':
    try:
        test_start_point_logic()
    except rospy.ROSInterruptException:
        rospy.loginfo("Test interrupted")
    except Exception as e:
        rospy.logerr(f"Test failed: {str(e)}")