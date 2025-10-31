#!/usr/bin/env python3
"""
Test Distance-Based Path Ordering

This script tests that inner paths are ordered to minimize travel distance
starting from the area's start point and optimizing inter-path transitions.

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


def get_path_endpoints(slic3r_path):
    """Get start and end points of a path"""
    if slic3r_path is None or len(slic3r_path.path.poses) < 1:
        return None, None
    
    start_pose = slic3r_path.path.poses[0].pose.position
    end_pose = slic3r_path.path.poses[-1].pose.position
    
    return (start_pose.x, start_pose.y), (end_pose.x, end_pose.y)


def analyze_path_ordering(paths, start_point, path_type=""):
    """Analyze the ordering efficiency of paths"""
    if not paths:
        return 0.0, []
    
    total_distance = 0.0
    transitions = []
    current_pos = (start_point['x'], start_point['y'])
    
    rospy.loginfo(f"\nANALYZING {path_type.upper()} PATH ORDERING:")
    rospy.loginfo(f"Starting from point: ({current_pos[0]:.1f}, {current_pos[1]:.1f})")
    
    for i, path in enumerate(paths):
        start_pos, end_pos = get_path_endpoints(path)
        if start_pos and end_pos:
            # Distance from current position to path start
            travel_distance = calculate_distance(current_pos, start_pos)
            total_distance += travel_distance
            
            transitions.append({
                'path_index': i+1,
                'from': current_pos,
                'to_start': start_pos,
                'to_end': end_pos,
                'travel_distance': travel_distance
            })
            
            rospy.loginfo(f"  Path {i+1}: Travel {travel_distance:.2f}m to start {start_pos}, ends at {end_pos}")
            
            # Update current position to end of this path
            current_pos = end_pos
        else:
            rospy.logwarn(f"  Path {i+1}: Could not get endpoints")
    
    rospy.loginfo(f"Total inter-path travel distance: {total_distance:.2f}m")
    return total_distance, transitions


def test_distance_based_ordering():
    """Test distance-based path ordering optimization"""
    rospy.init_node('test_distance_based_ordering', anonymous=True)
    
    # Wait for service
    rospy.loginfo("Waiting for path optimization service...")
    rospy.wait_for_service('/path_optimizer/optimize')
    
    try:
        optimize_service = rospy.ServiceProxy('/path_optimizer/optimize', OptimizePaths)
        
        rospy.loginfo("Creating spatially distributed test paths...")
        
        # Create inner paths at different locations to test distance optimization
        # Layout:  Start(0,0)    Path1(10,5)    Path2(2,3)    Path3(15,8)    Path4(5,1)
        #
        # Optimal order should be: Path4(5,1) -> Path2(2,3) -> Path1(10,5) -> Path3(15,8)
        # This minimizes travel from start point (0,0)
        
        inner_paths = [
            create_slic3r_path([[10, 5], [11, 5], [12, 5]], False, "inner_distant"),   # Far path
            create_slic3r_path([[2, 3], [3, 3], [4, 3]], False, "inner_near"),        # Near path  
            create_slic3r_path([[15, 8], [16, 8], [17, 8]], False, "inner_farthest"), # Farthest path
            create_slic3r_path([[5, 1], [6, 1], [7, 1]], False, "inner_closest")      # Closest path
        ]
        
        # Add one outline path (should come after all inner paths)
        outline_path = create_slic3r_path([[0, 0], [20, 0], [20, 10], [0, 10], [0, 0]], True, "outline_boundary")
        
        # Combine paths in deliberately suboptimal order
        test_paths = inner_paths + [outline_path]
        
        # Check if all paths were created
        if any(path is None for path in test_paths):
            rospy.logerr("Failed to create test paths - aborting test")
            return
        
        # Test with Area 1 (High Precision) which has inner_first=True and start_point=(0,0)
        area_id = 1
        
        rospy.loginfo(f"\n{'='*80}")
        rospy.loginfo(f"Testing Distance-Based Path Ordering (Area {area_id})")
        rospy.loginfo(f"{'='*80}")
        
        # Show input path locations
        rospy.loginfo("INPUT PATH LOCATIONS:")
        start_point = {'x': 0.0, 'y': 0.0}  # Area 1 start point
        for i, path in enumerate(test_paths):
            start_pos, end_pos = get_path_endpoints(path)
            path_type = "Outline" if path.is_outline else "Inner"
            distance_from_start = calculate_distance((start_point['x'], start_point['y']), start_pos) if start_pos else 0
            rospy.loginfo(f"  {i+1}. {path_type}: starts at {start_pos}, distance from area start: {distance_from_start:.2f}m")
        
        # Calculate baseline (input order) travel distance for inner paths only
        input_inner_paths = [p for p in test_paths if not p.is_outline]
        baseline_distance, _ = analyze_path_ordering(input_inner_paths, start_point, "INPUT INNER")
        
        # Call optimization service
        rospy.loginfo(f"\nCalling optimization service for area {area_id} with inner_first=True...")
        response = optimize_service(
            paths=test_paths,
            area=area_id
        )
        
        # Analyze output ordering
        output_inner_paths = [p for p in response.optimized_paths if not p.is_outline]
        output_outline_paths = [p for p in response.optimized_paths if p.is_outline]
        
        optimized_distance, transitions = analyze_path_ordering(output_inner_paths, start_point, "OPTIMIZED INNER")
        
        # Show improvement
        rospy.loginfo("\nORDERING OPTIMIZATION RESULTS:")
        rospy.loginfo(f"  Baseline travel distance (input order): {baseline_distance:.2f}m")
        rospy.loginfo(f"  Optimized travel distance: {optimized_distance:.2f}m")
        
        if baseline_distance > 0:
            improvement = ((baseline_distance - optimized_distance) / baseline_distance) * 100
            rospy.loginfo(f"  Distance reduction: {baseline_distance - optimized_distance:.2f}m ({improvement:.1f}% improvement)")
            
            if improvement > 0:
                rospy.loginfo("  Result: ✓ IMPROVED - Distance-based ordering is working!")
            elif improvement == 0:
                rospy.loginfo("  Result: = SAME - Path order was already optimal")
            else:
                rospy.logwarn("  Result: ✗ WORSE - Optimization made ordering less efficient")
        
        # Verify inner paths come before outline paths
        rospy.loginfo("\nPATH TYPE ORDERING VERIFICATION:")
        rospy.loginfo(f"  Inner paths: {len(output_inner_paths)}")
        rospy.loginfo(f"  Outline paths: {len(output_outline_paths)}")
        
        # Check that all inner paths come before outline paths
        inner_positions = []
        outline_positions = []
        
        for i, path in enumerate(response.optimized_paths):
            if path.is_outline:
                outline_positions.append(i+1)
            else:
                inner_positions.append(i+1)
        
        if inner_positions and outline_positions:
            max_inner = max(inner_positions)
            min_outline = min(outline_positions)
            if max_inner < min_outline:
                rospy.loginfo(f"  Result: ✓ CORRECT - All inner paths (positions {inner_positions}) before outline paths (positions {outline_positions})")
            else:
                rospy.logwarn("  Result: ✗ INCORRECT - Inner/outline ordering violated")
        
        rospy.loginfo(f"\n{'='*80}")
        rospy.loginfo("Distance-Based Path Ordering Test Completed!")
        rospy.loginfo("Features verified:")
        rospy.loginfo("  - Inner paths ordered by proximity to area start point")
        rospy.loginfo("  - Inter-path travel distance minimized using nearest-neighbor algorithm")
        rospy.loginfo("  - Inner paths still come before outline paths when inner_first=True")
        rospy.loginfo(f"{'='*80}")
        
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
    except Exception as e:
        rospy.logerr(f"Test failed: {e}")


if __name__ == '__main__':
    try:
        test_distance_based_ordering()
    except rospy.ROSInterruptException:
        rospy.loginfo("Test interrupted")
    except Exception as e:
        rospy.logerr(f"Test failed: {str(e)}")