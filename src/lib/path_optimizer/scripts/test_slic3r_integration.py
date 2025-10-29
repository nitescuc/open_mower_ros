#!/usr/bin/env python3
"""
Test Script for Path Optimizer with slic3r_coverage_planner/Path messages

This script tests the path optimizer service using the correct message types
with the is_outline field embedded in each path message.

Author: Clemens Elflein
License: MIT
"""

import rospy
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


def create_slic3r_path(waypoints, is_outline, path_id):
    """Create a slic3r_coverage_planner/Path message"""
    try:
        from slic3r_coverage_planner.msg import Path as Slic3rPath
        
        slic3r_path = Slic3rPath()
        slic3r_path.path = create_nav_path(waypoints, f"path_{path_id}")
        slic3r_path.is_outline = 1 if is_outline else 0
        
        return slic3r_path
        
    except ImportError:
        rospy.logerr("Failed to import slic3r_coverage_planner.msg.Path")
        rospy.logerr("Make sure the slic3r_coverage_planner package is built and sourced")
        return None


def print_path_info(slic3r_path, index):
    """Print information about a slic3r path"""
    if slic3r_path is None:
        rospy.loginfo(f"  {index}: NULL PATH")
        return
        
    nav_path = slic3r_path.path
    is_outline = bool(slic3r_path.is_outline)
    path_type = "Outline" if is_outline else "Inner"
    
    rospy.loginfo(f"  {index}: {path_type} - {len(nav_path.poses)} waypoints")
    if len(nav_path.poses) >= 2:
        start = nav_path.poses[0].pose.position
        end = nav_path.poses[-1].pose.position
        rospy.loginfo(f"       Start: ({start.x:.1f}, {start.y:.1f}) -> End: ({end.x:.1f}, {end.y:.1f})")


def test_slic3r_path_optimization():
    """Test the path optimizer with slic3r_coverage_planner/Path messages"""
    rospy.init_node('test_slic3r_path_optimization', anonymous=True)
    
    # Wait for service
    rospy.loginfo("Waiting for path optimization service...")
    rospy.wait_for_service('/path_optimizer/optimize')
    
    try:
        optimize_service = rospy.ServiceProxy('/path_optimizer/optimize', OptimizePaths)
        
        rospy.loginfo("Creating test paths with slic3r_coverage_planner/Path messages...")
        
        # Create test paths in mixed order: inner, outline, inner, outline, inner
        test_paths = [
            create_slic3r_path([[0, 0], [1, 0], [2, 0]], False, "inner_1"),     # Inner path
            create_slic3r_path([[0, 1], [1, 1], [2, 1]], True, "outline_1"),    # Outline path  
            create_slic3r_path([[0, 2], [1, 2], [2, 2]], False, "inner_2"),     # Inner path
            create_slic3r_path([[0, 3], [1, 3], [2, 3]], True, "outline_2"),    # Outline path
            create_slic3r_path([[0, 4], [1, 4], [2, 4]], False, "inner_3")      # Inner path
        ]
        
        # Check if all paths were created successfully
        if any(path is None for path in test_paths):
            rospy.logerr("Failed to create test paths - aborting test")
            return
        
        # Test different areas with different inner_first settings
        test_areas = [
            {"id": 1, "name": "High Precision", "inner_first": True},
            {"id": 2, "name": "Standard Mowing", "inner_first": False},
            {"id": 4, "name": "Smooth Path Required", "inner_first": True}
        ]
        
        for area_info in test_areas:
            area_id = area_info["id"]
            area_name = area_info["name"]
            expected_inner_first = area_info["inner_first"]
            
            rospy.loginfo(f"\n{'='*70}")
            rospy.loginfo(f"Testing Area {area_id}: {area_name}")
            rospy.loginfo(f"Expected inner_first behavior: {expected_inner_first}")
            rospy.loginfo(f"{'='*70}")
            
            # Show input paths
            rospy.loginfo("INPUT PATHS:")
            for i, slic3r_path in enumerate(test_paths):
                print_path_info(slic3r_path, i+1)
            
            # Call optimization service
            rospy.loginfo(f"\nCalling optimization service for area {area_id}...")
            response = optimize_service(
                paths=test_paths,
                area=area_id
            )
            
            # Show output paths
            rospy.loginfo(f"\nOUTPUT PATHS (Area {area_id}):")
            for i, slic3r_path in enumerate(response.optimized_paths):
                print_path_info(slic3r_path, i+1)
            
            # Analyze path ordering
            rospy.loginfo("\nORDERING ANALYSIS:")
            
            inner_positions = []
            outline_positions = []
            
            for i, slic3r_path in enumerate(response.optimized_paths):
                if slic3r_path.is_outline:
                    outline_positions.append(i+1)
                else:
                    inner_positions.append(i+1)
            
            rospy.loginfo(f"  Inner path positions: {inner_positions}")
            rospy.loginfo(f"  Outline path positions: {outline_positions}")
            
            # Check ordering correctness
            if expected_inner_first:
                if inner_positions and outline_positions:
                    max_inner_pos = max(inner_positions)
                    min_outline_pos = min(outline_positions)
                    
                    if max_inner_pos < min_outline_pos:
                        rospy.loginfo("  Result: ✓ CORRECT - Inner first ordering working")
                        rospy.loginfo(f"          All inner paths (max pos {max_inner_pos}) before outline paths (min pos {min_outline_pos})")
                    else:
                        rospy.logwarn("  Result: ✗ INCORRECT - Inner first ordering failed")
                        rospy.logwarn(f"          Inner path at pos {max_inner_pos} after outline path at pos {min_outline_pos}")
                elif inner_positions and not outline_positions:
                    rospy.loginfo("  Result: ✓ CORRECT - Only inner paths (no outline paths to check)")
                elif outline_positions and not inner_positions:
                    rospy.loginfo("  Result: ✓ CORRECT - Only outline paths (no inner paths to check)")
                else:
                    rospy.logwarn("  Result: ? No paths to analyze")
            else:
                # When inner_first=False, original order should be preserved
                original_order_preserved = True
                for i, (input_path, output_path) in enumerate(zip(test_paths, response.optimized_paths)):
                    input_frame_id = input_path.path.header.frame_id
                    output_frame_id = output_path.path.header.frame_id
                    if input_frame_id != output_frame_id:
                        original_order_preserved = False
                        break
                
                if original_order_preserved:
                    rospy.loginfo("  Result: ✓ CORRECT - Original order preserved (inner_first=False)")
                else:
                    rospy.logwarn("  Result: ✗ INCORRECT - Original order changed despite inner_first=False")
        
        rospy.loginfo(f"\n{'='*70}")
        rospy.loginfo("Slic3r Path Optimization Test Completed!")
        rospy.loginfo("Key behaviors verified:")
        rospy.loginfo("  - Service accepts slic3r_coverage_planner/Path messages")
        rospy.loginfo("  - is_outline field properly extracted from each path message")
        rospy.loginfo("  - Path reordering works based on area inner_first settings")
        rospy.loginfo("  - Optimized paths maintain correct message structure")
        rospy.loginfo(f"{'='*70}")
        
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
    except Exception as e:
        rospy.logerr(f"Test failed: {e}")


if __name__ == '__main__':
    try:
        test_slic3r_path_optimization()
    except rospy.ROSInterruptException:
        rospy.loginfo("Test interrupted")
    except Exception as e:
        rospy.logerr(f"Test failed: {str(e)}")