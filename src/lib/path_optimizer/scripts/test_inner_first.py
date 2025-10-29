#!/usr/bin/env python3
"""
Test Inner First Path Ordering

This script tests that when inner_first=True, paths with is_outline=False
come before paths with is_outline=True in the result.

Author: Clemens Elflein
License: MIT
"""

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from path_optimizer.srv import OptimizePaths
from std_msgs.msg import Header


def create_test_path(waypoints, path_id, frame_id="map"):
    """Create a Path message from waypoints with unique ID in frame_id"""
    path_msg = Path()
    path_msg.header = Header()
    path_msg.header.stamp = rospy.Time.now()
    path_msg.header.frame_id = f"{frame_id}_path_{path_id}"  # Unique identifier
    
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


def test_inner_first_ordering():
    """Test that inner_first correctly orders paths"""
    rospy.init_node('test_inner_first_ordering', anonymous=True)
    
    # Wait for service
    rospy.loginfo("Waiting for path optimization service...")
    rospy.wait_for_service('/path_optimizer/optimize')
    
    try:
        optimize_service = rospy.ServiceProxy('/path_optimizer/optimize', OptimizePaths)
        
        # Create test paths with clear identification
        rospy.loginfo("Creating test paths for inner_first ordering test...")
        
        # Create paths in mixed order: inner, outline, inner, outline, inner
        paths = [
            create_test_path([[0, 0], [1, 0], [2, 0]], "inner_1"),      # is_outline=False
            create_test_path([[0, 1], [1, 1], [2, 1]], "outline_1"),   # is_outline=True  
            create_test_path([[0, 2], [1, 2], [2, 2]], "inner_2"),     # is_outline=False
            create_test_path([[0, 3], [1, 3], [2, 3]], "outline_2"),   # is_outline=True
            create_test_path([[0, 4], [1, 4], [2, 4]], "inner_3")      # is_outline=False
        ]
        
        # Corresponding is_outline flags (mixed order)
        is_outline_flags = [False, True, False, True, False]
        
        path_types = ["Inner", "Outline", "Inner", "Outline", "Inner"]
        
        # Test areas with different inner_first settings
        test_areas = [
            {"id": 1, "name": "Front Yard", "inner_first": True},    # Should reorder
            {"id": 2, "name": "Standard Mowing", "inner_first": False}  # Should keep original order
        ]
        
        for area_info in test_areas:
            area_id = area_info["id"]
            area_name = area_info["name"]
            inner_first_setting = area_info["inner_first"]
            
            rospy.loginfo(f"\n{'='*70}")
            rospy.loginfo(f"Testing Area {area_id}: {area_name}")
            rospy.loginfo(f"inner_first setting: {inner_first_setting}")
            rospy.loginfo(f"{'='*70}")
            
            # Show input order
            rospy.loginfo("INPUT ORDER:")
            for i, (path_type, is_outline) in enumerate(zip(path_types, is_outline_flags)):
                rospy.loginfo(f"  {i+1}. {path_type} (is_outline={is_outline})")
            
            # Call optimization service
            response = optimize_service(
                paths=paths,
                is_outline=is_outline_flags,
                area=area_id
            )
            
            # Analyze output order
            rospy.loginfo(f"\nOUTPUT ORDER (Area {area_id}):")
            
            inner_positions = []
            outline_positions = []
            
            for i, output_path in enumerate(response.optimized_paths):
                # Identify original path by frame_id
                original_frame_id = output_path.header.frame_id
                
                # Find which original path this corresponds to
                original_index = -1
                for j, input_path in enumerate(paths):
                    if input_path.header.frame_id == original_frame_id:
                        original_index = j
                        break
                
                if original_index >= 0:
                    original_type = path_types[original_index]
                    original_is_outline = is_outline_flags[original_index]
                    rospy.loginfo(f"  {i+1}. {original_type} (is_outline={original_is_outline})")
                    
                    if original_is_outline:
                        outline_positions.append(i+1)
                    else:
                        inner_positions.append(i+1)
                else:
                    rospy.loginfo(f"  {i+1}. Unknown path")
            
            # Check ordering correctness
            rospy.loginfo("\nORDERING ANALYSIS:")
            rospy.loginfo(f"  Inner path positions: {inner_positions}")
            rospy.loginfo(f"  Outline path positions: {outline_positions}")
            
            if inner_first_setting:
                # When inner_first=True, all inner paths should come before all outline paths
                if inner_positions and outline_positions:
                    max_inner_pos = max(inner_positions)
                    min_outline_pos = min(outline_positions)
                    
                    if max_inner_pos < min_outline_pos:
                        rospy.loginfo(f"  Result: ✓ CORRECT - All inner paths (max pos {max_inner_pos}) come before outline paths (min pos {min_outline_pos})")
                    else:
                        rospy.logwarn(f"  Result: ✗ INCORRECT - Inner path at position {max_inner_pos} comes after outline path at position {min_outline_pos}")
                elif inner_positions and not outline_positions:
                    rospy.loginfo("  Result: ✓ CORRECT - Only inner paths present")
                elif outline_positions and not inner_positions:
                    rospy.loginfo("  Result: ✓ CORRECT - Only outline paths present")
                else:
                    rospy.loginfo("  Result: ? No paths to analyze")
            else:
                # When inner_first=False, order should be preserved
                original_order_preserved = True
                for i, output_path in enumerate(response.optimized_paths):
                    original_frame_id = output_path.header.frame_id
                    expected_frame_id = paths[i].header.frame_id
                    if original_frame_id != expected_frame_id:
                        original_order_preserved = False
                        break
                
                if original_order_preserved:
                    rospy.loginfo("  Result: ✓ CORRECT - Original order preserved (inner_first=False)")
                else:
                    rospy.logwarn("  Result: ✗ INCORRECT - Original order not preserved despite inner_first=False")
        
        rospy.loginfo(f"\n{'='*70}")
        rospy.loginfo("Inner First Ordering Test Completed!")
        rospy.loginfo("Expected behavior:")
        rospy.loginfo("  - When inner_first=True: All is_outline=False paths before is_outline=True paths")
        rospy.loginfo("  - When inner_first=False: Original order preserved")
        rospy.loginfo(f"{'='*70}")
        
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
    except Exception as e:
        rospy.logerr(f"Test failed: {e}")


if __name__ == '__main__':
    try:
        test_inner_first_ordering()
    except rospy.ROSInterruptException:
        rospy.loginfo("Test interrupted")
    except Exception as e:
        rospy.logerr(f"Test failed: {str(e)}")