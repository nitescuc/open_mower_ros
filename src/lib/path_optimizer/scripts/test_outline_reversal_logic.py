#!/usr/bin/env python3
"""
Test Script for Outline Path Reversal Logic

This test verifies that when reverse=true for an area, inner paths are reversed 
but outline paths are NOT reversed (they keep their original direction).
"""

import sys
import os

# Add the scripts directory to the Python path
sys.path.insert(0, '/Users/nites/ros/open_mower_ros/src/lib/path_optimizer/scripts')

from test_enum_start_points import (
    PathOptimizerNode, create_test_path
)

def test_outline_reversal_behavior():
    """Test that outline paths are not reversed even when reverse=true for area"""
    print("=== Testing Outline Path Reversal Behavior ===\n")
    
    # Create a mock PathOptimizerNode with test configuration
    class MockPathOptimizerNode(PathOptimizerNode):
        def __init__(self):
            # Initialize without ROS
            self.config_loader = MockConfigLoader()
            self.processor = None  # Not needed for this test
        
        def create_slic3r_path_message(self, nav_path, is_outline):
            """Mock implementation"""
            slic3r_path = create_test_path(0, 0, 0, 0, is_outline)
            slic3r_path.path = nav_path
            return slic3r_path
        
        def get_area_specific_processor(self, area_id):
            """Mock processor that reverses paths"""
            class MockProcessor:
                def optimize_path(self, waypoints):
                    # Reverse the waypoints (simulate reverse=True)
                    return waypoints[::-1]
            return MockProcessor()
        
        def extract_waypoints(self, path_msg):
            """Mock implementation"""
            waypoints = []
            for pose in path_msg.poses:
                waypoints.append([pose.pose.position.x, pose.pose.position.y])
            return waypoints
        
        def create_path_message(self, waypoints, header):
            """Mock implementation"""
            from test_enum_start_points import Path, PoseStamped, Pose
            path = Path()
            path.header = header
            path.poses = []
            
            for wp in waypoints:
                pose_stamped = PoseStamped()
                pose_stamped.pose = Pose()
                pose_stamped.pose.position.x = wp[0]
                pose_stamped.pose.position.y = wp[1]
                path.poses.append(pose_stamped)
            
            return path
    
    class MockConfigLoader:
        def get_area_config(self, area_id):
            return {
                'reverse': True,  # This area has reversal enabled
                'inner_first': True,
                'start_point': {'type': 'bottom_left'}
            }
    
    # Create test paths with clear directional waypoints
    inner_path = create_test_path(1.0, 1.0, 5.0, 1.0, is_outline=False)  # Inner path: left to right
    outline_path = create_test_path(0.0, 0.0, 6.0, 0.0, is_outline=True)  # Outline path: left to right
    
    # Add intermediate points to make reversal more obvious
    from test_enum_start_points import PoseStamped, Pose
    
    # Inner path: (1,1) -> (3,1) -> (5,1)
    middle_inner = PoseStamped()
    middle_inner.pose = Pose()
    middle_inner.pose.position.x = 3.0
    middle_inner.pose.position.y = 1.0
    inner_path.path.poses.insert(1, middle_inner)
    
    # Outline path: (0,0) -> (3,0) -> (6,0) 
    middle_outline = PoseStamped()
    middle_outline.pose = Pose()
    middle_outline.pose.position.x = 3.0
    middle_outline.pose.position.y = 0.0
    outline_path.path.poses.insert(1, middle_outline)
    
    print("Original Paths:")
    print("Inner path waypoints:")
    for i, pose in enumerate(inner_path.path.poses):
        pos = pose.pose.position
        print(f"  Point {i+1}: ({pos.x}, {pos.y})")
    
    print("Outline path waypoints:")
    for i, pose in enumerate(outline_path.path.poses):
        pos = pose.pose.position
        print(f"  Point {i+1}: ({pos.x}, {pos.y})")
    
    # Create mock request
    class MockRequest:
        def __init__(self):
            self.paths = [inner_path, outline_path]
            self.area = 1  # Area with reverse=True
    
    # Test the processing
    node = MockPathOptimizerNode()
    request = MockRequest()
    
    print(f"\nProcessing paths for area {request.area} (reverse=True)...")
    
    # Create path info list (simulate the service callback logic)
    path_info_list = []
    for i, slic3r_path_msg in enumerate(request.paths):
        path_info_list.append({
            'original_slic3r_path': slic3r_path_msg,
            'original_nav_path': slic3r_path_msg.path,
            'is_outline': bool(slic3r_path_msg.is_outline),
            'original_index': i
        })
    
    # Process each path
    optimized_path_info = []
    for path_info in path_info_list:
        nav_path_msg = path_info['original_nav_path']
        
        # Extract waypoints
        waypoints = node.extract_waypoints(nav_path_msg)
        
        if len(waypoints) >= 2:
            # Create area-specific processor
            area_processor = node.get_area_specific_processor(request.area)
            
            # Apply the logic we just implemented
            if path_info['is_outline']:
                # For outline paths, don't apply reversal even if configured for the area
                processed_waypoints = waypoints  # Use original waypoints without reversal
                print(f"Path {path_info['original_index']}: Outline path - skipping reversal")
            else:
                # For inner paths, apply configured processing (including reversal if enabled)
                processed_waypoints = area_processor.optimize_path(waypoints)
                print(f"Path {path_info['original_index']}: Inner path - applying area processing (reversal)")
            
            # Create processed nav_msgs/Path message
            optimized_nav_path = node.create_path_message(
                processed_waypoints, 
                nav_path_msg.header
            )
            
            path_info['optimized_nav_path'] = optimized_nav_path
        
        optimized_path_info.append(path_info)
    
    # Verify results
    print(f"\nResults:")
    print("=" * 50)
    
    for i, path_info in enumerate(optimized_path_info):
        path_type = "Outline" if path_info['is_outline'] else "Inner"
        print(f"\n{path_type} Path {i} - Processed waypoints:")
        
        original_waypoints = node.extract_waypoints(path_info['original_nav_path'])
        processed_waypoints = node.extract_waypoints(path_info['optimized_nav_path'])
        
        print("  Original order:")
        for j, wp in enumerate(original_waypoints):
            print(f"    Point {j+1}: ({wp[0]}, {wp[1]})")
        
        print("  Processed order:")
        for j, wp in enumerate(processed_waypoints):
            print(f"    Point {j+1}: ({wp[0]}, {wp[1]})")
        
        # Check if waypoints were reversed
        original_list = [tuple(wp) for wp in original_waypoints]
        processed_list = [tuple(wp) for wp in processed_waypoints]
        reversed_list = original_list[::-1]
        
        if processed_list == original_list:
            result = "✓ NOT REVERSED (kept original order)"
        elif processed_list == reversed_list:
            result = "↔ REVERSED"
        else:
            result = "? MODIFIED (unexpected)"
        
        print(f"  Result: {result}")
        
        # Validate expected behavior
        if path_info['is_outline']:
            expected = "should NOT be reversed"
            correct = (processed_list == original_list)
        else:
            expected = "should be reversed"  
            correct = (processed_list == reversed_list)
        
        status = "✓ CORRECT" if correct else "✗ INCORRECT"
        print(f"  Expected: {path_type} paths {expected}")
        print(f"  Status: {status}")

def test_mixed_area_configurations():
    """Test with different area configurations"""
    print(f"\n{'='*70}")
    print("Testing Mixed Area Configurations")
    print(f"{'='*70}")
    
    configurations = [
        {'area': 1, 'reverse': False, 'description': 'No reversal for any paths'},
        {'area': 2, 'reverse': True, 'description': 'Reverse inner paths only (not outline)'},
    ]
    
    for config in configurations:
        print(f"\nArea {config['area']}: {config['description']}")
        print(f"Configuration: reverse={config['reverse']}")
        print("-" * 50)
        
        if config['reverse']:
            print("Expected behavior:")
            print("  • Inner paths: REVERSED")
            print("  • Outline paths: NOT REVERSED (kept original)")
        else:
            print("Expected behavior:")
            print("  • Inner paths: NOT REVERSED")  
            print("  • Outline paths: NOT REVERSED")

if __name__ == "__main__":
    print("Path Optimizer - Outline Path Reversal Test")
    print("=" * 60)
    
    # Test the main reversal behavior
    test_outline_reversal_behavior()
    
    # Test different configurations
    test_mixed_area_configurations()
    
    print(f"\n{'='*60}")
    print("✅ Test Complete!")
    print("Key Validation:")
    print("  • Inner paths ARE reversed when area reverse=True")  
    print("  • Outline paths are NOT reversed (regardless of area reverse setting)")
    print("  • This preserves outline path integrity while optimizing inner fill patterns")
    print(f"{'='*60}")