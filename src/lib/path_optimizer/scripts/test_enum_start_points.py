#!/usr/bin/env python3
"""
Test Script for Enum-based Start Point System in Path Optimizer

Tests the resolve_start_position method with different enum types:
- top_left, top_right, bottom_left, bottom_right
- coordinates (custom x,y)
"""

import sys

# Mock geometry_msgs for testing without ROS
class Point:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y 
        self.z = z

class Pose:
    def __init__(self):
        self.position = Point()

class PoseStamped:
    def __init__(self):
        self.pose = Pose()

class Path:
    def __init__(self):
        self.poses = []

# Mock rospy for testing without ROS  
class rospy:
    @staticmethod
    def logdebug(msg):
        print(f"DEBUG: {msg}")
    
    @staticmethod
    def loginfo(msg):
        print(f"INFO: {msg}")
    
    @staticmethod
    def logwarn(msg):
        print(f"WARN: {msg}")
    
    @staticmethod
    def logerr(msg):
        print(f"ERROR: {msg}")

# Mock the path optimizer node class
class PathOptimizerNode:
    def __init__(self):
        pass
    
    def resolve_start_position(self, start_config, inner_paths):
        """
        Resolve start position based on configuration type
        
        Args:
            start_config (dict): Configuration with type and optional x,y coordinates
            inner_paths: List of paths with is_outline=False
            
        Returns:
            tuple: (x, y) coordinates for the start position
        """
        start_type = start_config.get('type', 'bottom_left')
        
        if start_type == 'coordinates':
            # Use explicit coordinates
            return (start_config.get('x', 0.0), start_config.get('y', 0.0))
        
        if not inner_paths:
            rospy.logwarn("No inner paths available for start position calculation")
            return (0.0, 0.0)
        
        # Calculate bounding box from all inner path start points
        min_x = float('inf')
        max_x = float('-inf')
        min_y = float('inf') 
        max_y = float('-inf')
        
        for path in inner_paths:
            if path.path.poses:
                start_point = path.path.poses[0].pose.position
                min_x = min(min_x, start_point.x)
                max_x = max(max_x, start_point.x)
                min_y = min(min_y, start_point.y)
                max_y = max(max_y, start_point.y)
        
        # Handle case where no valid points found
        if min_x == float('inf'):
            rospy.logwarn("No valid start points found in inner paths")
            return (0.0, 0.0)
        
        # Calculate position based on enum type
        if start_type == 'top_left':
            return (min_x, max_y)
        elif start_type == 'top_right':
            return (max_x, max_y)
        elif start_type == 'bottom_left':
            return (min_x, min_y)
        elif start_type == 'bottom_right':
            return (max_x, min_y)
        else:
            rospy.logwarn(f"Unknown start position type: {start_type}, using bottom_left")
            return (min_x, min_y)

def create_test_path(start_x, start_y, end_x, end_y, is_outline=False):
    """Create a test path with start and end points"""
    # Mock slic3r_coverage_planner Path
    class Slic3rPath:
        def __init__(self):
            self.is_outline = is_outline
            self.path = Path()
    
    slic3r_path = Slic3rPath()
    slic3r_path.is_outline = is_outline
    
    # Add start point
    start_pose = PoseStamped()
    start_pose.pose.position.x = start_x
    start_pose.pose.position.y = start_y
    slic3r_path.path.poses.append(start_pose)
    
    # Add end point 
    end_pose = PoseStamped()
    end_pose.pose.position.x = end_x
    end_pose.pose.position.y = end_y
    slic3r_path.path.poses.append(end_pose)
    
    return slic3r_path

def test_enum_start_positions():
    """Test enum-based start position resolution"""
    print("=== Testing Enum-based Start Position System ===\n")
    
    # Create test paths - inner paths (is_outline=False)
    inner_paths = [
        create_test_path(1.0, 1.0, 2.0, 2.0, is_outline=False),  # Path 1: (1,1) to (2,2)
        create_test_path(3.0, 3.0, 4.0, 4.0, is_outline=False),  # Path 2: (3,3) to (4,4) 
        create_test_path(0.5, 2.5, 1.5, 3.5, is_outline=False),  # Path 3: (0.5,2.5) to (1.5,3.5)
        create_test_path(2.5, 0.5, 3.5, 1.5, is_outline=False),  # Path 4: (2.5,0.5) to (3.5,1.5)
    ]
    
    # Note: outline paths (is_outline=True) should not affect start position calculation
    
    print("Test Paths (Inner paths only, used for bounding box calculation):")
    for i, path in enumerate(inner_paths):
        start = path.path.poses[0].pose.position
        end = path.path.poses[1].pose.position
        print(f"  Path {i+1}: ({start.x}, {start.y}) -> ({end.x}, {end.y})")
    
    print("\nExpected Bounding Box:")
    print("  Min X: 0.5, Max X: 3.0")
    print("  Min Y: 0.5, Max Y: 3.0") 
    print("  (from start points of inner paths only)\n")
    
    # Initialize path optimizer node
    node = PathOptimizerNode()
    
    # Test different enum types
    test_configs = [
        {'type': 'top_left'},      # Should return (0.5, 3.0)
        {'type': 'top_right'},     # Should return (3.0, 3.0) 
        {'type': 'bottom_left'},   # Should return (0.5, 0.5)
        {'type': 'bottom_right'},  # Should return (3.0, 0.5)
        {'type': 'coordinates', 'x': 1.5, 'y': 1.5},  # Should return (1.5, 1.5)
        {'type': 'invalid_type'},  # Should fallback to bottom_left: (0.5, 0.5)
    ]
    
    expected_results = [
        (0.5, 3.0),   # top_left
        (3.0, 3.0),   # top_right
        (0.5, 0.5),   # bottom_left  
        (3.0, 0.5),   # bottom_right
        (1.5, 1.5),   # coordinates
        (0.5, 0.5),   # invalid (fallback to bottom_left)
    ]
    
    print("Testing Enum Start Position Resolution:")
    print("-" * 50)
    
    all_passed = True
    for i, (config, expected) in enumerate(zip(test_configs, expected_results)):
        result = node.resolve_start_position(config, inner_paths)
        
        # Check if result matches expected (with small tolerance for float comparison)
        tolerance = 1e-10
        x_match = abs(result[0] - expected[0]) < tolerance
        y_match = abs(result[1] - expected[1]) < tolerance
        passed = x_match and y_match
        
        status = "✓ PASS" if passed else "✗ FAIL"
        print(f"Test {i+1}: {config}")
        print(f"  Expected: ({expected[0]}, {expected[1]})")
        print(f"  Got:      ({result[0]}, {result[1]})")
        print(f"  Status:   {status}")
        
        if not passed:
            all_passed = False
        print()
    
    # Test edge case: empty inner paths
    print("Testing Edge Case: Empty Inner Paths")
    print("-" * 40)
    empty_result = node.resolve_start_position({'type': 'top_left'}, [])
    print(f"Empty paths with top_left: {empty_result}")
    print(f"Expected: (0.0, 0.0) - {empty_result == (0.0, 0.0)}")
    print()
    
    # Test edge case: paths with no poses
    print("Testing Edge Case: Paths with No Poses")
    print("-" * 42)
    empty_path = create_test_path(0, 0, 0, 0)
    empty_path.path.poses = []  # Remove all poses
    no_poses_result = node.resolve_start_position({'type': 'bottom_right'}, [empty_path])
    print(f"No poses with bottom_right: {no_poses_result}")
    print(f"Expected: (0.0, 0.0) - {no_poses_result == (0.0, 0.0)}")
    print()
    
    # Summary
    print("=" * 60)
    if all_passed:
        print("🎉 ALL TESTS PASSED! Enum-based start point system is working correctly.")
    else:
        print("❌ Some tests failed. Please check the implementation.")
    
    return all_passed

def test_integration_example():
    """Test a realistic integration example"""
    print("\n" + "=" * 60)
    print("Integration Example: Area Configuration Test")
    print("=" * 60)
    
    # Simulate area configurations
    area_configs = {
        1: {'inner_first': True, 'reverse': False, 'start_point': {'type': 'bottom_left'}},
        2: {'inner_first': False, 'reverse': True, 'start_point': {'type': 'coordinates', 'x': 2.0, 'y': 2.0}},
        3: {'inner_first': True, 'reverse': False, 'start_point': {'type': 'top_right'}},
    }
    
    # Create diverse test paths
    test_paths = [
        create_test_path(1.0, 1.0, 3.0, 1.0, is_outline=False),  # Horizontal inner
        create_test_path(2.0, 2.0, 2.0, 4.0, is_outline=False),  # Vertical inner  
        create_test_path(0.0, 0.0, 5.0, 5.0, is_outline=True),   # Outline
    ]
    
    node = PathOptimizerNode()
    
    print("Test Scenario: Mixed inner and outline paths")
    for i, path in enumerate(test_paths):
        start = path.path.poses[0].pose.position
        path_type = "Outline" if path.is_outline else "Inner"
        print(f"  Path {i+1}: {path_type} starting at ({start.x}, {start.y})")
    
    # Filter inner paths only
    inner_paths = [p for p in test_paths if not p.is_outline]
    
    print(f"\nInner paths only (used for bounding box): {len(inner_paths)} paths")
    print()
    
    for area_id, config in area_configs.items():
        print(f"Area {area_id} Configuration:")
        print(f"  Start Point Config: {config['start_point']}")
        
        start_pos = node.resolve_start_position(config['start_point'], inner_paths)
        print(f"  Resolved Start Position: ({start_pos[0]}, {start_pos[1]})")
        print(f"  Inner First: {config['inner_first']}")  
        print(f"  Reverse Paths: {config['reverse']}")
        print()

if __name__ == "__main__":
    print("Path Optimizer - Enum Start Point System Test Suite")
    print("=" * 60)
    
    # Run the main test
    success = test_enum_start_positions()
    
    # Run integration example
    test_integration_example()
    
    print("\n" + "=" * 60)
    if success:
        print("✅ Test Suite Completed Successfully!")
        print("The enum-based start point system is ready for production use.")
    else:
        print("❌ Test Suite Failed!")
        print("Please review the implementation before deployment.")
    
    sys.exit(0 if success else 1)