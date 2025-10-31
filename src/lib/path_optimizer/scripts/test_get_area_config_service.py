#!/usr/bin/env python3
"""
Test script for GetAreaConfig service

This script tests the get_area_config service to ensure it correctly
returns area configurations.
"""

import rospy
from path_optimizer.srv import GetAreaConfig, GetAreaConfigRequest

def test_get_area_config():
    """Test the get_area_config service"""
    rospy.init_node('test_get_area_config', anonymous=True)
    
    # Wait for service to be available
    service_name = '/path_optimizer_node/get_area_config'
    print(f"Waiting for service {service_name}...")
    rospy.wait_for_service(service_name, timeout=5.0)
    
    try:
        # Create service proxy
        get_config = rospy.ServiceProxy(service_name, GetAreaConfig)
        
        # Test different area IDs
        test_areas = [1, 2, 3, 999]  # Include non-existent area to test fallback
        
        for area_id in test_areas:
            print(f"\n{'='*60}")
            print(f"Testing area {area_id}")
            print(f"{'='*60}")
            
            # Call service
            request = GetAreaConfigRequest()
            request.area = area_id
            response = get_config(request)
            
            # Display results
            print(f"Success: {response.success}")
            print(f"Message: {response.message}")
            
            if response.success:
                # Display structured fields
                print(f"\nConfiguration for {response.name}:")
                print(f"  Inner First: {response.inner_first}")
                print(f"  Reverse: {response.reverse}")
                print(f"  Outlines Count: {response.outlines_count}")
                print(f"  Start Point: ({response.start_point_x}, {response.start_point_y})")
                
                if response.has_fix_point:
                    print(f"  Fix Point: ({response.fix_point_x}, {response.fix_point_y})")
                else:
                    print("  Fix Point: Not set")
                
                print("\n✓ Configuration retrieved successfully")
            else:
                print(f"Error: {response.message}")
        
        print(f"\n{'='*60}")
        print("All tests completed!")
        print(f"{'='*60}")
        
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
    except rospy.ROSException as e:
        print(f"ROS error: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")

if __name__ == '__main__':
    try:
        test_get_area_config()
    except rospy.ROSInterruptException:
        pass
