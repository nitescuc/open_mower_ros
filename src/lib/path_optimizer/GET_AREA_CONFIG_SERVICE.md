# GetAreaConfig Service

## Overview
The `GetAreaConfig` service allows clients to fetch the configuration for a specific mowing area. This service is useful for debugging, UI displays, and external tools that need to know the optimization parameters being used for each area.

## Service Definition

**File:** `srv/GetAreaConfig.srv`

```
# Request: Area identifier
int32 area
---
# Response: Area configuration as structured fields
bool success
string message
string name
bool inner_first
bool reverse
int32 outlines_count
float64 start_point_x
float64 start_point_y
bool has_fix_point
float64 fix_point_x
float64 fix_point_y
```

## Request Fields
- `area` (int32): The ID of the area whose configuration you want to retrieve

## Response Fields

### Status Fields
- `success` (bool): Whether the service call succeeded
- `message` (string): Human-readable message describing the result
- `name` (string): Human-readable name for the area

### Optimization Settings
- `inner_first` (bool): If true, inner paths are placed before outline paths
- `reverse` (bool): If true, reverses the direction of inner paths (not applied to outline paths)
- `outlines_count` (int32): Number of outline paths expected (default: 4)

### Position Settings
- `start_point_x` (float64): X-coordinate of the starting point
- `start_point_y` (float64): Y-coordinate of the starting point
- `has_fix_point` (bool): Whether a fix point is configured for this area
- `fix_point_x` (float64): X-coordinate of the fix point (only valid if `has_fix_point` is true)
- `fix_point_y` (float64): Y-coordinate of the fix point (only valid if `has_fix_point` is true)

## Usage Examples

### Command Line (rosservice)

```bash
# Call the service for area 2
rosservice call /path_optimizer_node/get_area_config "area: 2"
```

### Python Example

```python
#!/usr/bin/env python3
import rospy
from path_optimizer.srv import GetAreaConfig

rospy.init_node('config_client')
rospy.wait_for_service('/path_optimizer_node/get_area_config')

get_config = rospy.ServiceProxy('/path_optimizer_node/get_area_config', GetAreaConfig)
response = get_config(area=2)

if response.success:
    print(f"Area: {response.name}")
    print(f"Inner first: {response.inner_first}")
    print(f"Reverse: {response.reverse}")
    print(f"Outlines count: {response.outlines_count}")
    print(f"Start point: ({response.start_point_x}, {response.start_point_y})")
    
    if response.has_fix_point:
        print(f"Fix point: ({response.fix_point_x}, {response.fix_point_y})")
    else:
        print("No fix point set")
else:
    print(f"Error: {response.message}")
```

### C++ Example

```cpp
#include <ros/ros.h>
#include <path_optimizer/GetAreaConfig.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "config_client");
    ros::NodeHandle nh;
    
    ros::ServiceClient client = nh.serviceClient<path_optimizer::GetAreaConfig>(
        "/path_optimizer_node/get_area_config"
    );
    
    path_optimizer::GetAreaConfig srv;
    srv.request.area = 2;
    
    if (client.call(srv)) {
        if (srv.response.success) {
            ROS_INFO("Area: %s", srv.response.name.c_str());
            ROS_INFO("Inner first: %s", srv.response.inner_first ? "true" : "false");
            ROS_INFO("Reverse: %s", srv.response.reverse ? "true" : "false");
            ROS_INFO("Outlines count: %d", srv.response.outlines_count);
            ROS_INFO("Start point: (%.2f, %.2f)", 
                     srv.response.start_point_x, srv.response.start_point_y);
            
            if (srv.response.has_fix_point) {
                ROS_INFO("Fix point: (%.2f, %.2f)", 
                         srv.response.fix_point_x, srv.response.fix_point_y);
            } else {
                ROS_INFO("No fix point set");
            }
        } else {
            ROS_ERROR("Service error: %s", srv.response.message.c_str());
        }
    } else {
        ROS_ERROR("Failed to call service");
    }
    
    return 0;
}
```

## Behavior

### Existing Area
When requesting a configuration for an area that exists in `areas_config.yaml`, the service returns that area's specific configuration.

### Non-existent Area
When requesting a configuration for an area that does NOT exist in the config file, the service returns the **default configuration** defined in the YAML file.

### Error Handling
If an error occurs (e.g., config file parsing error), the service returns:
- `success = false`
- An error message in the `message` field
- Default values for all configuration fields

## Testing

A test script is provided to verify the service:

```bash
# Make sure the path_optimizer_node is running
rosrun path_optimizer test_get_area_config_service.py
```

This script tests multiple area IDs (including non-existent ones) and displays the returned configurations.

## Integration

This service is automatically started when the `path_optimizer_node` is launched. The service is available at:

```
/path_optimizer_node/get_area_config
```

Or with namespace:
```
/<namespace>/path_optimizer_node/get_area_config
```

## See Also

- `OptimizePaths` service - The main path optimization service
- `config/areas_config.yaml` - Area configuration file format
- `PathOptimizerConfig` class - Configuration loader implementation
