# Path Optimizer

The `path_optimizer` package provides advanced path optimization algorithms specifically designed for autonomous mowing applications. It optimizes navigation paths to improve efficiency, reduce energy consumption, and ensure smooth mower operation.

## Overview

This package implements essential path ordering and processing for mowing operations:
- **Inner-First Ordering**: Reorders paths so inner areas are mowed before outline boundaries
- **Distance-Based Optimization**: Orders inner paths to minimize travel distance between segments
- **Path Reversal**: Reverses waypoint sequences within paths for optimal mowing direction
- **Automatic Start Point Selection**: Intelligently selects starting positions when not configured

## Features

- Path reordering with inner-first priority
- Distance-based path sequence optimization
- Waypoint reversal for optimal mowing direction  
- ROS integration with slic3r_coverage_planner messages
- Area-specific configuration support
- Automatic start point selection fallback

## Installation

This package is part of the OpenMower project and should be built with the rest of the workspace:

```bash
cd /path/to/open_mower_ros
catkin_make
```

## Usage

### Launch the Path Optimizer Node

```bash
# Basic launch with default parameters
roslaunch path_optimizer path_optimizer.launch

# Launch with custom configuration file
roslaunch path_optimizer path_optimizer.launch \
    areas_config_file:=/path/to/custom_areas_config.yaml
```

### Parameters

- `areas_config_file` (string, default: "$(find path_optimizer)/config/areas_config.yaml")
  - Path to YAML configuration file for area-specific path ordering settings
- `input_topic` (string, default: "/mower_logic/mowing_path")
  - Input path topic
- `output_topic` (string, default: "/path_optimizer/optimized_path")
  - Output optimized path topic

### Topics

#### Subscribed Topics
- `~/input_path` ([nav_msgs/Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html))
  - Input path to be optimized

#### Published Topics
- `~/optimized_path` ([nav_msgs/Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html))
  - Optimized output path

### Services

#### Service Servers
- `~/optimize` ([path_optimizer/OptimizePaths](srv/OptimizePaths.srv))
  - Batch optimization service for multiple paths with area-specific configurations
  - **Request:**
    - `slic3r_coverage_planner/Path[] paths`: Array of paths to optimize (each contains nav_msgs/Path and is_outline field)
    - `int32 area`: Area identifier for optimization configuration
  - **Response:**
    - `slic3r_coverage_planner/Path[] optimized_paths`: Array of optimized paths (reordered based on area settings)

- `~/get_area_config` ([path_optimizer/GetAreaConfig](srv/GetAreaConfig.srv))
  - Service to fetch configuration for a specific area
  - **Request:**
    - `int32 area`: Area identifier
  - **Response:**
    - `bool success`: Whether the request succeeded
    - `string message`: Status message
    - Structured configuration fields: `name`, `inner_first`, `reverse`, `outlines_count`, `start_point_x`, `start_point_y`, `has_fix_point`, `fix_point_x`, `fix_point_y`
  - **See:** [GET_AREA_CONFIG_SERVICE.md](GET_AREA_CONFIG_SERVICE.md) for detailed documentation

### Python API

You can also use the path optimizer directly in your Python code:

```python
from path_optimizer.path_optimizer_core import PathOptimizerCore
import numpy as np

# Create optimizer instance
optimizer = PathOptimizerCore(
    method='douglas_peucker',
    tolerance=0.1
)

# Optimize a path
waypoints = np.array([[0, 0], [1, 0.1], [2, 0], [3, 0.1], [4, 0]])
optimized = optimizer.optimize_path(waypoints)

# Get optimization statistics
stats = optimizer.get_path_statistics(waypoints, optimized)
print(f"Reduced from {stats['original_waypoints']} to {stats['optimized_waypoints']} waypoints")
```

### Service Usage

```bash
# Call the optimization service with rosservice
rosservice call /path_optimizer/optimize "paths: [path1, path2] is_outline: [false, true] area: 1"

# Test the service functionality
rosrun path_optimizer test_optimization_service.py

# Test path reordering specifically
rosrun path_optimizer test_path_reordering.py
```

```python
```python
import rospy
from path_optimizer.srv import OptimizePaths
from slic3r_coverage_planner.msg import Path as Slic3rPath

rospy.wait_for_service('/path_optimizer/optimize')
optimize_service = rospy.ServiceProxy('/path_optimizer/optimize', OptimizePaths)

# Create slic3r_coverage_planner/Path messages with embedded is_outline flags
slic3r_paths = []
for nav_path, is_outline in zip(input_nav_paths, [False, True, False]):
    slic3r_path = Slic3rPath()
    slic3r_path.path = nav_path
    slic3r_path.is_outline = 1 if is_outline else 0
    slic3r_paths.append(slic3r_path)

# Call service with slic3r paths and area
response = optimize_service(
    paths=slic3r_paths,
    area=area_id
)
optimized_slic3r_paths = response.optimized_paths
```
```

## Optimization Methods

### Douglas-Peucker
Classical line simplification algorithm that removes waypoints based on perpendicular distance from the simplified line.

**Pros:**
- Fast execution
- Preserves overall path shape
- Significant waypoint reduction

**Cons:**
- May create sharp corners
- Fixed tolerance can be limiting

### Spline Smoothing
Uses cubic spline interpolation to create smooth paths with improved dynamics.

**Pros:**
- Very smooth paths
- Good for mower dynamics
- Eliminates sharp turns

**Cons:**
- May deviate from original path
- Computationally intensive
- Can increase path length

### Combined Optimization
Two-stage process: first simplifies using Douglas-Peucker, then smooths with splines.

**Pros:**
- Balances efficiency and smoothness
- Configurable trade-offs
- Good general-purpose solution

### Mowing-Specific
Tailored optimization considering mower-specific constraints:
- Minimum turning radius
- Segment length constraints
- Sharp turn smoothing

**Pros:**
- Optimized for mowing applications
- Considers physical constraints
- Improves mower performance

## Configuration Examples

### High Accuracy (Surveying Mode)
```yaml
optimization_method: "douglas_peucker"
tolerance: 0.02
```

### Smooth Operation (Normal Mowing)
```yaml
optimization_method: "combined"
tolerance: 0.1
```

### Fast Processing (Large Areas)
```yaml
optimization_method: "douglas_peucker"
tolerance: 0.2
```

### Precision Mowing (Small Areas)
```yaml
optimization_method: "mowing_specific"
tolerance: 0.05
```

## YAML Configuration

### Configuration File Structure

The path optimizer uses YAML configuration files to define area-specific optimization parameters. The default configuration is located at `config/areas_config.yaml`.

```yaml
# Default optimization parameters
default:
  method: "combined"
  tolerance: 0.1
  max_iterations: 100
  inner_first: false      # Mow inner areas first
  reverse: false          # Reverse path direction
  start_point:            # Preferred starting point
    x: 0.0
    y: 0.0
  mowing_params:
    min_turn_radius: 0.5
    min_segment_length: 0.05
    max_sharp_turn: 2.2

# Area-specific configurations  
areas:
  1:
    name: "High Precision"
    method: "mowing_specific"
    tolerance: 0.05
    inner_first: true     # Start with inner boundaries
    reverse: false        # Keep original direction
    start_point:          # Specific start point for this area
      x: 0.0
      y: 0.0
    mowing_params:
      min_turn_radius: 0.3
      min_segment_length: 0.02
      
  2:
    name: "Standard Mowing" 
    method: "combined"
    tolerance: 0.1
    inner_first: false
    reverse: false
    start_point:
      x: 5.0
      y: 5.0
    # ... more areas
    
# Global optimization settings
global_settings:
  enable_turn_smoothing: true
  max_path_length_for_spline: 1000
  log_optimization_statistics: true
```

### Area-Specific Optimization

The service supports area-specific optimization configurations loaded from YAML:

- **Area 1**: High Precision - Tight spaces requiring maximum accuracy
- **Area 2**: Standard Mowing - Regular lawn areas  
- **Area 3**: Large Open Areas - Efficiency prioritized over precision
- **Area 4**: Smooth Path Required - Slopes and delicate areas
- **Area 5**: Narrow Passages - Tight spaces between obstacles
- **Area 6**: Rough Terrain - Uneven ground with rocks
- **Area 7**: Speed Priority - Large flat areas where speed matters

### Configuration Parameters

#### **Path Behavior Parameters**
- **`inner_first`** (boolean): Whether to prioritize inner areas/boundaries first with distance optimization
  - `true`: Reorder paths so inner paths (is_outline=false) come before outline paths, with inner paths ordered to minimize travel distance from the area's start_point
  - `false`: Maintain original path order regardless of outline status
  
- **`reverse`** (boolean): Whether to reverse the path direction (content)
  - `true`: Reverse the waypoint sequence within each path (last waypoint becomes first)
  - `false`: Keep original waypoint sequence

- **`start_point`** (object): Preferred starting coordinates for the area (used for distance optimization)
  - `x` (float): X-coordinate in meters (starting position for nearest-neighbor path ordering)
  - `y` (float): Y-coordinate in meters (used when inner_first=true to minimize travel distance)
  - **Automatic Fallback**: If not specified, the optimizer automatically selects the path with the lowest right position (minimum x + y coordinates)

#### **Example Usage Scenarios**
```yaml
# Area with complex inner boundaries (gardens, flower beds)
inner_first: true      # Mow inner areas before boundaries
start_point: {x: 2.0, y: 3.0}  # Near garden center

# Large open field - start from far edge, work back  
reverse: true          # Reverse waypoint sequence in each path
start_point: {x: 50.0, y: 50.0}  # Far corner

# Narrow passage - specific entry point required
inner_first: false     # Follow original path order
start_point: {x: 1.0, y: 1.0}  # Entry point coordinates
```

### Path Reordering Logic

When `inner_first=true` for an area, the service will reorder the optimized paths:

1. **Inner Paths First**: All paths with `is_outline=false` are placed first
2. **Outline Paths Last**: All paths with `is_outline=true` are placed at the end  
3. **Distance Optimization**: Inner paths are ordered to minimize travel distance
4. **Outline Preservation**: Outline paths maintain their original relative order

**Distance-Based Ordering for Inner Paths:**

When `inner_first=true`, inner paths are intelligently ordered using a nearest-neighbor algorithm:

1. **Starting Point Selection**:
   - **Configured**: Uses the area's `start_point` configuration if specified
   - **Automatic**: If no start_point configured, finds the path with the lowest right position (minimum x + y coordinates)
2. **Nearest Path**: Selects the inner path whose start point is closest to current position
3. **Travel Minimization**: Each subsequent path is chosen to minimize travel from the previous path's end point
4. **Greedy Optimization**: Continues until all inner paths are ordered optimally

**Example with Configured Start Point:**
```bash
# Area start point configured: (0, 0)
# Inner paths: A(start: 10,5), B(start: 2,3), C(start: 15,8), D(start: 5,1)
# 
# Optimal order: D(5,1) → B(2,3) → A(10,5) → C(15,8)
# This minimizes total inter-path travel distance from (0,0)

# Input: [A, B, C, D, Outline1] with inner_first=true
# Output: [D, B, A, C, Outline1] (inner paths reordered by distance)
```

**Example with Automatic Start Point Selection:**
```bash
# No start point configured in area
# Inner paths: A(start: 8,6), B(start: 5,8), C(start: 2,3), D(start: 10,2)
# Coordinate sums: A=14, B=13, C=5, D=12
# 
# Lowest right position: C(2,3) with sum=5, so start optimization from there
# Optimal order: C(2,3) → D(10,2) → B(5,8) → A(8,6)

# Input: [A, B, C, D, Outline1] with inner_first=true, no start_point
# Output: [C, D, B, A, Outline1] (started from lowest right position)
```

### Path Content Reversal Logic

When `reverse=true` for an area, the waypoint sequence within each path is reversed:

1. **Waypoint Reversal**: Each path's waypoints are reversed (first becomes last)
2. **Individual Processing**: Each path is reversed independently
3. **Direction Change**: Mower will travel the path in opposite direction

**Example:**
```bash
# Original path: A → B → C → D
# With reverse=true: D → C → B → A

# Multiple paths with reverse=true:
# Path1: [P1, P2, P3] → [P3, P2, P1]
# Path2: [Q1, Q2, Q3, Q4] → [Q4, Q3, Q2, Q1]
```

**Combined Effect:**
- `inner_first=true` changes the **order of paths** 
- `reverse=true` changes the **direction within each path**
- Both can be used together for complete control

Example usage:
```bash
# Optimize paths for high precision area
rosservice call /path_optimizer/optimize "paths: [...] area: 1"

# Optimize paths for large open area  
rosservice call /path_optimizer/optimize "paths: [...] area: 3"
```

### Configuration Testing

Test your YAML configuration:
```bash
# Test configuration loading
rosrun path_optimizer test_config.py

# Test with custom config file
python3 test_config.py /path/to/your/config.yaml
```

## Integration with OpenMower

The path optimizer integrates seamlessly with the OpenMower ecosystem:

1. **Input**: Receives paths from `mower_logic` or coverage planners
2. **Processing**: Optimizes paths based on selected algorithm
3. **Output**: Publishes optimized paths for navigation stack

### Typical Workflow

```
Coverage Planner → Path Optimizer → Local Planner → Mower Control
```

## Performance Considerations

- **Douglas-Peucker**: O(n log n) average case, suitable for real-time use
- **Spline Smoothing**: O(n) but with higher constants, may need larger tolerance for large paths
- **Combined**: Best balance of performance and quality
- **Memory Usage**: Minimal, processes paths in-place where possible

## Debugging and Visualization

Enable debug logging to see optimization statistics:

```bash
rosservice call /path_optimizer/set_logger_level ros.path_optimizer DEBUG
```

Use RViz to visualize input and output paths:
- Input path: Topic `~/input_path`
- Output path: Topic `~/optimized_path`

## Dependencies

- ROS (Melodic/Noetic)
- Python 3.6+
- NumPy
- SciPy
- geometry_msgs
- nav_msgs
- std_msgs

## License

MIT License - See LICENSE file for details.

## Contributing

Contributions are welcome! Please consider:
- New optimization algorithms
- Performance improvements
- Better mowing-specific constraints
- Enhanced documentation

## Troubleshooting

### Common Issues

**Path not being optimized:**
- Check topic names and remapping
- Verify input path has at least 2 waypoints
- Check tolerance parameters

**Optimization too aggressive:**
- Increase tolerance value
- Consider using 'combined' method instead of 'douglas_peucker'

**Path too rough:**
- Use 'spline' or 'combined' methods
- Decrease tolerance value
- Enable mowing-specific optimizations

**Performance issues:**
- Use 'douglas_peucker' for better performance
- Increase tolerance for faster processing
- Consider path splitting for very large paths

### Getting Help

- Check ROS logs for error messages
- Enable debug logging for detailed statistics
- Visualize paths in RViz to understand optimization behavior