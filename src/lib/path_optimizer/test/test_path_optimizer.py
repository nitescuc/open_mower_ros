#!/usr/bin/env python3
"""
Test script for Path Optimizer

This script demonstrates the usage of the path optimizer with sample data.
It can be used for testing and validation of optimization algorithms.
"""

import sys
import os
import numpy as np

# Add the src directory to Python path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from path_optimizer.path_optimizer_core import PathOptimizerCore


def create_test_paths():
    """Create various test paths for optimization"""
    
    # Test path 1: Noisy straight line
    straight_noisy = []
    for i in range(20):
        x = i * 0.5
        y = np.random.normal(0, 0.05)  # Small noise
        straight_noisy.append([x, y])
    
    # Test path 2: Zigzag pattern (common in mowing)
    zigzag = []
    for i in range(10):
        zigzag.append([i * 2, 0])
        zigzag.append([i * 2 + 1, 1])
        zigzag.append([i * 2 + 2, 0])
    
    # Test path 3: Curved path with many points
    curve = []
    for i in range(50):
        t = i * 0.1
        x = t
        y = np.sin(t) * 2
        curve.append([x, y])
    
    # Test path 4: Square path
    square = [
        [0, 0], [0.1, 0], [0.2, 0], [1, 0], [1.1, 0], [1.2, 0],
        [1, 0.1], [1, 0.2], [1, 1], [1, 1.1], [1, 1.2],
        [0.9, 1], [0.8, 1], [0, 1], [-0.1, 1], [-0.2, 1],
        [0, 0.9], [0, 0.8], [0, 0.1], [0, 0.05]
    ]
    
    return {
        'straight_noisy': np.array(straight_noisy),
        'zigzag': np.array(zigzag),
        'curve': np.array(curve),
        'square': np.array(square)
    }


def test_optimization_methods():
    """Test all optimization methods on sample paths"""
    
    print("Path Optimizer Test Suite")
    print("=" * 50)
    
    # Create test paths
    test_paths = create_test_paths()
    
    # Test all methods
    methods = ['douglas_peucker', 'spline', 'combined', 'mowing_specific']
    tolerances = [0.05, 0.1, 0.2]
    
    for path_name, waypoints in test_paths.items():
        print(f"\nTesting path: {path_name} ({len(waypoints)} waypoints)")
        print("-" * 40)
        
        for method in methods:
            print(f"\nMethod: {method}")
            
            for tolerance in tolerances:
                try:
                    # Create optimizer
                    optimizer = PathOptimizerCore(
                        method=method,
                        tolerance=tolerance
                    )
                    
                    # Optimize path
                    optimized = optimizer.optimize_path(waypoints)
                    
                    # Get statistics
                    stats = optimizer.get_path_statistics(waypoints, optimized)
                    
                    print(f"  Tolerance {tolerance:0.2f}: "
                          f"{stats['original_waypoints']} → {stats['optimized_waypoints']} "
                          f"({stats['reduction_ratio']:0.1%} reduction, "
                          f"{stats['length_change_ratio']:+0.1%} length change)")
                    
                except Exception as e:
                    print(f"  Tolerance {tolerance:0.2f}: ERROR - {str(e)}")


def benchmark_performance():
    """Benchmark optimization performance"""
    
    print("\n" + "=" * 50)
    print("Performance Benchmark")
    print("=" * 50)
    
    import time
    
    # Create large test path
    large_path = []
    for i in range(1000):
        t = i * 0.01
        x = t
        y = np.sin(t * 2) + np.random.normal(0, 0.02)
        large_path.append([x, y])
    
    large_path = np.array(large_path)
    methods = ['douglas_peucker', 'spline', 'combined', 'mowing_specific']
    
    print(f"Testing with {len(large_path)} waypoints")
    print("-" * 40)
    
    for method in methods:
        try:
            optimizer = PathOptimizerCore(method=method, tolerance=0.1)
            
            # Measure time
            start_time = time.time()
            optimized = optimizer.optimize_path(large_path)
            end_time = time.time()
            
            # Calculate statistics
            stats = optimizer.get_path_statistics(large_path, optimized)
            
            print(f"{method:20s}: {end_time - start_time:6.3f}s, "
                  f"{len(optimized):4d} waypoints, "
                  f"{stats['reduction_ratio']:5.1%} reduction")
                  
        except Exception as e:
            print(f"{method:20s}: ERROR - {str(e)}")


def visualize_optimization(save_plots=False):
    """Visualize optimization results (requires matplotlib)"""
    
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("\nMatplotlib not available, skipping visualization")
        return
    
    print("\n" + "=" * 50)
    print("Visualization (saved to plots/)")
    print("=" * 50)
    
    # Create plots directory if it doesn't exist
    if save_plots:
        os.makedirs('plots', exist_ok=True)
    
    test_paths = create_test_paths()
    
    for path_name, waypoints in test_paths.items():
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        fig.suptitle(f'Path Optimization: {path_name}', fontsize=14)
        
        methods = ['douglas_peucker', 'spline', 'combined', 'mowing_specific']
        
        for i, method in enumerate(methods):
            row, col = i // 2, i % 2
            ax = axes[row, col]
            
            try:
                optimizer = PathOptimizerCore(method=method, tolerance=0.1)
                optimized = optimizer.optimize_path(waypoints)
                stats = optimizer.get_path_statistics(waypoints, optimized)
                
                # Plot original and optimized paths
                ax.plot(waypoints[:, 0], waypoints[:, 1], 'b-o', 
                       alpha=0.5, markersize=3, label='Original')
                ax.plot(optimized[:, 0], optimized[:, 1], 'r-s', 
                       markersize=4, label='Optimized')
                
                ax.set_title(f'{method}\n{stats["original_waypoints"]} → '
                           f'{stats["optimized_waypoints"]} waypoints')
                ax.legend()
                ax.grid(True, alpha=0.3)
                ax.set_aspect('equal')
                
            except Exception as e:
                ax.text(0.5, 0.5, f'Error: {str(e)}', 
                       transform=ax.transAxes, ha='center', va='center')
                ax.set_title(f'{method} - Error')
        
        plt.tight_layout()
        
        if save_plots:
            plt.savefig(f'plots/{path_name}_optimization.png', dpi=150, bbox_inches='tight')
            print(f"Saved plot: plots/{path_name}_optimization.png")
        else:
            plt.show()
        
        plt.close()


if __name__ == '__main__':
    # Run tests
    test_optimization_methods()
    
    # Run performance benchmark
    benchmark_performance()
    
    # Create visualizations (save to files)
    visualize_optimization(save_plots=True)
    
    print("\n" + "=" * 50)
    print("Test completed successfully!")
    print("Check the plots/ directory for visualization results.")
    print("=" * 50)