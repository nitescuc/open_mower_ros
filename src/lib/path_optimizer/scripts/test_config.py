#!/usr/bin/env python3
"""
Configuration Test Script for Path Optimizer

This script tests the YAML configuration loading functionality.

Author: Clemens Elflein
License: MIT
"""

import os
import sys
import yaml
from path_optimizer.config_loader import PathOptimizerConfig


def test_yaml_loading():
    """Test YAML configuration loading"""
    print("=== Path Optimizer Configuration Test ===\n")
    
    # Test 1: Load configuration from file
    print("1. Testing configuration file loading...")
    try:
        # Get the path to the configuration file
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_file = os.path.join(script_dir, '..', 'config', 'areas_config.yaml')
        
        if os.path.exists(config_file):
            config_loader = PathOptimizerConfig(config_file)
            print(f"✓ Successfully loaded config from {config_file}")
        else:
            print(f"✗ Config file not found: {config_file}")
            print("  Using built-in defaults...")
            config_loader = PathOptimizerConfig()
            
    except Exception as e:
        print(f"✗ Error loading configuration: {e}")
        return False
    
    # Test 2: Check default configuration
    print("\n2. Testing default configuration...")
    try:
        default_config = config_loader.get_default_config()
        print(f"✓ Default method: {default_config.get('method', 'N/A')}")
        print(f"✓ Default tolerance: {default_config.get('tolerance', 'N/A')}")
        print(f"✓ Default max_iterations: {default_config.get('max_iterations', 'N/A')}")
    except Exception as e:
        print(f"✗ Error getting default config: {e}")
        return False
    
    # Test 3: Check available areas
    print("\n3. Testing available areas...")
    try:
        available_areas = config_loader.get_available_areas()
        print(f"✓ Found {len(available_areas)} configured areas:")
        for area_id, area_name in available_areas.items():
            print(f"   Area {area_id}: {area_name}")
    except Exception as e:
        print(f"✗ Error getting available areas: {e}")
        return False
    
    # Test 4: Check area-specific configurations
    print("\n4. Testing area-specific configurations...")
    test_areas = [1, 2, 3, 99]  # Include non-existent area
    
    for area_id in test_areas:
        try:
            area_config = config_loader.get_area_config(area_id)
            area_name = area_config.get('name', f'Area {area_id}')
            method = area_config.get('method', 'N/A')
            tolerance = area_config.get('tolerance', 'N/A')
            inner_first = area_config.get('inner_first', 'N/A')
            reverse = area_config.get('reverse', 'N/A')
            start_point = area_config.get('start_point', {'x': 'N/A', 'y': 'N/A'})
            
            print(f"   Area {area_id} ({area_name}):")
            print(f"     method={method}, tolerance={tolerance}")
            print(f"     inner_first={inner_first}, reverse={reverse}")
            print(f"     start_point=({start_point.get('x', 'N/A')}, {start_point.get('y', 'N/A')})")
            
        except Exception as e:
            print(f"✗ Error getting config for area {area_id}: {e}")
    
    # Test 5: Check global settings
    print("\n5. Testing global settings...")
    try:
        global_settings = config_loader.get_global_settings()
        print(f"✓ Global settings loaded: {len(global_settings)} parameters")
        for key, value in list(global_settings.items())[:5]:  # Show first 5
            print(f"   {key}: {value}")
        if len(global_settings) > 5:
            print(f"   ... and {len(global_settings) - 5} more")
    except Exception as e:
        print(f"✗ Error getting global settings: {e}")
        return False
    
    # Test 6: Validate configurations
    print("\n6. Testing configuration validation...")
    try:
        # Test valid config
        valid_config = {
            'method': 'douglas_peucker',
            'tolerance': 0.1,
            'max_iterations': 100,
            'inner_first': True,
            'reverse': False,
            'start_point': {'x': 1.5, 'y': 2.5}
        }
        is_valid = config_loader.validate_config(valid_config)
        print(f"✓ Valid config validation: {is_valid}")
        
        # Test invalid config
        invalid_config = {
            'method': 'invalid_method',
            'tolerance': -0.1,
            'max_iterations': 'not_a_number',
            'inner_first': 'not_boolean',
            'reverse': 'not_boolean',
            'start_point': 'not_a_dict'
        }
        is_valid = config_loader.validate_config(invalid_config)
        print(f"✓ Invalid config validation: {is_valid} (should be False)")
        
        # Test invalid start_point config
        invalid_start_point_config = {
            'method': 'douglas_peucker',
            'tolerance': 0.1,
            'max_iterations': 100,
            'inner_first': True,
            'reverse': False,
            'start_point': {'x': 'not_number', 'y': 2.5}
        }
        is_valid = config_loader.validate_config(invalid_start_point_config)
        print(f"✓ Invalid start_point config validation: {is_valid} (should be False)")
        
    except Exception as e:
        print(f"✗ Error in validation test: {e}")
        return False
    
    print("\n=== All tests completed successfully! ===")
    return True


def test_direct_yaml_loading():
    """Test direct YAML file loading"""
    print("\n=== Direct YAML Loading Test ===\n")
    
    try:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_file = os.path.join(script_dir, '..', 'config', 'areas_config.yaml')
        
        if not os.path.exists(config_file):
            print(f"✗ Config file not found: {config_file}")
            return False
        
        with open(config_file, 'r') as file:
            config_data = yaml.safe_load(file)
        
        print(f"✓ Successfully loaded YAML file: {config_file}")
        print(f"✓ Top-level keys: {list(config_data.keys())}")
        
        # Check structure
        if 'areas' in config_data:
            areas = config_data['areas']
            print(f"✓ Found {len(areas)} area configurations")
            
        if 'default' in config_data:
            default = config_data['default']
            print(f"✓ Default configuration: {default}")
            
        if 'global_settings' in config_data:
            global_settings = config_data['global_settings']
            print(f"✓ Global settings: {len(global_settings)} parameters")
        
        return True
        
    except Exception as e:
        print(f"✗ Error in direct YAML loading: {e}")
        return False


if __name__ == '__main__':
    try:
        # Test direct YAML loading first
        yaml_success = test_direct_yaml_loading()
        
        # Test configuration loader
        config_success = test_yaml_loading()
        
        if yaml_success and config_success:
            print("\n🎉 All configuration tests passed!")
            sys.exit(0)
        else:
            print("\n❌ Some tests failed!")
            sys.exit(1)
            
    except Exception as e:
        print(f"\n💥 Test script failed: {e}")
        sys.exit(1)