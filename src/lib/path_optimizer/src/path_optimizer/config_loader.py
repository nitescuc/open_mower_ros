#!/usr/bin/env python3
"""
Configuration Loader for Path Optimizer

This module handles loading and managing area-specific configurations
from YAML files for the path optimizer.

Author: Clemens Elflein
License: MIT
"""

import yaml
import os
import rospy
from typing import Dict, Any, Optional


class PathOptimizerConfig:
    """
    Configuration manager for path optimizer area settings
    """
    
    def __init__(self, config_file: Optional[str] = None):
        """
        Initialize configuration loader
        
        Args:
            config_file (str, optional): Path to YAML config file
        """
        self.config_data = {}
        self.default_config = {}
        self.areas_config = {}
        self.global_settings = {}
        
        if config_file:
            self.load_config(config_file)
        else:
            # Try to load default configuration
            self._load_default_config()
    
    def _load_default_config(self):
        """Load default configuration file from package"""
        try:
            # Try to get config file from ROS parameter
            config_file = rospy.get_param('~areas_config_file', None)
            
            if config_file and os.path.exists(config_file):
                self.load_config(config_file)
                return
            
            # Try default location in package
            import rospkg
            rospack = rospkg.RosPack()
            pkg_path = rospack.get_path('path_optimizer')
            default_config = os.path.join(pkg_path, 'config', 'areas_config.yaml')
            
            if os.path.exists(default_config):
                self.load_config(default_config)
            else:
                rospy.logwarn("No configuration file found, using built-in defaults")
                self._create_builtin_defaults()
                
        except Exception as e:
            rospy.logwarn(f"Failed to load default config: {e}, using built-in defaults")
            self._create_builtin_defaults()
    
    def _create_builtin_defaults(self):
        """Create built-in default configuration when no file is available"""
        self.default_config = {
            'method': 'combined',
            'tolerance': 0.1,
            'max_iterations': 100,
            'inner_first': False,
            'reverse': False,
            'outlines_count': 4,
            'start_point': {
                'x': 0.0,
                'y': 0.0
            },
            'fix_point': None,
            'mowing_params': {
                'min_turn_radius': 0.5,
                'min_segment_length': 0.05,
                'max_sharp_turn': 2.2
            }
        }
        
        self.areas_config = {
            1: {
                'name': 'High Precision',
                'method': 'mowing_specific',
                'tolerance': 0.05,
                'max_iterations': 150,
                'inner_first': True,
                'reverse': False,
                'outlines_count': 4,
                'start_point': {'x': 0.0, 'y': 0.0},
                'fix_point': None,
                'mowing_params': {
                    'min_turn_radius': 0.3,
                    'min_segment_length': 0.02,
                    'max_sharp_turn': 2.0
                }
            },
            2: {
                'name': 'Standard Mowing',
                'method': 'combined',
                'tolerance': 0.1,
                'max_iterations': 100,
                'inner_first': False,
                'reverse': False,
                'outlines_count': 4,
                'start_point': {'x': 5.0, 'y': 5.0},
                'fix_point': None,
                'mowing_params': {
                    'min_turn_radius': 0.5,
                    'min_segment_length': 0.05,
                    'max_sharp_turn': 2.2
                }
            },
            3: {
                'name': 'Large Open Areas',
                'method': 'douglas_peucker',
                'tolerance': 0.2,
                'max_iterations': 80,
                'inner_first': False,
                'reverse': True,
                'outlines_count': 4,
                'start_point': {'x': 10.0, 'y': 10.0},
                'fix_point': None,
                'mowing_params': {
                    'min_turn_radius': 0.8,
                    'min_segment_length': 0.1,
                    'max_sharp_turn': 2.5
                }
            }
        }
        
        self.global_settings = {
            'enable_turn_smoothing': True,
            'enable_segment_filtering': True,
            'enable_path_statistics': True,
            'max_path_length_for_spline': 1000,
            'max_deviation_from_original': 0.5,
            'preserve_start_end_points': True,
            'log_optimization_statistics': True,
            'log_area_selections': True
        }
    
    def load_config(self, config_file: str):
        """
        Load configuration from YAML file
        
        Args:
            config_file (str): Path to YAML configuration file
        """
        try:
            with open(config_file, 'r') as file:
                self.config_data = yaml.safe_load(file)
            
            # Extract configuration sections
            self.default_config = self.config_data.get('default', {})
            self.areas_config = self.config_data.get('areas', {})
            self.global_settings = self.config_data.get('global_settings', {})
            
            # Convert area keys to integers if they're strings
            self.areas_config = {
                int(k) if isinstance(k, str) and k.isdigit() else k: v
                for k, v in self.areas_config.items()
            }
            
            rospy.loginfo(f"Loaded path optimizer configuration from {config_file}")
            rospy.loginfo(f"Available areas: {list(self.areas_config.keys())}")
            
        except FileNotFoundError:
            rospy.logerr(f"Configuration file not found: {config_file}")
            self._create_builtin_defaults()
        except yaml.YAMLError as e:
            rospy.logerr(f"Error parsing YAML file {config_file}: {e}")
            self._create_builtin_defaults()
        except Exception as e:
            rospy.logerr(f"Error loading configuration: {e}")
            self._create_builtin_defaults()
    
    def get_area_config(self, area_id: int) -> Dict[str, Any]:
        """
        Get configuration for specific area
        
        Args:
            area_id (int): Area identifier
            
        Returns:
            Dict[str, Any]: Area configuration or default if not found
        """
        if area_id in self.areas_config:
            config = self.areas_config[area_id].copy()
            
            # Log area selection if enabled
            if self.global_settings.get('log_area_selections', True):
                area_name = config.get('name', f'Area {area_id}')
                rospy.loginfo(f"Using area config: {area_name} (ID: {area_id})")
            
            return config
        else:
            rospy.logdebug(f"Area {area_id} not found in config, using default")
            return self.default_config.copy()
    
    def get_default_config(self) -> Dict[str, Any]:
        """
        Get default configuration
        
        Returns:
            Dict[str, Any]: Default configuration
        """
        return self.default_config.copy()
    
    def get_global_settings(self) -> Dict[str, Any]:
        """
        Get global settings
        
        Returns:
            Dict[str, Any]: Global settings
        """
        return self.global_settings.copy()
    
    def get_available_areas(self) -> Dict[int, str]:
        """
        Get list of available areas with their names
        
        Returns:
            Dict[int, str]: Dictionary of area_id -> area_name
        """
        return {
            area_id: config.get('name', f'Area {area_id}')
            for area_id, config in self.areas_config.items()
        }
    
    def validate_config(self, config: Dict[str, Any]) -> bool:
        """
        Validate configuration parameters
        
        Args:
            config (Dict[str, Any]): Configuration to validate
            
        Returns:
            bool: True if valid, False otherwise
        """
        required_fields = ['method', 'tolerance', 'max_iterations', 'inner_first', 'reverse', 'start_point']

        for field in required_fields:
            if field not in config:
                rospy.logwarn(f"Missing required configuration field: {field}")
                return False
        
        # Validate method
        valid_methods = ['douglas_peucker', 'spline', 'combined', 'mowing_specific']
        if config['method'] not in valid_methods:
            rospy.logwarn(f"Invalid optimization method: {config['method']}")
            return False
        
        # Validate tolerance
        if not isinstance(config['tolerance'], (int, float)) or config['tolerance'] <= 0:
            rospy.logwarn(f"Invalid tolerance value: {config['tolerance']}")
            return False
        
        # Validate max_iterations
        if not isinstance(config['max_iterations'], int) or config['max_iterations'] <= 0:
            rospy.logwarn(f"Invalid max_iterations value: {config['max_iterations']}")
            return False
        
        # Validate inner_first
        if not isinstance(config['inner_first'], bool):
            rospy.logwarn(f"Invalid inner_first value: {config['inner_first']} (must be boolean)")
            return False
        
        # Validate reverse
        if not isinstance(config['reverse'], bool):
            rospy.logwarn(f"Invalid reverse value: {config['reverse']} (must be boolean)")
            return False
        
        # Validate start_point
        start_point = config['start_point']
        if not isinstance(start_point, dict):
            rospy.logwarn("Invalid start_point: must be a dictionary with x and y values")
            return False
        
        if 'x' not in start_point or 'y' not in start_point:
            rospy.logwarn("start_point must contain 'x' and 'y' fields")
            return False
        
        if not isinstance(start_point['x'], (int, float)) or not isinstance(start_point['y'], (int, float)):
            rospy.logwarn("start_point x and y must be numeric values")
            return False

        # Optional fix_point: if provided, it must be a dict with numeric x,y
        fix_point = config.get('fix_point', None)
        if fix_point is not None:
            if not isinstance(fix_point, dict):
                rospy.logwarn("fix_point must be a dictionary with x and y values")
                return False
            if 'x' not in fix_point or 'y' not in fix_point:
                rospy.logwarn("fix_point must contain 'x' and 'y' fields")
                return False
            if not isinstance(fix_point['x'], (int, float)) or not isinstance(fix_point['y'], (int, float)):
                rospy.logwarn("fix_point x and y must be numeric values")
                return False
        
        return True
    
    def reload_config(self, config_file: Optional[str] = None):
        """
        Reload configuration from file
        
        Args:
            config_file (str, optional): Path to config file, uses current if None
        """
        if config_file:
            self.load_config(config_file)
        else:
            self._load_default_config()
    
    def save_config(self, config_file: str):
        """
        Save current configuration to YAML file
        
        Args:
            config_file (str): Path to save configuration
        """
        try:
            config_data = {
                'default': self.default_config,
                'areas': self.areas_config,
                'global_settings': self.global_settings
            }
            
            with open(config_file, 'w') as file:
                yaml.dump(config_data, file, default_flow_style=False, indent=2)
            
            rospy.loginfo(f"Configuration saved to {config_file}")
            
        except Exception as e:
            rospy.logerr(f"Error saving configuration: {e}")
    
    def add_area_config(self, area_id: int, config: Dict[str, Any]):
        """
        Add or update area configuration
        
        Args:
            area_id (int): Area identifier
            config (Dict[str, Any]): Area configuration
        """
        if self.validate_config(config):
            self.areas_config[area_id] = config
            rospy.loginfo(f"Added/updated configuration for area {area_id}")
        else:
            rospy.logerr(f"Invalid configuration for area {area_id}")
    
    def remove_area_config(self, area_id: int):
        """
        Remove area configuration
        
        Args:
            area_id (int): Area identifier to remove
        """
        if area_id in self.areas_config:
            del self.areas_config[area_id]
            rospy.loginfo(f"Removed configuration for area {area_id}")
        else:
            rospy.logwarn(f"Area {area_id} not found in configuration")