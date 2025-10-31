#!/usr/bin/env python3
"""
Path Optimizer Core Module

This module contains simple path processing for mowing operations.
Supports only path reversal based on area configuration.

Author: Clemens Elflein
License: MIT
"""

import numpy as np
from typing import Optional


class PathOptimizerCore:
    """
    Simple path processing for mowing operations
    """
    
    def __init__(self, config: Optional[dict] = None):
        """
        Initialize the path processor
        
        Args:
            config (dict): Configuration parameters from YAML
        """
        # Load configuration parameters
        self.config = config or {}
        
        # Extract area-specific parameters with defaults
        self.reverse = self.config.get('reverse', False)
    
    def optimize_path(self, waypoints: np.ndarray) -> np.ndarray:
        """
        Process a path (only applies reversal if configured)
        
        Args:
            waypoints (np.ndarray): Input waypoints [[x1, y1], [x2, y2], ...]
            
        Returns:
            np.ndarray: Processed waypoints
        """
        if len(waypoints) < 2:
            return waypoints
        
        # Apply area-specific modifications
        processed_waypoints = self._apply_area_specific_modifications(waypoints)
        
        return processed_waypoints
    
    def _apply_area_specific_modifications(self, waypoints: np.ndarray) -> np.ndarray:
        """
        Apply area-specific path modifications
        
        Args:
            waypoints (np.ndarray): Input waypoints
            
        Returns:
            np.ndarray: Modified waypoints
        """
        result_waypoints = waypoints.copy()
        
        # Apply path reversal if configured
        if self.reverse:
            result_waypoints = self._reverse_path_content(result_waypoints)
        
        return result_waypoints
    
    def _reverse_path_content(self, waypoints: np.ndarray) -> np.ndarray:
        """
        Reverse the waypoint sequence within a path
        
        This changes the direction of travel along the path by reversing
        the order of waypoints (first becomes last, last becomes first).
        
        Args:
            waypoints (np.ndarray): Input waypoints
            
        Returns:
            np.ndarray: Waypoints with reversed sequence
        """
        return waypoints[::-1]
    
    def get_path_statistics(self, original_waypoints: np.ndarray, 
                          optimized_waypoints: np.ndarray) -> dict:
        """
        Get simple statistics about path processing
        
        Args:
            original_waypoints (np.ndarray): Original waypoints
            optimized_waypoints (np.ndarray): Processed waypoints
            
        Returns:
            dict: Statistics dictionary
        """
        return {
            'original_waypoints': len(original_waypoints),
            'optimized_waypoints': len(optimized_waypoints),
            'path_reversed': self.reverse,
            'processing_method': 'reversal_only'
        }