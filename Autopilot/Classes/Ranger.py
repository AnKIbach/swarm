#!/usr/bin/env python
'''
This class contains functions to range a value between two ranges

Questions: anhellesnes@fhs.mil.no
'''

from .Vector_class import Vector

class autoRange:
    '''Class for ranging value to a new value'''
    def __init__(self, old_min = 0.0, old_max = 0.0, new_min = 0.0, new_max = 0.0):
        '''Initialises ranges both old and new

        Args:
            old_min: Float of lowest value for old range, 0.0 by default
            old_max: Float of highest value for old range, 0.0 by default
            new_min: Float of lowest value for new range, 0.0 by default
            new_max: Float of highest value for new range, 0.0 by feault
        '''
        self.oldMin = old_min
        self.oldMax = old_max
        self.newMin = new_min
        self.newMax = new_max

        self.oldRange = 0.0
        self.newRange = 0.0

    def _clamp(self, value):
        if value < self.oldMin:
            value = self.oldMin
        elif value > self.oldMax:
            value = self.oldMax
        
        return value

    def new(self, old_value):
        '''Calculates the new value from new range 

        Args:
            old_value: Float of old value to range

        Returns:
            Float of new value based on ranges and old value 
        '''
        clamped_value = self._clamp(old_value)
        self.oldRange = (self.oldMax - self.oldMin)  
        self.newRange = (self.newMax - self.newMin)  
        newValue = ((clamped_value - self.oldMin) * (self.newRange) / self.oldRange) + self.newMin

        return newValue
