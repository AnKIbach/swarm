#!/usr/bin/env python
from Vector_class import Vector

class autoRange:
    def __init__(self, old_min = 0.0, old_max = 0.0, new_min = 0.0, new_max = 0.0):
        self.oldMin = old_min
        self.oldMax = old_max
        self.newMin = new_min
        self.newMax = new_max

        self.oldRange = 0.0
        self.newRange = 0.0

    def clamp(self, value):
        if value < self.oldMin:
            value = self.oldMin
        elif value > self.oldMax:
            value = self.oldMax
        
        return value

    def new(self, old_value):
        
        clamped_value = self.clamp(old_value)
        self.oldRange = (self.oldMax - self.oldMin)  
        self.newRange = (self.newMax - self.newMin)  
        newValue = ((clamped_value - self.oldMin) * (self.newRange) / self.oldRange) + self.newMin

        return newValue
