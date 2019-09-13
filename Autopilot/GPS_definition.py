#!/usr/bin/env python
import math as m
from Vector_definition import Vector

class GPS:
    def __init__(self, latitude = 0.0, longitude = 0.0): 
        self.lon = longitude
        self.lat = latitude
        self.distance = 0.0
        self.bearing = 0.0
        self.delta_lon = 0.0
        self.delta_lat = 0.0
        self.a = 0.0
        self.radius_earth = 6.371*m.pow(10,6)
        
        self.calculated_vector = Vector()
    
    def set(self, latitude, longitude):
        self.lat = latitude
        self.lon = longitude
        

    def calculate(self, other):
        if isinstance(self, (GPS, tuple)) and isinstance(other, (GPS, tuple)):
            delta_lat = other.lat - self.lat
            delta_lon = other.lon - self.lon
            self.lat, self.lon, other.lat, other.lon, delta_lat, delta_lon = map(m.radians, (self.lat,self.lon, other.lat, other.lon, delta_lat, delta_lon))

            a = m.sin(delta_lat * 0.5) ** 2 + m.cos(self.lat) * m.cos(other.lat) * m.sin(delta_lon * 0.5) ** 2
            c = 2 * m.atan2(m.sqrt(a), m.sqrt(1-a))
            distance = self.radius_earth * c 
            
            b = m.cos(self.lat) * m.sin(other.lat) - m.sin(self.lat) * m.cos(other.lat) * m.cos(delta_lon)
            bearing_rad = m.atan2(m.sin(delta_lon) * m.cos(other.lat), b)
            bearing = (m.degrees(bearing_rad) + 360 ) % 360 

            self.calculated_vector.set(distance, bearing)
            return self.calculated_vector

    def show(self):
        print("GPS coordinates: lat: ", self.lat, "lon: ", self.lon)



    
