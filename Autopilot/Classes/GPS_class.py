#!/usr/bin/env python
import math as m

from Vector_class import Vector

class GPS:
    '''Class helper to enable the storing and mathematical operations of GPS points (lat, lon)'''
    def __init__(self, latitude = 0.0, longitude = 0.0): 
        self.lat = latitude
        self.lon = longitude

        self.lat_rad = 0.0
        self.lon_rad = 0.0

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
            self.lat_rad, self.lon_rad, other.lat_rad, other.lon_rad, delta_lat_rad, delta_lon_rad = map(m.radians, (self.lat, self.lon, other.lat, other.lon, delta_lat, delta_lon))

            a = m.sin(delta_lat_rad * 0.5) ** 2 + m.cos(self.lat_rad) * m.cos(other.lat_rad) * m.sin(delta_lon_rad * 0.5) ** 2
            c = 2 * m.atan2(m.sqrt(a), m.sqrt(1-a))
            distance = self.radius_earth * c 
            
            b = m.cos(self.lat_rad) * m.sin(other.lat_rad) - m.sin(self.lat_rad) * m.cos(other.lat_rad) * m.cos(delta_lon_rad)
            bearing_rad = m.atan2(m.sin(delta_lon_rad) * m.cos(other.lat_rad), b)
            bearing = (m.degrees(bearing_rad) + 360 ) % 360 

            self.calculated_vector.set(distance, bearing)
            return self.calculated_vector

    def show(self):
        print("GPS coordinates: lat: ", round(self.lat,2), "lon: ", round(self.lon, 2))



    
