#!/usr/bin/env python
'''
This class contains and stores GPS points

Questions: anhellesnes@fhs.mil.no
'''

import math as m

from .Vector_class import Vector

class GPS:
    '''Class helper to enable the storing and operations with GPS points (lat, lon)'''

    def __init__(self, latitude = 0.0, longitude = 0.0): 
        '''initialise a GPS object with lat and lon

        Args:
            latitude: Float value of latitude in decimal, 0.0 by default
            longitude: Float value of longituce in decimal, 0.0 by default
        '''
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
        '''Simple set helper class to change value of GPS point

        args:
            latitude: Float value of latitude in decimal
            longitude: Float value of longituce in decimal
        '''
        self.lat = latitude
        self.lon = longitude
        

    def calculate(self, other):
        '''Calculates distance in m and bearing relative to 
        North from self to given GPS point -180 to 180 deg

        Args:
            other: GPS object of GPS to get distance to

        Returns:
            Vector instance containing distance and bearing to GPS point
        '''
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
        '''Helper to print latitude and longitude of self rounded of to 2 decimals'''
        print("GPS coordinates: lat: ", round(self.lat,2), "lon: ", round(self.lon, 2))
