#!/usr/bin/env python

import rosbag
import numpy as np
import matplotlib.pyplot as plt
from gmplot import gmplot

class plotter():
	def __init__(self):
		self.bag  = rosbag.Bag('/home/bach/test.bag')

		# set arrays to store data
		self.time = []
		self.gps_lat = []
		self.gps_lon = []
		# set a flag
		self.Iter = 0
	# read messages 
	def readmsg(self, topic_name):
		# open bag

		# loop over the topic to read evey message
		for topic, msg, t in self.bag.read_messages(topics=topic_name):
			sec         = t.to_nsec() 
			self.time.append((sec-1508618888979416609)*1e-9)

			self.gps_lat.append(msg.position.latitude)
			self.gps_lon.append(msg.position.longitude)

			self.Iter       += 1

		self.bag.close()
		# plot the data
		self._gmap_plot(self.gps_lat,self.gps_lon)
		

	# google map plotting
	def _gmap_plot(self, gps_lat, gps_lon):
		# Place map
		gmap = gmplot.GoogleMapPlotter(60.365791, 5.264471, 10)
		gmap.plot(gps_lat, gps_lon, 'test', edge_width=10)
		
		print("made")
		# Draw
		gmap.draw("/home/bach/my_map.html")


def main():
	# choose one topic  
	gm = plotter()

	topic_name = '/autopilot/current' #/position
	#topic_name = '/mavros/global_position/raw/fix'

	# choose a function
	gm.readmsg(topic_name)
	# showmsg(topic_name)
	# gmap_plot()
	print("done")


if __name__ == '__main__':
	main()