#!/usr/bin/env python

import sys, getopt
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
		
		self.wanted_speed = []
		self.wanted_angle = []
		self.current_speed = []
		self.current_angle = []
		self.change_speed = []
		self.change_angle = []
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


def main(argv):

	topic = ''
	
	try:
		opts, args = getopt.geotpt(argv,"hi:o:",["type=", "topic1=", "topic2=", "topic3="])
	except getopt.GetoptError:
		print('Plot.py -p <plot> -t <topic1> <topic2> <topic3>')
	
	for opt, arg in opts:


	for args in argv:
		if args == 'position':
			pass
	# choose one topic  
	gm = plotter()

	 #/position
	#topic_name = '/mavros/global_position/raw/fix'

	# choose a function
	gm.readmsg(topic_name)
	# showmsg(topic_name)
	# gmap_plot()
	print("done")


if __name__ == '__main__':
	main()