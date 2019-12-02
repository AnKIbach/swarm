#!/usr/bin/env python3
'''
This file is used for plotting data from rosbags to matplot
For help use plot.py -h functionability

utilizes additional modules for python: matplotlib, gmplot

Questions: anhellesnes@fhs.mil.no
'''

import sys, getopt
import rosbag
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as clr 
import matplotlib.axes as ax
from gmplot import gmplot

class plotter():
	def __init__(self):
		self.fig, self.ax = plt.subplots() 
		self.bag  = ''

		self.numTopics = 1
		
		self.time = []
		self.gps_lat = []
		self.gps_lon = []
		
		self.topic1_speed = []
		self.topic1_angle = []
		self.topic2_speed = []
		self.topic2_angle = []
		self.topic3_speed = []
		self.topic3_angle = []

		self.topics_speed = [self.topic1_speed,
							self.topic2_speed,
							self.topic3_speed]

		self.topics_angle = [self.topic1_angle,
							self.topic2_angle,
							self.topic3_angle]


		self.colors = ['b', 'g', 'r']

		self.Iter = 0

	def set_bag(self, bag):
		self.bag = rosbag.Bag(bag)

	def set_numTopics(self, topics):
		self.numTopics = topics

	def readPosmsg(self, topic_name):
		'''Function to read through rosbag and get position data

		Args:
			topic_name: string containing which topic to get
		'''

		# loop over the topic to read evey message
		for topic, msg, t in self.bag.read_messages(topics=topic_name):
			sec = t.to_nsec() 
			self.time.append((sec-1508618888979416609)*1e-9)

			self.gps_lat.append(msg.position.latitude)
			self.gps_lon.append(msg.position.longitude)

			self.Iter += 1

		self.bag.close()

	def readVelAngmsg(self, topic_name, num):
		'''Function to read through rosbag and get angular or velocity data

		Args:
			topic_name: string containing which topic to get
			num: int of which number topic to read data to
		'''
		# loop over the topic to read evey message
		for topic, msg, t in self.bag.read_messages(topics=topic_name):
			sec = t.to_nsec() 
			self.time.append((sec-1508618888979416609)*1e-9)

			self.topics_speed[num].append(msg.movement.velocity)
			self.topics_angle[num].append(msg.movement.bearing)

			self.Iter += 1

	def speed_plot(self, topic):
		'''Function to plot angles 

		Args: 
			list of topics to plot from bag
		'''
		print(self.topics_speed[0])
		for i in range(self.numTopics):
			plt.plot(self.topics_speed[i], label=str(topic[i])+"/velocity", color=self.colors[i])

		self.ax.set_title('Velocity', fontsize=24, loc='center')
		self.ax.set_xlabel('Time[s]', color='k')
		self.ax.set_ylabel('Velocity[m/s]', color='k')
		plt.grid(linestyle='--', linewidth=1)	
		plt.legend(loc=1, fontsize=10, title='Lines')
		plt.show()

	def angle_plot(self, topic): 
		'''Function to plot angles 

		Args: 
			list of topics to plot from bag
		'''
		for i in range(self.numTopics):
			plt.plot(self.topics_angle[i], label=str(topic[i])+"/bearing", color=self.colors[i])

		self.ax.set_title('Bearing', fontsize=24, loc='center')
		self.ax.set_xlabel('Time[s]', color='k')
		self.ax.set_ylabel('Beairng[deg]', color='k')
		plt.grid(linestyle='--', linewidth=1)
		plt.legend(loc=1, fontsize=10, title='Lines')
		plt.show()	

	# google map plotting
	def gmap_plot(self, topic):
		# Place map
		gmap = gmplot.GoogleMapPlotter(60.365791, 5.264471, 10)
		gmap.plot(self.gps_lat, self.gps_lon, title=str(topic), edge_width=10)
		gmap.apikey="AIzaSyBABJmXdbga-WjzxSTycegI68rWjXIKFVk"
		print("made")
		# Draw
		gmap.draw("my_map.html")

def main(argv):

	print(argv)
	topics = []
	
	gm = plotter()

	#Fetch arguments from terminal based on expected arguments
	try:
		opts, args = getopt.getopt(argv, "hb:p:1:2:3:", ["bag=", "plot=", "topic1=", "topic2=", "topic3="]) 
	except getopt.GetoptError as e:
		print('Plot.py -b <bag> -p <plot> -1 <topic1> -2 <topic2> -3 <topic3>')
		print(e)

	print(opts)

	#decifer arguments from terminal 
	for opt, arg in opts:
		if opt in ('-h', '--help'):
			print('Usage: Plot.py -b <bag> -p <plot> -1 <topic1> -2 <topic2> -3 <topic3>')
			sys.exit(2)
		elif opt in ('-b', '--bag'):
			gm.set_bag(arg)
		elif opt in ('-p', '--plot'):
			pltType = arg
		elif opt in ('-1', '-2', '-3', '--topic1', 'topic2', 'topic3'):
			topics.append(arg)

	print('REMAINDER: ', args)
	
	gm.set_numTopics(len(topics))

	#use right plot type and plot data
	try:
		if pltType == 'position':
			print("Topic: ", topics[0])
			gm.readPosmsg(topics[0])
			gm.gmap_plot(topics[0])
			print("done")

		elif pltType == 'speed':
			for i in range(len(topics)):
				gm.readVelAngmsg(topics[i], i)
			gm.speed_plot(topics)
			gm.bag.close
			print("done")

		elif pltType == 'angle':
			for i in range(len(topics)):
				gm.readVelAngmsg(topics[i], i)
			gm.angle_plot(topics)
			gm.bag.close
			print("done")
		else:
			print("invalid plot type chosen - options: <position>, <speed>, <angle>")
	
	except UnboundLocalError:
		pass


if __name__ == '__main__':
	main(sys.argv[1:])