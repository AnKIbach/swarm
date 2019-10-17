import rosbag
import numpy as np
import matplotlib.pyplot as plt
from gmplot import gmplot


# read messages 
def readmsg(topic_name):
	# open bag
	bag  = rosbag.Bag('data.bag')
	# set arrays to store data
	time = np.zeros([26])
	data1 = time.copy()
	data2 = time.copy()
	data3 = time.copy()
	gps_lat = np.zeros([26])
	gps_lon = np.zeros([26])
	# set a flag
	Iter = 0
	# loop over the topic to read evey message
	for topic, msg, t in bag.read_messages(topics=topic_name):
		sec         = t.to_nsec() 
		time[Iter]  = (sec-1508618888979416609)*1e-9
		data1[Iter] = msg.brake_torque_request
		data2[Iter] = msg.brake_torque_actual
		data3[Iter] = msg.wheel_torque_actual
		gps_lat[Iter] = msg.latitude
		gps_lon[Iter] = msg.longitude
		Iter       += 1

	bag.close()
	# plot the data
	gmap_plot(gps_lat,gps_lon)
	plt.plot(time,data1,label='brake_torque_request')
	plt.plot(time,data2,label='brake_torque_actual')
	plt.plot(time,data2,label='wheel_torque_actual')
	plt.xlabel('time (seconds)',fontsize = 20)
	plt.ylabel('torque', fontsize=20)
	plt.legend(loc=2)
	plt.grid()
	plt.show()
	
# just read and print the messages on teminal 
def showmsg(topic_name):
	bag  = rosbag.Bag('data.bag')
	Iter = 0
	for topic, msg, t in bag.read_messages(topics=topic_name):
		sec         =(t.to_nsec()-1508618888979416609)*1e-9
		Iter += 1
		print msg
	print Iter

# google map plotting
def gmap_plot(gps_lat, gps_lon):
	# Place map
	gmap = gmplot.GoogleMapPlotter(42.2994293333, -83.697904, 13)
	gmap.plot(gps_lat, gps_lon, 'cornflowerblue', edge_width=10)
	
	# Draw
	gmap.draw("my_map.html")


def main():
	# choose one topic  
	topic_name = '/vehicle/steering_report'
	topic_name = '/vehicle/brake_info_report'
	topic_name = '/vehicle/gps/fix'
	# choose a function
	readmsg(topic_name)
	showmsg(topic_name)
	gmap_plot()


if __name__ == '__main__':
	main()