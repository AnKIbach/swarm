# import gmplot package 
import gmplot 

latitude_list = [60.364858] 
  
longitude_list = [5.261016] 
  
gmap = gmplot.GoogleMapPlotter(60.364847, 5.262507, 17) 

# heatmap plot heating Type 
# points on the Google map 
gmap.heatmap( latitude_list, longitude_list ) 
    
gmap.draw( "C:\\Users\\Kim\\Desktop\\map14.html") 