#!/usr/bin/env python3
# coding: utf-8

# import modules
import xml.etree.ElementTree as ET
import sys
import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
from matplotlib.animation import FuncAnimation
import math

from car import Car
from obstacle import Obstacle
from lane import Lane
from road_segment import RoadSegment

# simulation settings
infilename = "grid3x3.net.xml" 
#infilename = "grid5x5.net.xml"
#infilename = "tsudanuma.net.xml"

#number_of_cars = 1000
number_of_cars = 20
#number_of_cars = 5

#sensitivity = 1.0
sensitivity = 0.1


# functions
def read_parse_netxml(infilename):
  # open file
  infile = open(infilename, "r")

  # parsing xml 
  root = ET.fromstring(infile.read())
  #print(root.tag, root.attrib)
  return root

def create_road_network(root):
  # read edge tagged data for reading the road network
  # create data structure of road network using NetworkX
  x_y_dic = {} # input: node's x,y pos, output: node id
  DG = nx.DiGraph() # Directed graph of road network
  edge_lanes_list = [] # list of lane instances
  node_id = 0
  for child in root:
    if child.tag == "edge":
      lane = Lane()
      if "from" in child.attrib and "to" in child.attrib:
        lane.add_from_to(child.attrib["from"], child.attrib["to"])
  
      for child2 in child:
        data_list  = child2.attrib["shape"].split(" ")
        node_id_list = []
        node_x_list = []; node_y_list = []
        distance_list = []
        data_counter = 0
        for data in data_list:
          node_x_list.append( float(data.split(",")[0]) )
          node_y_list.append( float(data.split(",")[1]) )
          if (float(data.split(",")[0]), float(data.split(",")[1])) not in x_y_dic.keys():
            node_id_list.append(node_id)
            DG.add_node(node_id, pos=(float(data.split(",")[0]), float(data.split(",")[1])))
            x_y_dic[ (float(data.split(",")[0]), float(data.split(",")[1])) ] = node_id
            node_id += 1
          else:
            node_id_list.append( x_y_dic[ (float(data.split(",")[0]), float(data.split(",")[1])) ] )
          if data_counter >= 1:
            distance_list.append( np.sqrt( (float(data.split(",")[0]) - old_node_x)**2 + (float(data.split(",")[1]) - old_node_y)**2) )
          old_node_x = float(data.split(",")[0])
          old_node_y = float(data.split(",")[1])
          data_counter += 1
        for i in range(len(node_id_list)-1):
          DG.add_edge(node_id_list[i], node_id_list[i+1], weight=distance_list[i], color="black", speed=float(child2.attrib["speed"])) # calculate weight here
        if "from" in child.attrib and "to" in child.attrib:
          lane.set_others(float(child2.attrib["speed"]), node_id_list, node_x_list, node_y_list)
          edge_lanes_list.append(lane)  # to modify here
  return x_y_dic, DG, edge_lanes_list


# generate a list of road segments for U-turn
def create_road_segments(edge_lanes_list):
  road_segments_list = []
  for i in range(len(edge_lanes_list)-1):
    for j in range(i+1, len(edge_lanes_list)):
      if edge_lanes_list[i].from_id == edge_lanes_list[j].to_id and edge_lanes_list[i].to_id == edge_lanes_list[j].from_id:
        road_segments_list.append(RoadSegment(edge_lanes_list[i], edge_lanes_list[j]))
        break
  return road_segments_list

# randomly select Orign and Destination lanes (O&D are different)
def select_OD_lanes():
  origin_lane_id = np.random.randint(len(edge_lanes_list))
  destination_lane_id = origin_lane_id
  while origin_lane_id == destination_lane_id:
    destination_lane_id = np.random.randint(len(edge_lanes_list))
  return origin_lane_id, destination_lane_id

def find_OD_node_ids(origin_lane_id, destination_lane_id):
  origin_node_id = x_y_dic[ ( edge_lanes_list[origin_lane_id].node_x_list[0], edge_lanes_list[origin_lane_id].node_y_list[0] ) ]
  destination_node_id = x_y_dic[ ( edge_lanes_list[destination_lane_id].node_x_list[-1], edge_lanes_list[destination_lane_id].node_y_list[-1] ) ]
  return origin_node_id, destination_node_id

def draw_road_network(DG):
  pos=nx.get_node_attributes(DG,'pos')
  edge_color = nx.get_edge_attributes(DG, "color")
  nx.draw(DG, pos, node_size=1, arrowsize=5, with_labels=True, font_size=0.8, font_color="red", edge_color=edge_color.values())

# For initializing animation settings
def init():
  line.set_data([], [])
  title.set_text("Simulation step: 0")
  return line, title, 

# main of animation update
def animate(time):
  global xdata, ydata

  xdata = []; ydata=[]

  for car in cars_list:
    if car.__class__.__name__ == "Car":
      x_new, y_new, goal_arrived_flag = car.move(DG, edges_cars_dic, sensitivity) 
      # update x_new and y_new
      xdata.append(x_new)
      ydata.append(y_new)
      # remove arrived cars from the list
      if car.goal_arrived == True:
        cars_list.remove( car )
    elif car.__class__.__name__ == "Obstacle":
      print("Obstacle #%d instance is called, skip!!!" %(car.obstacle_node_id) )
  
    # TODO: if the car encounters road closure, it U-turns.



  # check if all the cars arrive at their destinations
  if len(cars_list) == 0:
    print("Total simulation step: "+str(time-1))
    print("### End of simulation ###")
    sys.exit(0) # end of simulation, exit.

  line.set_data(xdata, ydata)
  title.set_text("Simulation step: "+str(time)+";  # of cars: "+str(len(cars_list)))

  return line, title, 


# Optimal Velocity Function
def V(b, current_max_speed):
  return 0.5*current_max_speed*(np.tanh(b-2) + np.tanh(2))


##### main #####
if __name__ == "__main__":
  # root: xml tree of input file 
  root = read_parse_netxml(infilename)
  
  # x_y_dic: node's x,y pos --> node id
  # DG: Directed graph of road network
  # edge_lanes_list: list of lane instances
  x_y_dic, DG, edge_lanes_list = create_road_network(root)
  
  # road_segments_list: list of road segment instances
  road_segments_list = create_road_segments(edge_lanes_list)
  
  # create cars
  edges_all_list = DG.edges()
  edges_cars_dic = {}
  for item in edges_all_list:
    edges_cars_dic[ item ] = []

  cars_list = []
  # 10 obstacles
  for i in range(10):
    origin_lane_id, destination_lane_id = select_OD_lanes()
    origin_node_id, destination_node_id = find_OD_node_ids(origin_lane_id, destination_lane_id)
    obstacle = Obstacle(origin_node_id, destination_node_id, origin_lane_id, i)
    obstacle.init(DG)
    cars_list.append(obstacle)
    edges_cars_dic[ ( edge_lanes_list[origin_lane_id].node_id_list[0], edge_lanes_list[origin_lane_id].node_id_list[1] ) ].append( obstacle )

  for i in range(len(edge_lanes_list)):
    # randomly select Orign and Destination lanes (O&D are different)
    origin_lane_id, destination_lane_id = select_OD_lanes()
  
    # find Orign and Destination node IDs
    origin_node_id, destination_node_id = find_OD_node_ids(origin_lane_id, destination_lane_id)
  
    # calculate a shortest path to go
    # Reference: https://networkx.github.io/documentation/latest/reference/algorithms/generated/networkx.algorithms.shortest_paths.weighted.dijkstra_path.html
    shortest_path = nx.dijkstra_path(DG, origin_node_id, destination_node_id)
  
    car = Car(origin_node_id, destination_node_id, shortest_path, origin_lane_id)
    car.init(DG) # initialization of car settings
    cars_list.append(car)
    edges_cars_dic[ ( edge_lanes_list[origin_lane_id].node_id_list[0], edge_lanes_list[origin_lane_id].node_id_list[1] ) ].append( car )
  
  
  # animation initial settings
  fig, ax = plt.subplots()
  xdata = []; ydata = []
  for i in range(len(cars_list)):
    xdata.append( cars_list[i].current_position[0] )
    ydata.append( cars_list[i].current_position[1] )
  line, = plt.plot([], [], color="green", marker="s", linestyle="", markersize=10)
  title = ax.text(20.0, -20.0, "", va="center")
  
  # draw road network
  draw_road_network(DG)
  
  print("### Start of simulation ###")
  ani = FuncAnimation(fig, animate, frames=range(1000), init_func=init, blit=True, interval= 100)
  plt.show()
