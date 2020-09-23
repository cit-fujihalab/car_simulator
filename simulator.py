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
import copy
from scipy import stats
from PIL import Image, ImageOps
import smopy

from car import Car
from lane import Lane
from road_segment import RoadSegment
from obstacle import Obstacle

# simulation settings
#infilename = "grid3x3.net.xml"
infilename = "grid5x5.net.xml"
#infilename = "tsudanuma.net.xml"

#png_infilename = "tsudanuma2-osm.png"


number_of_cars = 200
#number_of_cars = 50
#number_of_cars = 100
number_of_obstacles = 30

sensitivity = 1.0
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
  lane_dic = {}
  edge_length_dic = {}
  node_id = 0
  lane_id = 0

  DG = nx.DiGraph() # Directed graph of road network
  edge_lanes_list = [] # list of lane instances
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

          print("エッジ長とレーン番号の組",float(child2.attrib["length"]), lane_id)
          edge_length_dic[float(child2.attrib["length"])] = lane_id
          for i in range(len(node_x_list)):
            lane_dic[(x_y_dic[node_x_list[i],node_y_list[i]])] = lane_id
          lane_id += 1
          lane.set_others(float(child2.attrib["speed"]), node_id_list, node_x_list, node_y_list)
          edge_lanes_list.append(lane)  # to modify here

  return x_y_dic, lane_dic, edge_length_dic, DG, edge_lanes_list

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
def find_OD_node_and_lane():

  origin_lane_id = np.random.randint(len(edge_lanes_list))
  destination_lane_id = origin_lane_id
  while origin_lane_id == destination_lane_id:
    destination_lane_id = np.random.randint(len(edge_lanes_list))

  origin_node_id = x_y_dic[(edge_lanes_list[origin_lane_id].node_x_list[0], edge_lanes_list[origin_lane_id].node_y_list[0])]
  destination_node_id = x_y_dic[(edge_lanes_list[destination_lane_id].node_x_list[-1], edge_lanes_list[destination_lane_id].node_y_list[-1])]

  while origin_node_id in obstacle_node_id_list:
    origin_lane_id = np.random.randint(len(edge_lanes_list))
    origin_node_id = x_y_dic[(edge_lanes_list[origin_lane_id].node_x_list[0], edge_lanes_list[origin_lane_id].node_y_list[0])]

  while destination_node_id in obstacle_node_id_list or origin_lane_id == destination_lane_id:
      destination_lane_id = np.random.randint(len(edge_lanes_list))
      destination_node_id = x_y_dic[(edge_lanes_list[destination_lane_id].node_x_list[-1], edge_lanes_list[destination_lane_id].node_y_list[-1])]

  return origin_lane_id, destination_lane_id, origin_node_id, destination_node_id


def find_obstacle_lane_and_node():
  while True:
    obstacle_lane_id = np.random.randint(len(edge_lanes_list))
    obstacle_node_id = x_y_dic[(edge_lanes_list[obstacle_lane_id].node_x_list[-1], edge_lanes_list[obstacle_lane_id].node_y_list[-1])]
    oncoming_lane = None
    for i in range(len(edge_lanes_list) - 1):
      for j in range(i + 1, len(edge_lanes_list)):
        if edge_lanes_list[i].from_id == edge_lanes_list[j].to_id and edge_lanes_list[i].to_id == edge_lanes_list[j].from_id:
          if edge_lanes_list[obstacle_lane_id] == edge_lanes_list[i]:
            oncoming_lane = edge_lanes_list[j]
          elif edge_lanes_list[obstacle_lane_id] == edge_lanes_list[j]:
            oncoming_lane = edge_lanes_list[i]
    if oncoming_lane == None:
      if obstacle_node_id not in obstacle_node_id_list:
        break
    elif oncoming_lane != None:
      if x_y_dic[(oncoming_lane.node_x_list[-1], oncoming_lane.node_y_list[-1])] not in obstacle_node_id_list and obstacle_node_id not in obstacle_node_id_list:
        break
  obstacle_node_id_list.append(obstacle_node_id)
  pair_node_id_list.append(x_y_dic[(edge_lanes_list[obstacle_lane_id].node_x_list[0], edge_lanes_list[obstacle_lane_id].node_y_list[0])])
  #print("障害物ノードリスト : "+str(obstacle_node_id_list))

  return obstacle_lane_id, obstacle_node_id

def draw_road_network(DG):
  pos=nx.get_node_attributes(DG,'pos')
  edge_color = nx.get_edge_attributes(DG, "color")
  nx.draw(DG, pos, node_size=1, arrowsize=5, with_labels=True, font_size=0.8, font_color="red", edge_color=edge_color.values())

# For initializing animation settings
def init():
  line1.set_data([], [])
  line2.set_data([], [])
  title.set_text("Simulation step: 0")
  return line1, line2, title,

# main of animation update
def animate(time):
  global xdata,ydata,obstacle_x,obstacle_y
  global goal_time_list, number_of_shortest_path_changes_list, number_of_opportunistic_communication_list, moving_distance_list


  xdata = []; ydata=[]

  for car in cars_list:
    if car.__class__.__name__ == 'Car':
      x_new, y_new, goal_arrived_flag, car_forward_pt, diff_dist = car.move(edges_cars_dic, sensitivity, lane_dic, edge_length_dic)

      # update x_new and y_new
      #xdata.append(x_new)
      #ydata.append(y_new)

      # remove arrived cars from the list
      if car.goal_arrived == True:
        number_of_shortest_path_changes_list.append(car.number_of_shortest_path_changes)
        number_of_opportunistic_communication_list.append(car.number_of_opportunistic_communication)
        goal_time_list.append(car.elapsed_time)
        #moving_distance_list.append(car.moving_distance)
        cars_list.remove( car )

      # TODO: if the car encounters road closure, it U-turns.
      if car_forward_pt.__class__.__name__ != "Car" and diff_dist <= 20 :
        #print("U_turn start!")
        x_new, y_new = car.U_turn(edges_cars_dic, lane_dic, edge_lanes_list, x_y_dic, obstacle_node_id_list)

      xdata.append(x_new)
      ydata.append(y_new)
      oc_lane = 0
      #対向車線を決定 oc = oncoming = 対向
      #対向車線に車両があるとき、車両の持っている障害物の情報を渡す。
      for i in range(len(edge_lanes_list) - 1):
        for j in range(i + 1, len(edge_lanes_list)):
          if edge_lanes_list[i].from_id == edge_lanes_list[j].to_id and edge_lanes_list[i].to_id == edge_lanes_list[j].from_id:
            if edge_lanes_list[car.current_lane_id] == edge_lanes_list[i]:
              oc_lane = edge_lanes_list[j]
            elif edge_lanes_list[car.current_lane_id] == edge_lanes_list[j]:
              oc_lane = edge_lanes_list[i]


      if oc_lane  != 0:
        for oncoming_car in edges_cars_dic[(x_y_dic[(oc_lane.node_x_list[0], oc_lane.node_y_list[0])],x_y_dic[(oc_lane.node_x_list[1], oc_lane.node_y_list[1])])]:
          if oncoming_car.__class__.__name__ =="Car" and len(oncoming_car.obstacles_info_list) >= 1:
            for i in oncoming_car.obstacles_info_list:
              if i not in car.obstacles_info_list:
                #print("すれ違い通信開始")

                car.number_of_opportunistic_communication += 1
                car.obstacles_info_list.append(i)
                a = x_y_dic[(edge_lanes_list[lane_dic[car.obstacles_info_list[-1]]].node_x_list[0],edge_lanes_list[lane_dic[car.obstacles_info_list[-1]]].node_y_list[0])]
                if car.DG_copied.has_edge(a, car.obstacles_info_list[-1]) == True:
                  car.DG_copied.remove_edge(a, car.obstacles_info_list[-1])
                  #print((car.current_position[0], car.current_position[1]) in x_y_dic)
                  #現在地がcurrent_end_node_idのときの条件を追加
                  if (car.current_position[0], car.current_position[1]) in x_y_dic and x_y_dic[(car.current_position[0], car.current_position[1])] == car.shortest_path[car.current_sp_index + 1]:
                    #経路の再計算
                    try:
                      car.shortest_path = nx.dijkstra_path(car.DG_copied, x_y_dic[(car.current_position[0], car.current_position[1])], destination_node_id) #current_start_node_id?
                      break

                    except Exception:
                      destination_lane_id = np.random.randint(len(edge_lanes_list))
                      destination_node_id = x_y_dic[(edge_lanes_list[destination_lane_id].node_x_list[-1], edge_lanes_list[destination_lane_id].node_y_list[-1])]
                      while destination_node_id in obstacle_node_id_list or car.current_lane_id == destination_lane_id:
                        destination_lane_id = np.random.randint(len(edge_lanes_list))
                        destination_node_id = x_y_dic[(edge_lanes_list[destination_lane_id].node_x_list[-1], edge_lanes_list[destination_lane_id].node_y_list[-1])]
                    car.current_sp_index = 0
                    car.number_of_shortest_path_changes += 1


    #elif car.__class__.__name__ == 'Obstacle':
     # print("Obstacle #%d instance is called, skip!!!" % (car.obstacle_node_id))
    #elif car.__class__.__name__ == "Fire":


  obstacle_x = []; obstacle_y = []
  for obstacle in obstacles_list:
    x_new,y_new = obstacle.move(DG,edges_obstacles_dic,sensitivity)
    obstacle_x.append(x_new)
    obstacle_y.append(y_new)

  # check if all the cars arrive at their destinations
  if len(cars_list) - number_of_obstacles == 0:
    #print("経路変更回数"+str(number_of_shortest_path_changes_list))
    #print("すれ違い通信回数"+str(number_of_opportunistic_communication_list))
    #print("ゴールタイム"+str(goal_time_list))

    with open("結果.txt", 'w') as f:
      f.write("経路変更回数"+str(number_of_shortest_path_changes_list)+"\n")
      f.write ("すれ違い通信回数"+str(number_of_opportunistic_communication_list)+"\n")
      f.write("ゴールタイム"+str(goal_time_list)+"\n")
      f.write("総移動距離" + str(moving_distance_list) + "\n")

    print("Total simulation step: "+str(time-1))
    print("### End of simulation ###")
    sys.exit(0) # end of simulation, exit.

  line1.set_data(xdata, ydata)
  line2.set_data(obstacle_x,obstacle_y)
  title.set_text("Simulation step: " + str(time) + ";  # of cars: " + str(len(cars_list) - number_of_obstacles))

  return line1, line2, title,
      
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
  x_y_dic, lane_dic, edge_length_dic, DG, edge_lanes_list = create_road_network(root)
  print("edge_length_dic "+str(edge_length_dic))
  # road_segments_list: list of road segment instances
  road_segments_list = create_road_segments(edge_lanes_list)
  
  # create cars
  edges_all_list = DG.edges()
  edges_cars_dic = {}
  edges_obstacles_dic = {}
  edges_fires_dic = {}

  for item in edges_all_list:
    edges_obstacles_dic[ item ] = []
    edges_cars_dic[ item ] = []


  obstacles_list = []
  obstacle_node_id_list = []
  pair_node_id_list = []
  cars_list = []
  goal_time_list = []  # 移動完了時間リスト
  number_of_shortest_path_changes_list = []  # 経路変更数リスト
  number_of_opportunistic_communication_list = []  # すれ違い通信数リスト
  moving_distance_list = [] #総移動距離リスト


  #edges_all_list = DG.edges()
  #create obstacles
  while True:
    for i in range(number_of_obstacles):
      obstacle_lane_id, obstacle_node_id = find_obstacle_lane_and_node()
      obstacle = Obstacle(obstacle_node_id, obstacle_lane_id)
      obstacle.init(DG)
      obstacles_list.append(obstacle)
      cars_list.append(obstacle)
      edges_obstacles_dic[(edge_lanes_list[obstacle_lane_id].node_id_list[0], edge_lanes_list[obstacle_lane_id].node_id_list[1])].append(obstacle)
      edges_cars_dic[(edge_lanes_list[obstacle_lane_id].node_id_list[0], edge_lanes_list[obstacle_lane_id].node_id_list[1])].append(obstacle)
    if nx.is_weakly_connected(DG) == True:
      break

  #create cars
  DG_copied2 = copy.deepcopy(DG)
  for i in range(len(obstacle_node_id_list)):
    DG_copied2.remove_edge(pair_node_id_list[i],obstacle_node_id_list[i])
  for i in range(number_of_cars):
    # Reference: https://networkx.github.io/documentation/latest/reference/algorithms/generated/networkx.algorithms.shortest_paths.weighted.dijkstra_path.html
    origin_lane_id, destination_lane_id, origin_node_id, destination_node_id = find_OD_node_and_lane()
    while True:
      try:
        shortest_path = nx.dijkstra_path(DG_copied2, origin_node_id, destination_node_id)
        break
      except Exception:
        origin_lane_id, destination_lane_id, origin_node_id, destination_node_id = find_OD_node_and_lane()
    shortest_path = nx.dijkstra_path(DG, origin_node_id, destination_node_id)
    car = Car(origin_node_id, destination_node_id, destination_lane_id, shortest_path, origin_lane_id, DG)
    car.init(DG)  # initialization of car settings
    cars_list.append(car)
    edges_cars_dic[(edge_lanes_list[origin_lane_id].node_id_list[0], edge_lanes_list[origin_lane_id].node_id_list[1])].append(car)



  # animation initial settings
  fig, ax = plt.subplots()
  xdata = []; ydata = []
  for i in range(len(cars_list)):
    xdata.append( cars_list[i].current_position[0] )
    ydata.append( cars_list[i].current_position[1] )
  obstacle_x = []; obstacle_y = []
  for i in range(len(obstacles_list)):
    obstacle_x.append(obstacles_list[i].current_position[0])
    obstacle_y.append(obstacles_list[i].current_position[1])

  line1, = plt.plot([], [], color="green", marker="s", linestyle="", markersize=3)
  line2, = plt.plot([], [], color="red", marker="s", linestyle="", markersize=4)
  title = ax.text(20.0, -20.0, "", va="center")

#  img = Image.open(png_infilename)
#  img = img.crop((212, 176, 761, 574))#(left, upper, right, lower)
#  img = img.resize((2137, 1553), Image.ANTIALIAS) #(width, heigth)
#  img = ImageOps.flip(img) #上下反転
#  img_list = np.asarray(img)
  #plt.imshow(img_list)
  ## draw road network
  draw_road_network(DG)
  
  print("### Start of simulation ###")
  ani = FuncAnimation(fig, animate, frames=range(1000), init_func=init, blit=True, interval= 10)
  #ani.save("grid-sanimation.mp4", writer="ffmpeg")



  plt.show()
