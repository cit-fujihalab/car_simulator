import networkx as nx
import numpy as np
import math
from road_segment import RoadSegment
from pprint import pprint

class Car:
  def __init__(self, orig_node_id, dest_node_id, dest_lane_id, shortest_path, current_lane_id):
    self.orig_node_id  = orig_node_id #起点
    self.dest_node_id  = dest_node_id #終点
    self.dest_lane_id = dest_lane_id
    self.shortest_path = shortest_path #最短経路
    self.current_lane_id =  current_lane_id #現在のレーン
    self.current_sp_index = 0
    self.current_speed = 0.0
    self.current_start_node = []
    self.current_position = []
    self.current_end_node = []
    self.found_obstacles = []
    self.current_distance = 0.0
    self.goal_arrived = False

  def init(self, DG):
    current_start_node_id = self.shortest_path[ self.current_sp_index ]
    self.current_start_node = DG.nodes[ current_start_node_id ]["pos"]
    self.current_position = DG.nodes[ current_start_node_id ]["pos"]
    current_end_node_id = self.shortest_path[ self.current_sp_index+1]
    self.current_end_node = DG.nodes[ current_end_node_id ]["pos"]
    current_edge_attributes = DG.get_edge_data(current_start_node_id, current_end_node_id)
    self.current_max_speed = current_edge_attributes["speed"]
    self.current_distance = current_edge_attributes["weight"]

  # Optimal Velocity Function to determine the current speed
  def V(self, inter_car_distance):
    return 0.5*self.current_max_speed*(np.tanh(inter_car_distance-2) + np.tanh(2))

  # update car's speed
  #inter_car_distance = diff_dist
  def update_current_speed(self, sensitivity, inter_car_distance):
    if inter_car_distance < 20:
      self.current_speed = 0
    else:
      self.current_speed += sensitivity*( self.V(inter_car_distance) - self.current_speed )

  def move(self, DG, edges_cars_dic, sensitivity):
    # x_prev == self.current_position[0]
    # y_prev == self.current_position[1]
    direction_x = self.current_end_node[0] - self.current_position[0]
    direction_y = self.current_end_node[1] - self.current_position[1]
    arg = math.atan2(direction_y, direction_x)

    arrived_cars_list = []
    #x_new = None; y_new = None

    if np.sqrt((self.current_position[0] - self.current_end_node[0])**2 + (self.current_position[1] - self.current_end_node[1])**2) < self.current_speed: # to arrive at the terminal of edge

      self.current_sp_index += 1
      if self.current_sp_index >= len(self.shortest_path)-1: # arrived at the goal 
        self.goal_arrived = True
        x_new = self.current_end_node[0]
        y_new = self.current_end_node[1]

        current_start_node_id = self.shortest_path[ self.current_sp_index-1 ]
        current_end_node_id = self.shortest_path[ self.current_sp_index ]

        car_forward_index = edges_cars_dic[(current_start_node_id, current_end_node_id)].index(self)
        car_forward_pt = edges_cars_dic[(current_start_node_id, current_end_node_id)][car_forward_index]
        diff_dist = 50

        edges_cars_dic[ (current_start_node_id, current_end_node_id) ].remove( self )
        arrived_cars_list.append( self )


      else: # lane change
        x_new = self.current_end_node[0]
        y_new = self.current_end_node[1]

        current_start_node_id = self.shortest_path[ self.current_sp_index-1 ]
        current_end_node_id = self.shortest_path[ self.current_sp_index ]
        edges_cars_dic[ (current_start_node_id, current_end_node_id) ].remove( self )

        current_start_node_id = self.shortest_path[ self.current_sp_index ]
        self.current_start_node = DG.nodes[ current_start_node_id ]["pos"]
        self.current_position = DG.nodes[ current_start_node_id ]["pos"]
        current_end_node_id = self.shortest_path[ self.current_sp_index+1]
        self.current_end_node = DG.nodes[ current_end_node_id ]["pos"]
        current_edge_attributes = DG.get_edge_data(current_start_node_id, current_end_node_id)
        self.current_max_speed = current_edge_attributes["speed"]
        self.current_distance = current_edge_attributes["weight"]
        edges_cars_dic[ (current_start_node_id, current_end_node_id) ].append( self )

        if edges_cars_dic[(current_start_node_id, current_end_node_id)].index(self) > 0:
          car_forward_index = edges_cars_dic[(current_start_node_id, current_end_node_id)].index(self) - 1
          car_forward_pt = edges_cars_dic[(current_start_node_id, current_end_node_id)][car_forward_index]
          diff_dist = 50.0

        else:
          car_forward_index = edges_cars_dic[(current_start_node_id, current_end_node_id)].index(self)
          car_forward_pt = edges_cars_dic[(current_start_node_id, current_end_node_id)][car_forward_index]
          diff_dist = 50.0

    else: # move to the terminal of edge
      x_new = self.current_position[0] + self.current_speed*np.cos(arg)
      y_new = self.current_position[1] + self.current_speed*np.sin(arg)
      self.current_position = [x_new, y_new]
      current_start_node_id = self.shortest_path[ self.current_sp_index ]
      current_end_node_id = self.shortest_path[ self.current_sp_index+1 ]

      if edges_cars_dic[ (current_start_node_id, current_end_node_id) ].index( self ) > 0:
        car_forward_index = edges_cars_dic[ (current_start_node_id, current_end_node_id) ].index( self ) - 1
        car_forward_pt = edges_cars_dic[ (current_start_node_id, current_end_node_id) ][ car_forward_index ]
        diff_dist = np.sqrt( (car_forward_pt.current_position[0] - self.current_position[0])**2 + (car_forward_pt.current_position[1] - self.current_position[1])**2 )

      else:
        car_forward_index = edges_cars_dic[(current_start_node_id, current_end_node_id)].index(self)
        car_forward_pt = edges_cars_dic[(current_start_node_id, current_end_node_id)][car_forward_index]
        diff_dist = 50.0
      self.update_current_speed(sensitivity, diff_dist)

    return x_new, y_new, self.goal_arrived, car_forward_pt, diff_dist, self.shortest_path

  def U_turn(self,DG_copied,edges_cars_dic,lane_dic, edge_lanes_list, x_y_dic, obstacle_node_id_list):
    self.current_sp_index += 1

    x_new = self.current_end_node[0]
    y_new = self.current_end_node[1]

    # Uターン前のモデルの削除
    current_start_node_id = self.shortest_path[self.current_sp_index - 1]
    current_end_node_id = self.shortest_path[self.current_sp_index]
    edges_cars_dic[(current_start_node_id, current_end_node_id)].remove(self)
    pre_start_node_id = current_start_node_id
    pre_end_node_id = current_end_node_id

    #車線に関して
    #print(lane_dic)
    self.current_lane_id = lane_dic[self.shortest_path[self.current_sp_index]]
    print("今のノードの番号:"+str(self.shortest_path[self.current_sp_index]))#このノードの番号を用いてレーンを求める
    print("今いるレーンの番号:"+str(self.current_lane_id))

    for i in range(len(edge_lanes_list) - 1):
      for j in range(i + 1, len(edge_lanes_list)):
        if edge_lanes_list[i].from_id == edge_lanes_list[j].to_id and edge_lanes_list[i].to_id == edge_lanes_list[j].from_id:
          if edge_lanes_list[self.current_lane_id] == edge_lanes_list[i]:
            print("対向車線のレーンの番号"+str(j))
            print("車線変更後のノード番号"+str(x_y_dic[(edge_lanes_list[j].node_x_list[1], edge_lanes_list[j].node_y_list[1])]))
            current_start_node_id = x_y_dic[(edge_lanes_list[j].node_x_list[0], edge_lanes_list[j].node_y_list[0])]
            current_end_node_id = x_y_dic[(edge_lanes_list[j].node_x_list[-1], edge_lanes_list[j].node_y_list[-1])]

          elif edge_lanes_list[self.current_lane_id] == edge_lanes_list[j]:
            print("車線変更後のレーンの番号"+str(i))
            print("車線変更後のノード番号"+str(x_y_dic[(edge_lanes_list[i].node_x_list[1], edge_lanes_list[i].node_y_list[1])]))
            current_start_node_id = x_y_dic[(edge_lanes_list[i].node_x_list[0], edge_lanes_list[i].node_y_list[0])]
            current_end_node_id = x_y_dic[(edge_lanes_list[i].node_x_list[-1], edge_lanes_list[i].node_y_list[-1])]

    self.current_start_node = DG_copied.nodes[current_start_node_id]["pos"]
    self.current_position = DG_copied.nodes[current_start_node_id]["pos"]
    self.current_end_node = DG_copied.nodes[current_end_node_id]["pos"]

    #障害物を含むedgeの削除
    DG_copied.remove_edge(pre_start_node_id,pre_end_node_id)

    #最短経路の再計算
    print("車線変更前のshortest_path"+str(self.shortest_path))
    self.shortest_path = nx.dijkstra_path(DG_copied, current_start_node_id, self.dest_node_id)

    #try:
    #  self.shortest_path = nx.dijkstra_path(DG_copied, current_start_node_id, self.dest_node_id)
    #except:
    #  print("No Path")
    #  print("change dest_node")
    #  self.dest_lane_id = np.random.randint(len(edge_lanes_list))
    #  self.dest_node_id = x_y_dic[(edge_lanes_list[self.dest_lane_id].node_x_list[-1], edge_lanes_list[self.dest_lane_id].node_y_list[-1])]
    #  while self.dest_node_id in obstacle_node_id_list:
    #    self.dest_lane_id = np.random.randint(len(edge_lanes_list))
    #    self.dest_node_id = x_y_dic[(edge_lanes_list[self.dest_lane_id].node_x_list[-1], edge_lanes_list[self.dest_lane_id].node_y_list[-1])]
    #   try:
    #      self.shortest_path = nx.dijkstra_path(DG_copied, current_start_node_id, self.dest_node_id)
    #    except:
    #      print("No Path")

    print("車線変更後のshortest_path" + str(self.shortest_path))

    if len(self.shortest_path) == 1:
      print("じゃんぷしてる～")
    self.current_sp_index = 0# current_sp_indexのリセット

    current_start_node_id = self.shortest_path[self.current_sp_index]
    self.current_start_node = DG_copied.nodes[current_start_node_id]["pos"]
    self.current_position = DG_copied.nodes[current_start_node_id]["pos"]
    current_end_node_id = self.shortest_path[self.current_sp_index + 1]
    self.current_end_node = DG_copied.nodes[current_end_node_id]["pos"]
    current_edge_attributes = DG_copied.get_edge_data(current_start_node_id, current_end_node_id)
    self.current_max_speed = current_edge_attributes["speed"]
    self.current_distance = current_edge_attributes["weight"]
    edges_cars_dic[(current_start_node_id, current_end_node_id)].append(self)

    car_forward_index = edges_cars_dic[(current_start_node_id, current_end_node_id)].index(self) - 1
    car_forward_pt = edges_cars_dic[(current_start_node_id, current_end_node_id)][car_forward_index]
    if car_forward_pt not in self.found_obstacles:
      self.found_obstacles.append(car_forward_pt)
    print('U_turn end!')
    return x_new, y_new, DG_copied, self.shortest_path
