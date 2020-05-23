import networkx as nx
import numpy as np
import math 

class Obstacle:
  def __init__(self, orig_node_id, dest_node_id, shortest_path, current_lane_id):
    self.orig_node_id  = orig_node_id
    self.dest_node_id  = dest_node_id
    self.shortest_path = shortest_path
    self.current_lane_id =  current_lane_id
    self.current_sp_index = 0
    self.current_speed = 0.0
    self.current_start_node = []
    self.current_position = []
    self.current_end_node = []
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
  def update_current_speed(self, sensitivity, inter_car_distance):
    self.current_speed += 0.0

  def move(self, DG, edges_cars_dic, sensitivity):
    x_new = self.current_position[0]
    y_new = self.current_position[1]
    return x_new, y_new, self.goal_arrived
