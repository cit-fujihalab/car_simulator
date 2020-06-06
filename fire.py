import networkx as nx
import numpy as np
import math

class Fire:
  def __init__(self,orig_node_id,dest_node_id,current_lane_id,fire_node_id):
    self.orig_node_id = orig_node_id
    self.current_lane_id = current_lane_id
    self.current_position = []
    self.fire_node_id = fire_node_id

  def init(self, DG):
    current_node_id = self.orig_node_id
    self.current_position = DG.nodes[ current_node_id ]["pos"]

  def move(self, DG, edges_cars_dic, sensitivity):
    x_new = self.current_position[0]
    y_new = self.current_position[1]
    return x_new, y_new
