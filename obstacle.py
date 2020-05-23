import networkx as nx
import numpy as np
import math 

class Obstacle:
  def __init__(self, orig_node_id, dest_node_id, current_lane_id, obstacle_node_id):
    self.orig_node_id  = orig_node_id
    self.current_lane_id =  current_lane_id
    self.current_position = []

    # add obstacle node id
    self.obstacle_node_id = obstacle_node_id

  def init(self, DG):
    current_start_node_id = self.orig_node_id
    self.current_position = DG.nodes[ current_start_node_id ]["pos"]

