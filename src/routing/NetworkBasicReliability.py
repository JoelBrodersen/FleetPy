# -------------------------------------------------------------------------------------------------------------------- #
# standard distribution imports
# -----------------------------
import os
import logging

# additional module imports (> requirements)
# ------------------------------------------
import pandas as pd
import numpy as np

# src imports
# -----------
from src.routing.NetworkBasic import NetworkBasic
from src.routing.NetworkBasic import Edge as BasicEdge
from src.routing.NetworkBasic import Node as BasicNode

from src.routing.cpp_router.PyNetwork import PyNetwork

# -------------------------------------------------------------------------------------------------------------------- #
# global variables
# ----------------
from src.misc.globals import *
LOG = logging.getLogger(__name__)



INPUT_PARAMETERS_NetworkBasicReliability = {
    "doc" : "this routing class does all routing computations based on dijkstras algorithm and considers Traveltime Mean and Variance",
    "inherit" : "NetworkBase",
    "input_parameters_mandatory": [G_NETWORK_NAME],
    "input_parameters_optional": [G_NW_DYNAMIC_F],
    "mandatory_modules": [],
    "optional_modules": []
}
def read_node_line(columns):
    return Node(int(columns["node_index"]), int(columns["is_stop_only"]), float(columns["pos_x"]), float(columns["pos_y"]))

class Node(BasicNode):
    def __init__(self, node_index, is_stop_only, pos_x, pos_y, node_order=None):
        self.node_index = node_index
        self.is_stop_only = is_stop_only
        self.pos_x = pos_x
        self.pos_y = pos_y
        # 
        self.edges_to = {}  #node_obj -> edge
        self.edges_from = {}    #node_obj -> edge
        #
        self.travel_infos_from = {} #node_index -> (tt, dis)
        self.travel_infos_from_std = {} #node_index -> (std)
        self.travel_infos_to = {}   #node_index -> (tt, dis)
        self.travel_infos_to_std = {}   #node_index -> (std)
        #
        # attributes set during path calculations
        self.is_target_node = False     # is set and reset in computeFromNodes
        #attributes for forwards dijkstra
        self.prev = None
        self.settled = 1
        self.cost_index = -1
        self.cost = None
        # attributes for backwards dijkstra (for bidirectional dijkstra)
        self.next = None
        self.settled_back = 1
        self.cost_index_back = -1
        self.cost_back = None
    
    def add_next_edge_to(self, other_node, edge):
        #print("add next edge to: {} -> {}".format(self.node_index, other_node.node_index))
        self.edges_to[other_node] = edge
        self.travel_infos_to[other_node.node_index] = (edge.get_tt(),edge.get_distance(), edge.get_tt_std())

    def add_prev_edge_from(self, other_node, edge):
        self.edges_from[other_node] = edge
        self.travel_infos_from[other_node.node_index] = (edge.get_tt(),edge.get_distance(), edge.get_tt_std())

    def get_travel_infos_to(self, other_node_index):
        return self.travel_infos_to[other_node_index]

    def get_travel_infos_from(self, other_node_index):
        return self.travel_infos_from[other_node_index]
    

class Edge(BasicEdge):
    def __init__(self, edge_index, distance, travel_time,travel_time_std):
        self.edge_index = edge_index
        self.distance = distance
        self.travel_time = travel_time
        self.travel_time_std = travel_time_std
    
    def set_tt_std(self, travel_time_std):
        self.travel_time_std = travel_time_std
    
    def get_tt_std(self):
        """
        :return: (current) travel time standard deviation on edge
        """
        return self.travel_time_std
    
    def get_tt_mean_std(self):
        """
        :return: (travel time mean, travel time standard deviation) tuple
        """
        return (self.travel_time, self.travel_time_std)

class NetworkBasicReliability(NetworkBasic):
    def __init__(self, network_name_dir, network_dynamics_file_name=None, scenario_time=None):
        self.nodes = []     #list of all nodes in network (index == node.node_index)
        self.network_name_dir = network_name_dir
        self._tt_infos_from_folder = True
        self._current_tt_factor = None
        self.travel_time_file_infos = self._load_tt_folder_path(network_dynamics_file_name=network_dynamics_file_name)
        self.loadNetwork(network_name_dir, network_dynamics_file_name=network_dynamics_file_name, scenario_time=scenario_time)
        self.current_dijkstra_number = 1    #used in dijkstra-class
        self.sim_time = 0   # TODO #
        self.zones = None   # TODO #
        with open(os.sep.join([self.network_name_dir, "base","crs.info"]), "r") as f:
            self.crs = f.read()    
      
    def loadNetwork(self, network_name_dir, network_dynamics_file_name=None, scenario_time=None):
        nodes_f = os.path.join(network_name_dir, "base", "nodes.csv")
        print(f"Loading nodes from {nodes_f} ...")
        nodes_df = pd.read_csv(nodes_f)
        self.nodes = nodes_df.apply(read_node_line, axis=1)
        edges_f = os.path.join(network_name_dir, "base", "edges.csv")
        print(f"Loading edges from {edges_f} ...")
        edges_df = pd.read_csv(edges_f)
        for _, row in edges_df.iterrows():
            o_node = self.nodes[row[G_EDGE_FROM]]
            d_node = self.nodes[row[G_EDGE_TO]]
            tmp_edge = Edge(edge_index=(o_node, d_node), distance=row[G_EDGE_DIST], travel_time= row[G_EDGE_TT],travel_time_std= row[G_EDGE_TT_STD])
            o_node.add_next_edge_to(d_node, tmp_edge)
            d_node.add_prev_edge_from(o_node, tmp_edge)
        print("... {} nodes loaded!".format(len(self.nodes)))
        for node in self.nodes:
            print(node, node.travel_infos_from)
        breakpoint()
        if scenario_time is not None:
            latest_tt = None
            if len(self.travel_time_file_folders.keys()) > 0:
                tts = sorted(list(self.travel_time_file_folders.keys()))
                for tt in tts:
                    if tt > scenario_time:
                        break
                    latest_tt = tt
                self.load_tt_file(latest_tt)
    
    def load_tt_file(self, scenario_time):
        """
        loads new travel time files for scenario_time
        """
        self._reset_internal_attributes_after_travel_time_update()
        f = self.travel_time_file_folders[scenario_time]
        tt_file = os.path.join(f, "edges_td_att.csv")
        tmp_df = pd.read_csv(tt_file)
        print(tmp_df)
        tmp_df.set_index(["from_node","to_node"], inplace=True)
        for edge_index_tuple, new_tt in tmp_df["edge_tt"].iteritems():
            self._set_edge_tt(edge_index_tuple[0], edge_index_tuple[1], new_tt)
        
        for edge_index_tuple, new_tt_std in tmp_df["edge_std"].iteritems():
            self._set_edge_tt_std(edge_index_tuple[0], edge_index_tuple[1], new_tt_std)
    
    
    def _set_edge_tt_std(self, o_node_index, d_node_index, new_travel_time_std):
            o_node = self.nodes[o_node_index]
            d_node = self.nodes[d_node_index]
            edge_obj = o_node.edges_to[d_node]
            edge_obj.set_tt_std(new_travel_time_std)
            o_node.travel_infos_to_std[d_node_index] = new_travel_time_std
            d_node.travel_infos_from_std[o_node_index] = new_travel_time_std

    def get_section_infos(self, start_node_index, end_node_index):
        """
        :param start_node_index_index: index of start_node of section
        :param end_node_index: index of end_node of section
        :return: (travel time, distance, travel time std); if no section between nodes (None, None)
        """
        if self._current_tt_factor is None:
            return self.nodes[start_node_index].get_travel_infos_to(end_node_index)
        else:
            tt, dis, tt_std = self.nodes[start_node_index].get_travel_infos_to(end_node_index)
            return (tt * self._current_tt_factor, dis, tt_std)
    
    def return_route_infos(self, route, rel_start_edge_position, start_time):
        """
        This method returns the information travel information along a route. The start position is given by a relative
        value on the first edge [0,1], where 0 means that the vehicle is at the first node.
        :param route: list of nodes
        :param rel_start_edge_position: float [0,1] determining the start position
        :param start_time: can be used as an offset in case the route is planned for a future time
        :return: (arrival time, distance to travel)
        """
        arrival_time = start_time
        distance = 0
        _, start_tt, start_dis = self.get_section_overhead( (route[0], route[1], rel_start_edge_position), from_start=False)
        arrival_time += start_tt
        distance += start_dis
        if len(route) > 2:
            for i in range(2, len(route)):
                tt, dis, _ = self.get_section_infos(route[i-1], route[i])
                arrival_time += tt
                distance += dis
        return (arrival_time, distance)

    def get_section_overhead(self, position, from_start=True, customized_section_cost_function=None):
        """This method computes the section overhead for a certain position.

        :param position: (current_edge_origin_node_index, current_edge_destination_node_index, relative_position)
        :param from_start: computes already traveled travel_time and distance,
                           if False: computes rest travel time (relative_position -> 1.0-relative_position)
        :param customized_section_cost_function: customized routing objective function
        :return: (cost_function_value, travel time, travel_distance)
        """
        if position[1] is None:
            return 0.0, 0.0, 0.0
        all_travel_time, all_travel_distance, all_travel_time_std = self.get_section_infos(position[0], position[1])
        overhead_fraction = position[2]
        if not from_start:
            overhead_fraction = 1.0 - overhead_fraction
        all_travel_cost = all_travel_time
        if customized_section_cost_function is not None:
            pass
            ##TODO all_travel_cost = customized_section_cost_function(all_travel_time, all_travel_distance, self.nodes[position[1]])
        #return all_travel_cost * overhead_fraction, all_travel_time * overhead_fraction, all_travel_distance * overhead_fraction