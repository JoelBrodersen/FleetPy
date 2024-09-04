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
from src.routing.NetworkBasic import read_node_line
from src.routing.NetworkBasic import Edge as BasicEdge
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

class Edge(BasicEdge):
    def __init__(self, edge_index, distance, travel_time,travel_time_std=None):
        self.edge_index = edge_index
        self.distance = distance
        self.travel_time = travel_time
        self.travel_time_std = travel_time_std
       
    def set_tt_std(self, travel_time_std):
        self.travel_time_std = travel_time_std

    def get_tt(self):
        """
        :return: (current) mean travel time on edge
        """
        return self.travel_time
    
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
         super().__init__(network_name_dir, network_dynamics_file_name, scenario_time)

    def _set_edge_tt_std(self, o_node_index, d_node_index, new_travel_time_std):
            o_node = self.nodes[o_node_index]
            d_node = self.nodes[d_node_index]
            edge_obj = o_node.edges_to[d_node]
            print(edge_obj, new_travel_time_std)
            breakpoint()
            edge_obj.set_tt_std(new_travel_time_std)
            """
            new_tt, dis = edge_obj.get_tt_distance()
            o_node.travel_infos_to[d_node_index] = (new_tt, dis)
            d_node.travel_infos_from[o_node_index] = (new_tt, dis)
            """ 
            print(edge_obj.get_tt_mean_std())  
    def loadNetwork(self, network_name_dir, network_dynamics_file_name=None, scenario_time=None):
        nodes_f = os.path.join(network_name_dir, "base", "nodes.csv")
        print(f"Loading nodes from {nodes_f} ...")
        nodes_df = pd.read_csv(nodes_f)
        self.nodes = nodes_df.apply(read_node_line, axis=1)
        #
        edges_f = os.path.join(network_name_dir, "base", "edges.csv")
        print(f"Loading edges from {edges_f} ...")
        edges_df = pd.read_csv(edges_f)
        print(edges_df)
        for _, row in edges_df.iterrows():
            o_node = self.nodes[row[G_EDGE_FROM]]
            d_node = self.nodes[row[G_EDGE_TO]]
            tmp_edge = Edge((o_node, d_node), row[G_EDGE_DIST], row[G_EDGE_TT])
            o_node.add_next_edge_to(d_node, tmp_edge)
            d_node.add_prev_edge_from(o_node, tmp_edge)
        print("... {} nodes loaded!".format(len(self.nodes)))
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
    

        