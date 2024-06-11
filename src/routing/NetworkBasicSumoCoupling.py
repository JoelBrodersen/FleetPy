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
from src.routing.NetworkBasicWithStore import NetworkBasicWithStore
from src.routing.routing_imports.Router import Router

# -------------------------------------------------------------------------------------------------------------------- #
# global variables
# ----------------
from src.misc.globals import *
LOG = logging.getLogger(__name__)

class NetworkBasicSumoCoupling(NetworkBasicWithStore):
    """ this network is used for an external coupling with SUMO
    the only difference is the implementation of the method "external_update_edge_travel_times"
    to update travel times based on external input
    """

    def external_update_edge_travel_times(self, new_travel_time_dict):
        '''This method takes new edge travel time information from sumo and updates the network in the fleet simulation
        :param new_travel_time_dict: dict edge_id (o_node, d_node) -> edge_traveltime [s]
        :return None'''
        self._reset_internal_attributes_after_travel_time_update()
        for edge_index_tuple, new_tt in new_travel_time_dict.items():
            o_node = self.nodes[edge_index_tuple[0]]
            #print(f'o_node {o_node}')
            d_node = self.nodes[edge_index_tuple[1]]
            #print(f'd_node {d_node}')
            edge_obj = o_node.edges_to[d_node]
            edge_obj.set_tt(new_tt)