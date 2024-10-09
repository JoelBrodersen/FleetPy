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
from src.routing.NetworkBasicReliabilityWithStore import NetworkBasicReliabilityWithStore

from src.routing.NetworkBasic import Edge as BasicEdge
from src.routing.NetworkBasic import Node as BasicNode

#from src.routing.cpp_router.PyNetwork import PyNetwork
from src.routing.routing_imports.RouterReliability import RouterReliability
from src.routing.routing_imports.Router import Router

# -------------------------------------------------------------------------------------------------------------------- #
# global variables
# ----------------
from src.misc.globals import *
LOG = logging.getLogger(__name__)



class NetworkBasicReliabilitySumoCoupling(NetworkBasicReliabilityWithStore):
    """ this network is used for an external coupling with SUMO
    the only difference is the implementation of the method "external_update_edge_travel_times"
    to update travel times based on external input
    """

    def external_update_edge_travel_times(self, new_travel_time_dict):
        '''This method takes new edge travel time information from sumo and updates the network in the fleet simulation
        :param new_travel_time_dict: dict edge_id (o_node, d_node) -> edge_traveltime [s]
        :return None'''
        self._reset_internal_attributes_after_travel_time_update()
        for (o_node,d_node), (new_tt,new_tt_std) in new_travel_time_dict.items():
            self._set_edge_tt(o_node, d_node, new_tt, new_tt_std)        
            print(f"Updated {len(new_travel_time_dict)} edges in FP Routing Engine with simulated values")