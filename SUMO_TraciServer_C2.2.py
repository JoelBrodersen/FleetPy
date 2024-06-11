#!/usr/bin/python
# IN Traci conda env
from abc import abstractmethod
import os, sys
import pandas as pd
import csv
import logging 
from operator import itemgetter
import statistics
import sys, getopt
from time import perf_counter
from typing import Tuple
import pathlib
from datetime import datetime
from tqdm import tqdm
import xml.etree.ElementTree
import time
import numpy as np

from src.SUMOcontrolledSim import SUMOcontrolledSim
from src.misc.init_modules import load_simulation_environment
import src.misc.config as config
from src.misc.globals import *
import src.evaluation.standard as eval


import traci
import traci.constants as tc
import sumolib

""" 
This script can be used to create a FleetPy-Simulation coupled to SUMO.
Customers, requests and the control of fleet vehicles are controled in FleetPy while vehicle movements are conducted in SUMO.
When a new vehicle route is available, the vehicle is created in SUMO and drives on the computed route to its destination. The vehicle is deleted once it reaches the destination in SUMO (also for boarding processes and created again after the boarding process)
The script requires as input the usual FleetPy config files (constant config and scenario config) and additionally the .sumocfg file.
The network used by FleetPy has to be synchronized to the SUMO-network. Therefore, the script preprocessing\networks\network_from_sumo.py can be used to create the corresponding FleetPy network representation.
The corresponding FleetPy-demand files have to be created manually.
Additionally, the vehicle_types (str-names) used in the FleetPy have to be defined in SUMO, too. Those will be used as fleet vehicle types
"""

LOG = logging.getLogger(__name__)

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

t1_start = perf_counter()

def fleetpy_v_id_to_sumo_v_id(op_vid_tuple : Tuple[int, int]) -> str:
    """ converts the fleetpy vehicle id tuple (op_id, vid) into the sumo string id (fp_{op_id}_{v_id})"""
    return f"fp_{op_vid_tuple[0]}_{op_vid_tuple[1]}"

def sumo_v_id_to_fleetpy_v_id(sumo_v_id_str : str) -> Tuple[int, int]:
    """ converts the sumo v_id into the fleetpy vehicle id (op_id, vid)"""
    _, op_id, vid = sumo_v_id_str.split("_")
    return (int(op_id), int(vid))

def setup_fleetsimulation(constant_config_file, scenario_file, n_cpu_per_sim=1, evaluate=1, log_level="debug",
                keep_old=False, continue_next_after_error=False) -> SUMOcontrolledSim:
    """
    This function combines constant study parameters and scenario parameters.
    Then it sets up a pool of workers and starts a simulation for each scenario.
    The required parameters are stated in the documentation.

    :param constant_config_file: this file contains all input parameters that remain constant for a study
    :type constant_config_file: str
    :param scenario_file: this file contain all input parameters that are varied for a study
    :type scenario_file: str
    :param n_parallel_sim: number of parallel simulation processes
    :type n_parallel_sim: int
    :param n_cpu_per_sim: number of cpus for a single simulation
    :type n_cpu_per_sim: int
    :param evaluate: 0: no automatic evaluation / != 0 automatic simulation after each simulation
    :type evaluate: int
    :param log_level: hierarchical output to the logging file. Possible inputs with hierarchy from low to high:
            - "verbose": lowest level -> logs everything; even code which could scale exponentially
            - "debug": standard debugging logger. code which scales exponentially should not be logged here
            - "info": basic information during simulations (default)
            - "warning": only logs warnings
    :type log_level: str
    :param keep_old: does not start new simulation if result files are already available in scenario output directory
    :type keep_old: bool
    :param continue_next_after_error: continue with next simulation if one the simulations threw an error (only SP)
    :type continue_next_after_error: bool
    """
    # read constant and scenario config files
    constant_cfg = config.ConstantConfig(constant_config_file)
    scenario_cfgs = config.ScenarioConfig(scenario_file)

    # set constant parameters from function arguments
    const_abs = os.path.abspath(constant_config_file)
    study_name = os.path.basename(os.path.dirname(os.path.dirname(const_abs)))
    
    if study_name == "scenarios":
        print("ERROR! The path of the config files is not longer up to date!")
        print("See documentation/Data_Directory_Structure.md for the updated directory structure needed as input!")
        exit()
      
    if constant_cfg.get(G_STUDY_NAME) is not None and study_name != constant_cfg.get(G_STUDY_NAME):
        print("ERROR! {} from constant config is not consitent with study directory: {}".format(constant_cfg[G_STUDY_NAME], study_name))
        print("{} is now given directly by the folder name !".format("G_STUDY_NAME"))
        exit()
    constant_cfg[G_STUDY_NAME] = study_name
    constant_cfg["n_cpu_per_sim"] = n_cpu_per_sim
    constant_cfg["evaluate"] = evaluate
    constant_cfg["log_level"] = log_level
    constant_cfg["keep_old"] = keep_old
    
    
    # combine constant and scenario parameters into verbose scenario parameters
    for i, scenario_cfg in enumerate(scenario_cfgs):
        scenario_cfgs[i] = constant_cfg + scenario_cfg
        
    scenario_cfgs[0][G_SIM_START_TIME] += scenario_cfgs[0].get(G_SUMO_SIM_TIME_OFFSET, 0)
    
    SF = load_simulation_environment(scenario_cfgs[0])
    
    return SF

def setup_traci(sumo_config, results_path, sumoBinary, seed, sumo_fcd_output=False, sumo_edge_output=False, sumo_lane_output=True):
    """ TODO can you put all theses specifications into sumo_config?"""
    if not os.path.isdir(os.path.join(results_path, "SumoDumps")):
        os.mkdir(os.path.join(results_path, "SumoDumps"))
    Trajectoriespath = os.path.join(results_path, "SumoDumps", "Trajectories.xml") 
    TripInfoPath = os.path.join(results_path, "SumoDumps", "TripInfo.xml") 
    vehRoutePath = os.path.join(results_path, "SumoDumps", "vehRoutes.xml") 
    collisionPath = os.path.join(results_path, "SumoDumps", "collisionPath.xml") 
    statisticsPath = os.path.join(results_path, "SumoDumps", "statistics.xml") 
    fullOutputPath = os.path.join(results_path, "FullOutput.xml")
    edges_output = os.path.join(results_path, "SumoDumps", "edge-output.xml")
    additional_file = os.path.join(results_path, "SumoDumps", "additional.xml")
    sumo_cnfg_path = os.path.dirname(sumo_config)
    sumoCmd = [sumoBinary, "-c", sumo_config ,"--collision.action","warn",
               "--step-length","1","--time-to-teleport","300","--tripinfo-output",TripInfoPath,
               "--vehroute-output",vehRoutePath,"--vehroute-output.exit-times","--vehroute-output.incomplete","--collision-output",collisionPath,"--statistic-output",statisticsPath,"--start", "--seed", str(seed)]
    fcdPath = os.path.join(results_path, "SumoDumps", "fcd-output.xml")
    if sumo_fcd_output:
        sumoCmd += ["--fcd-output", fcdPath, "--fcd-output.geo"]

    """ TODO fix this"""
    if sumo_edge_output:
        raise NotImplementedError("Edgeoutput requires specifing the path in an additional file. This is not easily doable in this script. Manually setting the path is needed! ")
        with open(additional_file, "w") as f:
            f.write(f'<additional>\n<edgeData id="edge_1" file="{edges_output}"/>\n</additional>')
        print("warning: you should not do that!!! SUMO_TraciServer")
        sumoCmd += ["--additional-files", additional_file+f",{os.path.join(sumo_cnfg_path, 'routes_calibrated_v1.rou.xml')}"+f",{os.path.join(sumo_cnfg_path, 'vehicles_2018_calibrated_v1.rou.xml')}"]
    # if sumo_lane_output:
    #     sumoCmd += ["--lanedata-output", lane_output]
    traci.start(sumoCmd)
    
def setup_network_translation(FleetPy : SUMOcontrolledSim):
    """Edge ID Dict Translator 
    :param sumo_edge_id_to_fs_edge --> SUMO_EDGE_ID : (FP_START_NODE,FP_END_NODE)
    :param fs_edge_to_sumo_edge_id -->  (FP_START_NODE,FP_END_NODE):SUMO_EDGE_ID
    :param sumo_node_list --> [J1,J1,J2...]
    :param fs_edge_to_ff_tt --> (FP_START_NODE,FP_END_NODE): TRAVEL_TIME
    :param fs_edge_to_len --> (FP_START_NODE,FP_END_NODE): DISTANCE
    """
    nw_path = FleetPy.dir_names[G_DIR_NETWORK]
    edge_df = pd.read_csv(os.path.join(nw_path, "base", "edges.csv"))
    node_df = pd.read_csv(os.path.join(nw_path, "base", "nodes.csv"))

    sumo_edge_id_to_fs_edge = {}
    fs_edge_to_sumo_edge_id = {}
    fs_edge_to_ff_tt = {}
    fs_edge_to_len ={}
    for _, row in edge_df.iterrows():
        if row["from_node"] != row["to_node"]:
            sumo_edge_id = row["source_edge_id"]
            if sumo_edge_id != sumo_edge_id:
                print(f"Warning: no sumo edge for {row['from_node']} -> {row['to_node']} exists!")
                continue
            sumo_edge_id = str(sumo_edge_id)
            start_node_index = row["from_node"]
            end_node_index = row["to_node"]
            sumo_edge_id_to_fs_edge[sumo_edge_id] = (start_node_index, end_node_index)  ## SUMO_EDGE_ID : (FP_START_NODE,FP_END_NODE)
            fs_edge_to_sumo_edge_id[(start_node_index, end_node_index)] = sumo_edge_id ## (FP_START_NODE,FP_END_NODE):SUMO_EDGE_ID 
            fs_edge_to_ff_tt[(start_node_index, end_node_index)] = row["travel_time"] ## (FP_START_NODE,FP_END_NODE): TRAVEL_TIME
            fs_edge_to_len[(start_node_index, end_node_index)] = row["distance"] ## (FP_START_NODE,FP_END_NODE): DISTANCE

    sumo_node_list = node_df.source_node_id
    sumo_node_list = sumo_node_list.values.tolist()

    return sumo_edge_id_to_fs_edge, fs_edge_to_sumo_edge_id, sumo_node_list, fs_edge_to_ff_tt, fs_edge_to_len

##########Handover functions########

def get_current_vehicle_positions(fleetsim, sumo_edge_id_to_fs_edge):
    """ this function reads the positions of the specified vehicles from sumo and returns a dictionary
    :param vehicle_ids: list integer of sumo vehicle ids to read positions
    :param vehicle_to_position_dict: dictionary (operator_id, fleetsim vehicle id) -> fleetsim network position (tuple (o_node, d_node, frac_position) or (o_node, None, None))
            for a single operator operator_id = 0
    :return: """
    fleetsim_vehicles = fleetsim.get_vehicle_and_op_ids()
    vehicle_to_position_dict = {}
    for opid_vid_tuple in fleetsim_vehicles:
        sumo_vid = fleetpy_v_id_to_sumo_v_id(opid_vid_tuple)
        if str(sumo_vid) in traci.vehicle.getIDList():
            LOG.debug("Vehicle is in IDList and position should be updated")
            currentLane = traci.vehicle.getLaneID(str(sumo_vid))
            laneLength = traci.lane.getLength(currentLane)
            currentLanePosition = traci.vehicle.getLanePosition(str(sumo_vid)) #returns something like: 20.267898023103246 (Other format needed?!)
            frac_position = currentLanePosition/laneLength
            currentEdge = traci.lane.getEdgeID(currentLane)
            if sumo_edge_id_to_fs_edge.get(currentEdge) is not None:# currentEdge in sumo_edge_id_to_fs_edge.keys():
                o_node, d_node = sumo_edge_id_to_fs_edge[currentEdge]
                CurrentPosition = (o_node, d_node, frac_position)
                LOG.debug("Vehicle is on edge")
            else:
                last_pos = fleetsim.sim_vehicles[opid_vid_tuple].pos
                CurrentPosition = last_pos
            vehicle_to_position_dict[opid_vid_tuple] = CurrentPosition
        else:
            LOG.debug(f"Position of {sumo_vid} remained the same")

    return vehicle_to_position_dict

def update_routes_and_add_vehicles(fleetsim: SUMOcontrolledSim, initializedVehicles, sim_time, sumo_node_list, fs_edge_to_sumo_edge_id, opvid_to_veh_type):
    '''This functions reads a dict of routes with vehicleIDs as strings (key) and a list of edg (value) and sets the taxi routes accordingly ->(vehicleID, [e1,e2,e3])
    :param vehicle_ids:  list string of sumo vehicle ids
    :return: None'''
    # Receive new routes from FleetPy
    route_dict = fleetsim.get_new_vehicle_routes(sim_time)
    arrivedVehicles = []
    for opid_vid_tuple in route_dict.keys():
        sumo_vid = fleetpy_v_id_to_sumo_v_id(opid_vid_tuple)
        route = route_dict[opid_vid_tuple]
        
        sumoRoute = []
        #Translate Route to sumoRoute
        for i in range(0, len(route)-1):
            o_node = route[i]
            d_node = route[i+1]
            
            #If edge is an internal edge it does not need to be added to the sumoRoute, if route only consisted of internal edges, it would not be a sumoRoute
            if o_node != d_node:
                try:
                    edgeID = fs_edge_to_sumo_edge_id[o_node,d_node]
                    if not edgeID[0] == "i": # internal edges start with "i"
                        sumoRoute.append(edgeID)
                except KeyError:
                    LOG.warning(f'There is a KeyError in the Route which is {o_node} -> {d_node} : {route}')#No occurence
                    continue
        
        route_name = "Route_"+str(sumo_vid)+"_"+str(sim_time) 
        

        # If route only consisted of internal edges, it would not be a sumoRoute
        LOG.debug(f'vehicle {str(sumo_vid)} should get sumoRoute {sumoRoute} which is FleetPy route {route}')
        
        if len(sumoRoute) > 0:
            traci.route.add(route_name, sumoRoute)  # TODO does this lead to an infinite amount of routes in long simulations?
            #Check if vehicle is currently in the simulation
            if sumo_vid in initializedVehicles:
                # 1 A) Vehicle is Loaded during this time step for SUMO-Simulation 
                if sumo_vid in traci.simulation.getLoadedIDList():
                    edgeID = traci.vehicle.getRoadID(sumo_vid)
                    currentRoute = traci.vehicle.getRoute(sumo_vid)
                    if sumoRoute != currentRoute:
                        if sumoRoute[0] == edgeID:# Check if current edgeID is the first element of the sumoRoute
                            is_valid_route = True
                            try:
                                traci.vehicle.setRoute(sumo_vid,sumoRoute)
                                if traci.vehicle.isRouteValid(sumo_vid) is False:
                                    LOG.warning(f'Route of {sumo_vid} is not valid') # No occurence
                                    is_valid_route = False
                            except:
                                LOG.warning(f'Route of {sumo_vid} could not be set 1: {sumoRoute}')
                                is_valid_route = False
                            if not is_valid_route:
                                LOG.warning(f"Vehicle {sumo_vid} has an invalid route {sumoRoute}")
                                LOG.info("Use SUMO rerouter")
                                try:
                                    traci.vehicle.changeTarget(sumo_vid, sumoRoute[-1])
                                    traci.vehicle.rerouteTraveltime(sumo_vid)
                                    LOG.info(f"Vehicle {sumo_vid} has been rerouted to {sumoRoute[-1]} on {traci.vehicle.getRoute(sumo_vid)}")
                                except:
                                    LOG.warning(f"Vehicle {sumo_vid} could not be rerouted")
                    elif sumoRoute == currentRoute:
                        LOG.debug(f"Vehicle {sumo_vid} is Loaded and SumoRoute {sumoRoute} is current Route {currentRoute}")
                        pass        
                    else: #If current edgeID is not the first element look into it. 
                        LOG.debug(f'No new route is set for: {sumo_vid}')
                        LOG.debug(f'newRoute: {traci.vehicle.getRoute(sumo_vid)}')
                        LOG.debug(f'Current route {traci.vehicle.getRoute(sumo_vid)}')
                        LOG.debug(f'Vid: {sumo_vid}, Is in initialized Vehicles: {sumo_vid in initializedVehicles}, RoadID {traci.vehicle.getRoadID(sumo_vid)}, SumoRoute {sumoRoute}')
                # 1 B) Vehicle is already in Simulation
                elif sumo_vid in traci.vehicle.getIDList():
                    edgeID = traci.vehicle.getRoadID(sumo_vid)
                    currentRoute = traci.vehicle.getRoute(sumo_vid)
                    if sumoRoute != currentRoute:
                        if sumoRoute[0] == edgeID:# Check if current edgeID is the first element of the sumoRoute
                            is_valid_route = True
                            try:
                                traci.vehicle.setRoute(sumo_vid,sumoRoute)
                                if traci.vehicle.isRouteValid(sumo_vid) is False:
                                    LOG.warning(f'Route of {sumo_vid} is not valid') # No occurence
                                    is_valid_route = False
                            except:
                                LOG.warning(f'Route of {sumo_vid} could not be set 2: {sumoRoute}')
                                is_valid_route = False
                            if not is_valid_route:
                                LOG.warning(f"Vehicle {sumo_vid} has an invalid route {sumoRoute}")
                                LOG.info("Use SUMO rerouter")
                                try:
                                    traci.vehicle.changeTarget(sumo_vid, sumoRoute[-1])
                                    traci.vehicle.rerouteTraveltime(sumo_vid)
                                    LOG.info(f"Vehicle {sumo_vid} has been rerouted to {sumoRoute[-1]} on {traci.vehicle.getRoute(sumo_vid)}")
                                except:
                                    LOG.warning(f"Vehicle {sumo_vid} could not be rerouted")
                    elif sumoRoute == currentRoute:
                        LOG.debug(f"Vehicle {sumo_vid} is Loaded and in Network and SumoRoute {sumoRoute} is current Route {currentRoute}")
                        pass 
                # 1 C) Vehicle initialized but not in simulation
                else:
                    try:
                        traci.vehicle.setRoute(sumo_vid,sumoRoute)
                    except:
                        try:
                            LOG.debug(f"Vehicle {sumo_vid}has been initialized but is not in the network")#Occured 795 times
                            traci.vehicle.addFull(vehID=sumo_vid, routeID=route_name, typeID=opvid_to_veh_type[opid_vid_tuple])
                        except:
                            LOG.debug(f"Vehicle {sumo_vid}, is not loaded, not in the network and couldn't be added")
                            try:
                                LOG.info(f"try rerouting")
                                traci.vehicle.changeTarget(sumo_vid, sumoRoute[-1])
                                traci.vehicle.rerouteTraveltime(sumo_vid)
                                LOG.info(f"Vehicle {sumo_vid} has been rerouted to {sumoRoute[-1]} on {traci.vehicle.getRoute(sumo_vid)}")
                            except:
                                LOG.warning(f"Vehicle {sumo_vid} could not be rerouted (3)")
                        pass
                    #LOG.debug(f"Vehicle {str(vid)} had been initialized and is being readded")
                    #traci.vehicle.remove(str(vid))
                    #traci.vehicle.add(vehID=str(vid), routeID=route_name, typeID="drt")
                    #edgeID = traci.vehicle.getRoadID(str(vid))
                    #traci.vehicle.setRoute(str(vid),sumoRoute)
                    #if traci.vehicle.isRouteValid(str(vid)) is False:
                    #LOG.warning(f'Route of {str(vid)} is not valid')
            # 2) Vehicle is not in the simulation, but it should be possible to add it.        
            else: 
                try:
                    traci.vehicle.addFull(vehID=sumo_vid, routeID=route_name, typeID=opvid_to_veh_type[opid_vid_tuple])
                    if traci.vehicle.isRouteValid(sumo_vid) is False:
                        LOG.warning(f'Route of {sumo_vid} is not valid')
                    initializedVehicles.append(sumo_vid)
                except:
                    LOG.debug(f'Vehicle {sumo_vid} could not be added')#No occurence
                    pass
        else:   #If the route is only internal vehicles that have been initialized are immediately considered to be "arrived", if not they are added at a "HelpRoute" first. 
                #This should only happen very early in the Simulation and not within the evaluation time
            try:
                #if str(vid) in traci.vehicle.getIDList():
                if sumo_vid in initializedVehicles:
                    arrivedVehicles.append(sumo_vid)
                    LOG.debug(f'Vehicle {sumo_vid} is immediately considered to be arrived')#No occurence
                else:# SumoRoute only consists of internal edge and the vehicle has not been initialize
                    try:    # TODO i dont understand this part
                        LOG.debug(f'SUMO Route vehicle {sumo_vid}')#Occured 7 times 
                        arrivedVehicles.append(sumo_vid)
                        LOG.debug(f'Vehicle {sumo_vid} is immediately considered to be arrived (2)') #This now works
                    except:
                        LOG.debug(f"Error is thrown here")
            except:
                LOG.debug("Something went wrong while adding the route in traci")#6 Occurences
    LOG.debug(f"initialized Vehciles {initializedVehicles}")
    return arrivedVehicles, initializedVehicles

def absolute_to_relative_position(vehID):
    LanePositionInMeter = traci.vehicle.getLanePosition(vehID)
    LaneID = traci.vehicle.getLaneID(vehID)
    LaneLength = traci.lane.getLength(LaneID)
    relativePosition = LanePositionInMeter/LaneLength
    return relativePosition

def update_arrived_vehicles(arrivedVids, fleetsim, vehicle_to_position_dict, sim_time, vids_try_again):
    arrivedVehicleIDs = list(traci.simulation.getArrivedIDList()) #vehciles that have reached destination and have been removed in this sim timestep
    LOG.debug(f"traci.simulation.getArrivedIDList() {list(traci.simulation.getArrivedIDList())}") #Arrived vehicles in problematic situations from this
    LOG.debug(f"arrivedVids {arrivedVids}")
    for sumo_vid in arrivedVehicleIDs:
        if sumo_vid in traci.vehicle.getIDList():
            LOG.debug("vehicle is being removed")
            traci.vehicle.remove(sumo_vid)
    arrivedVehicleIDs.append(arrivedVids)
    LOG.debug(f"arrivedVids {arrivedVehicleIDs}")
    #### updating postion to be the last  step of route ###########
    for sumo_vid in arrivedVehicleIDs:
        if sumo_vid in traci.simulation.getLoadedIDList():
            if sumo_vid in traci.vehicle.getIDList():
                route = traci.vehicle.getRoute(sumo_vid)
                lastEdge = route[-1]
                if lastEdge in sumo_edge_id_to_fs_edge.keys():
                    o_node, d_node = sumo_edge_id_to_fs_edge[lastEdge]
                    frac_position = 1.0
                CurrentPosition = (o_node, d_node, frac_position)
                LOG.debug(f"Manually updated current position is: {CurrentPosition} and route is {route}") #This never happens
                vehicle_to_position_dict[sumo_v_id_to_fleetpy_v_id(sumo_vid)] = CurrentPosition
                EdgeLength = traci.edge.getLength(lastEdge)
                laneID = lastEdge+"_0"
                traci.vehicle.moveTo(sumo_vid,laneID,EdgeLength)
            else:
                LOG.debug(f"vehcile {sumo_vid}is not in ID List but arrived")#780 occurences
        else:
            LOG.debug(f"Vehicle has not been loaded but arrived")

    fleetsim.update_vehicle_positions(vehicle_to_position_dict, sim_time)
    ####
    fleetsim_vehicles= fleetsim.get_vehicle_and_op_ids()
    vids_reached_destination = []
    for opid_vid_tuple in fleetsim_vehicles:
        if fleetpy_v_id_to_sumo_v_id(opid_vid_tuple) in arrivedVehicleIDs:
            vids_reached_destination.append(opid_vid_tuple)
        
    LOG.debug(f"vids-reached_destination is {vids_reached_destination} and arrivedVeicleIDs is {arrivedVehicleIDs}")    
    fleetsim.vehicles_reached_destination(sim_time,vids_reached_destination)
    LOG.debug(f'vids_try_again: {vids_try_again}')
    

def get_current_sumo_Network_speeds(edge_to_veh_current_speed_list):
    vehicles_in_net = traci.vehicle.getIDList()
    if len(vehicles_in_net)==0:
        return edge_to_veh_current_speed_list
    visited_vehicles =[]
    for vehicle_id in vehicles_in_net:
        if vehicle_id in visited_vehicles: 
            continue
        #if not vehicle_id.startswith("fp_"):
            #continue
        vehicle_edge = traci.vehicle.getRoadID(vehicle_id) ##Edge that the vehicle was the last timestep
        VehiclesOnEdge = traci.edge.getLastStepVehicleIDs(vehicle_edge) ## Get all vehicles on this edge
        if vehicle_edge.startswith(":"): ##Internal Edges are considered to have constant traveltimes and are therfore skipped. 
            visited_vehicles.append(VehiclesOnEdge)
            continue            
        
        for Vid in VehiclesOnEdge:
            #if not Vid.startswith("fp_"):
                #continue
            VehicleSpeed = traci.vehicle.getSpeed(Vid)          
            # First time visit to this edge by any vehicle
            if vehicle_edge not in edge_to_veh_current_speed_list:
                 edge_to_veh_current_speed_list[vehicle_edge] = {Vid:[VehicleSpeed]}
            # Edge already visited by other vehicles but not this one
            elif vehicle_edge in edge_to_veh_current_speed_list and Vid not in edge_to_veh_current_speed_list[vehicle_edge]: 
                edge_to_veh_current_speed_list[vehicle_edge][Vid] = [VehicleSpeed]
            # Edge already visited by this vehicle --> add time to list
            elif vehicle_edge in edge_to_veh_current_speed_list and Vid in edge_to_veh_current_speed_list[vehicle_edge]:
                 edge_to_veh_current_speed_list[vehicle_edge][Vid].append(VehicleSpeed)
            
    return edge_to_veh_current_speed_list 

    """
             Shortcut for empty net and intitalisation (FreeFlow Speed)
            if len(vehicles_in_net)==0 or sim_time==0: 
        for edgeID in sumo_edge_id_to_fs_edge.keys():
            freeflowTraveltime = fs_edge_to_ff_tt[sumo_edge_id_to_fs_edge[edgeID]]
            currentEdgeTraveltime = freeflowTraveltime          
                #currentEdgeTraveltime = traci.edge.getTraveltime(edgeID)
            try:
                edge_to_current_tt_list[edgeID].append(currentEdgeTraveltime)
            except KeyError:
                edge_to_current_tt_list[edgeID] = [currentEdgeTraveltime]
                """         
    """
        ## For all Edges that not have been visited by a vehicle this timestep we asume freeflow speed
        for edgeID in sumo_edge_id_to_fs_edge.keys():
            if edgeID in visited_edges:
                continue
        """
        
    """
            freeflowTraveltime = fs_edge_to_ff_tt[sumo_edge_id_to_fs_edge[edgeID]]
            currentEdgeTraveltime = freeflowTraveltime   
            try:
                edge_to_current_tt_list[edgeID].append(currentEdgeTraveltime)
            except KeyError:
                edge_to_current_tt_list[edgeID] = [currentEdgeTraveltime]
            """
            
                
    """
        for edgeID in sumo_edge_id_to_fs_edge.keys():
            #print(f'edgeID = {edgeID}')
            if edgeID.startswith("i"):  # internal
                continue
            else:
                VehiclesOnEdge = traci.edge.getLastStepVehicleIDs(edgeID)
                now = datetime.now()
                laneID = edgeID+"_0"
                length = traci.lane.getLength(laneID)
                if VehiclesOnEdge:
                    VehicleTravelTimes = []
                    for Vid in VehiclesOnEdge:
                        VehicleSpeed = traci.vehicle.getSpeed(Vid)
                        if VehicleSpeed < 0.1:
                            waitingTime = traci.edge.getWaitingTime(str(edgeID))
                            freeflowTraveltime = fs_edge_to_ff_tt[sumo_edge_id_to_fs_edge[edgeID]]
                            currentTravelTime = freeflowTraveltime + waitingTime
                            #print(f'currentTravelTime: {currentTravelTime}')
                            VehicleTravelTimes.append(currentTravelTime)
                        else:
                            currentTravelTime = length/VehicleSpeed
                            VehicleTravelTimes.append(currentTravelTime)
                    currentEdgeTraveltime =statistics.mean(VehicleTravelTimes)
                else: ##No vehicles on this Edge --> FreeFlow Speed from dict (faster)             
                    freeflowTraveltime = fs_edge_to_ff_tt[sumo_edge_id_to_fs_edge[edgeID]]
                    currentEdgeTraveltime = freeflowTraveltime          
                
                    #currentEdgeTraveltime = traci.edge.getTraveltime(edgeID)
             
        """
    

def save_tt_to_csv(sumo_edge_to_avg_tt, sim_time, resultsPath):
    path = os.path.join(resultsPath, "EdgeTravelTimes", "new_travel_times.csv")
    export_tt_list = [(sumo_edge, avg_tt, sim_time) for sumo_edge, avg_tt in sumo_edge_to_avg_tt.items()]
    if os.path.isfile(path):
        with open(path, 'a') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerows(export_tt_list)
    else:
        if not os.path.isdir(os.path.join(resultsPath, "EdgeTravelTimes")):
            os.mkdir(os.path.join(resultsPath, "EdgeTravelTimes"))
        traveltime_fields = ["edges","travel_times_0","sim_time"]    
        with open(path, 'w') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(traveltime_fields)
            writer.writerows(export_tt_list)

def update_edge_traveltimes(edge_to_veh_current_speed_list, sumo_edge_id_to_fs_edge, fs_edge_to_len, sim_time, resultsPath):
    sumo_edge_to_avg_tt = {}
    fs_edge_to_avg_tt = {}
    for sumo_edge_id in edge_to_veh_current_speed_list:
        edge_veh_list =[]
        for veh_id in edge_to_veh_current_speed_list[sumo_edge_id]:
            avg_veh_speed = np.mean(edge_to_veh_current_speed_list[sumo_edge_id][veh_id])
            edge_veh_list.append(avg_veh_speed)
        avg_edge_speed = np.mean(edge_veh_list)
        if avg_edge_speed == 0 and len(edge_to_veh_current_speed_list[sumo_edge_id])<5:
            continue
        elif avg_edge_speed == 0 and len(edge_to_veh_current_speed_list[sumo_edge_id])>=5:
            avg_edge_speed = 0.01
        edge_len = fs_edge_to_len[sumo_edge_id_to_fs_edge[sumo_edge_id]]
        avg_tt = round(edge_len/avg_edge_speed,3)
        fs_edge = sumo_edge_id_to_fs_edge[sumo_edge_id]
        sumo_edge_to_avg_tt[sumo_edge_id] = avg_tt
        fs_edge_to_avg_tt[fs_edge] = avg_tt   
    save_tt_to_csv(sumo_edge_to_avg_tt, sim_time, resultsPath)
    return fs_edge_to_avg_tt

def run_simulation(fleetsim : SUMOcontrolledSim, sumo_edge_id_to_fs_edge, fs_edge_to_sumo_edge_id, sumo_node_list, fs_edge_to_ff_tt,fs_edge_to_len,
                   update_fleetsim_traveltimes = True, update_travel_statistics_time_step = 900):

    edge_to_veh_current_speed_list = {}
    vids_try_again = []
    initializedVehicles = []
    vehicle_to_position_dict = {}
    resultsPath = fleetsim.dir_names[G_DIR_OUTPUT]
    sim_time_offset = fleetsim.scenario_parameters.get(G_SUMO_SIM_TIME_OFFSET, 0)
    end_time = fleetsim.scenario_parameters[G_SIM_END_TIME]
    # get vehicle types of the simulation vehicles
    opvid_to_veh_type = {op_vid : veh.veh_type for op_vid, veh in fleetsim.sim_vehicles.items()} # {(op_id,veh_id):"veh_type"}

    #Check for old EdgeTravelTimes and delete them if they are still there
    if os.path.isfile(os.path.join(resultsPath, "EdgeTravelTimes", "new_travel_times.csv")):
        os.remove(os.path.join(resultsPath, "EdgeTravelTimes", "new_travel_times.csv"))

    ### SIMULATION
    step = 0
    last_time = -1
    while True:


        # 1) fleetpy time step  
        sim_time = int(traci.simulation.getTime()) # sumo time in milliseconds
        sim_time += sim_time_offset
        if sim_time > end_time:
            break
        if sim_time != last_time: # avoid same time step again due to rounding
            leg_status_dict = fleetsim.step(sim_time) # fleetpy timestep and computing new routes
            last_time = sim_time
        if sim_time % 60 == 0:
            print("{}: current simtime: {}/{}".format(fleetsim.scenario_parameters[G_SCENARIO_NAME], sim_time, end_time))
        
       #if sim_time%60==0:
           # for (op_id, veh_id) in fleetsim.sim_vehicles:
               # print((op_id, veh_id),fleetsim.sim_vehicles[(op_id, veh_id)])
               # if len(fleetsim.sim_vehicles[(op_id, veh_id)].pax)>0:
                    #for passenger in fleetsim.sim_vehicles[(op_id, veh_id)].pax:
                        #print(f"id: {passenger.rid} {passenger.o_node} --> {passenger.d_node}")
        
        # 2) check for new rounds and finished boarding processes
        arrivedVehicles, initializedVehicles = update_routes_and_add_vehicles(fleetsim, initializedVehicles, sim_time, sumo_node_list, fs_edge_to_sumo_edge_id, opvid_to_veh_type)
       
        # 3) sumo time step
        traci.simulationStep()
                
        # 4) get current network speeds
        if sim_time%2==0 and update_travel_statistics_time_step<24*3600:
            edge_to_veh_current_speed_list = get_current_sumo_Network_speeds(edge_to_veh_current_speed_list)
        

        # 5) send new travel times to fleetsim
        if sim_time%update_travel_statistics_time_step==0:
            new_travel_times = update_edge_traveltimes(edge_to_veh_current_speed_list, sumo_edge_id_to_fs_edge, fs_edge_to_len, sim_time, resultsPath)
            
            edge_to_veh_current_speed_list = {}
            if update_fleetsim_traveltimes:
                fleetsim.update_network_travel_times(new_travel_times, sim_time)

          

        # 6) collect the current positions of all fleet vehicles in SUMO
        vehicle_to_position_dict = get_current_vehicle_positions(fleetsim, sumo_edge_id_to_fs_edge)
        now = datetime.now()
        
        # 7) set the new positions in FleetPy
        fleetsim.update_vehicle_positions(vehicle_to_position_dict, sim_time)

        # 8) check for vehicles that arrived at their destination
        vids_try_again = update_arrived_vehicles(arrivedVehicles, fleetsim, vehicle_to_position_dict, sim_time, vids_try_again)

        for opid_vid_tuple in leg_status_dict:
            sumo_vid = fleetpy_v_id_to_sumo_v_id(opid_vid_tuple)
            LOG.debug(f"legstatus for {opid_vid_tuple} with vid {sumo_vid} is {leg_status_dict[opid_vid_tuple][0]}")
            if leg_status_dict[opid_vid_tuple][0] == 10 and sumo_vid not in traci.vehicle.getIDList():
                try:
                    pos = leg_status_dict[opid_vid_tuple][1]
                    LOG.debug(f"pos is {pos}")
                    o_node = pos[0]
                    d_node = o_node+1
                    sumoEdgeID= fs_edge_to_sumo_edge_id[(o_node, d_node)]
                    route = []
                    route.append(sumoEdgeID)
                    route_Name = "route_"+sumo_vid+"_"+str(sim_time)
                    if sumo_vid not in traci.simulation.getLoadedIDList():
                        LOG.debug(f"Vehicle {sumo_vid} is not laoded")
                    LOG.debug(f"Is this tried?")
                    #traci.vehicle.remove(str(vid))
                    LOG.debug(f"Has the vehicle been removed?")
                    traci.vehicle.addFull(sumo_vid,route_Name, typeID=opvid_to_veh_type[opid_vid_tuple])
                    LOG.debug(f"Is this succcessfull?")
                except:
                    pass
    
        step+=1

    t1_stop = perf_counter()
    time_elapsed = []
    time_elapsed.append(t1_stop)
    timefile = resultsPath+"Computationaltime.csv"
    #print(timefile)
    #print(time_elapsed)
    with open(timefile, 'w', newline = '') as csvfile:
        my_writer = csv.writer(csvfile, delimiter = ' ')
        my_writer.writerow(time_elapsed)
    EvaluationPath = resultsPath
    #print(EvaluationPath)
    eval.standard_evaluation(EvaluationPath, evaluation_start_time =None, evaluation_end_time =None, print_comments=True, dir_names_in = {})
    eval.evaluate_folder(EvaluationPath,evaluation_start_time = None, evaluation_end_time = None, print_comments = False)
    traci.close()
    sys.stdout.flush()
    

def setup_and_run_sumo_simulation(constant_config, scenario_config, sumo_config, sumoBinary="sumo-gui", log_level = "info", sumo_fcd_output=False):
    """ this method runs the coupled sumo and fleetpy simulation
    :param constant_config: fleetpy const config file
    :param scenario_config: fleetpy scenario config file (! only one scenario can be computed at once -> same format as constant config !)
    :param sumo_config: sumo configuration file
    :param sumoBinary: (str) if "sumo-gui", then sumo gui is shown, other option is "sumo" 
    :param log_level: Fleetpy log level specifaction
    :param sumo_fcd_output: if True, all vehicle coordinates per timestep is written to a file"""
    
    start_time = time.time()

    # start FleetPy
    FleetPy = setup_fleetsimulation(constant_config, scenario_config, log_level=log_level)
    resultsPath = FleetPy.dir_names[G_DIR_OUTPUT]
    seed = FleetPy.scenario_parameters[G_RANDOM_SEED]
    

    # setup traci connection to SUMO
    setup_traci(sumo_config, resultsPath, sumoBinary, seed, sumo_fcd_output=sumo_fcd_output)
    
    
    # Edge ID Dict Translator
    sumo_edge_id_to_fs_edge, fs_edge_to_sumo_edge_id, sumo_node_list, fs_edge_to_ff_tt, fs_edge_to_len = setup_network_translation(FleetPy)
  

    # Get interval in which new network statistics are gathered and sent to FleetPy to updated network (if not given, no statistics are gathered)
    travel_time_interval = FleetPy.scenario_parameters.get(G_SUMO_STAT_INT)
    if travel_time_interval is None:
        update_fleetsim_traveltimes = False
        update_travel_statistics_time_step = 10000000000000
    else:
        update_fleetsim_traveltimes = True
        update_travel_statistics_time_step = travel_time_interval
    
    end_time_setup = time.time()
    # run the simulation
    run_simulation(FleetPy, sumo_edge_id_to_fs_edge, fs_edge_to_sumo_edge_id, sumo_node_list, fs_edge_to_ff_tt,fs_edge_to_len,
                   update_fleetsim_traveltimes=update_fleetsim_traveltimes, update_travel_statistics_time_step=update_travel_statistics_time_step)
    
    end_time_simulation = time.time()

    LOG.info(f"Simulation completed. \n Initialisation time: {end_time_setup-start_time} \n Simulation time: {end_time_simulation-end_time_setup} \n Total time: {end_time_simulation-start_time}")

if __name__ == "__main__":
    
    try:
        constant_config = sys.argv[1]
        scenario_config = sys.argv[2]
        sumo_config = sys.argv[3]
        if len(sys.argv) > 4:
            sumoBinary = sys.argv[4]
        else:
            sumoBinary = "sumo-gui"
        if len(sys.argv) > 5:
            log_level = sys.argv[5]
        else:
            log_level = "info"
    except:
        print("something is wrong with the input given: ", sys.argv)
        exit()
        
    setup_and_run_sumo_simulation(constant_config, scenario_config, sumo_config, sumoBinary=sumoBinary, log_level=log_level)
    
