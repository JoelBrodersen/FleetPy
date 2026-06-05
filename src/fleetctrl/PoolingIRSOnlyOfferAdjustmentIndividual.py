import logging
import time
import numpy as np
from scipy.stats import norm

from src.simulation.Offers import TravellerOffer
from src.fleetctrl.FleetControlBase import FleetControlBase
from src.fleetctrl.PoolingIRSOnly import PoolingInsertionHeuristicOnly
from src.fleetctrl.PoolingIRSOnlyOfferAdjustment import PoolingInsertionHeuristicOnlyOfferAdjustment

from src.fleetctrl.planning.PlanRequest import PlanRequest
from src.fleetctrl.pooling.objectives import return_pooling_objective_function
from src.fleetctrl.pooling.immediate.insertion import insertion_with_heuristics
from src.misc.globals import *
LOG = logging.getLogger(__name__)
LARGE_INT = 100000

INPUT_PARAMETERS_PoolingInsertionHeuristicOnlyOfferAdjustment = {
    "doc" : "this class represents a ride-pooling MoD-operator. the operators uses an insertion heuristic for assignment",
    "inherit" : "FleetControlBase",
    "input_parameters_mandatory": [],
    "input_parameters_optional": [],
    "mandatory_modules": [],
    "optional_modules": []
}

class PoolingInsertionHeuristicOnlyOfferAdjustmentIndividual(PoolingInsertionHeuristicOnlyOfferAdjustment):
    """This class extends the PoolingInsertionHeuristicOnly class to include offer adjustment functionality.

    IMPORTANT NOTE:
    Both the new and the previously assigned plan are stored and await an instant response of the request. Therefore,
    this fleet control module is only consistent for the ImmediateOfferSimulation class.
    """
    def __init__(self, op_id, operator_attributes, list_vehicles, routing_engine, zone_system, scenario_parameters,
                 dir_names, op_charge_depot_infra=None, list_pub_charging_infra= []):
        """The specific attributes for the fleet control module are initialized. Strategy specific attributes are
        introduced in the children classes.

        :param op_id: operator id
        :type op_id: int
        :param operator_attributes: dictionary with keys from globals and respective values
        :type operator_attributes: dict
        :param list_vehicles: simulation vehicles; their assigned plans should be instances of the VehicleRouteLeg class
        :type list_vehicles: list
        :param routing_engine: routing engine
        :type routing_engine: Network
        :param scenario_parameters: access to all scenario parameters (if necessary)
        :type scenario_parameters: dict
        :param dirnames: directories for output and input
        :type dirnames: dict
        :param op_charge_depot_infra: reference to a OperatorChargingAndDepotInfrastructure class (optional) (unique for each operator)
        :type OperatorChargingAndDepotInfrastructure: OperatorChargingAndDepotInfrastructure
        :param list_pub_charging_infra: list of PublicChargingInfrastructureOperator classes (optional) (accesible for all agents)
        :type list_pub_charging_infra: list of PublicChargingInfrastructureOperator
        """
        super().__init__(op_id, operator_attributes, list_vehicles, routing_engine, zone_system, scenario_parameters,
                         dir_names=dir_names, op_charge_depot_infra=op_charge_depot_infra, list_pub_charging_infra=list_pub_charging_infra)

    def user_request(self, rq, sim_time):
        super().user_request(rq, sim_time)

   
    def get_segment_offer_tt(self,prq, tt, var):
        #print(f"Calculating offer_tt for prq {prq.rid} with tt: {tt}, var: {var}, routing_behavior_group: {prq.routing_behavior_group}")
        if prq.routing_behavior_group is None:
            raise ValueError(f"Routing behavior group is not defined for prq {prq.rid}.")
        else:
            for rb_group_config in self.scenario_parameters["routing_behavior_config"]:
                if rb_group_config["key"] == prq.routing_behavior_group:
                    rb_group_key = rb_group_config["key"]
                    rb_group_share = rb_group_config["share"]
                    rb_group_tt_adjustment = rb_group_config["tt_adjustment"]
                    break

        tt_adjustment_params = rb_group_tt_adjustment.split("_")
        ## Deterministic adjustment
        if tt_adjustment_params[0] == "det" and len(tt_adjustment_params) == 2:
            offer_tt = float(tt_adjustment_params[1])*tt
        ## Probabilistic adjustment
        elif tt_adjustment_params[0] == "prob" and len(tt_adjustment_params) == 2:
            offer_tt = self.normal_percentile(tt, var, float(tt_adjustment_params[1]))
        else:
            raise ValueError(f"Invalid routing behavior group: {prq.routing_behavior_group}. Expected format: 'det_x' or 'prob_x' where x is a number.")

        return offer_tt

    def _create_user_offer(self, prq, simulation_time, assigned_vehicle_plan=None, offer_dict_without_plan={}):
        super()._create_user_offer(prq, simulation_time, assigned_vehicle_plan=assigned_vehicle_plan, offer_dict_without_plan=offer_dict_without_plan)
        

    def assign_vehicle_plan(self, veh_obj, vehicle_plan, sim_time, force_assign=False, assigned_charging_task=None, add_arg=None):
        super().assign_vehicle_plan(veh_obj, vehicle_plan, sim_time, force_assign=force_assign, assigned_charging_task=assigned_charging_task, add_arg=add_arg)

    def lock_current_vehicle_plan(self, vid):
        super().lock_current_vehicle_plan(vid)

    def _lock_vid_rid_pickup(self, sim_time, vid, rid):
        super()._lock_vid_rid_pickup(sim_time, vid, rid)

