import logging
LOG = logging.getLogger(__name__)

try:
    from . import PriorityQueue_python3 as PQ
    from src.routing.routing_imports.Router import Router
except:
    try:
        import src.routing.routing_imports.PriorityQueue_python3 as PQ
    except:
        raise ImportError("couldnt import PriorityQueue_python3")

RAISE_ERROR = False

def shortest_travel_time_cost_function(travel_time, travel_distance, current_node_index):
    """ standard version of a customized_section_cost_function for computing time shortest routes"""
    return travel_time

class RouterReliability(Router):
    """class for computing dijkstra-algorithms in various ways
    nw is a graph network -> TODO
    computes one to many
    start_node: index of start_node
    destination_nodes: list of indices of destination nodes
    mode: None: standard dijkstra | "bidirectional": bidirectional dijkstra (one after another for one to many)
    time_radius: breaks after this radius is reached and returns "no route found" ->negative travel time and travel distance
    max_steps: breaks after max_steps dijkstra steps
    max_settled_nodes: breaks after max_settled_nodes nodes are settled
    forward_flag: if False -> backwards dijkstra is performed, start_node is start of dijkstra (returned route ends with start_node)
    dijkstra_number: sets dijkstra number in nodes (this current signal, if a node has been touched by this dijkstra computation|increased by one after each dijkstra if None)
    with_arc_flags: if True -> arc_flag filtering for next arcs is used
    ch_flag: contraction hierarchy is used
    """
    def __init__(self, nw, start_node, destination_nodes = [], mode = None, time_radius = None, max_settled_targets = None, forward_flag = True, ch_flag = False, customized_section_cost_function = None):
        self.nw = nw
        self.start = start_node
        self.back_end = None
        if not forward_flag:
            self.back_end = start_node
        self.destination_nodes = {}
        for d in destination_nodes:
            self.destination_nodes[d] = True
            self.nw.nodes[d].is_target_node = True
        self.number_destinations = len(destination_nodes)
        if max_settled_targets is not None and max_settled_targets < self.number_destinations:
            self.number_destinations = max_settled_targets
        self.time_radius = time_radius
        self.forward_flag = forward_flag
        self.mode = mode
        self.ch_flag = ch_flag

        if self.ch_flag:
            self.start_hc_val = self.nw.nodes[start_node].ch_value
            self.end_hc_val = 999999999999999

        self.customized_section_cost_function = customized_section_cost_function
        if self.customized_section_cost_function == None:
            if self.ch_flag:
                print("WARNING IN ROUTER: Contraction Hierachies disabled! Only time shortest computations feasible!")
                self.ch_flag = False
            self.customized_section_cost_function = shortest_travel_time_cost_function

        self.dijkstra_number = self.nw.current_dijkstra_number + 1
        self.n_settled = 0

    def dijkstraStepForwards(self, frontier, current_node_obj, current_cost):
        """ one dijkstra step forward
        checks all nodes at end of outgoing arcs
        arcs are filtered depending on preprocessing flags (ch/arcs)
        sets node attributes settled/cost_index/cost/prev
        """
        #print("{} {}".format(current_node_obj.id, current_cost))
        current_node_obj.settled = self.dijkstra_number
        self.n_settled += 1

        if current_node_obj.node_index != self.start and current_node_obj.must_stop():
            return

        if self.time_radius is not None and self.time_radius < current_cost:
            return

        next_nodes_and_edges = current_node_obj.get_next_node_edge_pairs(ch_flag = self.ch_flag)
        for next_node_obj, next_edge_obj in next_nodes_and_edges:
            edge_tt, edge_tt_std = next_edge_obj.get_tt_mean_std()
            new_end_cost = current_cost + self.customized_section_cost_function(edge_tt, edge_tt_std)
            if next_node_obj.settled != self.dijkstra_number:
                if next_node_obj.cost_index != -self.dijkstra_number:
                    next_node_obj.cost = (new_end_cost, current_node_obj.cost[1] + edge_tt, current_node_obj.cost[2] + edge_tt_std )
                    next_node_obj.prev = current_node_obj
                    next_node_obj.cost_index = -self.dijkstra_number
                    frontier.addTask(next_node_obj, new_end_cost)
                else:
                    if next_node_obj.cost[0] > new_end_cost:
                        next_node_obj.cost = (new_end_cost, current_node_obj.cost[1] + edge_tt, current_node_obj.cost[2] + edge_tt_std )
                        next_node_obj.prev = current_node_obj
                        frontier.addTask(next_node_obj, new_end_cost)
    
    def dijkstraStepBackwards(self, frontier, current_node_obj, current_cost):
        """ one dijkstra step backward
        checks all nodes at start of incoming arcs
        arcs are filtered depending on preprocessing flags (ch/arcs)
        sets node attributes settled_back/cost_index_back/cost_back/next
        """
        current_node_obj.settled_back = self.dijkstra_number
        self.n_settled += 1

        if current_node_obj.node_index != self.back_end and current_node_obj.must_stop():
            return

        next_nodes_and_edges = current_node_obj.get_prev_node_edge_pairs(ch_flag = self.ch_flag)

        for next_node_obj, next_edge_obj in next_nodes_and_edges:
            edge_tt, edge_tt_std = next_edge_obj.get_tt_mean_std()
            new_end_cost = current_cost + self.customized_section_cost_function(edge_tt, edge_tt_std)
            if next_node_obj.settled_back != self.dijkstra_number:
                if next_node_obj.cost_index_back != -self.dijkstra_number:
                    next_node_obj.cost_back = (new_end_cost, current_node_obj.cost_back[1] + edge_tt, current_node_obj.cost_back[2] + edge_tt_std )
                    next_node_obj.next = current_node_obj
                    next_node_obj.cost_index_back = -self.dijkstra_number
                    frontier.addTask(next_node_obj, new_end_cost)
                else:
                    if next_node_obj.cost_back[0] > new_end_cost:
                        next_node_obj.cost_back = (new_end_cost, current_node_obj.cost_back[1] + edge_tt, current_node_obj.cost_back[2] + edge_tt_std )
                        next_node_obj.next = current_node_obj
                        frontier.addTask(next_node_obj, new_end_cost)
