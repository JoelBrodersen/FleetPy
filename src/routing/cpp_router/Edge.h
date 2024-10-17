#pragma once

class Edge {
private:
	int start_node_;
	int end_node_;
	double travel_time_;
	double travel_distance_;
	double edge_std_;
	double edge_cfv_;
public:
	Edge(int start_node, int end_node, double travel_time, double travel_distance);
	int getStartNode();
	int getEndNode();
	double getTravelTime();
	double getTravelDistance();
	double getTravelTimeStd();
    double getCostFunctionValue();
    void setNewTravelTimeStd(double edge_std);
    void setNewTravelTime(double travel_time);
    void setNewEdgeCfv(double travel_time);
};