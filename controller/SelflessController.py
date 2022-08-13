from controller.RouteController import RouteController
from core.Util import ConnectionInfo, Vehicle
import numpy as np
import traci
import math
import copy
from queue import PriorityQueue
import random


class SelflessPolicy_Reader(RouteController):

    def __init__(self, connection_info, file_name, Round_name):
        super().__init__(connection_info)
        self.density=None
        self.flow=None
        self.dists=None
        self.lens=None
        self.time=None
        self.halting=None
        self.wait_time=None
        self.on_edge=None



    #Calculates the number of vehicles per meter on an edge
    def calculate_density(self):
        self.density={edge:0 for edge in self.connection_info.edge_list}
        lanes_list=traci.lane.getIDList()
        for lane in lanes_list:
            edge=traci.lane.getEdgeID(lane)
            if edge in self.density:
                self.density[edge]=max(traci.edge.getLastStepVehicleNumber(edge),0.01)/self.connection_info.edge_length_dict[edge]
        self.calculate_flow()

    #Calculates the number of vehicles that traverse an edge per second
    def calculate_flow(self):
        self.flow={edge:0 for edge in self.connection_info.edge_list}
        for key,value in self.density.items():
            if key in self.flow:
                self.flow[key]=max(traci.edge.getLastStepMeanSpeed(key),0.01)*self.density[key]

    #Estimate the time it takes for a vehicle to traverse an edge
    def calculate_time(self):
        self.time={edge:0 for edge in self.connection_info.edge_list}
        lanes_list=traci.lane.getIDList()
        for lane in lanes_list:
            edge=traci.lane.getEdgeID(lane)
            if edge in self.time:
                if traci.edge.getLastStepMeanSpeed(edge)==0:
                    self.time[edge]=1
                else:
                    self.time[edge]=self.connection_info.edge_length_dict[edge]/traci.edge.getLastStepMeanSpeed(edge)

    #Calculate the length of an edge
    def calculate_dists(self):
        self.dists={edge:0 for edge in self.connection_info.edge_list}
        lanes_list=traci.lane.getIDList()
        for lane in lanes_list:
            edge=traci.lane.getEdgeID(lane)
            if edge in self.dists:
                self.dists[edge]=self.connection_info.edge_length_dict[edge]

    #Calculates the length of an edge that the vehicles take up
    def calculate_len(self):
        self.lens={edge:0 for edge in self.connection_info.edge_list}
        lanes_list=traci.lane.getIDList()
        for lane in lanes_list:
            edge=traci.lane.getEdgeID(lane)
            if edge in self.lens:
                self.lens[edge]=max(traci.edge.getLastStepOccupancy(edge),random.random())*self.connection_info.edge_length_dict[edge]

    #Calculates the percent of vehicles on an edge that halt (reduce speed to <0.1 m/s)
    def calculate_halting(self):
        self.halting={edge:0 for edge in self.connection_info.edge_list}
        lanes_list=traci.lane.getIDList()
        for lane in lanes_list:
            edge=traci.lane.getEdgeID(lane)
            if edge in self.halting:
                if traci.edge.getLastStepVehicleNumber(edge)==0:
                    self.halting[edge]=0.01
                else:
                    self.halting[edge]=traci.edge.getLastStepHaltingNumber(edge)/traci.edge.getLastStepVehicleNumber(edge)


    #Calculate the total waiting time of all vehicles on an edge
    def calculate_wait_time(self):
        self.wait_time={edge:0 for edge in self.connection_info.edge_list}
        lanes_list = traci.lane.getIDList()
        for lane in lanes_list:
            edge = traci.lane.getEdgeID(lane)
            if edge in self.wait_time:
                self.wait_time[edge]=traci.edge.getWaitingTime(edge)

    #Calculates percentage of vehicles that are on each edge
    def calculate_on_edge(self,vehicles):
        self.on_edge = {edge: 0 for edge in self.connection_info.edge_list}
        lanes_list = traci.lane.getIDList()
        for lane in lanes_list:
            edge = traci.lane.getEdgeID(lane)
            if edge in self.on_edge:
                self.on_edge[edge] = traci.edge.getLastStepVehicleNumber(edge)/max(len(vehicles),1)


    def scale_help(self,metric):
        """
        Computes the minimum and maximum values in a dictionary
        :param metric: A dictionary of features
        :return: The minimum and maximum of the dictionary
        """
        max_metric=0
        min_metric=1e10
        for key,val in metric.items():
            max_metric=max(max_metric,val)
            min_metric=min(min_metric,val)
        return min_metric,max_metric

    def scale_to_dist(self,min_dist,max_dist,curr_val,min_metric,max_metric):
        """
        Scales a metric to the distance dictionary's scale
        :param min_dist: Minimum value in distance dictionary
        :param max_dist: Maximum value in distance dictionary
        :param curr_val: The current value of the metric that must be scaled
        :param min_metric: The minimum value of the metric
        :param max_metric: The maximum value of the metric
        :return: The scaled value
        """
        return (min_dist+(max_dist-min_dist)*(curr_val-min_metric))/(max_metric-min_metric)

    def floyd_warshall(self):
        """
        Creates a matrix that defines the weights between any 2 edges
        A helper matrix is also used to track the shortest paths between any 2 edges
        :param vehicles: list of vehicles
        :return: The weight matrix and the helper matrix
        """
        inf=1e10
        min_density,max_density = self.scale_help(self.density)
        min_dist,max_dist=self.scale_help(self.dists)
        min_flow,max_flow=self.scale_help(self.flow)
        min_len,max_len=self.scale_help(self.lens)

        E=len(self.connection_info.edge_list)
        graph=[[inf if i!=j else 0 for j in range(E)] for i in range(E)]
        next = [[-1] * E for i in range(E)]
        for i in range(E):
            next[i][i]=i
        for edge in self.connection_info.edge_list:
            i=self.connection_info.edge_index_dict[edge]
            for direction,outgoing_edge in self.connection_info.outgoing_edges_dict[edge].items():
                j=self.connection_info.edge_index_dict[outgoing_edge]
                scaled_density=self.scale_to_dist(min_dist,max_dist,self.density[edge],min_density,max_density)
                scaled_flow=self.scale_to_dist(min_dist,max_dist,self.flow[edge],min_flow,max_flow)
                scaled_len=self.scale_to_dist(min_dist,max_dist,self.lens[edge],min_len,max_len)
                graph[i][j] = max(self.halting[edge]*scaled_density+(1-self.halting[edge])*self.dists[edge],0) #226.5892
                next[i][j]=j
        for k in range(E):
            for i in range(E):
                for j in range(E):
                    if graph[i][k]+graph[k][j]<graph[i][j]:
                        graph[i][j]=min(graph[i][j],graph[i][k]+graph[k][j])
                        next[i][j]=next[i][k]
        return graph,next

    def FindPath(self,next,start,destination):
        """
        Finds the optimal path that a vehicle should take based on the weights
        :param next: helper matrix that traces the shortest paths
        :param start: vehicle's current edge
        :param destination: vehicle's destination edge
        :return: a list of edges that the vehicle should follow
        """
        if next[start][destination]==-1:
            return []
        else:
            path=[]
            path.append(start)
            while start!=destination:
                start=next[start][destination]
                path.append(start)
        return path



    def make_decisions(self, vehicles, connection_info):
        """
        Computes the next direction for each vehicle
        :param vehicles: list of all vevhicles
        :param connection_info: information about the map
        :return: a dictionary that contains the direction that each vehicle should take
        """
        self.calculate_density()
        self.calculate_time()
        self.calculate_dists()
        self.calculate_len()
        self.calculate_halting()
        self.calculate_wait_time()
        self.calculate_on_edge(vehicles)
        graph,next=self.floyd_warshall()
        local_targets = {}

        for vehicle in vehicles:
            start=self.connection_info.edge_index_dict[vehicle.current_edge]
            destination=self.connection_info.edge_index_dict[vehicle.destination]
            path = self.FindPath(next, start, destination)
            edge_path = []
            for edge in path:
                for key, value in self.connection_info.edge_index_dict.items():
                    if edge == value:
                        edge_path.append(key)

            decision_list = []
            for i in range(len(edge_path) - 1):
                for direction, outgoing_edge in self.connection_info.outgoing_edges_dict[edge_path[i]].items():
                    if outgoing_edge == edge_path[i + 1]:
                        decision_list.append(direction)

            local_targets[vehicle.vehicle_id]=self.compute_local_target(decision_list,vehicle)

        return local_targets







