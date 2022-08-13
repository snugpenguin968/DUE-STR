from controller.RouteController import RouteController
from core.Util import ConnectionInfo, Vehicle
import numpy as np
import traci
import math
import copy

class FWController(RouteController):

    def __init__(self, connection_info, file_name, Round_name):
        super().__init__(connection_info)
        self.routes = {}  # = to route with regards to directions from current edge is retruend by make decisions
        self.location = {}  # used to help with the operation of returning make decisions and routes
        self.position = {}
        self.routes_edges = {}  # same as routes but has the explicit id for each edge (ow the edge)
        self.trips = {}
        self.time = {}
        self.visited = []

    def calculate_dists(self):
        """
        Computes and returns a dictionary of edge lengths for each road in use
        """
        dists={edge:0 for edge in self.connection_info.edge_list}
        lanes_list=traci.lane.getIDList()
        for lane in lanes_list:
            edge=traci.lane.getEdgeID(lane)
            if edge in dists:
                dists[edge]=self.connection_info.edge_length_dict[edge]
        return dists


    def floyd_warshall(self,dists):
        """
        Builds the matrix of shortest distances between any 2 edges
        :param dists: A dictionary of distances
        :return: A matrix of shortest distances between each pair of edges
                 Also sets up a matrix that helps in tracing the optimal path between two edges
        """

        inf=99999
        E=len(self.connection_info.edge_list)
        graph=[[inf if i!=j else 0 for j in range(E)] for i in range(E)]
        next = [[-1] * E for i in range(E)]
        for i in range(E):
            next[i][i]=i
        for edge in self.connection_info.edge_list:
            i=self.connection_info.edge_index_dict[edge]
            for direction,outgoing_edge in self.connection_info.outgoing_edges_dict[edge].items():
                j=self.connection_info.edge_index_dict[outgoing_edge]
                graph[i][j]=dists[edge]
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
        Finds the optimal path for each vehicle
        :param next: a helper matrix that tracks the optimal path
        :param start: the vehicle's current edge
        :param destination: the vehicle's target edge
        :return: the optimal path
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
        :param vehicles: all of the vehicles in action
        :param connection_info: information about the map
        :return: the next edge that each vehicle should travel to
        """
        dists=self.calculate_dists()
        graph,next=self.floyd_warshall(dists)
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