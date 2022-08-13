from controller.RouteController import RouteController
from core.Util import ConnectionInfo, Vehicle
import numpy as np
import traci
import math
import copy
from core.Estimation import *
from core.STR_SUMO import *
import core.Run_id
from xml.dom import minidom




class ShortestTimeController(RouteController):

    def __init__(self, connection_info, file_name, Round_name):
        super().__init__(connection_info)
        self.routes = {}  # = to route with regards to directions from current edge is retruend by make decisions
        self.location = {}  # used to help with the operation of returning make decisions and routes
        self.position = {}
        self.routes_edges = {}  # same as routes but has the explicit id for each edge (ow the edge)
        self.trips = {}
        self.time = {}
        self.visited=[]
        # file_name = "static2.rou.xml"
        doc = minidom.parse("./configurations/Rounds/" + Round_name + '/' + file_name)
        # print("FILE NAME:",file_name)
        veh = doc.getElementsByTagName("vehicle")
        # self.
        # print(self.trips)
        # print("brudda")
        for t in veh:
            vid = t.getAttribute("id")
            route_tag = t.getElementsByTagName("route")[0]
            route = route_tag.getAttribute("edges")
            root = route.split(' ')
            self.trips[vid] = root

        trip_info = {}
        est_travel_time = {}
        distance_route = {}
        num_edges = {}
        sharing_route = {}
        num_on_edges = {}
        for x in self.trips.keys():
            est_travel_time[x] = 0
            distance_route[x] = 0
            num_edges[x] = len(self.trips[x])

            for y in self.trips[x]:
                est_travel_time[x] += connection_info.edge_length_dict[y] / connection_info.edge_speed_dict[y]
                distance_route[x] += connection_info.edge_length_dict[y]
                if y not in num_on_edges.keys():
                    num_on_edges[y] = []
                if x not in num_on_edges[y]:
                    num_on_edges[y].append(x)

        for x in self.trips.keys():
            sharing_route[x] = 0
            temp_ids = []
            for y in self.trips[x]:
                # print(type(temp_ids))
                # print(type(num_on_edges[y]))
                temp_ids = set(num_on_edges[y] + list(temp_ids))
            sharing_route[x] = len(list(temp_ids))
            # print()
            trip_info[x] = [est_travel_time[x], sharing_route[x], num_edges[x], distance_route[x]]

        core.Run_id.route_information = trip_info  # =[est_travel_time[x],sharing_route[x],num_edges[x],distance_route[x]]
        core.Run_id.trips = self.trips
        core.Run_id.Controller_version = "N/A"


    def ModifiedDijkstra(self, vehicle, connection_info, current):
        """

        :param vehicle: The current vehicle
        :param connection_info: Information about the map
        :param current: The vehicle's start edge
        :return: Lists containing the shortest distance in terms of directions and edges
        """
        decision_list = []
        unvisited = {edge: 1000000000 for edge in self.connection_info.edge_list}  # map of unvisited edges
        visited = {}  # map of visited edges
        current_edge = current

        current_distance = self.connection_info.edge_length_dict[current_edge] / self.connection_info.edge_speed_dict[
            current_edge]
        unvisited[current_edge] = current_distance
        path_lists = {edge: [] for edge in
                      self.connection_info.edge_list}  # stores shortest path to each edge using directions
        edge_path_list = {edge: [] for edge in self.connection_info.edge_list}
        while True:
            if current_edge not in self.connection_info.outgoing_edges_dict.keys():
                continue

            for direction, outgoing_edge in self.connection_info.outgoing_edges_dict[current_edge].items():

                if outgoing_edge not in unvisited:
                    continue
                edge_length = self.connection_info.edge_length_dict[outgoing_edge] / \
                              self.connection_info.edge_speed_dict[outgoing_edge]  # self.edge_speed_dict
                new_distance = current_distance + edge_length
                if new_distance < unvisited[outgoing_edge]:
                    unvisited[outgoing_edge] = new_distance
                    current_path = copy.deepcopy(path_lists[current_edge])
                    current_path.append(direction)
                    path_lists[outgoing_edge] = copy.deepcopy(current_path)

                    current_edge_path = copy.deepcopy(edge_path_list[current_edge])
                    current_edge_path.append(outgoing_edge)
                    edge_path_list[outgoing_edge] = copy.deepcopy(current_edge_path)

            visited[current_edge] = current_distance

            del unvisited[current_edge]

            if not unvisited:  # fairly certain this means that there is no other route
                return [], []
                break

            if current_edge == vehicle.destination:
                break

            possible_edges = [edge for edge in unvisited.items() if edge[1]]

            current_edge, current_distance = sorted(possible_edges, key=lambda x: x[1])[0]

        for direction in path_lists[vehicle.destination]:
            decision_list.append(direction)
        this_edge_list = []
        for edge in edge_path_list[vehicle.destination]:
            this_edge_list.append(edge)

        return decision_list, this_edge_list

    def make_decisions(self, vehicles, connection_info):
        """
        make_decisions algorithm uses Dijkstra's Algorithm to find the shortest path to each individual vehicle's destination
        :param vehicles: list of vehicles on the map
        :param connection_info: information about the map (roads, junctions, etc)
        """
        local_targets = {}
        check={vehicle.vehicle_id:[] for vehicle in vehicles}
        for vehicle in vehicles:
            self.visited.append(vehicle.vehicle_id)
            all_times=[]
            all_edges=[]
            #At least 1 route to the destination must exist


            #Consider each edge connected to the vehicle's current edge

            for direction,outgoing_edge in self.connection_info.outgoing_edges_dict[vehicle.current_edge].items():
                directions,edges=self.ModifiedDijkstra(vehicle,connection_info,outgoing_edge) #Run Dijkstra's on each outgoing edge to find multiple paths to target

                #edges.insert(0, outgoing_edge)
                edges.insert(0, vehicle.current_edge) #Account for current edge
                if len(edges)==1: #Ensure that a path exists
                    edges.append(vehicle.destination)
                if edges:
                    if edges in check[vehicle.vehicle_id]:
                        continue
                    else:
                        check[vehicle.vehicle_id].append(edges)
                all_edges.append(edges)

            #Estimate total time it takes to reach the target
            for edge_list in all_edges:
                est = 0
                for e in edge_list:
                    est += connection_info.edge_length_dict[e] / connection_info.edge_speed_dict[e]
                all_times.append(est)

            #Sort

            edges_dict={}
            for li,time in zip(all_edges,all_times):
                edges_dict[tuple(li)]=time


            sorted_edge_pairs=sorted(edges_dict.items(),key=lambda x:x[1])
            #sorted_direction_pairs=sorted(directions_dict.items(),key=lambda x:x[1])
            sorted_edge_list=[list(i[0]) for i in sorted_edge_pairs]
            #sorted_direction_list=[list(i[0]) for i in sorted_direction_pairs]
            final_edges_dict=dict(sorted_edge_pairs)
            #final_directions_dict=dict(sorted_direction_pairs)

            idx = 0 #Select the path that takes the least time

            edge_list=sorted_edge_list[idx]
            #decision_list=sorted_direction_list[idx]
            decision_list=[]



            for i in range(len(edge_list)-1):
                curr=edge_list[i]
                next=edge_list[i+1]
                for direction, outgoing_edge in self.connection_info.outgoing_edges_dict[curr].items():

                    if outgoing_edge==next:
                        decision_list.append(direction)

            self.routes_edges.update({vehicle.vehicle_id: edge_list})
            self.routes.update({vehicle.vehicle_id:decision_list})

            #local_targets[vehicle.vehicle_id] = self.compute_local_target(decision_list, vehicle)

            idx2 = self.routes_edges[vehicle.vehicle_id].index(vehicle.current_edge)
            local_targets[vehicle.vehicle_id] = self.routes_edges[vehicle.vehicle_id][idx2 + 1]

        return local_targets