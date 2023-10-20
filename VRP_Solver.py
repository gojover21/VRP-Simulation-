import numpy as np
import pandas as pd
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def print_solution(manager, routing, assignment, demands, labels_mapping):
    total_distance = 0
    total_children_picked = 0
    for vehicle_id in range(manager.GetNumberOfVehicles()):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_distance = 0
        route_children = 0
        while not routing.IsEnd(index):
            children = demands[manager.IndexToNode(index)]
            plan_output += ' {} ({} children) ->'.format(labels_mapping[manager.IndexToNode(index)], children)
            route_children += children
            previous_index = index
            index = assignment.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
        plan_output += ' {}\n'.format(labels_mapping[manager.IndexToNode(index)])
        plan_output += 'Distance of the route: {} km\n'.format(route_distance)
        plan_output += 'Children picked up: {}\n'.format(route_children)
        print(plan_output)
        total_distance += route_distance
        total_children_picked += route_children
    print('Total Distance of all routes: {} km'.format(total_distance))
    print('Total children picked up: {}'.format(total_children_picked))

def read_distance_matrix_from_csv(file_path):
    df = pd.read_csv(file_path, index_col=0)
    labels_mapping = {idx: label for idx, label in enumerate(df.index)}
    return df.values.astype(int), labels_mapping

def main():
    # Read distance matrix and labels mapping from the CSV file
    distance_matrix, labels_mapping = read_distance_matrix_from_csv(r"C:\Users\akash\Downloads\Distance_AAA.csv")
    
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), 6, 0)
    routing = pywrapcp.RoutingModel(manager)

    # Fixed capacity for each vehicle
    vehicle_capacity = 30

    # Replace random demand with the exact number of kids to be picked at the bus stops
    demands = [0, 1, 6, 7, 3, 4, 5, 8, 4, 1, 4, 4, 5, 7, 2, 3, 3, 15, 8, 1, 9, 3, 1, 6, 3, 4, 1, 10, 2, 2, 3, 1, 3, 2, 4, 2, 3, 4, 1, 6, 3, 1, 2, 1, 1, 1]

    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return demands[from_node]

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        [vehicle_capacity] * 6,  # capacities of all vehicles
        True,  # start cumul to zero
        'Capacity'
    )
    
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    assignment = routing.SolveWithParameters(search_parameters)
    if assignment:
        print_solution(manager, routing, assignment, demands, labels_mapping)

if __name__ == '__main__':
    main()
