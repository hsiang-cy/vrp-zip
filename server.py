from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import grpc
from concurrent import futures
import vrp_pb2
import vrp_pb2_grpc
from grpc_reflection.v1alpha import reflection

def solve_vrp(distance_matrix, num_locations, num_vehicles, depot, locations, vehicles):
    manager = pywrapcp.RoutingIndexManager(num_locations, num_vehicles, depot)
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return locations[from_node].cargo  # 每個地點的貨物數量

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(demand_callback_index, 0, [vehicle.capacity for vehicle in vehicles], True, 'Capacity')

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_parameters.time_limit.seconds = 30

    solution = routing.SolveWithParameters(search_parameters)

    if solution:
        routes = []
        total_cost = 0
        for vehicle_id in range(num_vehicles):
            index = routing.Start(vehicle_id)
            route = []
            vehicle_cost = vehicles[vehicle_id].fixed_cost  # 固定出車費用
            while not routing.IsEnd(index):
                route.append(manager.IndexToNode(index))
                index = solution.Value(routing.NextVar(index))
            route.append(manager.IndexToNode(routing.End(vehicle_id)))

            # 計算行駛成本
            for i in range(len(route) - 1):
                vehicle_cost += vehicles[vehicle_id].cost_per_km * distance_matrix[route[i]][route[i+1]]

            routes.append(route)
            total_cost += vehicle_cost

        return routes, total_cost

    return None, None

class VrpServiceServicer(vrp_pb2_grpc.VrpServiceServicer):
    def SolveVrp(self, request, context):
        num_locations = request.num_locations
        distance_matrix = [request.distance_matrix[i:i+num_locations]
                          for i in range(0, len(request.distance_matrix), num_locations)]
        locations = request.locations
        vehicles = request.vehicles
        routes, total_cost = solve_vrp(distance_matrix, num_locations, request.num_vehicles, request.depot, locations, vehicles)
        
        if routes:
            response = vrp_pb2.VrpResponse()
            for route in routes:
                r = response.routes.add()
                r.nodes.extend(route)
            return response
        context.set_code(grpc.StatusCode.INVALID_ARGUMENT)
        context.set_details(f'No solution found, total cost: {total_cost}')
        return vrp_pb2.VrpResponse()

def serve():
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    vrp_pb2_grpc.add_VrpServiceServicer_to_server(VrpServiceServicer(), server)

    SERVICE_NAMES = (
        vrp_pb2.DESCRIPTOR.services_by_name['VrpService'].full_name,
        reflection.SERVICE_NAME,
    )
    reflection.enable_server_reflection(SERVICE_NAMES, server)

    server.add_insecure_port('[::]:50051')
    server.start()
    server.wait_for_termination()

if __name__ == '__main__':
    serve()
