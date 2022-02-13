




import matplotlib.pyplot as plt
import sys
from math import sqrt, log
from itertools import product
from mip import Model, xsum, minimize, OptimizationStatus


"""Simple Vehicles Routing Problem (VRP).

   This is a sample using the routing library python wrapper to solve a VRP
   problem.
   A description of the problem can be found here:
   http://en.wikipedia.org/wiki/Vehicle_routing_problem.

   Distances are in meters.
"""

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


def create_data_model():
    """Stores the data for the problem."""
    data = {}
    data['distance_matrix'] = [
        [
            0, 548, 776, 696, 582, 274, 502, 194, 308, 194, 536, 502, 388, 354,
            468, 776, 662
        ],
        [
            548, 0, 684, 308, 194, 502, 730, 354, 696, 742, 1084, 594, 480, 674,
            1016, 868, 1210
        ],
        [
            776, 684, 0, 992, 878, 502, 274, 810, 468, 742, 400, 1278, 1164,
            1130, 788, 1552, 754
        ],
        [
            696, 308, 992, 0, 114, 650, 878, 502, 844, 890, 1232, 514, 628, 822,
            1164, 560, 1358
        ],
        [
            582, 194, 878, 114, 0, 536, 764, 388, 730, 776, 1118, 400, 514, 708,
            1050, 674, 1244
        ],
        [
            274, 502, 502, 650, 536, 0, 228, 308, 194, 240, 582, 776, 662, 628,
            514, 1050, 708
        ],
        [
            502, 730, 274, 878, 764, 228, 0, 536, 194, 468, 354, 1004, 890, 856,
            514, 1278, 480
        ],
        [
            194, 354, 810, 502, 388, 308, 536, 0, 342, 388, 730, 468, 354, 320,
            662, 742, 856
        ],
        [
            308, 696, 468, 844, 730, 194, 194, 342, 0, 274, 388, 810, 696, 662,
            320, 1084, 514
        ],
        [
            194, 742, 742, 890, 776, 240, 468, 388, 274, 0, 342, 536, 422, 388,
            274, 810, 468
        ],
        [
            536, 1084, 400, 1232, 1118, 582, 354, 730, 388, 342, 0, 878, 764,
            730, 388, 1152, 354
        ],
        [
            502, 594, 1278, 514, 400, 776, 1004, 468, 810, 536, 878, 0, 114,
            308, 650, 274, 844
        ],
        [
            388, 480, 1164, 628, 514, 662, 890, 354, 696, 422, 764, 114, 0, 194,
            536, 388, 730
        ],
        [
            354, 674, 1130, 822, 708, 628, 856, 320, 662, 388, 730, 308, 194, 0,
            342, 422, 536
        ],
        [
            468, 1016, 788, 1164, 1050, 514, 514, 662, 320, 274, 388, 650, 536,
            342, 0, 764, 194
        ],
        [
            776, 868, 1552, 560, 674, 1050, 1278, 742, 1084, 810, 1152, 274,
            388, 422, 764, 0, 798
        ],
        [
            662, 1210, 754, 1358, 1244, 708, 480, 856, 514, 468, 354, 844, 730,
            536, 194, 798, 0
        ],
    ]
    data['num_vehicles'] = 4
    data['depot'] = 0
    return data


def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    print(f'Objective: {solution.ObjectiveValue()}')
    max_route_distance = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_distance = 0
        while not routing.IsEnd(index):
            plan_output += ' {} -> '.format(manager.IndexToNode(index))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)
        plan_output += '{}\n'.format(manager.IndexToNode(index))
        plan_output += 'Distance of the route: {}m\n'.format(route_distance)
        print(plan_output)
        max_route_distance = max(route_distance, max_route_distance)
    print('Maximum of the route distances: {}m'.format(max_route_distance))



def main():
    """Entry point of the program."""
    # Instantiate the data problem.
    data = create_data_model()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)


    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance constraint.
    dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        3000,  # vehicle maximum travel distance
        True,  # start cumul to zero
        dimension_name)
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(data, manager, routing, solution)
    else:
        print('No solution found !')


if __name__ == '__main__':
    main()





sys.exit()
# possible plants
F = [1, 2, 3, 4, 5, 6]

# possible plant installation positions
pf = {1: (1, 38), 2: (31, 40), 3: (23, 59), 4: (76, 51), 5: (93, 51), 6: (63, 74)}

# maximum plant capacity
c = {1: 1955, 2: 1932, 3: 1987, 4: 1823, 5: 1718, 6: 1742}

# clients
C = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]

# position of clients
pc = {1: (94, 10), 2: (57, 26), 3: (74, 44), 4: (27, 51), 5: (78, 30), 6: (23, 30),
      7: (20, 72), 8: (3, 27), 9: (5, 39), 10: (51, 1)}

# demands
d = {1: 302, 2: 273, 3: 275, 4: 266, 5: 287, 6: 296, 7: 297, 8: 310, 9: 302, 10: 309}

# plotting possible plant locations
for i, p in pf.items():
    plt.scatter((p[0]), (p[1]), marker="^", color="purple", s=50)
    plt.text((p[0]), (p[1]), "$f_%d$" % i)

# plotting location of clients
for i, p in pc.items():
    plt.scatter((p[0]), (p[1]), marker="o", color="black", s=15)
    plt.text((p[0]), (p[1]), "$c_{%d}$" % i)

plt.text((20), (78), "Region 1")
plt.text((70), (78), "Region 2")
plt.plot((50, 50), (0, 80))

dist = {(f, c): round(sqrt((pf[f][0] - pc[c][0]) ** 2 + (pf[f][1] - pc[c][1]) ** 2), 1)
        for (f, c) in product(F, C) }

m = Model()

z = {i: m.add_var(ub=c[i]) for i in F}  # plant capacity

# Type 1 SOS: only one plant per region
for r in [0, 1]:
    # set of plants in region r
    Fr = [i for i in F if r * 50 <= pf[i][0] <= 50 + r * 50]
    m.add_sos([(z[i], i - 1) for i in Fr], 1)

# amount that plant i will supply to client j
x = {(i, j): m.add_var() for (i, j) in product(F, C)}

# satisfy demand
for j in C:
    m += xsum(x[(i, j)] for i in F) == d[j]

# SOS type 2 to model installation costs for each installed plant
y = {i: m.add_var() for i in F}
for f in F:
    D = 6  # nr. of discretization points, increase for more precision
    v = [c[f] * (v / (D - 1)) for v in range(D)]  # points
    # non-linear function values for points in v
    vn = [0 if k == 0 else 1520 * log(v[k]) for k in range(D)]
    # w variables
    w = [m.add_var() for v in range(D)]
    m += xsum(w) == 1  # convexification
    # link to z vars
    m += z[f] == xsum(v[k] * w[k] for k in range(D))
    # link to y vars associated with non-linear cost
    m += y[f] == xsum(vn[k] * w[k] for k in range(D))
    m.add_sos([(w[k], v[k]) for k in range(D)], 2)

# plant capacity
for i in F:
    m += z[i] >= xsum(x[(i, j)] for j in C)

# objective function
m.objective = minimize(
    xsum(dist[i, j] * x[i, j] for (i, j) in product(F, C)) + xsum(y[i] for i in F) )

m.optimize()

plt.savefig("location.pdf")

if m.num_solutions:
    print("Solution with cost {} found.".format(m.objective_value))
    print("Facilities capacities: {} ".format([z[f].x for f in F]))
    print("Facilities cost: {}".format([y[f].x for f in F]))

    # plotting allocations
    for (i, j) in [(i, j) for (i, j) in product(F, C) if x[(i, j)].x >= 1e-6]:
        plt.plot(
            (pf[i][0], pc[j][0]), (pf[i][1], pc[j][1]), linestyle="--", color="darkgray"
        )

    plt.savefig("location-sol.pdf")