#definitions
import random
import threading 
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import time
from datetime import datetime, timedelta


dist_matrix =  [[0, 548, 776, 696, 582, 274, 502, 194, 308, 194, 536, 502, 388, 354, 468, 776, 662],
      [548, 0, 684, 308, 194, 502, 730, 354, 696, 742, 1084, 594, 480, 674, 1016, 868, 1210],
      [776, 684, 0, 992, 878, 502, 274, 810, 468, 742, 400, 1278, 1164, 1130, 788, 1552, 754],
      [696, 308, 992, 0, 114, 650, 878, 502, 844, 890, 1232, 514, 628, 822, 1164, 560, 1358],
      [582, 194, 878, 114, 0, 536, 764, 388, 730, 776, 1118, 400, 514, 708, 1050, 674, 1244],
      [274, 502, 502, 650, 536, 0, 228, 308, 194, 240, 582, 776, 662, 628, 514, 1050, 708],
      [502, 730, 274, 878, 764, 228, 0, 536, 194, 468, 354, 1004, 890, 856, 514, 1278, 480],
      [194, 354, 810, 502, 388, 308, 536, 0, 342, 388, 730, 468, 354, 320, 662, 742, 856],
      [308, 696, 468, 844, 730, 194, 194, 342, 0, 274, 388, 810, 696, 662, 320, 1084, 514],
      [194, 742, 742, 890, 776, 240, 468, 388, 274, 0, 342, 536, 422, 388, 274, 810, 468],
      [536, 1084, 400, 1232, 1118, 582, 354, 730, 388, 342, 0, 878, 764, 730, 388, 1152, 354],
      [502, 594, 1278, 514, 400, 776, 1004, 468, 810, 536, 878, 0, 114, 308, 650, 274, 844],
      [388, 480, 1164, 628, 514, 662, 890, 354, 696, 422, 764, 114, 0, 194, 536, 388, 730],
      [354, 674, 1130, 822, 708, 628, 856, 320, 662, 388, 730, 308, 194, 0, 342, 422, 536],
      [468, 1016, 788, 1164, 1050, 514, 514, 662, 320, 274, 388, 650, 536, 342, 0, 764, 194],
      [776, 868, 1552, 560, 674, 1050, 1278, 742, 1084, 810, 1152, 274, 388, 422, 764, 0, 798],
      [662, 1210, 754, 1358, 1244, 708, 480, 856, 514, 468, 354, 844, 730, 536, 194, 798, 0]]

# Define the duration for accepting inputs (in seconds)
duration  = 60

# maintain time
order_time = []

# dict
order_list = []

# rest ready time
waiting_times = [random.randint(1, 15) for _ in range(10)]
# waiting_times = [2,4,5,7,6,8,4,2,1,7]





def create_data_model(dist_matrix,pickups,vehicles):
    #print("create data model start")
    """Stores the data for the problem."""
    data = {}
    data["distance_matrix"] = dist_matrix
    data["pickups_deliveries"] = pickups
    data["num_vehicles"] = vehicles
    data["depot"] = 0
    #print("create model end")
    #print(data)
    return data


def print_solution(data, manager, routing, solution):
    #print("print soln start")
    """Prints solution on console."""
    #print(f"Objective: {solution.ObjectiveValue()}")
    total_distance = 0
    ans = []
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        plan_output = f"Route for vehicle {vehicle_id}:\n"
        route_distance = 0
        while not routing.IsEnd(index):
            plan_output += f" {manager.IndexToNode(index)} -> "
            ans.append(manager.IndexToNode(index))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id
            )
        plan_output += f"{manager.IndexToNode(index)}\n"
        plan_output += f"Distance of the route: {route_distance}m\n"
        #print(plan_output)
        total_distance += route_distance
    #print(f"Total Distance of all routes: {total_distance}m")
    #print("print soln end")
    return ans


def routing(dist_matrix,pickups,vehicles):
    #print("Routing start")
    """Entry point of the program."""
    # Instantiate the data problem.
    data = create_data_model(dist_matrix,pickups,vehicles)
    #print(data)
    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(data["distance_matrix"]), data["num_vehicles"], data["depot"]
    )
    #print("check1")
    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)
    #print("check2")

    # Define cost of each arc.
    def distance_callback(from_index, to_index):
        """Returns the manhattan distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["distance_matrix"][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    #print("check3")
    # Add Distance constraint.
    dimension_name = "Distance"
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        3000,  # vehicle maximum travel distance
        True,  # start cumul to zero
        dimension_name,
    )
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)
    #print("check4")
    # Define Transportation Requests.
    for request in data["pickups_deliveries"]:
        pickup_index = manager.NodeToIndex(request[0])
        #print(pickup_index)
        delivery_index = manager.NodeToIndex(request[1])
        #print(delivery_index)
        routing.AddPickupAndDelivery(pickup_index, delivery_index)
        routing.solver().Add(
            routing.VehicleVar(pickup_index) == routing.VehicleVar(delivery_index)
        )
        #print("check4.1")
        routing.solver().Add(
            distance_dimension.CumulVar(pickup_index)
            <= distance_dimension.CumulVar(delivery_index)
        )
    #print("check5")
    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION
    )
    #print("check6")

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)
    #print("Routing end")
    # Print solution on console.
    if solution:
        return print_solution(data, manager, routing, solution)
    else:
        return []


def preprocessing(order_list):
    rows_cols_keep = [0,]
    for i in range(len(order_list)):
        if (order_list[i][0] not in rows_cols_keep):
          rows_cols_keep.append(order_list[i][0])
        if (order_list[i][1] not in rows_cols_keep):
          rows_cols_keep.append(order_list[i][1])
    # Delete rows except for the given values
    distance_matrix_subset = [dist_matrix[i] for i in rows_cols_keep]

    # Delete columns except for the given values
    distance_matrix_subset = [[row[j] for j in rows_cols_keep] for row in distance_matrix_subset]
    #print(distance_matrix_subset)
    sorted_rows_cols_keep = sorted(rows_cols_keep)
    map_indexes1 = {}
    map_indexes2 = {}
    for i in range(0,len(rows_cols_keep)):
        map_indexes1[rows_cols_keep[i]] = i 
        map_indexes2[i] = rows_cols_keep[i]
    #print(map_indexes2)
    for i in range(len(order_list)):
        order_list[i][0] = map_indexes1[order_list[i][0]]
        order_list[i][1] = map_indexes1[order_list[i][1]]
    #print(order_list)
    
    ans = routing(distance_matrix_subset,order_list,1)
    #print(ans)
    ans_main = []
    for i in ans:
        ans_main.append(map_indexes2[i])
    #print(ans_main)
    ans_string = ""
    for i in ans_main:
        ans_string += str(i)+" -> "
    ans_string = ans_string[:-3]
    #ans_string += str(ans_main[0])
    print(ans_string)
    print("=======================================================================================")
        

def main():
    # Get the starting time
    start_time = time.time()
    #print(order_time)
    #print(order_list)
    # Continue accepting inputs until time is up
    # num_orders = 0
    while (time.time() - start_time) < duration:
        
        #print(order_time)
        #print(order_list)
        i = 0
        while(i < len(order_time)):
            if (time.time() - order_time[i] >= 10):
                #print("cond 1")
                order_time.pop(i)
                i-=1
                input_thread = threading.Thread(target=preprocessing,args=(order_list.pop(i),))
                input_thread.start()
                input_thread.join()
                #preprocessing(order_list.pop(i))
            elif (len(order_list[i]) > 1):
                #print("cond2")
                order_time.pop(i)
                i-=1
                input_thread = threading.Thread(target=preprocessing,args=(order_list.pop(i),))
                input_thread.start()
                input_thread.join()
            i+=1
        try:
            user_input = input("Enter restaurant number and customer number separated by a space (e.g., '3 5'): ")
            restaurant_number, customer_number = map(int, user_input.split())
            # Check if the entered numbers are within the range of 1-10
            if 1 <= restaurant_number <= 10 and 1 <= customer_number <= 10:
                # num_orders += 1
                # Get the current timestamp for the order
                order_timestamp = time.time()
                # Store the pair along with the order timestamp in the list
                if (not(order_list)):
                    #print("cond3")
                    order_list.append([[restaurant_number-1, customer_number-1]])
                    order_time.append(order_timestamp)
                else:
                    for i in order_list:
                        r,c = i[0][0],i[0][1]
                        #print("Dist bet c : ",dist_matrix[c][customer_number-1])
                        #print("Dist bet r : ",dist_matrix[r][restaurant_number-1])
                        #print(waiting_times)
                        #print(order_timestamp," ",waiting_times[restaurant_number-1]," ",order_time[order_list.index(i)]," ",waiting_times[r])
                        if (order_timestamp +waiting_times[restaurant_number-1] - (order_time[order_list.index(i)]+waiting_times[r]) <= 10):
                            if (restaurant_number-1 == r and customer_number-1 == c):
                                #print("cond4")
                                order_list[order_list.index(i)].append([restaurant_number-1,customer_number-1])
                                break
                            elif (r == restaurant_number-1 and dist_matrix[c][customer_number-1] <1000 ):    # 500 = 1 km
                                #print("cond5")
                                order_list[order_list.index(i)].append([restaurant_number-1,customer_number-1])
                                break
                            elif (c == customer_number-1 and dist_matrix[r][restaurant_number-1] <1000 ):    # 500 = 1 km
                                #print("cond6")
                                order_list[order_list.index(i)].append([restaurant_number-1,customer_number-1])
                                break
                            else:
                                #print("cond7")
                                order_list.append([[restaurant_number-1, customer_number-1]])
                                order_time.append(order_timestamp)
                                break
                        else:
                            #print("cond8")
                            order_list.append([[restaurant_number-1, customer_number-1]])
                            order_time.append(order_timestamp)
                            break

            else:
                print("Please enter numbers within the range of 1-10.")
        except ValueError:
            print("Invalid input format. Please enter two integers separated by a space.")

    print("------------------------------------------------------------------------")
    #print(order_list)
    while(order_list):
        preprocessing(order_list.pop(0))   

main()








# pickups=[]
# # Check if any of the orders from the same restaurant are placed within 10 minutes of each other
# for i in range(len(restaurant_customer_pairs)):
#     for j in range(i+1, len(restaurant_customer_pairs)):
#         if restaurant_customer_pairs[i][0] == restaurant_customer_pairs[j][0]:
#             time_diff = abs(restaurant_customer_pairs[i][2] - restaurant_customer_pairs[j][2])
#             if time_diff <= timedelta(minutes=10):
#               pickups.extend([restaurant_customer_pairs[i][:2],restaurant_customer_pairs[j][:2]])
#               print(f"Orders from Restaurant {restaurant_customer_pairs[i][0]} were placed within 10 minutes of each other.")
# print(pickups)