import heapq

# Task 3
# Define the graph as an adjacency list with variable travel times
# Each island (node) has neighbors with the travel times to reach them
# Sample Graph:
graph = {
    'origin': [('island1', 5), ('island2', 10)],
    'island1': [('island3', 2), ('island4', 4)],
    'island2': [('island4', 7)],
    'island3': [('island5', 3)],
    'island4': [('island5', 1)],
    'island5': []
}

# Dijkstra's algorithm to find shortest path from origin other islands
def dijkstra(graph, start):
    # initialize distances dictionary with infinity
    distances = {node: float('inf') for node in graph}
    # distance to the start node is 0
    distances[start] = 0 
    # priority queue to hold nodes to explore
    priority_queue = [(0, start)]  # Priority queue to hold nodes to explore

    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)

        # skip this node if we've already found a shorter path to it
        if current_distance > distances[current_node]:
            continue

        # explore neighbors
        for neighbor, weight in graph[current_node]:
            distance = current_distance + weight

            # only update if found shorter path
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(priority_queue, (distance, neighbor))

    return distances

# Function to distribute resources using available canoes
def distribute_resource(graph, origin, islands, num_canoes):
    #find the shortest paths from the origin to all other islands using dijkstra
    shortest_paths = dijkstra(graph, origin)

    #start canoe tracking
    canoes = [{'id': i, 'available': True} for i in range(num_canoes)]
    # store each canoe's trip for tracking
    trips = []  

    for island in islands:
        if island != origin and shortest_paths[island] != float('inf'):
            #assign an available canoe
            assigned_canoe = next((canoe for canoe in canoes if canoe['available']), None)

            if assigned_canoe:
                #mark canoe as unavailable during trip
                assigned_canoe['available'] = False
                travel_time = shortest_paths[island]

                #store trip details
                trips.append((assigned_canoe['id'], origin, island, travel_time))
                print(f"Canoe traveled from {origin} to {island} taking {travel_time} units of time")

                #simulate return of the canoe, for the sake of time one simple approach is to assume they return immediately after a trip
                assigned_canoe['available'] = True  # set back to available after trip

    return trips

# Example use:
origin = 'origin'
islands = ['origin', 'island1', 'island2', 'island3', 'island4', 'island5']
num_canoes = 3  # Number of canoes available

trips = distribute_resource(graph, origin, islands, num_canoes)
print("\nTrip Log:")
for trip in trips:
    canoe_id, start, destination, time = trip
    print(f"Canoe went from {start} to {destination}, taking {time} units of time")
print("\nDistribution Finished");


# Task 2
# Using graph variable above as an adjacency list with variable travel times
# Define the capacities of routes as a separate dictionary
# Each key is a tuple (start, end) representing the route and its max capacity
capacities = {
    ('origin', 'island1'): 50,
    ('origin', 'island2'): 30,
    ('island1', 'island3'): 20,
    ('island1', 'island4'): 15,
    ('island2', 'island4'): 25,
    ('island3', 'island5'): 10,
    ('island4', 'island5'): 40
}

def spread_resource_across_islands(graph, capacities, start, initial_resource):
    # Priority queue to manage the islands by shortest travel time
    pq = [(0, start, initial_resource)]  # (travel_time, current_island, available_resource)
    # Dictionary to track resources received at each island
    resources_distributed = {island: 0 for island in graph}
    resources_distributed[start] = initial_resource
    
    # Dijkstra's algorithm with limited capacity constraint
    while pq:
        current_time, island, available_resource = heapq.heappop(pq)

        # Distribute resource to neighboring islands
        for neighbor, travel_time in graph[island]:
            new_time = current_time + travel_time
            route_capacity = capacities.get((island, neighbor), 0)
            
            # Determine the amount of resource to send on this route
            resource_to_send = min(available_resource, route_capacity)

            # Only continue if we have resources to send
            if resource_to_send > 0:
                # Add resource to the neighbor's distribution
                resources_distributed[neighbor] += resource_to_send

                # Push neighbor into the priority queue with updated travel time and resource
                heapq.heappush(pq, (new_time, neighbor, resource_to_send))

    return resources_distributed

# Example usage
initial_resource = 100  # Starting resource amount on 'Ni'ihau'
result = spread_resource_across_islands(graph, capacities, 'origin', initial_resource)
print("Resource distribution across islands:", result)
