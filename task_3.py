import heapq

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
