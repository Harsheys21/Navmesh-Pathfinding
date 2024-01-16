import math
from math import inf, sqrt
from heapq import heappop, heappush

def find_path (source_point, destination_point, mesh):

    """
    Searches for a path from source_point to destination_point through the mesh

    Args:
        source_point: starting point of the pathfinder
        destination_point: the ultimate goal the pathfinder must reach
        mesh: pathway constraints the path adheres to

    Returns:

        A path (list of points) from source_point to destination_point if exists
        A list of boxes explored by the algorithm
    """
    path = []
    boxes = {}
    # breadth_first_search(path, boxes, mesh, source_point, destination_point)
    # boxes, path = dijkstras_shortest_path(source_point, destination_point, mesh["boxes"], mesh["adj"])
    boxes, path = astar_shortest_path(source_point, destination_point, mesh["boxes"], mesh["adj"])
    return path, boxes.keys()


def breadth_first_search(path, boxes, mesh, source_point, destination_point):
    detail_points = {}
    frontier = []
    goal = None
    path_found = False
    # find source and destination boxes
    for box in mesh["boxes"]:
        if box[0] <= source_point[0] and box[1] >= source_point[0] and box[2] <= source_point[1] and box[3] >= source_point[1]:
            boxes[box] = None
            detail_points[box] = source_point
            frontier.append(box)

        if box[0] <= destination_point[0] and box[1] >= destination_point[0] and box[2] <= destination_point[1] and box[3] >= destination_point[1]:
            goal = box
            detail_points[box] = destination_point
    
    neighbors = mesh["adj"]

    # implement breadth first search
    while frontier:
        current = frontier.pop(0)

        if current == goal:
            path_found = True
            break

        for next_node in neighbors[current]:
            if next_node not in boxes:
                frontier.append(next_node)
                current_point = detail_points[current]
                boxes[next_node] = current
                bx = [max(current[0], next_node[0]), min(current[1], next_node[1])]
                by = [max(current[2], next_node[2]), min(current[3], next_node[3])]
                xcord = 0
                ycord = 0
                if current_point[0] < bx[0]:
                    xcord = min(bx)
                elif current_point[0] > bx [1]:
                    xcord = max(bx)
                else:
                    xcord = current_point[0]

                if current_point[1] < by[0]:
                    ycord = min(by)
                elif current_point[1] > by [1]:
                    ycord = max(by)
                else:
                    ycord = current_point[1]

                detail_points[next_node] = (xcord, ycord)

    if path_found == False:
        print("No path!")
    else:
        print("Path found!")
        came_from = boxes[goal]
        path.append(destination_point)
        while came_from is not None:
            path.append(detail_points[came_from])
            came_from = boxes[came_from]

    path.append(source_point)

def euclidean_distance(source_point, destination_point):
    x1, y1 = source_point
    x2, y2 = destination_point
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)    


def dijkstras_shortest_path(initial_position, destination, graph, adj):
    detail_points = {} 
    queue = []
    paths = {}                              # maps cells to previous cells on path
    pathcosts = {}
    goal = None
    start = None

    # find source and destination boxes
    for box in graph:
        if box[0] <= initial_position[0] and box[1] >= initial_position[0] and box[2] <= initial_position[1] and box[3] >= initial_position[1]:
            paths[box] = None
            pathcosts[box] = 0
            start = box
            detail_points[box] = initial_position
            heappush(queue, (0, box))  # maintain a priority queue of cells

        if box[0] <= destination[0] and box[1] >= destination[0] and box[2] <= destination[1] and box[3] >= destination[1]:
            goal = box
            detail_points[box] = destination

    # going through the queue
    while queue:
        priority, cell = heappop(queue)      # pop the priority cell
        # is goal is reached
        if cell == goal:
            path = []
            path.append(destination)
            path.append(detail_points[goal])
            came_from = paths[goal]
            while came_from is not None:
                path.append(detail_points[came_from])
                came_from = paths[came_from]
            print("Path found!")
            return paths, path
        
        # investigate child nodes
        for child in adj[cell]:
            # calculate cost along this path to child
            
            cost_to_child = priority + transition_cost(cell, child)

            if child not in pathcosts or cost_to_child < pathcosts[child]:
                pathcosts[child] = cost_to_child            # update the cost
                heappush(queue, (cost_to_child, child))     # put the child on the priority queue
                paths[child] = cell                         # set the backpointer
                
                
                # setting detail points
                current_point = detail_points[cell]

                detail_points[child] = detail_p(current_point,cell, child)

    print("No path!")       
    return paths, []

def astar_shortest_path(initial_position, destination, graph, adj):
    detail_points = {} 
    queue = []
    paths = {}                              # maps cells to previous cells on path
    pathcosts = {}
    goal = None
    start = None

    # find source and destination boxes
    for box in graph:
        if box[0] <= initial_position[0] and box[1] >= initial_position[0] and box[2] <= initial_position[1] and box[3] >= initial_position[1]:
            paths[box] = None
            pathcosts[box] = 0
            start = box
            detail_points[box] = initial_position
            heappush(queue, (0, box))  # maintain a priority queue of cells

        if box[0] <= destination[0] and box[1] >= destination[0] and box[2] <= destination[1] and box[3] >= destination[1]:
            goal = box
            detail_points[box] = destination

    # going through the queue
    while queue:
        priority, cell = heappop(queue)      # pop the priority cell
        # is goal is reached
        if cell == goal:
            path = []
            path.append(destination)
            path.append(detail_points[goal])
            came_from = paths[goal]
            while came_from is not None:
                path.append(detail_points[came_from])
                came_from = paths[came_from]
            print("Path found!")
            return paths, path
        
        # investigate child nodes
        for child in adj[cell]:
            # calculate cost along this path to child
            
            cost_to_child = pathcosts[cell]

            if child not in pathcosts or cost_to_child < pathcosts[child]:
                # setting detail points
                current_point = detail_points[cell]
                detail_points[child] = detail_p(current_point,cell, child)

                pathcosts[child] = cost_to_child            # update the cost
                priority = cost_to_child + euclidean_distance(detail_points[child], destination)
                heappush(queue, (priority, child))     # put the child on the priority queue
                paths[child] = cell                         # set the backpointer

    print("No path!")       
    return paths, []


def transition_cost(cell, cell2):
    distance = sqrt((cell2[0] - cell[0])**2 + (cell2[2] - cell[2])**2)
    return distance 

def heuristic(a, b):
    # Manhattan distance on a square grid
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def detail_p(current_point, current, next_node):
    bx = [max(current[0], next_node[0]), min(current[1], next_node[1])]
    by = [max(current[2], next_node[2]), min(current[3], next_node[3])]

    xcord = 0
    ycord = 0

    if current_point[0] < bx[0]:
        xcord = min(bx)
    elif current_point[0] > bx [1]:
        xcord = max(bx)
    else:
        xcord = current_point[0]

    if current_point[1] < by[0]:
        ycord = min(by)
    elif current_point[1] > by [1]:
        ycord = max(by)
    else:
        ycord = current_point[1]

    return xcord, ycord