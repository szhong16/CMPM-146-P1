from heapq import heappop, heappush
from math import inf, sqrt

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

    # extract args
    allBoxes = mesh['boxes']
    # print(allBoxes)
    adj = mesh['adj']

    # find source and destination boxes
    src = find_box(allBoxes, source_point)
    print("src:")
    print(src)
    dst = find_box(allBoxes, destination_point)
    print("dst:")
    print(dst)

    # check if arguments exist
    if dst == None or src == None:
        print("path not found")
        return path, boxes

    print("source_point, destination_point:")
    print(source_point, destination_point)
  
    path.append(source_point)

    # for graph search algorithm
    queue = [] # queue of boxes (priority, curr_box, goal_box)
    heappush(queue, (0, src, dst))
    heappush(queue, (0, dst, src))
    # forward direction tables src -> dst
    detail_points_forward = {src : source_point}
    came_from_forward = {src : None}
    cost_so_far_forward = {src : 0}
    # backward direction tables src <- dst
    detail_points_backward = {dst : destination_point}
    came_from_backward = {dst : None}
    cost_so_far_backward = {dst : 0}
 
    # A* searching algorithm
    while queue:
        current_cost, current_box, current_goal= heappop(queue)
        print("current_priority: ", current_cost)
        # print("current_goal: ", current_goal)
        if current_box == current_goal or came_from_forward.get(current_box) is not None or came_from_backward.get(current_box) is not None:
            print("current_box: ", current_box)
            print("current_goal: ", current_goal)
            
            if current_box == current_goal:
                print("current_box == current_goal")
                if current_goal == dst:
                    box_path = path_to_box(current_box, came_from_forward)
                    print("found destination: current_box == dst")
                    path = box_path_to_detail_point_path(box_path, detail_points_forward)
                    break
                elif current_goal == src:
                    box_path = path_to_box(current_box, came_from_backward)
                    print("found destination: current_box == src")
                    path = box_path_to_detail_point_path(box_path, detail_points_backward)
                    break

            # if two directions intersects
            else:
                if (came_from_forward.get(current_box) is not None and current_goal != dst) or (came_from_backward.get(current_box) is not None and current_goal != src):
                    box_path_1 = path_to_box(current_box, came_from_forward)
                    print("came_from_forward box_path_1: ", box_path_1)
                    box_path_2 = path_to_box(current_box, came_from_backward)
                    print("came_from_backward box_path_2: ", box_path_2)
                    box_path_2 = list(reversed(box_path_2))
                    path = box_path_to_detail_point_path(box_path_1, detail_points_forward) + box_path_to_detail_point_path(box_path_2, detail_points_backward)
                    # box_path = box_path_1 + box_path_2
                    print("found destination: current_box != current_goal")
                    break
        
        for next in adj[current_box]:
            # chech which is the source point
            if current_goal == dst:
                # print("current_goal == dst")
                detail_point_and_cost = find_detail_points(current_box, next, detail_points_forward, source_point)
                new_cost = current_cost + detail_point_and_cost[1]  
                if next not in cost_so_far_forward or new_cost < cost_so_far_forward[next]:
                    boxes[next] = current_box
                    if next not in detail_points_forward.keys():
                        detail_points_forward[next] = detail_point_and_cost[0]
                    cost_so_far_forward[next] = new_cost
                    priority = new_cost + euclidean_distance(current_goal, detail_point_and_cost[0])
                    print("in current_goal == dst, priority = ", priority)
                    heappush(queue, (priority, next, current_goal))
                    came_from_forward[next] = current_box
            elif current_goal == src:  
                boxes[next] = current_box
                # print("current_goal == src")
                detail_point_and_cost = find_detail_points(current_box, next, detail_points_backward, destination_point)
                new_cost = current_cost + detail_point_and_cost[1]  
                if next not in cost_so_far_backward or new_cost < cost_so_far_backward[next]:
                    if next not in detail_points_backward.keys():
                        detail_points_backward[next] = detail_point_and_cost[0]
                    cost_so_far_backward[next] = new_cost
                    priority = new_cost + euclidean_distance(current_goal, detail_point_and_cost[0])
                    print("in current_goal == src, priority = ", priority)
                    heappush(queue, (priority, next, current_goal))
                    came_from_backward[next] = current_box
    
    # path.append(source_point)
    print("--------------------")

    # box_path = list(dict.fromkeys(box_path))
    # print("box_path = ", box_path)
    # for next in box_path:
    #     print("in 'for next in box_path', box = ", next)
    #     path.append(detail_points[next])
    path.append(destination_point)
    print("path:")
    print(path)
    return path, boxes

# given a box_path, find the exact landing points to a box from detail_points list
def box_path_to_detail_point_path(box_path, detail_points):
    path = []
    for next in box_path:
        path.append(detail_points[next])
    return path

# given a coordinates of a point, find the box that the point is located at
def find_box(box_list, point):
    for box in box_list:
        if (point[0] > box[0] and point[0] < box[1] and 
            point[1] > box[2] and point[1] < box[3]) :
            return box

# find the final path 
def path_to_box(box, paths):
    if box == None:
        return []
    # print("in path_to_box, box = ", box)
    return path_to_box(paths[box], paths) + [box]

# find the detail points to land on boxes. Returns (point(x,y), cost from box_1 to box_2)
def find_detail_points(box_1, box_2, detail_points, source_point):
    # ranges of next point
    x_range = [max(box_1[0], box_2[0]), min(box_1[1], box_2[1])]
    y_range = [max(box_1[2], box_2[2]), min(box_1[3], box_2[3])]

    # find bxmin, bxmax
    if x_range[0] <= x_range[1]:
        bxmin = x_range[0]
        bxmax = x_range[1]
    else:
        bxmin = x_range[1]
        bxmax = x_range[0]

    # find bymin, bymax
    if y_range[0] <= y_range[1]:
        bymin = y_range[0]
        bymax = y_range[1]
    else:
        bymin = y_range[1]
        bymax = y_range[0]

    detail_point = (max(bxmin, min(bxmax, source_point[0])), max(bymin, min(bymax,source_point[1])))
    cost = euclidean_distance(detail_point, detail_points[box_1])
    return (detail_point, cost)

def euclidean_distance(point_1, point_2):
    return sqrt((point_1[0] - point_2[0])**2 + (point_1[1] - point_2[1])**2) * 0.5

def transition_cost(level, cell, cell2):
    distance = sqrt((cell2[0] - cell[0])**2 + (cell2[1] - cell[1])**2)
    average_cost = (level['spaces'][cell] + level['spaces'][cell2])/2
    return distance * average_cost