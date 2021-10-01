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

    if dst == None or src == None:
        print("path not found")
        return path, boxes

    print("source_point, destination_point:")
    print(source_point, destination_point)

    pathFromSrc = []
    pathFromDst = []

    path.append(source_point)
    pathFromSrc.append(source_point)
    pathFromDst.append(destination_point)

    # for graph search algorithm
    detail_points = {}
    detail_points[src] = {src : source_point} # the point we will go next
    detail_points[dst] = {dst : destination_point}
    queue = [] # queue of boxes
    came_from_src = {src : None} # remember which point we came from for check the cost
    came_from_dst = {dst : None}
    cost_so_far_src = {src: 0}
    cost_so_far_dst = {dst: 0}
    heappush(queue, (0, src, 'destination'))
    heappush(queue, (0, dst, 'source'))

    # A* searching algorithm
    # append() add on end, insert() add on front
    while queue:
        current_cost, current_box, current_goal = heappop(queue)

        if current_goal == 'destination' and current_box in came_from_dst:
            print('meet each other')
            print('meet at: ', current_box)
            box_path_dst2src = path_to_box(current_box, came_from_dst)
            box_path_src2dst = path_to_box(current_box, came_from_src)

            # now we want store box_path_dst2src in the beginning of the path, vice versa
            if box_path_dst2src is not None:
                path.insert(detail_points[box_path_dst2src])
            if box_path_src2dst is not None:
                path.append(detail_points[box_path_src2dst])
            print("found destination")
            return path, boxes
            # break

        if current_goal == 'source' and current_box in came_from_src:
            print('meet each other')
            print('meet at: ', current_box)
            box_path_dst2src = path_to_box(current_box, came_from_dst)
            box_path_src2dst = path_to_box(current_box, came_from_src)

            # now we want store box_path_dst2src in the beginning of the path, vice versa
            if box_path_dst2src is not None:
                path.insert(detail_points[box_path_dst2src])
            if box_path_src2dst is not None:
                path.append(detail_points[box_path_src2dst])
            print("found destination")
            return path, boxes
            # break

        for next in adj[current_box]:
            boxes[next] = current_box
            # return a pair, detail_point_and_cost[1] = cost, detail_point_and_cost[0] = point we go next
            if current_goal == 'destination':
                detail_point_and_cost = find_detail_points(current_box, next, detail_points, source_point, destination_point)
                new_cost = current_cost + detail_point_and_cost[1]
                if next not in cost_so_far_src or new_cost < cost_so_far_src[next]:
                    detail_points[next] = detail_point_and_cost[0]
                    cost_so_far_src[next] = new_cost
                    priority = new_cost + euclidean_distance(destination_point, detail_point_and_cost[0]) # f(n) = g(n) + h(n)
                    heappush(queue, (priority, next, 'destination'))
                    came_from_src[next] = current_box
            if current_goal == "source":
                detail_point_and_cost = find_detail_points(current_box, next, detail_points, destination_point, source_point)
                new_cost = current_cost + detail_point_and_cost[1]
                if next not in cost_so_far_dst or new_cost < cost_so_far_dst[next]:
                    detail_points[next] = detail_point_and_cost[0]
                    cost_so_far_dst[next] = new_cost
                    priority = new_cost + euclidean_distance(destination_point, detail_point_and_cost[0]) # f(n) = g(n) + h(n)
                    heappush(queue, (priority, next, 'source'))
                    came_from_dst[next] = current_box

    path.append(source_point)
    print("--------------------")

    path.append(destination_point)
    print("path:")
    print(path)
    return path, boxes

# given a coordinates of a point, find the box that the point is located at
def find_box(box_list, point):
    for box in box_list:
        if (point[0] > box[0] and point[0] < box[1] and
            point[1] > box[2] and point[1] < box[3]) :
            return box

# find the final path
def path_to_box(box, paths):
    if box == None: # if box is not exists
        return []
    print("in path_to_box, box = ", box)
    return path_to_box(paths[box], paths) + [box]

# find the detail points to land on boxes. Returns (point(x,y), cost from box_1 to box_2)
def find_detail_points(box_1, box_2, detail_points, source_point, destination_point):
    # ranges of next point
    x_range = [max(box_1[0], box_2[0]), min(box_1[1], box_2[1])]
    y_range = [max(box_1[2], box_2[2]), min(box_1[3], box_2[3])]

    # got this formula from professor max(Bxmin, min(Bxmax, Sx))
    # Bxmin = min x on the boundary
    # Bxmax = max x on the boundary
    # this formula is also works for y
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
