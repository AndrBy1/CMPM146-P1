from heapq import heappop, heappush
import math

def euclidean_distance(source, destination):
    y1, x1 = source
    y2, x2 = destination
    return math.sqrt((x2-x1)**2 + (y2-y1)**2)
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
    source_point_box = None
    destination_point_box = None
    detail_points = {}

    for box in mesh['boxes']:
        x1, x2, y1, y2 = box
        if x1 <= source_point[0] <= x2 and y1 <= source_point[1] <= y2:
            source_point_box = box
            detail_points[source_point_box] = source_point
        if x1 <= destination_point[0] <= x2 and y1 <= destination_point[1] <= y2:
            destination_point_box = box
            detail_points[destination_point_box] = destination_point
        if source_point_box is not None and destination_point_box is not None:
            break

    if not (source_point_box and destination_point_box):
        print('No valid source or destination box found!')
        return [], []
    queue = []
    forward_prev = {source_point_box: None}
    forward_cost = {source_point_box: 0}
    backward_prev = {destination_point_box: None}
    backward_cost = {destination_point_box: 0}
    heappush(queue, (0, source_point_box, destination_point_box))

    while queue:
        priority, curr_box, curr_goal = heappop(queue)
        print("curr_box: ", curr_box)
        print("curr_goal: ", curr_goal)
        boxes[curr_box] = True

        if curr_box == curr_goal:
            break
        for neighbors, goal_neighbors in zip(mesh['adj'][curr_box], mesh['adj'][curr_goal]):
            
            curr_detail_point = detail_points[curr_box]
            x1, x2, y1, y2 = neighbors
            current_x, current_y = curr_detail_point

            neighbor_detail_point = (min(max(current_x, x1), x2),min(max(current_y, y1), y2))
            cost_to_neighbor = forward_cost[curr_box] + euclidean_distance(detail_points[curr_box], neighbor_detail_point)
            
            if neighbors not in forward_cost or cost_to_neighbor < forward_cost[neighbors]:
                forward_cost[neighbors] = cost_to_neighbor 
                forward_prev[neighbors] = curr_box
                euclidean = euclidean_distance(neighbor_detail_point, destination_point)
                heappush(queue, (cost_to_neighbor + euclidean, neighbors, curr_goal)) 
                detail_points[neighbors] = neighbor_detail_point #okay well we have 20 minutes
            
            curr_goal_point = detail_points[curr_goal]
            gx1, gx2, gy1, gy2 = goal_neighbors
            goal_x, goal_y = curr_goal_point

            goal_neighbor_detail_point = (min(max(goal_x, gx1), gx2),min(max(goal_y, gy1), gy2))
            cost_to_goal_neighbor = backward_cost[curr_goal] + euclidean_distance(detail_points[curr_goal], goal_neighbor_detail_point)
            
            if goal_neighbors not in backward_cost or cost_to_goal_neighbor < backward_cost[goal_neighbors]:
                backward_cost[goal_neighbors] = cost_to_goal_neighbor
                backward_prev[goal_neighbors] = curr_goal
                euclidean = euclidean_distance(goal_neighbor_detail_point, source_point)
                heappush(queue, (cost_to_goal_neighbor + euclidean, goal_neighbors, curr_box)) 
                detail_points[goal_neighbors] = goal_neighbor_detail_point

    if destination_point_box in forward_prev:
        curr_box = destination_point_box 
        while curr_box is not None: #
            path.append(detail_points[curr_box])
            curr_box = forward_prev[curr_box]
        path.reverse()
   
    return path, boxes.keys()
