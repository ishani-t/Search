# search.py
# ---------------
# Licensing Information:  You are free to use or extend this projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to the University of Illinois at Urbana-Champaign
#
# Created by Kelvin Ma (kelvinm2@illinois.edu) on 01/24/2021

"""
This is the main entry point for MP3. You should only modify code
within this file -- the unrevised staff files will be used for all other
files and classes when code is run, so be careful to not modify anything else.
"""
# Search should return the path.
# The path should be a list of tuples in the form (row, col) that correspond
# to the positions of the path taken by your search algorithm.
# maze is a Maze object based on the maze from the file specified by input filename
# searchMethod is the search method specified by --method flag (bfs,dfs,astar,astar_multi,fast)


# Feel free to use the code below as you wish
# Initialize it with a list/tuple of objectives
# Call compute_mst_weight to get the weight of the MST with those objectives
# TODO: hint, you probably want to cache the MST value for sets of objectives you've already computed...
# Note that if you want to test one of your search methods, please make sure to return a blank list
#  for the other search methods otherwise the grader will not crash.
class MST:
    def __init__(self, objectives):
        self.elements = {key: None for key in objectives}

        # TODO: implement some distance between two objectives
        # ... either compute the shortest path between them, or just use the manhattan distance between the objectives
        self.distances   = {
                (i, j): abs(j[0] - i[0]) + abs(j[1] - i[1])
                for i, j in self.cross(objectives)
            }

    # Prim's algorithm adds edges to the MST in sorted order as long as they don't create a cycle
    def compute_mst_weight(self):
        weight      = 0
        for distance, i, j in sorted((self.distances[(i, j)], i, j) for (i, j) in self.distances):
            if self.unify(i, j):
                weight += distance
        return weight

    # helper checks the root of a node, in the process flatten the path to the root
    def resolve(self, key):
        path = []
        root = key
        while self.elements[root] is not None:
            path.append(root)
            root = self.elements[root]
        for key in path:
            self.elements[key] = root
        return root

    # helper checks if the two elements have the same root they are part of the same tree
    # otherwise set the root of one to the other, connecting the trees
    def unify(self, a, b):
        ra = self.resolve(a)
        rb = self.resolve(b)
        if ra == rb:
            return False
        else:
            self.elements[rb] = ra
            return True

    # helper that gets all pairs i,j for a list of keys
    def cross(self, keys):
        return (x for y in (((i, j) for j in keys if i < j) for i in keys) for x in y)

import collections

def bfs(maze):
    """
    Runs BFS for part 1 of the assignment.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    root = maze.start # first vertex of path should be start
    target = maze.waypoints[0] # last vertex should be waypoints[0]
    
    # base case - you're already at the waypoint you're searching for
    if(root == target):
        return [root]

    queue = collections.deque([root]) # hint: the deque type, available in the collections module, may be useful
    paths = {root: None} # dictionary will store a the paths we take so we can backtrack from target to root - start root has no predecessors so value is None
    seen = [] # all nodes we've seen

    while queue:
        current = queue.popleft()

        if current not in seen:
            seen.append(current)
            
            # if we reach the first waypoint, stop looking
            if current == target:
                break

            else: # otherwise, search neighbors
                i, j = current
                neighbors = maze.neighbors(i, j) # method declaration: neighbors(self, i, j)
                for neighbor in neighbors:
                    if neighbor not in seen:
                        paths[neighbor] = current # key: neighbor, value: previous node - needed to go back to root
                        queue.append(neighbor)
    
    # After finding the target, have to backtrack to root - use paths dict
    backwards = []
    while True:
        backwards.append(current)
        current = paths[current]

        # if we find start node, exit while loop --> value at start is None. 
        if current == None:
            break
    backwards.reverse()

    return backwards


import heapq
def astar_single(maze):
    """
    Runs A star for part 2 of the assignment.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    root = maze.start # first vertex of path should be start
    target = maze.waypoints[0] # last vertex should be waypoints[0]

    # base case - you're already at the waypoint you're searching for
    if(root == target):
        return [root]
    
    paths = {root: None}
    seen = {root: 0} # seen will have key of visited nodes and value of distance to that node
    

    queue = []
    heapq.heapify(queue)
    current = root
    while True:
        if(current == target):        # exit once the target node has been found
            break

        else: 
            i, j = current
            neighbors = maze.neighbors(i, j)
            
            for neighbor in neighbors:
                if neighbor not in seen:
                    paths[neighbor] = current
                    seen[neighbor] = seen[current] + 1

                    # update using manhattan distance, heuristic instead of just doing BFS
                    man_distance = abs(neighbor[0] - target[0]) + abs(neighbor[1] - target[1])
                    cost = man_distance + seen[current] # seen[current] for heuristic
                    # need to put distance in tuple first for sorting
                    heapq.heappush(queue, (cost, neighbor))

            
            current_entry = heapq.heappop(queue)
            current = current_entry[1]

    # backtrack from target to root, the same as before
    backwards = []
    while True:
        backwards.append(current)
        current = paths[current]

        if (current == None):
            break
    backwards.reverse()
    return backwards


def astar_multiple(maze):
    """
    Runs A star for part 3 of the assignment in the case where there are
    multiple objectives.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    root = maze.start
    targets = maze.waypoints
    start_tuple = (root, targets)  # need to store where we are and how many waypoints are left 
    
    paths = {start_tuple: None}
    seen = {start_tuple: 0} # g
    queue = [ (0, start_tuple) ] # queue: (cost, (current node, remaining targets))
    
    costs = {start_tuple: 0}     # f: store costs to decide which waypoint to go to next

    while queue:
        node = heapq.heappop(queue)
        current = node[1] # current gives a queue entry: (cost, (current node, remaining targets))

        # EXIT CONDITION: if there are no remaining targets in S, we've reached final waypoint
        coords, remaining_targets = current
        if len(remaining_targets) == 0:
            break

        i, j = coords
        neighbors = maze.neighbors(i, j)
        for n in neighbors:
            
            # if the neighbor is one of the waypoints, we need to remove it from the remaining targets list
            new_remaining_targets = []
            for target in remaining_targets:
                if target == n:
                    continue
                new_remaining_targets.append(target)
            neighbor = (n, tuple(new_remaining_targets))
                        
            # find distance to closest remaining waypoint
            closest_distance = 0
            if(len(new_remaining_targets)) > 0:
                closest_distance = 9999999999999999999
                for waypoint in new_remaining_targets:
                    man_dist = abs(n[0] - waypoint[0]) + abs(n[1] - waypoint[1])
                    if man_dist < closest_distance:
                        closest_distance = man_dist
                
            # find mst_weight for all reamining waypoints
            mst_weight = MST(new_remaining_targets).compute_mst_weight()

            # heuristic: distance to nearest waypoint + MST length for waypoints in s, f = h + g
            cost = closest_distance + mst_weight + seen[current] + 1

            # if there is a less costly way to get here, don't go
            if neighbor in costs and cost > costs.get(neighbor):
                continue
            
            # update all our mappings
            if neighbor not in costs:       
                heapq.heappush(queue, (cost, neighbor))
            paths[neighbor] = current
            seen[neighbor] = seen[current] + 1
            costs[neighbor] = cost
    
    # backtrack from target to root, the same as before
    backwards = []
    while True:
        backwards.append(current[0])
        current = paths[current]
        
        if (current == None):
            break
    backwards.reverse()
    return backwards


def fast(maze):
    """
    Runs suboptimal search algorithm for extra credit/part 4.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    return []


