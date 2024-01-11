#!/usr/bin/env python
# coding: utf-8
from typing import List, Tuple, Dict, Callable, Any
from copy import deepcopy
import math

# define the full world
full_world = [
['🌾', '🌾', '🌾', '🌾', '🌾', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾'],
['🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌾', '🌾', '🗻', '🗻', '🗻', '🗻', '🗻', '🗻', '🗻', '🌾', '🌾'],
['🌾', '🌾', '🌾', '🌾', '🗻', '🗻', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🗻', '🗻', '🗻', '🪨', '🪨', '🪨', '🗻', '🗻', '🪨', '🪨'],
['🌾', '🌾', '🌾', '🌾', '🪨', '🗻', '🗻', '🗻', '🌲', '🌲', '🌲', '🌲', '🐊', '🐊', '🌲', '🌲', '🌲', '🌲', '🌲', '🌾', '🌾', '🪨', '🪨', '🗻', '🗻', '🪨', '🌾'],
['🌾', '🌾', '🌾', '🪨', '🪨', '🗻', '🗻', '🌲', '🌲', '🌾', '🌾', '🐊', '🐊', '🐊', '🐊', '🌲', '🌲', '🌲', '🌾', '🌾', '🌾', '🪨', '🗻', '🗻', '🗻', '🪨', '🌾'],
['🌾', '🪨', '🪨', '🪨', '🗻', '🗻', '🪨', '🪨', '🌾', '🌾', '🌾', '🌾', '🐊', '🐊', '🐊', '🐊', '🐊', '🌾', '🌾', '🌾', '🌾', '🌾', '🪨', '🗻', '🪨', '🌾', '🌾'],
['🌾', '🪨', '🪨', '🗻', '🗻', '🪨', '🪨', '🌾', '🌾', '🌾', '🌾', '🪨', '🗻', '🗻', '🗻', '🐊', '🐊', '🐊', '🌾', '🌾', '🌾', '🌾', '🌾', '🪨', '🌾', '🌾', '🌾'],
['🌾', '🌾', '🪨', '🪨', '🪨', '🪨', '🪨', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🪨', '🗻', '🗻', '🗻', '🐊', '🐊', '🐊', '🌾', '🌾', '🪨', '🪨', '🪨', '🌾', '🌾'],
['🌾', '🌾', '🌾', '🪨', '🪨', '🪨', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🪨', '🪨', '🗻', '🗻', '🌾', '🐊', '🐊', '🌾', '🌾', '🪨', '🪨', '🪨', '🌾', '🌾', '🌾'],
['🌾', '🌾', '🌾', '🐊', '🐊', '🐊', '🌾', '🌾', '🪨', '🪨', '🪨', '🗻', '🗻', '🗻', '🗻', '🌾', '🌾', '🌾', '🐊', '🌾', '🪨', '🪨', '🪨', '🌾', '🌾', '🌾', '🌾'],
['🌾', '🌾', '🐊', '🐊', '🐊', '🐊', '🐊', '🌾', '🪨', '🪨', '🗻', '🗻', '🗻', '🪨', '🌾', '🌾', '🌾', '🌾', '🌾', '🪨', '🗻', '🗻', '🗻', '🪨', '🌾', '🌾', '🌾'],
['🌾', '🐊', '🐊', '🐊', '🐊', '🐊', '🌾', '🌾', '🪨', '🗻', '🗻', '🪨', '🌾', '🌾', '🌾', '🌾', '🐊', '🐊', '🌾', '🌾', '🪨', '🗻', '🗻', '🪨', '🌾', '🌾', '🌾'],
['🐊', '🐊', '🐊', '🐊', '🐊', '🌾', '🌾', '🪨', '🪨', '🗻', '🗻', '🪨', '🌾', '🐊', '🐊', '🐊', '🐊', '🌾', '🌾', '🌾', '🪨', '🗻', '🪨', '🌾', '🌾', '🌾', '🌾'],
['🌾', '🐊', '🐊', '🐊', '🐊', '🌾', '🌾', '🪨', '🌲', '🌲', '🪨', '🌾', '🌾', '🌾', '🌾', '🐊', '🐊', '🐊', '🐊', '🌾', '🌾', '🪨', '🌾', '🌾', '🌾', '🌾', '🌾'],
['🌾', '🌾', '🌾', '🌾', '🗻', '🌾', '🌾', '🌲', '🌲', '🌲', '🌲', '🪨', '🪨', '🪨', '🪨', '🌾', '🐊', '🐊', '🐊', '🌾', '🌾', '🪨', '🗻', '🪨', '🌾', '🌾', '🌾'],
['🌾', '🌾', '🌾', '🗻', '🗻', '🗻', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🗻', '🗻', '🗻', '🪨', '🪨', '🌾', '🐊', '🌾', '🪨', '🗻', '🗻', '🪨', '🌾', '🌾', '🌾'],
['🌾', '🌾', '🗻', '🗻', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🗻', '🗻', '🗻', '🌾', '🌾', '🗻', '🗻', '🗻', '🌾', '🌾', '🌾', '🌾', '🌾'],
['🌾', '🌾', '🌾', '🗻', '🗻', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🗻', '🗻', '🗻', '🗻', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾'],
['🌾', '🌾', '🌾', '🗻', '🗻', '🗻', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌾', '🌾', '🌾', '🪨', '🪨', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾'],
['🌾', '🌾', '🌾', '🌾', '🗻', '🗻', '🗻', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🐊', '🐊', '🐊', '🐊'],
['🌾', '🌾', '🪨', '🪨', '🪨', '🪨', '🗻', '🗻', '🌲', '🌲', '🌲', '🌲', '🌲', '🌾', '🗻', '🌾', '🌾', '🌾', '🌾', '🌾', '🐊', '🐊', '🐊', '🐊', '🐊', '🐊', '🐊'],
['🌾', '🌾', '🌾', '🌾', '🪨', '🪨', '🪨', '🗻', '🗻', '🗻', '🌲', '🌲', '🗻', '🗻', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🐊', '🐊', '🐊', '🐊', '🐊', '🐊', '🐊'],
['🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🪨', '🪨', '🪨', '🗻', '🗻', '🗻', '🗻', '🌾', '🌾', '🌾', '🌾', '🪨', '🪨', '🌾', '🌾', '🐊', '🐊', '🐊', '🐊', '🐊', '🐊'],
['🌾', '🪨', '🪨', '🌾', '🌾', '🪨', '🪨', '🪨', '🪨', '🪨', '🌾', '🌾', '🌾', '🌾', '🌾', '🪨', '🪨', '🗻', '🗻', '🪨', '🪨', '🌾', '🐊', '🐊', '🐊', '🐊', '🐊'],
['🪨', '🗻', '🪨', '🪨', '🪨', '🪨', '🌾', '🌾', '🌾', '🌾', '🌾', '🗻', '🗻', '🗻', '🪨', '🪨', '🗻', '🗻', '🌾', '🗻', '🗻', '🪨', '🪨', '🐊', '🐊', '🐊', '🐊'],
['🪨', '🗻', '🗻', '🗻', '🪨', '🌾', '🌾', '🌾', '🌾', '🌾', '🪨', '🪨', '🗻', '🗻', '🗻', '🗻', '🪨', '🪨', '🪨', '🪨', '🗻', '🗻', '🗻', '🐊', '🐊', '🐊', '🐊'],
['🪨', '🪨', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🪨', '🪨', '🪨', '🪨', '🪨', '🌾', '🌾', '🌾', '🌾', '🪨', '🪨', '🪨', '🌾', '🌾', '🌾']
]

# define the small world
small_world = [
    ['🌾', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲'],
    ['🌾', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲'],
    ['🌾', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲'],
    ['🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾'],
    ['🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌾'],
    ['🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌾'],
    ['🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌾']
]

# define moves
MOVES = [(0,-1), (1,0), (0,1), (-1,0)]
# define costs
COSTS = { '🌾': 1, '🌲': 3, '🪨': 5, '🐊': 7}

"""
The add_to_frontier algorithm treats the frontier as a priority queue and adds the value to the frontier in the correct sorted location. The frontier is sorted in ascending order by cost. The function updates the frontier in-place and returns None.

Note: the implementation of this algorithm using a for-loop could be inefficient if the frontier is long.
"""
def add_to_frontier(value: Dict[str, Any], frontier: List[Dict[str, Any]]) -> None:
    for index, frontier_value in enumerate(frontier): 
        if frontier_value["cost"] > value["cost"]:
            frontier.insert(index, value)
            return 
    # append if no insert location was found
    frontier.append(value)

"""
The get_successors function takes the current_position, the world, and the available moves and the direction of the move in the form [[(2, 2), (0, 1)],..], where (0, 1) represents right. The function uses these values to find valid moves on the board from the current position.
"""
def get_successors(current_position: Tuple[int, int], world: List[List[str]], moves: List[Tuple[int, int]]) -> List[List[Tuple[int, int]]]:
    valid_successors = []
    for move in moves:
        row, col = current_position[0] + move[0], current_position[1] + move[1]
        # Check if the new position is within range
        if col >= len(world[0]) or col < 0:
            continue
        elif row >= len(world) or row < 0:
            continue
        else:
            current_terrian = world[row][col]
            if current_terrian == '🗻':
                continue
            else:
                valid_successors.append((row, col))
    return valid_successors

"""
The get_path_direction function is a helper of the pretty_print_path function. This helper determines which symbol to print on the map based on the direction of the path.
"""
def get_path_offsets(path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
    offsets = []
    for index, position in enumerate(path):
        direction = (0, 0)
        if index == len(path)-1:
            offsets.append(direction)
            continue
        row, col = position[0], position[1]
        next_row, next_col = path[index+1][0], path[index+1][1]
        if next_row > row:
            direction = (1, 0)
        elif next_row < row:
            direction = (-1, 0)
        elif next_col > col:
            direction = (0, 1)
        elif next_col < col: 
            direction = (0, -1)
        offsets.append(direction)
    return offsets

"""
The seen_nodes function takes a list of dictionaries and determines if the position coordinates of the value parameter are already in the list.
"""
def seen_node(value: Dict[str, Any], seen_list: List[Dict[str, Any]]) -> bool:
    for seen_value in seen_list:
        if value["position"] == seen_value["position"]:
            return True
    return False

"""
The heuristic function used the Manhattan Distance to estimate the cost from the current state to the goal. The Manhattan Distance is useful for search patterns that search adjacent nodes on a graph.
"""
def heuristic(current_position: Tuple[int, int], goal: Tuple[int, int]) -> int:
    return abs(goal[0] - current_position[0]) + abs(goal[1] - current_position[1])

""" 
The get_best_path function take a list of all paths that the [a_star_search](a_star_search) function found to the goal and returns the path with the lowest cost.
"""
def get_best_path(goal_paths: List[Dict[str, Any]]) -> List[Tuple[int, int]]:
    best_path = []
    best_cost = math.inf
    for path in goal_paths:
        if path["g"] < best_cost:
            best_path = path["path"]
            best_cost = path["g"]
    return best_path  

"""
The a_star_search function applies the A\* search algorithm to find a path to the goal position from the starting position. The A\* search algorithm is an informed search algorithm that uses a heuristic function to predict how far each successor position is from the goal position. The algorithm calls the [get_successors](get_successors) function to retrieve all valid moves from the current position using the problem constraints. For every successor returned, the A\* search algorithm calculates the cost of that position using the forumala f(n) = g(n) + h(n), where g(n) is the cost of the child position and h(n) is the prediction from the [heuristic](heuristic) function. Next, the algorithm adds every successor position with its predicted cost of f(n) to the frontier. The A\* search function leverages a priority queue for the search frontier so that the algorithm always explores the position with the lowest f(n) first. The algorithm uses a dictionary of the form {"position": Tuple[int, int], "cost": int, "path": List[Tuple[int, int], "g": int}, where g is the cost from the initial node to the current node, to represent a frontier value. The path metadata helps recover the cheapest path found for the [pretty_print_path](#pretty_print_path) function.
"""
def a_star_search(world: List[List[str]], start: Tuple[int, int], goal: Tuple[int, int], costs: Dict[str, int], moves: List[Tuple[int, int]], heuristic: Callable) -> List[Tuple[int, int]]:
    frontier, explored, goal_paths = [], [], []
    # reverse input to be of the form (row, col) and add starting value
    start, goal = (start[1], start[0]), (goal[1], goal[0])
    frontier.append({"position": (start[0], start[1]), "cost": heuristic(start, goal), "path": [start], "g": 0}) 
    while len(frontier) > 0:
        current_position = frontier.pop(0)
        position_coordinates = current_position["position"]
        successors = get_successors(position_coordinates, world, moves)
        if position_coordinates == goal:
            goal_paths.append({"path": current_position["path"], "g": current_position["g"]})
            
        for successor in successors:
            total_cost = costs[world[successor[0]][successor[1]]] + current_position["g"]
            estimated_cost = total_cost + heuristic(successor, goal)
            frontier_value = {
                "position": successor, 
                "cost": estimated_cost, 
                "path": current_position["path"] + [successor],
                "g": total_cost}
            if not seen_node(frontier_value, explored) and not seen_node(frontier_value, frontier):
                add_to_frontier(frontier_value, frontier)
        explored.append(current_position)
    best_path = get_best_path(goal_paths)
    return get_path_offsets(best_path)


"""
The pretty_print_path function takes the path returned by the [a_star_search](a_star_search) function and prints the path on the world using the symbols ⏩, ⏪, ⏫, ⏬. The symbol 🎁 represents the goal on the board. The [get_path_direction](get_path_direction) function helps determine which symbol to print for the current position on the path. The pretty_print_path function also calculates the cost at each step of the path and outputs the total cost for the path.
"""
def pretty_print_path(world: List[List[str]], path: List[Tuple[int, int]], start: Tuple[int, int], goal: Tuple[int, int], costs: Dict[str, int]) -> int:
    symbols = {(0, 1): '⏩', (0, -1): '⏪', (-1, 0): '⏫', (1, 0): '⏬', 'goal': '🎁', (0, 0): '❌'}
    cost = 0
    new_world = deepcopy(world)
    current_position = start
    for index, move in enumerate(path):
        curr_row, curr_col = current_position[0], current_position[1]
        # break if we are at the end of the path
        if index == len(path)-1:
            if (curr_row, curr_col) == goal:
                new_world[curr_row][curr_col] = symbols["goal"]
            break
            
        cost += costs[world[curr_row][curr_col]]
        new_world[curr_row][curr_col] = symbols[move]
        current_position = (current_position[0] + move[0], current_position[1] + move[1])
    # print the map by unpacking the list at each row
    #for row in new_world:
    #    print(*row)    
    return cost, new_world


# ## Comments
# 
# Another solution to this problem is to return the first path found by A* search rather than selecting the best path of all paths found. This solution would be more space and time efficient. However, it could yield a sub-optimal result. 