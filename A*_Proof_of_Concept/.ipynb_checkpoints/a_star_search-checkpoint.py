#!/usr/bin/env python
# coding: utf-8

# # Module 1 - Programming Assignment
# 
# ## General Directions
# 
# 1. You must follow the Programming Requirements outlined on Canvas.
# 2. The Notebook should be cleanly and fully executed before submission.
# 3. You should change the name of this file to be your JHED id. For example, `jsmith299.ipynb` although Canvas will change it to something else... :/
# 
# <div style="background: lemonchiffon; margin:20px; padding: 20px;">
#     <strong>Important</strong>
#     <p>
#         You should always read the entire assignment before beginning your work, so that you know in advance what the requested output will be and can plan your implementation accordingly.
#     </p>
# </div>

# # State Space Search with A* Search
# 
# You are going to implement the A\* Search algorithm for navigation problems.
# 
# 
# ## Motivation
# 
# Search is often used for path-finding in video games. Although the characters in a video game often move in continuous spaces,
# it is trivial to layout a "waypoint" system as a kind of navigation grid over the continuous space. Then if the character needs
# to get from Point A to Point B, it does a line of sight (LOS) scan to find the nearest waypoint (let's call it Waypoint A) and
# finds the nearest, LOS waypoint to Point B (let's call it Waypoint B). The agent then does a A* search for Waypoint B from Waypoint A to find the shortest path. The entire path is thus Point A to Waypoint A to Waypoint B to Point B.
# 
# We're going to simplify the problem by working in a grid world. The symbols that form the grid have a special meaning as they
# specify the type of the terrain and the cost to enter a grid cell with that type of terrain:
# 
# ```
# token   terrain    cost 
# 🌾       plains     1
# 🌲       forest     3
# 🪨       hills      5
# 🐊       swamp      7
# 🗻       mountains  impassible
# ```
# 
# We can think of the raw format of the map as being something like:
# 
# ```
# 🌾🌾🌾🌾🌲🌾🌾
# 🌾🌾🌾🌲🌲🌲🌾
# 🌾🗻🗻🗻🌾🌾🌾
# 🌾🌾🗻🗻🌾🌾🌾
# 🌾🌾🗻🌾🌾🌲🌲
# 🌾🌾🌾🌾🌲🌲🌲
# 🌾🌾🌾🌾🌾🌾🌾
# ```

# ## The World
# 
# Given a map like the one above, we can easily represent each row as a `List` and the entire map as `List of Lists`:

# In[138]:


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


# ## Warning
# 
# One implication of this representation is that (x, y) is world[ y][ x] so that (3, 2) is world[ 2][ 3] and world[ 7][ 9] is (9, 7). Yes, there are many ways to do this. I picked this representation because when you look at it, it *looks* like a regular x, y cartesian grid and it's easy to print out.
# 
# It is often easier to begin your programming by operating on test input that has an obvious solution. If we had a small 7x7 world with the following characteristics:

# In[139]:


small_world = [
    ['🌾', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲'],
    ['🌾', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲'],
    ['🌾', '🌲', '🌲', '🌲', '🌲', '🌲', '🌲'],
    ['🌾', '🌾', '🌾', '🌾', '🌾', '🌾', '🌾'],
    ['🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌾'],
    ['🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌾'],
    ['🌲', '🌲', '🌲', '🌲', '🌲', '🌲', '🌾']
]


# **what do you expect the policy would be?** Think about it for a bit. This will help you with your programming and debugging.

# ## States and State Representation
# 
# The canonical pieces of a State Space Search problem are the States, Actions, Transitions and Costs. 
# 
# We'll start with the state representation. For the navigation problem, a state is the current position of the agent, `(x,y)`. The entire set of possible states is implicitly represented by the world map.

# ## Actions and Transitions
# 
# Next we need to specify the actions. In general, there are a number of different possible action sets in such a world. The agent might be constrained to move north/south/east/west or diagonal moves might be permitted as well (or really anything). When combined with the set of States, the *permissible* actions forms the Transition set.
# 
# Rather than enumerate the Transition set directly, for this problem it's easier to calculate the available actions and transitions on the fly. This can be done by specifying a *movement model* as offsets to the current state and then checking to see which of the potential successor states are actually permitted. This can be done in the successor function mentioned in the pseudocode.
# 
# One such example of a movement model is shown below.

# In[140]:


MOVES = [(0,-1), (1,0), (0,1), (-1,0)]


# ## Costs
# 
# We can encode the costs described above in a `Dict`:

# In[141]:


COSTS = { '🌾': 1, '🌲': 3, '🪨': 5, '🐊': 7}


# ## Specification
# 
# You will implement a function called `a_star_search` that takes the parameters and returns the value as specified below. The return value is going to look like this:
# 
# `[(0,1), (0,1), (0,1), (1,0), (1,0), (1,0), (1,0), (1,0), (1,0), (0,1), (0,1), (0,1)]`
# 
# You should also implement a function called `pretty_print_path`. 
# The `pretty_print_path` function prints an ASCII representation of the path generated by the `a_star_search` on top of the terrain map. 
# For example, for the test world, it would print this:
# 
# ```
# ⏬🌲🌲🌲🌲🌲🌲
# ⏬🌲🌲🌲🌲🌲🌲
# ⏬🌲🌲🌲🌲🌲🌲
# ⏩⏩⏩⏩⏩⏩⏬
# 🌲🌲🌲🌲🌲🌲⏬
# 🌲🌲🌲🌲🌲🌲⏬
# 🌲🌲🌲🌲🌲🌲🎁
# ```
# 
# using ⏩,⏪,⏫ ⏬ to represent actions and `🎁` to represent the goal. (Note the format of the output...there are no spaces, commas, or extraneous characters). You are printing the path over the terrain.
# This is an impure function (because it has side effects, the printing, before returning anything).
# 
# Note that in Python:
# ```
# > a = ["*", "-", "*"]
# > "".join(a)
# *-*
# ```
# Do not print raw data structures; do not insert unneeded/requested spaces!
# 
# ### Additional comments
# 
# As Python is an interpreted language, you're going to need to insert all of your functions *before* the actual `a_star_search` function implementation. 
# Do not make unwarranted assumptions (for example, do not assume that the start is always (0, 0).
# Do not refer to global variables, pass them as parameters (functional programming).
# 
# Simple and correct is better than inefficient and incorrect, or worse, incomplete.
# For example, you can use a simple List, with some helper functions, as a Stack or a Queue or a Priority Queue.
# Avoid the Python implementations of HeapQ, PriorityQueue implementation unless you are very sure about what you're doing as they require *immutable* keys.

# In[142]:


from typing import List, Tuple, Dict, Callable, Any
from copy import deepcopy


# *add as many markdown and code cells here as you need for helper functions. We have added `heuristic` for you*

# <a id="add_to_frontier"></a>
# ## add_to_frontier
# 
# *The add_to_frontier algorithm treats the frontier as a priority queue and adds the value to the frontier in the correct sorted location. The frontier is sorted in ascending order by cost. The function updates the frontier in-place and returns None.* **Used by**: [a_star_search](#a_star_search)
# 
# * **value** Dict[str, Tuple[int, int] | int | List[Tuple[int, int]]]: The values will be of the form {"position": (0, 0), "cost": 0, "path": [(0, 0)], "g": 0}
# * **frontier** List[Dict[str, Tuple[int, int] | int | List[Tuple[int, int]]]]: The frontier wll contain a list of the values outlined above in ascending order by cost. 
# 
# **returns** None.
# 
# Note: the implementation of this algorithm using a for-loop could be inefficient if the frontier is long. 

# In[143]:


def add_to_frontier(value: Dict[str, Any], frontier: List[Dict[str, Any]]) -> None:
    for index, frontier_value in enumerate(frontier): 
        if frontier_value["cost"] > value["cost"]:
            frontier.insert(index, value)
            return 
    # append if no insert location was found
    frontier.append(value)


# In[144]:


# assertions/unit tests
test_frontier = [
    {"position": (0, 0), "cost": 1, "path": [(0, 0)]},
    {"position": (0, 1), "cost": 2, "path": [(0, 0), (0, 1)]},
    {"position": (0, 2), "cost": 5, "path": [(0, 0), (0, 1), (0, 2)]}
]

# insert at front
test_value_1 = {"position": (1, 0), "cost": 0, "path": [(0, 0), (1, 0)]}
add_to_frontier(test_value_1, test_frontier)
assert test_frontier[0] == test_value_1
test_frontier.remove(test_value_1)

# insert in middle
test_value_2 = {"position": (2, 0), "cost": 3, "path": [(0, 0), (1, 0), (2, 0)]}
add_to_frontier(test_value_2, test_frontier)
assert test_frontier[2] == test_value_2
test_frontier.remove(test_value_2)

# insert at end
test_value_3 = {"position": (3, 0), "cost": 5, "path": [(0, 0), (1, 0), (3, 0)]}
add_to_frontier(test_value_3, test_frontier)
assert test_frontier[-1] == test_value_3
test_frontier.remove(test_value_3)


# <a id="get_successors"></a>
# ## get_successors
# 
# *The get_successors function takes the current_position, the world, and the available moves and the direction of the move in the form [[(2, 2), (0, 1)],..], where (0, 1) represents right. The function uses these values to find valid moves on the board from the current position.* **Used by**: [a_star_search](#a_star_search)
# 
# * **current_position** Tuple[int, int]: The current location on the board. 
# * **world** List[List[str]]: The world to explore.
# * **moves** List[Tuple[int, int]]: Valid actions that the agent may take.  
# 
# **returns** List[List[Tuple[int, int]]]: The valid moves from the current location.

# In[145]:


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


# In[146]:


# assertions/unit tests
assert get_successors((0,1), small_world, MOVES) == [(0, 0), (1, 1), (0, 2)]
# check lower bound of map
assert get_successors((0,0), small_world, MOVES) == [(1, 0), (0, 1)]
# check upper bound of map
assert get_successors((6,6), small_world, MOVES) == [(6, 5), (5, 6)]


# <a id="get_path_direction"></a>
# ## get_path_direction
# 
# *The get_path_direction function is a helper of the pretty_print_path function. This helper determines which symbol to print on the map based on the direction of the path.* **Used by**: [pretty_print_path](#pretty_print_path)
# 
# * **row** int: The row of the current position in the path 
# * **col** int: The column of the current position in the path
# * **next_row** int: The row of the next position in the path
# * **next_col** int: The column of the next position in the path
# 
# **returns** str: A string representing the direction of the path.

# In[147]:


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


# In[148]:


# assertions/unit tests
assert get_path_offsets([(0, 0), (0, 1)]) == [(0, 1), (0, 0)]
assert get_path_offsets([(5, 6), (6, 6)]) == [(1, 0), (0, 0)]
assert get_path_offsets([(5, 6), (4, 6)]) == [(-1, 0), (0, 0)]


# <a id="seen_nodes"></a>
# ## seen_nodes
# 
# *The seen_nodes function takes a list of dictionaries and determines if the position coordinates of the value parameter are already in the list.* **Used by**: [a_star_search](#a_star_search)
# 
# * **value** Dict[str, Tuple[int, int]|int|List[Tuple[int, int]]]: the current position on the board with position metadata.  
# * **seen_list** List[Dict[str, Tuple[int, int]|int|List[Tuple[int, int]]]]: The list of nodes already observed by the algorithm.
# 
# **returns** boolean: True if the coorindates are in the list, False otherwise. 

# In[149]:


def seen_node(value: Dict[str, Any], seen_list: List[Dict[str, Any]]) -> bool:
    for seen_value in seen_list:
        if value["position"] == seen_value["position"]:
            return True
    return False


# In[150]:


# assertions/unit tests
test_seen_list = [
    {"position": (0, 0), "cost": 1, "path": [(0, 0)]},
    {"position": (0, 1), "cost": 2, "path": [(0, 0), (0, 1)]},
    {"position": (0, 2), "cost": 5, "path": [(0, 0), (0, 1), (0, 2)]}
]
# case coordinates different, cost and path the same
assert(seen_node({"position": (0, 3), "cost": 1, "path": [(0, 0)]}, test_seen_list)) == False
# case all node metadata is different
assert(seen_node({"position": (0, 3), "cost": 2, "path": [(0, 1)]}, test_seen_list)) == False
# case coordinates seen
assert(seen_node({"position": (0, 1), "cost": 2, "path": [(0, 1)]}, test_seen_list)) == True


# <a id="heuristic"></a>
# ## heuristic
# 
# *The heuristic function used the Manhattan Distance to estimate the cost from the current state to the goal. The Manhattan Distance is useful for search patterns that search adjacent nodes on a graph.* **Used by**: [a_star_search](#a_star_search)
# 
# * **current_position** Tuple[int, int]: the bot's current position on the board, `(x, y)`.
# * **goal** Tuple[int, int]: the desired goal position for the bot, `(x, y)`.
# 
# **returns** int: the estimated cost to reach the goal from the current position.

# In[151]:


def heuristic(current_position: Tuple[int, int], goal: Tuple[int, int]) -> int:
    return abs(goal[0] - current_position[0]) + abs(goal[1] - current_position[1])


# In[152]:


# assertions/unit tests
assert heuristic((3, 2), (6, 3)) == 4
assert heuristic((3, 3), (0, 0)) == 6
assert heuristic((0, 0), (5, 4)) == 9


# <a id="get_best_path"></a>
# ## get_best_path
# 
# *The get_best_path function take a list of all paths that the [a_star_search](a_star_search) function found to the goal and returns the path with the lowest cost.* **Used by**: [a_star_search](#a_star_search)
# 
# * **goal_paths** List[Dict[str, List[Tuple[int, int]] | int]]: a list of dictionaries that contains all paths to the goal found by the a_star algorithm, and the cost of each path. 
# 
# **returns** List[Tuple[int, int]]: the path with the lowest cost
# 

# In[153]:


import math

def get_best_path(goal_paths: List[Dict[str, Any]]) -> List[Tuple[int, int]]:
    best_path = []
    best_cost = math.inf
    for path in goal_paths:
        if path["g"] < best_cost:
            best_path = path["path"]
            best_cost = path["g"]
    return best_path  


# In[154]:


# assertions/unit tests
test_1_goal_paths = [
    {"path": [(0, 1), (0, 2), (0, 3)], "g": 10},
    {"path": [(0, 1), (1, 2), (0, 3)], "g": 11},
    {"path": [(1, 1), (0, 2), (0, 3)], "g": 12}
]
assert get_best_path(test_1_goal_paths) == [(0, 1), (0, 2), (0, 3)]

test_2_goal_paths = [
    {"path": [(0, 1), (0, 2), (0, 3)], "g": 20},
    {"path": [(0, 1), (1, 2), (0, 3)], "g": 11},
    {"path": [(1, 1), (0, 2), (0, 3)], "g": 12}
]
assert get_best_path(test_2_goal_paths) == [(0, 1), (1, 2), (0, 3)]

test_3_goal_paths = [
    {"path": [(0, 1), (0, 2), (0, 3)], "g": 20},
    {"path": [(0, 1), (1, 2), (0, 3)], "g": 30},
    {"path": [(1, 1), (0, 2), (0, 3)], "g": 12}
]
assert get_best_path(test_3_goal_paths) == [(1, 1), (0, 2), (0, 3)]


# <a id="a_star_search"></a>
# ## a_star_search
# 
# *The a_star_search function applies the A\* search algorithm to find a path to the goal position from the starting position. The A\* search algorithm is an informed search algorithm that uses a heuristic function to predict how far each successor position is from the goal position. The algorithm calls the [get_successors](get_successors) function to retrieve all valid moves from the current position using the problem constraints. For every successor returned, the A\* search algorithm calculates the cost of that position using the forumala f(n) = g(n) + h(n), where g(n) is the cost of the child position and h(n) is the prediction from the [heuristic](heuristic) function. Next, the algorithm adds every successor position with its predicted cost of f(n) to the frontier. The A\* search function leverages a priority queue for the search frontier so that the algorithm always explores the position with the lowest f(n) first. The algorithm uses a dictionary of the form {"position": Tuple[int, int], "cost": int, "path": List[Tuple[int, int], "g": int}, where g is the cost from the initial node to the current node, to represent a frontier value. The path metadata helps recover the cheapest path found for the [pretty_print_path](#pretty_print_path) function.*
# 
# * **world** List[List[str]]: the actual context for the navigation problem.
# * **start** Tuple[int, int]: the starting location of the bot, `(x, y)`.
# * **goal** Tuple[int, int]: the desired goal position for the bot, `(x, y)`.
# * **costs** Dict[str, int]: is a `Dict` of costs for each type of terrain in **world**.
# * **moves** List[Tuple[int, int]]: the legal movement model expressed in offsets in **world**.
# * **heuristic** Callable: is a heuristic function, $h(n)$.
# 
# 
# **returns** List[Tuple[int, int]]: the offsets needed to get from start state to the goal as a `List`.

# In[155]:


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


# Note: algorithm tested below.

# ## pretty_print_path
# 
# *The pretty_print_path function takes the path returned by the [a_star_search](a_star_search) function and prints the path on the world using the symbols ⏩, ⏪, ⏫, ⏬. The symbol 🎁 represents the goal on the board. The [get_path_direction](get_path_direction) function helps determine which symbol to print for the current position on the path. The pretty_print_path function also calculates the cost at each step of the path and outputs the total cost for the path.*
# 
# * **world** List[List[str]]: the world (terrain map) for the path to be printed upon.
# * **path** List[Tuple[int, int]]: the path from start to goal, in offsets.
# * **start** Tuple[int, int]: the starting location for the path.
# * **goal** Tuple[int, int]: the goal location for the path.
# * **costs** Dict[str, int]: the costs for each action.
# 
# **returns** int - The path cost.

# In[156]:


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

# ## To think about for future assignments...

# This first assignment may not have been difficult for you if you've encountered A* search before in your Algorithms course. In preparation for future assignments that build on State Space Search, you can think about the following or even do an implementation if you like. You should **not** submit it as part of this assignment.
# 
# In several future assignments, we will have a need for a "plain ol'" Depth First Search algorithm.
# 
# 1. Implement DFS Search to solve the problem presented in this programming assignment. Try to be as general as possible (don't hard code anything you can pass as a formal parameter).
# 2. Can you implement DFS Search as a higher order function and supply your own `is_goal`, `successors`, and `path` functions? How do you handle *state*?
# 3. Can you write a version of DFS that returns all the solutions?
# 
# In one future assignment a Breadth First Search algorithm will be very handy. Can you implement a search algorithm that changes whether it uses DFS or BFS by parameterization?

# ## Before You Submit...
# 
# 1. Did you provide output exactly as requested?
# 2. Did you follow the Programming Requirements on Canvas?
# 
# Do not submit any other files.
