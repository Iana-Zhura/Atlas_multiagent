import math
import numpy as np
import heapq


class State:
    def __init__(self, value, loc):
        self.value = value  # character representation on the map file
        self.key = np.inf
        self.path_cost = np.inf
        self.time_expanded = 0  # used to generate the path.
        self.loc = loc  # tuple: (x, y)
        self.successors = []
        self.accessed_diagonally = False  # was parent adjacent diagonally.
        self.jps_pruned = False  # if pruned, then consider it as a wall.
        self.parent = None

    def __str__(self):
        return "( id:" + str(self.get_hash_id()) + ", key: " + str(self.key) + " )"

    def __repr__(self):
        return "( id:" + str(self.get_hash_id()) + ", key: " + str(self.key) + " )"

    def reset_state(self):
        self.successors = []
        self.accessed_diagonally = False
        self.key = np.inf
        self.path_cost = np.inf
        self.time_expanded = 0
        self.jps_pruned = False
        self.parent = None

    def get_hash_id(self):
        """
        returns the unique id for the state. (Ex '@50_45')
        """
        return str(self.value) + "_" + str(self.loc[0]) + "_" + str(self.loc[1])


class JPS(object):
    def __init__(self, matrix_map, use_diagonals=True, diagonal_cost=math.sqrt(2), jps=True):
        self._use_diagonals = use_diagonals
        self._diagonal_cost = diagonal_cost
        self._state_map = self.states_from_matrix(matrix_map)
        self._goal = (0, 0)
        self._map_size = matrix_map.shape
        self._heuristic_function = self._euclidean_distance_heuristic
        self._jps = jps
        self.expanded = 0

    @staticmethod
    def states_from_matrix(matrix_map):
        result = [[State(matrix_map[i, j], (i, j)) for j in range(
            matrix_map.shape[1])] for i in range(matrix_map.shape[0])]
        return result

    def find_path(self, start, goal):
        self._refresh_state_map()
        start_state = self._state_map[start[0]][start[1]]
        self._goal = goal
        parent_state = self.search(start_state)
        path = [parent_state]
        while parent_state.parent is not None:
            parent_state = parent_state.parent
            path.append(parent_state)
        path = reversed(path)
        return np.array([x.loc for x in path])

    def _goal_function(self, state):
        return state.loc == self._goal

    def _manhattan_distance_heuristic(self, point):
        return (abs(point[0] - self._goal[0])) + (abs(point[1] - self._goal[1]))

    def _euclidean_distance_heuristic(self, point):
        return math.sqrt((point[0] - self._goal[0]) ** 2 + (point[1] - self._goal[1]) ** 2)

    # Used tp reset all the states in a map. Used when TEST_ALL_CASES is true.
    def _refresh_state_map(self):
        for i in range(0, len(self._state_map)):
            for j in range(0, len(self._state_map[i])):
                self._state_map[i][j].reset_state()
        self.expanded = 0

    def _update_cost(self, state, parent_state):
        """
        Calculates the f, or cost of the function where f = g + h
        path_cost: used to update state if a short path to state was found.
        """
        cost_delta = self._diagonal_cost if state.accessed_diagonally else 1
        path_cost = parent_state.path_cost + cost_delta
        if state.path_cost > path_cost:
            state.path_cost = path_cost
            state.time_expanded = parent_state.time_expanded + 1
            h = self._heuristic_function(state.loc)
            state.key = h + path_cost
            state.parent = parent_state

    def _get_successors(self, state):
        """
        Expands all states that are reachable and adjacent.
        """
        if len(state.successors) > 0:
            return state.successors
        locations = self._get_locations(state.loc)
        # Check all state if they are a wall or reachable.
        for (x, y, diagonal) in locations:
            if self._in_range(x, y):
                state.successors.append(self._state_map[x][y])
                self._state_map[x][y].accessed_diagonally = diagonal
        return state.successors

    def _get_locations(self, loc):
        locations = [
            (loc[0] - 1, loc[1], False),
            (loc[0], loc[1] - 1, False),
            (loc[0] + 1, loc[1], False),
            (loc[0], loc[1] + 1, False)
        ]
        if self._use_diagonals:
            locations.append((loc[0] - 1, loc[1] - 1, True))
            locations.append((loc[0] - 1, loc[1] + 1, True))
            locations.append((loc[0] + 1, loc[1] - 1, True))
            locations.append((loc[0] + 1, loc[1] + 1, True))
        return locations

    def _get_successors_jps(self, state):
        """
        Expands all states that are reachable and adjacent,
        then tries to "jump" from that expanded state.
        """
        if len(state.successors) > 0:
            return state.successors
        locations = self._get_locations(state.loc)
        for (x, y, diagonal) in locations:
            if self._in_range(x, y):
                path_cost = self._diagonal_cost if diagonal else 1
                jump_state = self._jump_successor(state, state, x - state.loc[0], y - state.loc[1], diagonal, path_cost)
                if jump_state is not None:
                    jump_state.jps_pruned = True
                    state.successors.append(jump_state)
                    self._state_map[x][y].accessed_diagonally = diagonal
        return state.successors

    def _jump_successor(self, state, direction_state, dx, dy, diagonal, curr_path_cost):
        """
        Jumps to the next adjacent node if no forced neighbours were found.
        direction_state: our parent node.
        dx, dy: change in movement from direction_state to our expanded state.
                Ex: dx = 1, dy = 1. We are moving diagonally down and right.
        diagonal: whether we are moving diagonal. Used for pruning cases.
        curr_path_cost: keep track of the cost of moving when we jump.
                        JUMPS ARENT FREE!
        returns a state. None if we go out of range when expanding or if
                         found state is a dead end.
        """
        current_x = direction_state.loc[0]
        current_y = direction_state.loc[1]
        next_x = current_x + dx
        next_y = current_y + dy

        # Update path cost as we move through states
        new_path_cost = curr_path_cost
        if diagonal:
            new_path_cost += self._diagonal_cost
        else:
            new_path_cost += 1

        # Is next movement a wall or out of bounds?
        if not self._in_range(next_x, next_y):
            if diagonal:
                direction_state.path_cost = curr_path_cost
                return direction_state
            else:
                direction_state.jps_pruned = True
                return None
        # If clear, check if it has been pruned before continuing.
        else:
            if self._state_map[next_x][next_y].jps_pruned:
                return None

        # Is next movement the goal?
        if self._goal_function(self._state_map[next_x][next_y]):
            self._state_map[next_x][next_y].path_cost = curr_path_cost
            return self._state_map[next_x][next_y]

        # Check for forced neighbours
        forced_neighbour = False
        if diagonal:
            neighbour_1 = ((next_x - dx), next_y)
            neighbour_2 = (next_x, (next_y - dy))
            if not self._in_range(neighbour_1[0], neighbour_1[1]):
                forced_neighbour = True
            elif not self._in_range(neighbour_2[0], neighbour_2[1]):
                forced_neighbour = True
        else:
            forced_neighbour = self._has_forced_neighbours(current_x, current_y, dx, dy, next_x, next_y)

        # Diagonal Case for forced neighbours
        if diagonal:
            if forced_neighbour:
                return self._state_map[next_x][next_y]
            horizontal_node = self._jump_successor(state, self._state_map[next_x][next_y], dx, 0, False, new_path_cost)
            vertical_node = self._jump_successor(state, self._state_map[next_x][next_y], 0, dy, False, new_path_cost)
            if horizontal_node is not None or vertical_node is not None:
                self._state_map[next_x][next_y].path_cost = curr_path_cost
                return self._state_map[next_x][next_y]
            else:
                direction_state.jps_pruned = True

        # Horizontal and Vertical Case for forced neighbours
        else:
            if forced_neighbour:
                self._state_map[next_x][next_y].path_cost = curr_path_cost
                return self._state_map[next_x][next_y]

        # If nothing blocking or no forced neighbours, continue down path.
        return self._jump_successor(state, self._state_map[next_x][next_y], dx, dy, diagonal, new_path_cost)

    def _has_forced_neighbours(self, cx, cy, dx, dy, nx, ny):
        """
        Looks at our neighbours to see if they are all optimally
        reachable via parent node 'p' or next node 'x'.
        Ex 1:
        [1][3][5]
        [p][x][ ]
        [2][4][6]
        1)In this case 1 and 2 are trivially optimally reachable from p
        because they are adjacent. Therefore we dont check them.
        2)3 and 4 can be reached optimally by going diagonal from p.
        3)5 and 6 can be reached optimally by going from p->x->5 or 6.
        Ex 2:
        [1][@][5]
        [p][x][ ]
        [2][4][@]
        1)In this case 1 and 2 are trivially optimally reachable ...
        2)5 cannot be reached optimally by going diagonal from p.
        this is called a forced neighbour. Although we can get to 5
        from x, we cannot guarantee optimality for 5.
        3)4 has a similar problem but we personally chose to consider
        this as a future case and therefore a forced neighbour.
        Ex 3:
        [1][@][@]
        [p][x][ ]
        [2][@][@]
        1)In this case 1 and 2 are trivially optimally reachable...
        2)Considered to be the same case as Ex 1, except there are
        no neighbours so we dont need to concern ourselves with them.
        returns True if forced neighbour is found. False otherwise.
        """
        curr_neighbour_1_x = (cx + dy)
        curr_neighbour_1_y = (cy + dx)

        curr_neighbour_2_x = (cx - dy)
        curr_neighbour_2_y = (cy - dx)

        next_neighbour_1_x = (nx + dy)
        next_neighbour_1_y = (ny + dx)

        next_neighbour_2_x = (nx - dy)
        next_neighbour_2_y = (ny - dx)

        c_1 = self._in_range(curr_neighbour_1_x, curr_neighbour_1_y)
        n_1 = self._in_range(next_neighbour_1_x, next_neighbour_1_y)
        if c_1 != n_1:
            return True

        c_2 = self._in_range(curr_neighbour_2_x, curr_neighbour_2_y)
        n_2 = self._in_range(next_neighbour_2_x, next_neighbour_2_y)
        if c_2 != n_2:
            return True

        return False

    def _in_range(self, x, y):
        index_check = (0 <= x <= self._map_size[0] - 1) and (0 <= y <= self._map_size[1] - 1)
        wall_check = True
        if index_check:
            wall_check = self._state_map[x][y].value == 0
        return index_check and wall_check

    def search(self, start):
        closed = set()
        open_list = []
        start.key = 0
        start.path_cost = 0
        heapq.heappush(open_list, (start.key, start.loc))
        current_expanded_path = []
        current_state = start
        previous_state = current_state
        while len(open_list) != 0:
            loc = heapq.heappop(open_list)[1]
            current_state = self._state_map[loc[0]][loc[1]]
            if current_state.get_hash_id() in closed:
                continue
            self.expanded += 1

            # Add node to path
            if current_state.time_expanded > len(current_expanded_path) - 1:
                current_expanded_path.append(current_state)
            else:
                current_expanded_path[current_state.time_expanded] = current_state

            if self._goal_function(current_state):
                print("solution found with: " + str(self.expanded) + " nodes expanded.")
                print("with path length: " + str(current_state.time_expanded))
                return current_state

            # add to closed dict so that we dont expand this node again.
            closed.add(current_state.get_hash_id())
            if self._jps:
                states = self._get_successors_jps(current_state)
                current_state.jps_pruned = True
            else:
                states = self._get_successors(current_state)
            for state in states:
                if not (state.get_hash_id() in closed):
                    self._update_cost(state, current_state)
                    heapq.heappush(open_list, (state.key, state.loc))
            previous_state = current_state
        print("no solution found")
        return previous_state
