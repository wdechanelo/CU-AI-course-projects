import sys
import resource
from collections import deque
from timeit import Timer

search_method = sys.argv[1]
goal_state = [[i, i + 1, i + 2] for i in range(0, 8, 3)]


def flatten_list(state):
    return [item for sublist in state for item in sublist]


def make_chunks(my_list, len_of_chunks):
    length = len(my_list)
    return [my_list[i:i+len_of_chunks] for i in range(0, length-1, len_of_chunks)]


def get_puzzle_grid(entered_grid=sys.argv[2]):

    simple_grid = entered_grid.split(',')
    simple_grid = [int(i) for i in simple_grid]
    if len(set(simple_grid)) == 9:
        puzzle_grid = make_chunks(simple_grid, 3)
    else:
        puzzle_grid = None
    return puzzle_grid


class Board:

    def __init__(self, current_state=None, parent_state=None, taken_action=None, depth=0):
        if not current_state:
            self.current_state = get_puzzle_grid()
        else:
            self.current_state = current_state
        self.parent_state = parent_state
        self.previous_action = taken_action
        self.state_id = hash(tuple(flatten_list(self.current_state)))
        self.depth=depth

    def print_board(self):
        for i in range(3):
            print(self.current_state[i])

    def available_moves(self):
        moves = ['up', 'down', 'left', 'right']
        flattened_state = flatten_list(self.current_state)
        index_of_zero = flattened_state.index(0)
        # we remove the right of moving left or right
        if index_of_zero % 3 == 0:
            moves.remove('left')
        elif index_of_zero % 3 == 2:
            moves.remove('right')
        # we remove the right of moving up and down
        if index_of_zero < 3:
            moves.remove('up')
        elif index_of_zero > 5:
            moves.remove('down')
        return moves

    def get_neighbors_states(self):
        return [Board(self.move_tile(move), self.current_state, move, self.depth+1)
                for move in self.available_moves()]

    def move_tile(self, direction):

        approved_moves = self.available_moves()
        if direction in approved_moves:
            flattened_state = flatten_list(self.current_state)
            index_of_zero = flattened_state.index(0)
            new_index_of_zero = index_of_zero
            if direction == 'up':
                new_index_of_zero = index_of_zero - 3
            elif direction == 'down':
                new_index_of_zero = index_of_zero + 3
            elif direction == 'left':
                new_index_of_zero = index_of_zero - 1
            else:
                new_index_of_zero = index_of_zero + 1
            temp_tile = flattened_state[new_index_of_zero]
            flattened_state[new_index_of_zero] = 0
            flattened_state[index_of_zero] = temp_tile
            new_state = make_chunks(flattened_state, 3)
            return new_state
        else:
            return self.current_state


class BreadFirstSearch:

    def __init__(self):
        self.current_board = Board()
        self.fringe = deque([])
        self.fringe_id = set([])
        self.explored_states = {}
        self.counter = 0

    def get_max_depth(self):
        max_depth = 0
        for node in self.fringe:
            if node.depth > max_depth:
                max_depth = node.depth
        return max_depth
        """for node_key in self.explored_states:
            if self.explored_states[node_key].depth > max_depth:
                max_depth = self.explored_states[node_key].depth"""

    def search(self):
        path = []
        search_depth = 0
        self.fringe.append(self.current_board)
        self.fringe_id.add(self.current_board.state_id)
        while self.fringe:
            state = self.fringe.popleft()
            self.explored_states[state.state_id] = state
            if state.state_id == hash(tuple(flatten_list(goal_state))):
                parent_id = hash(tuple(flatten_list(state.parent_state)))
                path.append(state.previous_action)
                while parent_id is not None:
                    search_depth += 1
                    e_state = self.explored_states[parent_id]
                    if e_state.parent_state is not None:
                        move = e_state.previous_action
                        path.insert(0, move)
                        parent_id = hash(tuple(flatten_list(e_state.parent_state)))
                    else:
                        parent_id = None
                #print("search depth : {0}".format(search_depth))
                #print("path length : {0} ".format(len(path)))
                #print("nodes_expanded : {0}".format(len(self.explored_states)-1))
                return search_depth, path
            else:
                for f_state in state.get_neighbors_states():
                    if f_state.state_id not in (self.explored_states and self.fringe_id):
                        self.fringe.append(f_state)
                        self.fringe_id.add(f_state.state_id)
        return "failure"


class DepthFirstSearch:

    def __init__(self):
        self.current_board = Board()
        self.fringe = []
        self.fringe_id = set([])
        self.explored_states = {}

    def get_max_depth(self):
        max_depth = 0
        for node_key in self.explored_states:
            if self.explored_states[node_key].depth > max_depth:
                max_depth = self.explored_states[node_key].depth
        return max_depth


    def search(self):
        path = []
        search_depth = 0
        max_search_depth = 0
        self.fringe.append(self.current_board)
        self.fringe_id.add(self.current_board.state_id)
        count = 0
        while self.fringe:
            state = self.fringe.pop()
            count += 1
            self.explored_states[state.state_id] = state
            if state.state_id == hash(tuple(flatten_list(goal_state))):
                parent_id = hash(tuple(flatten_list(state.parent_state)))
                path.append(state.previous_action)
                while parent_id is not None:
                    search_depth += 1
                    e_state = self.explored_states[parent_id]
                    if e_state.parent_state is not None:
                        move = e_state.previous_action
                        path.insert(0, move)
                        parent_id = hash(tuple(flatten_list(e_state.parent_state)))
                    else:
                        parent_id = None
                #print("search depth : {0}".format(search_depth))
                #print("path length : {0} ".format(len(path)))
                #print("nodes_expanded : {0}".format(len(self.explored_states)-1))
                return search_depth, path
            else:
                for f_state in state.get_neighbors_states()[::-1]:
                    if f_state.state_id not in (self.explored_states and self.fringe_id):
                        self.fringe.append(f_state)
                        self.fringe_id.add(f_state.state_id)
        return "failure"


if __name__ == '__main__':

    search_algo = {'bfs': BreadFirstSearch(), 'dfs': DepthFirstSearch()}
    sa = search_algo[search_method]
    setup = '''from __main__ import (sa)'''
    result = sa.search()

    path = result[1]
    cost_of_path = len(path)
    nodes_expanded = len(sa.explored_states)-1
    search_depth = result[0]
    max_depth = sa.get_max_depth()
    exec_time = Timer("sa.search()", setup=setup).timeit(number=10000)
    rss = float(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)/1024

    print("path_to_goal: {0}".format(path))
    print("cost_of_path: {0}".format(cost_of_path))
    print("nodes_expanded: {0}".format(nodes_expanded))
    print("search_depth: {0}".format(search_depth))
    print("max_search_depth : {0}".format(max_depth))
    print("max_ram_usage : {0}".format(rss))
    print("execution_time : {0}".format(exec_time))

    with open('output.txt', 'w') as output:
        output.writelines("path_to_goal: {}\n".format(path))
        output.writelines("cost_of_path: {}\n".format(cost_of_path))
        output.writelines("nodes_expanded: {}\n".format(nodes_expanded))
        output.writelines("search_depth: {}\n".format(search_depth))
        output.writelines("max_search_depth: {}\n".format(max_depth))
        output.writelines("running_time: {}\n".format(exec_time))
        output.writelines("max_ram_usage: {}\n".format(rss))
    output.close()