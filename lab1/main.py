import heapq
import sys


class PriorityQueue:

    def __init__(self):
        self.heap = []
        self.count = 0

    def push(self, item, priority):
        entry = (priority, self.count, item)
        heapq.heappush(self.heap, entry)
        self.count += 1

    def pop(self):
        (_, _, item) = heapq.heappop(self.heap)
        return item

    def isEmpty(self):
        return len(self.heap) == 0

    def update(self, item, priority):
        # If item already in priority queue with higher priority, update its priority and rebuild the heap.
        # If item already in priority queue with equal or lower priority, do nothing.
        # If item not in priority queue, do the same thing as self.push.
        for index, (p, c, i) in enumerate(self.heap):
            if i == item:   # item是node.state
                if p <= priority:
                    break
                del self.heap[index]
                self.heap.append((priority, c, item))
                heapq.heapify(self.heap)
                break
        else:
            self.push(item, priority)


class node:
    """define node"""

    def __init__(self, state, parent, path_cost, action,):
        self.state = state          # A,B,C,D等字母
        self.parent = parent        # 一个node
        self.path_cost = path_cost  # 当前已知的距离
        self.action = action        # 一个列表，包含node的路径，每个action是个三元组


class problem:
    """searching problem"""

    def __init__(self, initial_state, actions):
        self.initial_state = initial_state
        self.actions = actions

    def search_actions(self, state):
        action_state = []
        for act in self.actions:
            if act[0] == state or act[1] == state:
                action_state.append(act)
        return action_state

    def solution(self, node):
        path = [node.state]
        while True:
            if node.parent == '':
                break
            path.insert(0, node.parent.state)
            node = node.parent
        return path

    def transition(self, state, action):
        if action[0] == state:
            return action[1]
        else:
            return action[0]

    def goal_test(self, state):
        return state == 'Goal'

    def step_cost(self, state1, action, state2):
        if (state1 == action[0]) and (state2 == action[1]):
            return int(action[2])
        else:
            print("Step error!")
            sys.exit()

    def child_node(self, node_begin, action):
        if node_begin.state == action[0]:
            return node(action[1], node_begin, node_begin.path_cost + int(action[2]), '')
        else:
            return node(action[0], node_begin, node_begin.path_cost + int(action[2]), '')


def UCS(problem):
    node_test = node(problem.initial_state, '', 0, '')
    frontier = PriorityQueue()
    frontier.push(node_test.state, node_test.path_cost)
    state2node = {node_test.state: node_test}
    explored = []
    while not frontier.isEmpty():
        node_test = state2node[frontier.pop()]
        if problem.goal_test(node_test.state):
            return problem.solution(node_test)
        explored.append(node_test.state)
        for action in problem.search_actions(node_test.state):
            child = problem.child_node(node_test, action)
            if child.state not in explored:
                frontier.update(child.state, child.path_cost)
            if child.state not in state2node.keys():
                state2node[child.state] = child
            elif child.path_cost < state2node[child.state].path_cost:
                state2node[child.state] = child

    return "Unreachable"


if __name__ == '__main__':
    Actions = []
    while True:
        a = input().strip()
        if a != 'END':
            a = a.split()
            Actions += [a]
        else:
            break
    graph_problem = problem('Start', Actions)
    answer = UCS(graph_problem)
    s = "->"
    if answer == 'Unreachable':
        print(answer)
    else:
        path = s.join(answer)
        print(path)
