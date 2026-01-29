# imports priority queue module
import heapq

class Node:

    __slots__ = ("parent", "state", 'g', 'h', 'f')

    def __init__(self, state: tuple, g: int, h: int, f: int):
        self.parent = None
        self.state = state
        self.g = g
        self.h = h
        self.f = f

class Astar:

    __slots__ = ("initial_state", "goal_state", "wind",
                 "moves", "pq", "root", "explored",
                 "goal_state_mapping")

    def __init__(self, initial_state: tuple, goal_state: tuple, wind: dict):
        self.initial_state = initial_state
        self.goal_state = goal_state
        self.wind = wind
        self.moves = [
            ('w', -1, self.wind['e']),
            ('n', -3, self.wind['s']),
            ('e', 1, self.wind['w']),
            ('s', 3, self.wind['n'])
        ]

        self.pq = []
        self.root = None
        self.explored = set()
        self.goal_state_mapping = self.getGoalStateMapping()

        self.initializeRoot()

    def getGoalStateMapping(self) -> dict:
        mapping = {}

        for i in range(9):
            row, col = self.getIndexCoordinates(i)
            mapping[self.goal_state[i]] = (row, col)

        return mapping

    @staticmethod
    def getIndexCoordinates(i: int) -> (int, int):
        row = i // 3
        col = i % 3

        return row, col

    def initializeRoot(self):
        g = 0
        h = self.calculateHeuristic(self.initial_state)
        f = self.getF(g, h)

        self.root = Node(self.initial_state, g, h, f)

        heapq.heappush(self.pq, (f, 0, self.root))

    def getTilesOutOfPlace(self, s: tuple) -> int:
        count = 0

        for i in range(9):
            if s[i] != 0:
                if s[i] != self.goal_state[i]:
                    count += 1

        return count

    def getManhattanDistance(self, s: tuple) -> int:
        distance = 0

        for index, tile in enumerate(s):

            if tile == 0:
                continue

            cur_row, cur_col = self.getIndexCoordinates(index)
            goal_row, goal_col = self.goal_state_mapping[tile]
            # print(f"tile: {tile}, cur: ({cur_row}, {cur_col}), goal: ({goal_row}, {goal_col})")

            row_steps = abs(goal_row - cur_row)
            col_steps = abs(goal_col - cur_col)
            # print(f"tile: {tile}, row_steps: {row_steps}, col_steps: {col_steps}")

            row_cost = 0
            col_cost = 0

            if goal_row > cur_row:  # tile moves south
                row_cost = row_steps * self.wind['s']
            elif goal_row < cur_row:  # tile moves north
                row_cost = row_steps * self.wind['n']

            if goal_col > cur_col:  # tile moves east
                col_cost = col_steps * self.wind['e']
            elif goal_col < cur_col:  # tile moves west
                col_cost = col_steps * self.wind['w']

            # print(f"row_cost: {row_cost}, col_cost: {col_cost}")
            distance += row_cost + col_cost
            # print(f"distance: {distance}")

        return distance

    def calculateHeuristic(self, s: tuple) -> int:
        return self.getTilesOutOfPlace(s) + self.getManhattanDistance(s)

    @staticmethod
    def getF(g: int, h: int) -> int:
        return g + h

    def isGoal(self, node):
        return node.state == self.goal_state

    def expand(self, node):
        children = []

        current_state = node.state
        blank_idx = current_state.index(0)
        row, col = self.getIndexCoordinates(blank_idx)

        for direction, delta, cost in self.moves:

            # legality check
            if direction == 'w' and col == 0:
                continue
            if direction == 'e' and col == 2:
                continue
            if direction == 'n' and row == 0:
                continue
            if direction == 's' and row == 2:
                continue

            # proceeds to expand to legal states
            new_blank_idx = blank_idx + delta

            # converts current state from tuple to list
            state_list = list(current_state)
            # swaps blank tile with legal neighbor
            state_list[new_blank_idx], state_list[blank_idx] = state_list[blank_idx], state_list[new_blank_idx]
            # converts new state back from list to tuple
            state_tuple = tuple(state_list)

            # calculates total cost
            g = node.g + cost
            # calculates total heuristic
            h = self.calculateHeuristic(state_tuple)
            # calculates total f
            f = self.getF(g, h)

            # instantiates state expansion node
            expansion_node = Node(
                state=state_tuple,
                g=g,
                h=h,
                f=f
            )

            children.append(expansion_node)

        return children

    @staticmethod
    def printNode(node, expansion_count):
        state = list(node.state)

        blank_idx = state.index(0)
        state[blank_idx] = '-'

        print(f"{state[0]} {state[1]} {state[2]}\n"
              f"{state[3]} {state[4]} {state[5]}\n"
              f"{state[6]} {state[7]} {state[8]}")

        print(f"{node.g} | {node.h}")
        print(f" #{expansion_count}\n")

    def explore(self):
        expansion_counter = 1
        order_counter = 1

        while self.pq:
            priority, _, node = heapq.heappop(self.pq)

            # skips node if already in explored set
            if node.state in self.explored:
                continue

            # adds current state to the explored set
            self.explored.add(node.state)

            # prints current state grid
            self.printNode(node, expansion_counter)
            # increments the expansion count
            expansion_counter += 1

            # checks for goal
            if self.isGoal(node):
                break

            # performs expansions
            expansions = self.expand(node)

            # adds expansions to the frontier
            for expansion in expansions:
                heapq.heappush(self.pq, (expansion.f, order_counter, expansion))
                order_counter += 1

if __name__ == "__main__":

    initial_s = (
        1, 6, 2,
        5, 7, 8,
        0, 4, 3
    )

    goal_s = (
        7, 8, 1,
        6, 0, 2,
        5, 4, 3
    )

    wind_vals = {
        "w": 1,
        "n": 2,
        "e": 3,
        "s": 2
    }

    a_star_search = Astar(initial_state=initial_s,
                          goal_state=goal_s,
                          wind=wind_vals)

    a_star_search.explore()