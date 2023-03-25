from blockworld import BlockWorld
from queue import PriorityQueue
from typing import List, Tuple, Dict
import numpy as np

Action = Tuple[str, str]


class BlockWorldHeuristic(BlockWorld):
    def __init__(self, num_blocks=5, state=None):
        BlockWorld.__init__(self, num_blocks, state)

    def heuristic(self, goal):
        priority = 0.

        self_state = self.get_state()
        goal_state = goal.get_state()

        for set in self_state:
            # for each block in the set
            for i in reversed(range(len(set))):
                box_number = len(set) - 1 - i

                for goal_set in goal_state:
                    if (set[i] in goal_set):
                        already_one_correct = False
                        for j in range(len(goal_set)):
                            if j > box_number:
                                break
                            if (box_number == 0 and set[-1 - box_number] != goal_set[-1 - j]):
                                priority += 1
                                break

                            if (box_number == j and set[-1 - box_number] != goal_set[-1 - j]):
                                priority += 2
                                break

                            if set[-1 - j] != goal_set[-1 - j]:
                                if already_one_correct:
                                    priority += 2
                                else:
                                    priority += 1
                                break
                            elif (j == 0):
                                already_one_correct = True

        return priority


class AStar():

    # def to_string(self, state):
    #     ret = "["
    #     for i in range(len(state.state)):
    #         ret += "["
    #         for j in range(len(state.state[i])):
    #             ret += str(state.state[i][j])
    #             if j != len(state.state[i]) - 1:
    #                 ret += ", "
    #         ret += "], "
    #     ret += "]"
    #     return ret

    def search(self, start, goal):
        opened = PriorityQueue()
        closed = dict()

        num_of_blocks = len(start.get_state())

        opened.put((0, start, None, None, 0))

        while not opened.empty():
            priority, state, prev_state, prev_action, cost = opened.get()

            if goal.__eq__(state):
                closed[state] = prev_action, prev_state
                return self.reconstruct_path(closed, start, state)

            if state in closed:  # State already visited ...
                continue

            else:
                closed[state] = prev_action, prev_state

            for action in state.get_actions():


                #next_state = BlockWorldHeuristic(num_of_blocks, self.to_string(state))
                next_state = state.clone()
                next_state.apply(action)

                if (next_state in closed):
                    continue

                new_priority = next_state.heuristic(goal) + cost

                opened.put((new_priority, next_state, state, action, cost+1))

        return []

    def reconstruct_path(self, closed: Dict[BlockWorld, Tuple[Action, BlockWorld]],
                         init_state: BlockWorld,
                         last_state: BlockWorld) -> List[Action]:
        action, prev_state = closed[last_state]
        path = [action]
        while prev_state != init_state:
            action, prev_state = closed[prev_state]
            path.append(action)
        return list(reversed(path))


if __name__ == '__main__':
    # Here you can test your algorithm. You can try different N values, e.g. 6, 7.
    N = 9

    start = BlockWorldHeuristic(N)
    goal = BlockWorldHeuristic(N)

    print("Searching for a path:")
    print(f"{start} -> {goal}")
    print()

    astar = AStar()
    path = astar.search(start, goal)

    if path is not None:
        print("Found a path:")
        print(path)

        print("\nHere's how it goes:")

        s = start.clone()
        print(s)

        for a in path:
            s.apply(a)
            print(s)

    else:
        print("No path exists.")

    print("Total expanded nodes:", BlockWorld.expanded)
