from blockworld import BlockWorld
from queue import PriorityQueue
from typing import List, Tuple, Dict, Callable
from copy import deepcopy

Action = Tuple[str, str]


class BlockWorldHeuristic(BlockWorld):
	def __init__(self, num_blocks=5, state=None):
		BlockWorld.__init__(self, num_blocks, state)

	def heuristic(self, goal):
		priority = 0.

		self_state = self.get_state()
		goal_state = goal.get_state()

		for set in self_state:
			for goal_set in goal_state:
				for i in range(len(goal_set)):
					if (i >= len(set)): break
					if (goal_set[-1 - i] == set[-1 - i]):
						priority -= 1

		return priority

class AStar():
	def search(self, start, goal):
		opened = PriorityQueue()
		closed = dict()

		opened.put((0, start, None, None))

		while not opened.empty():
			priority, state, prev_state, prev_action = opened.get()

			if goal.__eq__(state):
				closed[state] = prev_action, prev_state
				return self.reconstruct_path(closed, start, state)

			if state in closed:  # State already visited ...
				continue
			else:
				closed[state] = prev_action, prev_state

			for action in state.get_actions():
				next_state = state.clone()
				next_state.apply(action)

				if next_state in closed:
					continue

				new_priority = priority + next_state.heuristic(goal) + 1


				opened.put((new_priority, next_state, state, action))

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