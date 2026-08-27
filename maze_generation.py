"""Topology generation and validation for the EGB320 grid maze.

This module deliberately has no CoppeliaSim dependencies.  It describes a maze as the
unit wall segments between grid intersections and leaves ``mazebot_lib`` responsible
for turning those segments into simulator objects.
"""

from collections import deque
from dataclasses import dataclass, field
import math
import random


DIRECTIONS = ('N', 'E', 'S', 'W')
DIRECTION_DELTAS = {
	'N': (0, -1),
	'E': (1, 0),
	'S': (0, 1),
	'W': (-1, 0),
}
OPPOSITE_DIRECTION = {
	'N': 'S',
	'E': 'W',
	'S': 'N',
	'W': 'E',
}
VICTIM_LEVELS = ('L1', 'L2', 'L3')


# The original hand-authored maze remains available as the default preset.  Wall
# segments use grid-intersection coordinates, not world coordinates.
PRESET_MAZE_SEGMENTS = (
	((4, 0), (4, 1)),

	((1, 1), (2, 1)),
	((2, 1), (2, 2)),

	((3, 1), (3, 2)),
	((3, 2), (3, 3)),

	((5, 1), (6, 1)),
	((6, 1), (6, 2)),
	((6, 2), (6, 3)),

	((5, 1), (5, 2)),
	((5, 2), (5, 3)),
	((5, 3), (5, 4)),

	((1, 2), (2, 2)),
	((1, 3), (2, 3)),
	((2, 2), (2, 3)),

	((3, 2), (4, 2)),
	((4, 2), (5, 2)),

	((1, 3), (1, 4)),

	((2, 3), (3, 3)),
	((2, 3), (2, 4)),
	((2, 4), (2, 5)),
	((2, 5), (2, 6)),

	((1, 4), (1, 5)),

	((3, 4), (4, 4)),
	((4, 4), (5, 4)),

	((6, 4), (7, 4)),

	((3, 4), (3, 5)),

	((1, 5), (1, 6)),
	((1, 6), (1, 7)),
	((4, 5), (4, 6)),

	((5, 4), (5, 5)),
	((5, 5), (5, 6)),
	((5, 5), (6, 5)),

	((4, 6), (5, 6)),
	((5, 6), (6, 6)),

	((3, 6), (3, 7)),
)

PRESET_BASE_CELL = (0, 6)
PRESET_BASE_YAW = math.pi / 2.0
PRESET_VICTIM_CELLS = {
	'L1': (1, 2),
	'L2': (5, 1),
	'L3': (4, 5),
}


@dataclass
class MazeLayout:
	"""Complete grid-topology result returned by a maze generator."""

	rows: int
	columns: int
	wall_segments: list
	base_cell: tuple
	victim_cells: dict
	mode: str = 'random'
	seed: int = None
	generation_attempt: int = 1
	distances_from_base: dict = field(default_factory=dict)
	hazard_cells: list = field(default_factory=list)


def validate_victim_count(victim_count):
	"""Return a validated victim count in the supported inclusive range 1--3."""
	if (not isinstance(victim_count, int) or isinstance(victim_count, bool) or
			not 1 <= victim_count <= len(VICTIM_LEVELS)):
		raise ValueError('number of victims must be an integer from 1 to 3')
	return victim_count


def normalise_grid_segment(start_point, end_point):
	"""Return a direction-independent, hashable key for a grid wall segment."""
	return tuple(sorted((tuple(start_point), tuple(end_point))))


def cell_side_segment(cell, side):
	"""Return the grid-intersection segment forming one side of ``cell``."""
	column, row = cell
	segments = {
		'N': ((column, row), (column + 1, row)),
		'E': ((column + 1, row), (column + 1, row + 1)),
		'S': ((column, row + 1), (column + 1, row + 1)),
		'W': ((column, row), (column, row + 1)),
	}
	try:
		return segments[side]
	except KeyError:
		raise ValueError(f"Unknown cell side {side!r}; expected one of {DIRECTIONS}")


def _validate_cell(cell, rows, columns, description):
	try:
		column, row = cell
	except (TypeError, ValueError):
		raise ValueError(f"{description} must be a (column, row) pair (got {cell!r})")
	if not (isinstance(column, int) and isinstance(row, int)):
		raise ValueError(f"{description} coordinates must be integers (got {cell!r})")
	if not (0 <= column < columns and 0 <= row < rows):
		raise ValueError(
			f"{description} {tuple(cell)} is outside the {rows} x {columns} maze")
	return (column, row)


def validate_wall_segments(rows, columns, wall_segments):
	"""Validate wall geometry and return its normalised segment-key set."""
	wall_keys = set()
	for wall_index, segment in enumerate(wall_segments):
		try:
			start_point, end_point = segment
			start_column, start_row = start_point
			end_column, end_row = end_point
		except (TypeError, ValueError):
			raise ValueError(
				f"Wall {wall_index} must contain two (column, row) endpoints")

		for point in (start_point, end_point):
			column, row = point
			if not (isinstance(column, int) and isinstance(row, int)):
				raise ValueError(f"Wall endpoint coordinates must be integers: {point}")
			if not (0 <= column <= columns):
				raise ValueError(
					f"Wall endpoint column {column} out of range [0, {columns}]: {point}")
			if not (0 <= row <= rows):
				raise ValueError(
					f"Wall endpoint row {row} out of range [0, {rows}]: {point}")

		if start_point == end_point:
			raise ValueError(f"Wall segment has identical start and end point: {start_point}")
		if start_column != end_column and start_row != end_row:
			raise ValueError(
				f"Maze walls must be axis-aligned; diagonal segment "
				f"{start_point} -> {end_point} is not supported")
		if abs(end_column - start_column) + abs(end_row - start_row) != 1:
			raise ValueError(
				f"Maze walls must span exactly one cell edge: "
				f"{start_point} -> {end_point}")

		wall_key = normalise_grid_segment(start_point, end_point)
		if wall_key in wall_keys:
			raise ValueError(f"Duplicate maze wall segment: {wall_key}")
		wall_keys.add(wall_key)
	return wall_keys


def get_cell_wall_sides(rows, columns, wall_segments, cell):
	"""Return booleans indicating whether the N/E/S/W cell sides are closed."""
	cell = _validate_cell(cell, rows, columns, 'Cell')
	wall_keys = (
		wall_segments if isinstance(wall_segments, set)
		else {normalise_grid_segment(*segment) for segment in wall_segments}
	)
	column, row = cell
	boundaries = {
		'N': row == 0,
		'E': column == columns - 1,
		'S': row == rows - 1,
		'W': column == 0,
	}
	return {
		side: boundaries[side] or normalise_grid_segment(
			*cell_side_segment(cell, side)) in wall_keys
		for side in DIRECTIONS
	}


def build_open_adjacency(rows, columns, wall_segments):
	"""Build a cell adjacency map from the internal wall representation."""
	wall_keys = (
		wall_segments if isinstance(wall_segments, set)
		else {normalise_grid_segment(*segment) for segment in wall_segments}
	)
	adjacency = {
		(column, row): []
		for row in range(rows)
		for column in range(columns)
	}
	for row in range(rows):
		for column in range(columns):
			cell = (column, row)
			for side in ('E', 'S'):
				delta_column, delta_row = DIRECTION_DELTAS[side]
				neighbour = (column + delta_column, row + delta_row)
				if neighbour not in adjacency:
					continue
				segment_key = normalise_grid_segment(*cell_side_segment(cell, side))
				if segment_key not in wall_keys:
					adjacency[cell].append(neighbour)
					adjacency[neighbour].append(cell)
	return adjacency


def shortest_path_distances(adjacency, start_cell):
	"""Return unweighted shortest-path distances from ``start_cell``."""
	distances = {start_cell: 0}
	queue = deque([start_cell])
	while queue:
		cell = queue.popleft()
		for neighbour in adjacency[cell]:
			if neighbour in distances:
				continue
			distances[neighbour] = distances[cell] + 1
			queue.append(neighbour)
	return distances


def _open_side_for_dead_end(rows, columns, wall_keys, cell):
	walls = get_cell_wall_sides(rows, columns, wall_keys, cell)
	open_sides = [side for side in DIRECTIONS if not walls[side]]
	if len(open_sides) != 1:
		return None
	return open_sides[0]


def _marker_assignments_for_dead_ends(rows, columns, wall_keys, adjacency):
	"""Return terminal marker wall keys and reject two markers sharing one wall."""
	marker_segments = {}
	marker_sides = {}
	for cell, neighbours in adjacency.items():
		if len(neighbours) != 1:
			continue
		open_side = _open_side_for_dead_end(rows, columns, wall_keys, cell)
		if open_side is None:
			raise ValueError(f"Dead-end topology and wall sides disagree at cell {cell}")
		marker_side = OPPOSITE_DIRECTION[open_side]
		marker_key = normalise_grid_segment(*cell_side_segment(cell, marker_side))
		if marker_key in marker_segments:
			raise ValueError(
				f"Dead-end marker wall conflict: cells {marker_segments[marker_key]} "
				f"and {cell} would share wall {marker_key}")
		marker_segments[marker_key] = cell
		marker_sides[cell] = marker_side
	return marker_sides


def validate_maze_layout(layout):
	"""Validate challenge constraints and return useful derived topology details.

	Configured victim levels must be the ordered prefix L1, L1/L2, or L1/L2/L3. L1
	must lie in the short portion of the route-distance range, L2 (when present) in the
	middle portion, and L3 (when present) must be farther than both. Every generated
	victim and the base must be a dead end.
	"""
	if layout.rows <= 0 or layout.columns <= 0:
		raise ValueError(
			f"Maze rows and columns must be positive (got {layout.rows} x {layout.columns})")

	wall_keys = validate_wall_segments(
		layout.rows, layout.columns, layout.wall_segments)
	base_cell = _validate_cell(
		layout.base_cell, layout.rows, layout.columns, 'Base cell')

	victim_count = validate_victim_count(len(layout.victim_cells))
	configured_levels = VICTIM_LEVELS[:victim_count]
	labels = set(layout.victim_cells)
	expected_labels = set(configured_levels)
	if labels != expected_labels:
		raise ValueError(
			"victim_cells must contain an ordered level prefix: L1, L1/L2, or "
			f"L1/L2/L3 (got {sorted(labels)})")

	victim_cells = {}
	for label in configured_levels:
		victim_cells[label] = _validate_cell(
			layout.victim_cells[label], layout.rows, layout.columns,
			f"Victim '{label}' cell")
	if len(set(victim_cells.values())) != victim_count:
		raise ValueError('Victim cells must be unique')
	if base_cell in victim_cells.values():
		raise ValueError(f"Base cell {base_cell} must not coincide with a victim cell")

	adjacency = build_open_adjacency(layout.rows, layout.columns, wall_keys)
	distances = shortest_path_distances(adjacency, base_cell)
	cell_count = layout.rows * layout.columns
	if len(distances) != cell_count:
		raise ValueError(
			f"Maze must be fully connected: {len(distances)} of {cell_count} cells "
			f"are reachable from base {base_cell}")
	if len(adjacency[base_cell]) != 1:
		raise ValueError(
			f"Base cell {base_cell} must have exactly one entry/path "
			f"(found {len(adjacency[base_cell])})")

	for label, cell in victim_cells.items():
		if len(adjacency[cell]) != 1:
			raise ValueError(
				f"Victim '{label}' at cell {cell} must have one entry and three walls "
				f"(found {len(adjacency[cell])} entries)")

	victim_distances = {
		label: distances[cell] for label, cell in victim_cells.items()
	}
	ordered_distances = [victim_distances[label] for label in configured_levels]
	if any(
			first >= second
			for first, second in zip(ordered_distances, ordered_distances[1:])):
		raise ValueError(
			"Victim shortest-path distances must increase with victim level "
			f"(got {victim_distances})")

	# These deliberately broad bands preserve the approved preset while preventing a
	# random selection that calls almost-equally-distant leaves short/medium/long.
	maximum_route_distance = max(distances.values())
	distance_l1 = victim_distances['L1']
	short_upper = max(4, math.ceil(maximum_route_distance * 0.40))
	if distance_l1 > short_upper:
		raise ValueError(
			f"L1 victim is not in the short-distance band: distance {distance_l1}, "
			f"maximum {short_upper}")
	if 'L2' in victim_distances:
		distance_l2 = victim_distances['L2']
		medium_lower = max(
			distance_l1 + 1, math.floor(maximum_route_distance * 0.40))
		medium_upper = max(
			medium_lower, math.ceil(maximum_route_distance * 0.80))
		if not medium_lower <= distance_l2 <= medium_upper:
			raise ValueError(
				f"L2 victim is not in the medium-distance band: distance {distance_l2}, "
				f"expected [{medium_lower}, {medium_upper}]")

	marker_sides = _marker_assignments_for_dead_ends(
		layout.rows, layout.columns, wall_keys, adjacency)
	hazard_cells = sorted(
		cell for cell, neighbours in adjacency.items()
		if len(neighbours) == 1 and
		cell != base_cell and cell not in victim_cells.values())

	return {
		'wall_keys': wall_keys,
		'adjacency': adjacency,
		'distances_from_base': distances,
		'victim_distances': victim_distances,
		'dead_end_marker_sides': marker_sides,
		'hazard_cells': hazard_cells,
	}


def create_preset_maze(victim_count=3):
	"""Return the original approved 7 x 7 maze as a :class:`MazeLayout`."""
	victim_count = validate_victim_count(victim_count)
	layout = MazeLayout(
		rows=7,
		columns=7,
		wall_segments=list(PRESET_MAZE_SEGMENTS),
		base_cell=PRESET_BASE_CELL,
		victim_cells={
			label: PRESET_VICTIM_CELLS[label]
			for label in VICTIM_LEVELS[:victim_count]
		},
		mode='preset',
		seed=None,
	)
	details = validate_maze_layout(layout)
	layout.distances_from_base = details['distances_from_base']
	layout.hazard_cells = details['hazard_cells']
	return layout


class _DisjointSet:
	def __init__(self, values):
		self.parent = {value: value for value in values}
		self.rank = {value: 0 for value in values}

	def find(self, value):
		parent = self.parent[value]
		if parent != value:
			self.parent[value] = self.find(parent)
		return self.parent[value]

	def union(self, first, second):
		first_root = self.find(first)
		second_root = self.find(second)
		if first_root == second_root:
			return False
		if self.rank[first_root] < self.rank[second_root]:
			first_root, second_root = second_root, first_root
		self.parent[second_root] = first_root
		if self.rank[first_root] == self.rank[second_root]:
			self.rank[first_root] += 1
		return True


def _all_neighbour_edges(rows, columns, excluded_cell=None):
	cells = {
		(column, row)
		for row in range(rows)
		for column in range(columns)
		if (column, row) != excluded_cell
	}
	edges = []
	# Stable ordering before shuffling makes an integer seed reproducible across
	# processes and Python hash-table layouts, not merely within one interpreter.
	for cell in sorted(cells, key=lambda value: (value[1], value[0])):
		for delta_column, delta_row in ((1, 0), (0, 1)):
			neighbour = (cell[0] + delta_column, cell[1] + delta_row)
			if neighbour in cells:
				edges.append((cell, neighbour))
	return cells, edges


def _random_open_tree(rows, columns, base_cell, base_open_side, rng):
	"""Generate a random spanning tree while forcing the base to have degree one."""
	delta_column, delta_row = DIRECTION_DELTAS[base_open_side]
	base_neighbour = (
		base_cell[0] + delta_column,
		base_cell[1] + delta_row,
	)
	_validate_cell(base_neighbour, rows, columns, 'Base opening neighbour')

	cells, candidate_edges = _all_neighbour_edges(
		rows, columns, excluded_cell=base_cell)
	if not cells:
		raise ValueError('Random maze requires at least one non-base cell')
	rng.shuffle(candidate_edges)
	sets = _DisjointSet(cells)
	open_edges = set()
	for first, second in candidate_edges:
		if sets.union(first, second):
			open_edges.add(tuple(sorted((first, second))))
			if len(open_edges) == len(cells) - 1:
				break
	if len(open_edges) != len(cells) - 1:
		raise ValueError(
			"Removing the configured base cell disconnects this grid; choose a boundary "
			"base cell or a larger maze")
	open_edges.add(tuple(sorted((base_cell, base_neighbour))))
	return open_edges


def _walls_from_open_edges(rows, columns, open_edges):
	wall_segments = []
	for row in range(rows):
		for column in range(columns):
			cell = (column, row)
			for side in ('E', 'S'):
				delta_column, delta_row = DIRECTION_DELTAS[side]
				neighbour = (column + delta_column, row + delta_row)
				if not (0 <= neighbour[0] < columns and 0 <= neighbour[1] < rows):
					continue
				if tuple(sorted((cell, neighbour))) not in open_edges:
					wall_segments.append(cell_side_segment(cell, side))
	return wall_segments


def _choose_victim_cells(adjacency, distances, base_cell, rng):
	dead_ends = [
		cell for cell, neighbours in adjacency.items()
		if cell != base_cell and len(neighbours) == 1
	]
	if len(dead_ends) < 3:
		return None

	minimum_distance = min(distances[cell] for cell in dead_ends)
	maximum_distance = max(distances[cell] for cell in dead_ends)
	level_one_options = [
		cell for cell in dead_ends if distances[cell] == minimum_distance
	]
	level_three_options = [
		cell for cell in dead_ends if distances[cell] == maximum_distance
	]
	level_one = rng.choice(level_one_options)
	level_three = rng.choice(level_three_options)
	if level_one == level_three:
		return None

	short_upper = max(4, math.ceil(maximum_distance * 0.40))
	if minimum_distance > short_upper:
		return None
	medium_lower = max(minimum_distance + 1, math.floor(maximum_distance * 0.40))
	medium_upper = max(medium_lower, math.ceil(maximum_distance * 0.80))
	level_two_options = [
		cell for cell in dead_ends
		if cell not in (level_one, level_three) and
		medium_lower <= distances[cell] <= medium_upper
	]
	if not level_two_options:
		return None
	medium_target = (minimum_distance + maximum_distance) / 2.0
	best_error = min(
		abs(distances[cell] - medium_target) for cell in level_two_options)
	level_two = rng.choice([
		cell for cell in level_two_options
		if abs(distances[cell] - medium_target) == best_error
	])
	if not minimum_distance < distances[level_two] < maximum_distance:
		return None
	return {
		'L1': level_one,
		'L2': level_two,
		'L3': level_three,
	}


def generate_random_maze(
		rows=7, columns=7, base_cell=PRESET_BASE_CELL,
		base_open_side='N', seed=None, maximum_attempts=1000,
		victim_count=3):
	"""Generate a connected random challenge maze.

	A randomized Kruskal spanning tree supplies the topology.  The base is removed while
	the tree is made and attached afterward through one forced opening, which guarantees
	that it has only one path.  Victims are selected from dead ends by shortest-path
	distance, and layouts with ambiguous shared marker walls are discarded.
	"""
	if not (isinstance(rows, int) and isinstance(columns, int)):
		raise ValueError('Random maze rows and columns must be integers')
	victim_count = validate_victim_count(victim_count)
	if rows <= 1 or columns <= 1 or rows * columns < 8:
		raise ValueError(
			'Random challenge mazes require at least 8 cells and at least 2 rows/columns')
	base_cell = _validate_cell(base_cell, rows, columns, 'Base cell')
	base_open_side = str(base_open_side).upper()
	if base_open_side not in DIRECTIONS:
		raise ValueError(
			f"base_open_side must be one of {DIRECTIONS} (got {base_open_side!r})")
	if not isinstance(maximum_attempts, int) or maximum_attempts <= 0:
		raise ValueError('maximum_attempts must be a positive integer')
	if seed is not None and (not isinstance(seed, int) or isinstance(seed, bool)):
		raise ValueError('Random maze seed must be an integer or None')
	actual_seed = (
		random.SystemRandom().getrandbits(64) if seed is None else seed
	)
	rng = random.Random(actual_seed)

	last_reason = 'no suitable victim dead ends were available'
	for attempt in range(1, maximum_attempts + 1):
		open_edges = _random_open_tree(
			rows, columns, base_cell, base_open_side, rng)
		wall_segments = _walls_from_open_edges(rows, columns, open_edges)
		wall_keys = {normalise_grid_segment(*segment) for segment in wall_segments}
		adjacency = build_open_adjacency(rows, columns, wall_keys)
		distances = shortest_path_distances(adjacency, base_cell)
		all_victim_cells = _choose_victim_cells(
			adjacency, distances, base_cell, rng)
		if all_victim_cells is None:
			continue
		victim_cells = {
			label: all_victim_cells[label]
			for label in VICTIM_LEVELS[:victim_count]
		}

		layout = MazeLayout(
			rows=rows,
			columns=columns,
			wall_segments=wall_segments,
			base_cell=base_cell,
			victim_cells=victim_cells,
			mode='random',
			seed=actual_seed,
			generation_attempt=attempt,
		)
		try:
			details = validate_maze_layout(layout)
		except ValueError as error:
			last_reason = str(error)
			continue
		layout.distances_from_base = details['distances_from_base']
		layout.hazard_cells = details['hazard_cells']
		return layout

	raise RuntimeError(
		f"Could not generate a valid {rows} x {columns} challenge maze after "
		f"{maximum_attempts} attempts (seed {actual_seed}); last rejection: {last_reason}")
