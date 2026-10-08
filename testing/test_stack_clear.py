import sys
sys.path.insert(0, r"c:\Users\LENOVO\Documents\antigravity\serene-pythagoras")

from functions import FirefightingSimulation, astar_search, manhattan_distance

# Preset 1: Small Office
grid = [
    [0, 0, 0, 0, 0, 0],
    [0, 15, 2, 3, 2, 0],
    [0, 1, 0, 0, 4, 0],
    [0, 2, 1, 5, 3, 0],
    [0, 1, 8, 9, 2, 0],
    [0, 0, 0, 0, 0, 0]
]
fires = [(3, 4), (4, 2), (1, 3), (2, 4), (3, 1), (4, 3)]

sim = FirefightingSimulation(
    grid=grid,
    station_pos=(1, 1),
    fire_sources=fires,
    fire_spread_interval=8,
    single_cost=2,
    splash_cost=4
)

# Test behavior when all fires are dead:
sim.active_fires = {} # no fires
sim.dfs_stack = [(3, 1), (4, 2), (4, 3), (3, 4)]
sim.agent_pos = (1, 2)

print("Agent at (1, 2), Station at (1, 1). Active fires: 0. DFS Stack has 4 items.")
sim.step()
print("Next pos:", sim.agent_pos, "| Goal:", sim.current_goal, "| Action:", sim.last_action, "| PlannedPathLen:", len(sim.planned_path))
