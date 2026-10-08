import sys
import os
sys.path.insert(0, r"c:\Users\LENOVO\Documents\antigravity\serene-pythagoras")

from functions import FirefightingSimulation, astar_search, find_safe_stand_pos, manhattan_distance, DIRS_4, DIRS_8

print("==================================================")
print("DEEP ANALYSIS & CODE INSPECTION SUITE")
print("==================================================")

# Test 1: Fire Spreading Rate Growth Analysis
def analyze_spread_rate():
    print("\n--- TEST 1: Fire Spreading Growth Analysis ---")
    grid = [[1]*8 for _ in range(8)]
    sim = FirefightingSimulation(grid, (0, 0), [(4, 4)], fire_spread_interval=8, single_cost=2, splash_cost=4)
    # Don't step agent, just trigger spread 4 times
    print(f"Initial: {len(sim.active_fires)} fires at {list(sim.active_fires.keys())}")
    for i in range(1, 5):
        sim.spread_fire()
        print(f"Spread cycle {i}: {len(sim.active_fires)} active fires!")

analyze_spread_rate()

# Test 2: Planned Path Stale Bug Analysis
def analyze_planned_path_stale():
    print("\n--- TEST 2: Planned Path Stale State Analysis ---")
    grid = [
        [1, 1, 1, 1],
        [1, 0, 0, 1],
        [1, 1, 1, 1]
    ]
    sim = FirefightingSimulation(grid, (0, 0), [(2, 3)], fire_spread_interval=100, single_cost=2, splash_cost=4)
    for s in range(1, 10):
        if sim.is_completed: break
        sim.step()
        print(f"Step {s:02d} | Pos: {sim.agent_pos} | Goal: {sim.current_goal:<18} | PlannedPath: {sim.planned_path} (len={len(sim.planned_path)}) | Action: {sim.last_action}")

analyze_planned_path_stale()

# Test 3: Diagonal Leap Verification
def analyze_diagonal_leap():
    print("\n--- TEST 3: Diagonal Move Verification ---")
    grid = [
        [1, 1, 1],
        [1, 1, 1],
        [1, 1, 1]
    ]
    # Agent at (0, 0), fire at (1, 1) [diagonal]
    sim = FirefightingSimulation(grid, (0, 0), [(1, 1)], fire_spread_interval=100, single_cost=2, splash_cost=4)
    sim.active_fires = {(1, 1): 0}
    print(f"Step 0: Agent at {sim.agent_pos}")
    sim.step() # Extinguishes from (0, 0)
    print(f"Step 1: Action: {sim.last_action} | Recently Extinguished: {sim.recently_extinguished}")
    sim.step() # Steps into extinguished block
    man_dist = manhattan_distance((0, 0), sim.agent_pos)
    print(f"Step 2: Agent moved from (0, 0) to {sim.agent_pos} | Manhattan Distance = {man_dist} | Action: {sim.last_action}")
    if man_dist == 2:
        print(">>> CONFIRMED: Agent leapt diagonally across 2 Manhattan blocks in 1 tick!")

analyze_diagonal_leap()
