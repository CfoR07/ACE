import sys
import os
sys.path.insert(0, r"c:\Users\LENOVO\Documents\antigravity\serene-pythagoras")

from functions import FirefightingSimulation, astar_search, find_safe_stand_pos

def test_blocked_target_deadlock():
    print("=== TEST: High Priority Fire Behind Another Fire ===")
    # Corridor with Fire A at (0, 3) and High Priority Fire B (priority 10) at (0, 5)
    # Agent at (0, 0). Fire A blocks the only way to Fire B.
    grid = [
        [1, 1, 1, 1, 1, 10],
        [0, 0, 0, 0, 0, 0]
    ]
    station = (0, 0)
    fires = [(0, 3), (0, 5)]

    sim = FirefightingSimulation(
        grid=grid,
        station_pos=station,
        fire_sources=fires,
        fire_spread_interval=100,
        single_cost=2,
        splash_cost=4
    )
    # Force both fires active
    sim.active_fires = {(0, 3): 0, (0, 5): 0}
    sim.fire_clusters = [
        {'id': 1, 'source': (0, 3), 'fires': {(0, 3)}, 'zone_priority': 1, 'spawn_tick': 0},
        {'id': 2, 'source': (0, 5), 'fires': {(0, 5)}, 'zone_priority': 10, 'spawn_tick': 0}
    ]

    for step in range(1, 15):
        pos_before = sim.agent_pos
        sim.step()
        print(f"Step {step:02d} | Pos: {sim.agent_pos} | Goal: {sim.current_goal} | Action: {sim.last_action} | Active: {list(sim.active_fires.keys())}")
        if sim.agent_pos == pos_before and not sim.is_completed and "Extinguish" not in sim.last_action:
            print(">>> DEADLOCK DETECTED! Agent is stuck at", sim.agent_pos, "even though fires remain!")
            break

test_blocked_target_deadlock()
