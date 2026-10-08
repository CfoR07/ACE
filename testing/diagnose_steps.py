import sys
sys.path.insert(0, r"c:\Users\LENOVO\Documents\antigravity\serene-pythagoras")

from functions import FirefightingSimulation, manhattan_distance

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

print("=== STEP-BY-STEP TRACE OF PRESET 1 ===")
for step in range(1, 50):
    if sim.is_completed:
        print(f"Simulation completed at step {step}!")
        break
    pos_before = sim.agent_pos
    sim.step()
    pos_after = sim.agent_pos
    
    dr = pos_after[0] - pos_before[0]
    dc = pos_after[1] - pos_before[1]
    is_diag = (dr != 0 and dc != 0)
    dist = manhattan_distance(pos_before, pos_after)
    
    flag = ""
    if is_diag:
        flag += " [DIAGONAL LEAP!]"
    if dist > 1:
        flag += f" [JUMP DIST={dist}!]"
    if "Backtracking" in sim.last_action and len(sim.active_fires) > 0:
        flag += f" [BACKTRACKING WHILE {len(sim.active_fires)} FIRES BURNING!]"
        
    print(f"Step {step:02d} | Tick {sim.ticks:02d} | Pos: {pos_before} -> {pos_after} (dr={dr:+d}, dc={dc:+d}) | Goal: {sim.current_goal:<18} | StackDepth={len(sim.dfs_stack)} | ActiveFires={len(sim.active_fires)} | PlannedPathLen={len(sim.planned_path)}{flag}")
    print(f"        Action: {sim.last_action}")
