import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from functions import FirefightingSimulation, manhattan_distance

# Define candidate test scenarios (around 25 total)
SCENARIOS = []

# --- PRESET 1 CANDIDATES (6x6) ---
p1_grid = [
    [0, 0, 0, 0, 0, 0],
    [0, 15, 2, 3, 2, 0],
    [0, 1, 0, 0, 4, 0],
    [0, 2, 1, 5, 3, 0],
    [0, 1, 8, 9, 2, 0],
    [0, 0, 0, 0, 0, 0]
]

p1_fire_candidates = [
    # C01: Slide 9 pure (Server Room 8, Office 3 & 4, Corridor 3)
    {"name": "P1_C01_Slide9_Canonical", "grid": p1_grid, "station": (1, 1), "fires": [(4, 2), (3, 4), (2, 4), (1, 3)], "spread": 10, "single": 2, "splash": 4},
    # C02: Current Preset 1 with room fires (no hallway fire at 3,1)
    {"name": "P1_C02_RoomFires_Dynamic", "grid": p1_grid, "station": (1, 1), "fires": [(4, 2), (3, 4), (2, 4), (1, 3), (4, 3), (4, 4)], "spread": 8, "single": 2, "splash": 4},
    # C03: Server Room + Double Office Cluster + Lab
    {"name": "P1_C03_Cluster_Focus", "grid": p1_grid, "station": (1, 1), "fires": [(4, 2), (4, 3), (3, 4), (2, 4), (1, 3)], "spread": 9, "single": 2, "splash": 4},
    # C04: Dual Room Clusters
    {"name": "P1_C04_Dual_Room", "grid": p1_grid, "station": (1, 1), "fires": [(4, 2), (3, 4), (2, 4), (1, 4), (4, 3)], "spread": 8, "single": 2, "splash": 4},
    # C05: Slide 9 with delayed spawn
    {"name": "P1_C05_Slide9_Delayed", "grid": p1_grid, "station": (1, 1), "fires": [(4, 2), (3, 4), (1, 3), (2, 4), (4, 3)], "spread": 8, "single": 2, "splash": 4},
    # C06: High spread test
    {"name": "P1_C06_HighSpread", "grid": p1_grid, "station": (1, 1), "fires": [(4, 2), (3, 4), (1, 3), (2, 4)], "spread": 6, "single": 2, "splash": 4},
    # C07: Standard Preset 1 as currently in app
    {"name": "P1_C07_CurrentApp", "grid": p1_grid, "station": (1, 1), "fires": [(3, 4), (4, 2), (1, 3), (2, 4), (3, 1), (4, 3)], "spread": 8, "single": 2, "splash": 4},
    # C08: Compact clean office run
    {"name": "P1_C08_CleanOffice", "grid": p1_grid, "station": (1, 1), "fires": [(4, 2), (3, 4), (2, 4), (1, 3), (4, 1)], "spread": 10, "single": 2, "splash": 4},
]
SCENARIOS.extend(p1_fire_candidates)

# --- PRESET 2 CANDIDATES (8x8) ---
p2_grid = [
    [0, 0, 0, 0, 0, 0, 0, 0],
    [0, 15, 2, 2, 0, 4, 5, 0],
    [0, 1, 1, 2, 0, 3, 4, 0],
    [0, 0, 3, 0, 0, 2, 0, 0],
    [0, 5, 6, 7, 2, 1, 8, 0],
    [0, 4, 0, 0, 0, 0, 9, 0],
    [0, 3, 2, 1, 2, 8, 10, 0],
    [0, 0, 0, 0, 0, 0, 0, 0]
]

p2_fire_candidates = [
    # C09: Current app Preset 2
    {"name": "P2_C09_CurrentApp", "grid": p2_grid, "station": (1, 1), "fires": [(4, 1), (6, 6), (2, 5), (4, 6), (5, 6), (6, 3)], "spread": 10, "single": 2, "splash": 5},
    # C10: High Priority Hazard Lab Focus (6,6=10, 5,6=9, 6,5=8)
    {"name": "P2_C10_HazardLab_Cluster", "grid": p2_grid, "station": (1, 1), "fires": [(6, 6), (5, 6), (6, 5), (4, 2), (2, 5)], "spread": 10, "single": 2, "splash": 4},
    # C11: Multi-Zone Sweep (Hazard Lab + Central Lab + Upper Lab)
    {"name": "P2_C11_MultiZone_Sweep", "grid": p2_grid, "station": (1, 1), "fires": [(6, 6), (4, 3), (2, 6), (5, 6), (4, 2), (1, 6)], "spread": 12, "single": 2, "splash": 4},
    # C12: High Splash Opportunities in Lab
    {"name": "P2_C12_LabSplash_Focus", "grid": p2_grid, "station": (1, 1), "fires": [(6, 6), (5, 6), (4, 6), (4, 2), (4, 3), (2, 5)], "spread": 10, "single": 2, "splash": 4},
    # C13: Priority 10 + Priority 7 + Priority 5
    {"name": "P2_C13_Priority_Hierarchy", "grid": p2_grid, "station": (1, 1), "fires": [(6, 6), (4, 3), (1, 6), (5, 6), (4, 1)], "spread": 10, "single": 2, "splash": 4},
    # C14: Deep Room Exploration Lab
    {"name": "P2_C14_DeepRoom_Exploration", "grid": p2_grid, "station": (1, 1), "fires": [(6, 6), (6, 5), (4, 3), (4, 2), (2, 6), (2, 5)], "spread": 12, "single": 2, "splash": 4},
    # C15: Fast Lab Containment
    {"name": "P2_C15_Fast_Containment", "grid": p2_grid, "station": (1, 1), "fires": [(6, 6), (4, 2), (2, 5), (5, 6)], "spread": 10, "single": 2, "splash": 4},
    # C16: Spread Heavy Lab
    {"name": "P2_C16_SpreadHeavy_Lab", "grid": p2_grid, "station": (1, 1), "fires": [(6, 6), (4, 3), (2, 5), (5, 6), (4, 1)], "spread": 8, "single": 2, "splash": 4},
]
SCENARIOS.extend(p2_fire_candidates)

# --- PRESET 3 CANDIDATES (10x10) ---
p3_grid = [
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 15, 2, 3, 1, 0, 2, 3, 4, 0],
    [0, 2, 0, 0, 2, 0, 3, 0, 5, 0],
    [0, 3, 0, 8, 7, 6, 5, 0, 6, 0],
    [0, 2, 0, 9, 10, 8, 4, 0, 7, 0],
    [0, 1, 0, 0, 0, 0, 3, 0, 8, 0],
    [0, 2, 3, 4, 5, 2, 1, 0, 9, 0],
    [0, 0, 2, 0, 0, 3, 2, 0, 5, 0],
    [0, 1, 1, 2, 3, 4, 5, 6, 4, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
]

p3_fire_candidates = [
    # C17: Current app Preset 3
    {"name": "P3_C17_CurrentApp", "grid": p3_grid, "station": (1, 1), "fires": [(4, 4), (3, 6), (8, 2), (6, 3), (1, 8), (4, 6), (6, 5)], "spread": 12, "single": 2, "splash": 6},
    # C18: Central High-Value Vault Cluster (4,4=10, 4,3=9, 3,3=8, 4,5=8)
    {"name": "P3_C18_CentralVault_Cluster", "grid": p3_grid, "station": (1, 1), "fires": [(4, 4), (4, 3), (3, 4), (6, 4), (1, 8), (8, 6)], "spread": 12, "single": 2, "splash": 5},
    # C19: Balanced Warehouse Zones
    {"name": "P3_C19_Balanced_Zones", "grid": p3_grid, "station": (1, 1), "fires": [(4, 4), (3, 5), (6, 3), (8, 5), (1, 8), (3, 8)], "spread": 12, "single": 2, "splash": 5},
    # C20: Splash Heavy Central Storage
    {"name": "P3_C20_SplashHeavy_Storage", "grid": p3_grid, "station": (1, 1), "fires": [(4, 4), (3, 4), (4, 5), (3, 5), (6, 4), (8, 5)], "spread": 12, "single": 2, "splash": 4},
    # C21: Deep Aisle Navigation
    {"name": "P3_C21_DeepAisle_Run", "grid": p3_grid, "station": (1, 1), "fires": [(4, 4), (4, 3), (6, 3), (8, 2), (3, 6), (6, 8)], "spread": 14, "single": 2, "splash": 5},
    # C22: High Spread Warehouse
    {"name": "P3_C22_HighSpread_Warehouse", "grid": p3_grid, "station": (1, 1), "fires": [(4, 4), (3, 5), (6, 3), (8, 6), (1, 8)], "spread": 10, "single": 2, "splash": 5},
    # C23: Warehouse Priority Showcase
    {"name": "P3_C23_Priority_Showcase", "grid": p3_grid, "station": (1, 1), "fires": [(4, 4), (4, 5), (6, 8), (8, 6), (3, 3)], "spread": 12, "single": 2, "splash": 5},
    # C24: Warehouse Perimeter + Core
    {"name": "P3_C24_Perimeter_Core", "grid": p3_grid, "station": (1, 1), "fires": [(4, 4), (3, 4), (1, 3), (6, 2), (8, 6)], "spread": 12, "single": 2, "splash": 5},
    # C25: Clean Sweep 10x10
    {"name": "P3_C25_CleanSweep", "grid": p3_grid, "station": (1, 1), "fires": [(4, 4), (4, 3), (3, 4), (6, 4), (8, 5), (1, 8)], "spread": 12, "single": 2, "splash": 4},
]
SCENARIOS.extend(p3_fire_candidates)

print(f"Total Scenarios to Evaluate: {len(SCENARIOS)}")

results = []

for sc in SCENARIOS:
    name = sc["name"]
    grid = sc["grid"]
    station = sc["station"]
    fires = sc["fires"]
    spread = sc["spread"]
    single_c = sc["single"]
    splash_c = sc["splash"]

    sim = FirefightingSimulation(grid, station, fires, fire_spread_interval=spread, single_cost=single_c, splash_cost=splash_c)
    
    steps = 0
    max_steps = 150
    splash_count = 0
    single_count = 0
    enter_count = 0
    backtrack_count = 0
    max_stack = 0
    in_fire_count = 0
    diagonal_moves = 0
    prev_pos = sim.agent_pos

    while not sim.is_completed and steps < max_steps:
        steps += 1
        curr_pos = sim.agent_pos
        sim.step()
        new_pos = sim.agent_pos

        # Check diagonal
        dr = abs(new_pos[0] - curr_pos[0])
        dc = abs(new_pos[1] - curr_pos[1])
        if dr > 0 and dc > 0:
            diagonal_moves += 1

        # Check burn
        if new_pos in sim.active_fires:
            in_fire_count += 1

        # Track actions
        action = sim.last_action
        if "Splash" in action:
            splash_count += 1
        elif "Single" in action:
            single_count += 1
        elif "Entering" in action:
            enter_count += 1
        elif "Backtracking" in action:
            backtrack_count += 1

        if len(sim.dfs_stack) > max_stack:
            max_stack = len(sim.dfs_stack)

    score_aesthetic = 0
    # Desirable traits:
    # 1. Has splash (at least 1 or 2)
    if splash_count >= 1: score_aesthetic += 20
    if splash_count >= 2: score_aesthetic += 10
    # 2. Has DFS enter & backtrack (at least 2)
    if backtrack_count >= 1: score_aesthetic += 15
    if backtrack_count >= 2: score_aesthetic += 10
    if max_stack >= 1: score_aesthetic += 10
    if max_stack >= 2: score_aesthetic += 10
    # 3. Has single extinguish
    if single_count >= 1: score_aesthetic += 10
    # 4. Completed cleanly
    if sim.is_completed: score_aesthetic += 25
    # Penalties:
    if in_fire_count > 0: score_aesthetic -= 100
    if diagonal_moves > 0: score_aesthetic -= 100
    if not sim.is_completed: score_aesthetic -= 50

    results.append({
        "name": name,
        "completed": sim.is_completed,
        "steps": steps,
        "ticks": sim.ticks,
        "dist": sim.total_distance,
        "extinguished": sim.fires_extinguished_count,
        "splash": splash_count,
        "single": single_count,
        "enter": enter_count,
        "backtrack": backtrack_count,
        "max_stack": max_stack,
        "burns": in_fire_count,
        "diagonal": diagonal_moves,
        "score": score_aesthetic
    })

# Print ranked results by preset
for group_name, prefix in [("Preset 1 (6x6)", "P1_"), ("Preset 2 (8x8)", "P2_"), ("Preset 3 (10x10)", "P3_")]:
    print(f"\n==================== {group_name} RANKINGS ====================")
    group_res = [r for r in results if r["name"].startswith(prefix)]
    group_res.sort(key=lambda x: (x["completed"], -x["burns"], x["score"], -x["steps"]), reverse=True)
    for r in group_res:
        fires_str = ""
        # Find fires from candidate
        cand = next(c for c in SCENARIOS if c["name"] == r["name"])
        f_str = "; ".join([f"{f[0]},{f[1]}" for f in cand["fires"]])
        print(f"[{r['name']}] Score: {r['score']} | Done: {r['completed']} | Steps: {r['steps']} | Ticks: {r['ticks']} | Splash: {r['splash']} | Single: {r['single']} | Backtrack: {r['backtrack']} | MaxStack: {r['max_stack']} | Burns: {r['burns']}")
        print(f"   Fires Config: \"{f_str}\" (spread={cand['spread']}, single={cand['single']}, splash={cand['splash']})")
