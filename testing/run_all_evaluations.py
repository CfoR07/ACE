import os
import sys
import shutil
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

sys.path.insert(0, r"c:\Users\LENOVO\Documents\antigravity\serene-pythagoras")
from functions import FirefightingSimulation, astar_search, manhattan_distance, DIRS_4, DIRS_8

TESTING_DIR = r"c:\Users\LENOVO\Documents\antigravity\serene-pythagoras\testing"
SCREENSHOTS_DIR = os.path.join(TESTING_DIR, "screenshots")
os.makedirs(SCREENSHOTS_DIR, exist_ok=True)

# Render grid frame mimicking Streamlit UI
def render_frame(sim, test_name, step_idx, out_path):
    rows, cols = sim.rows, sim.cols
    fig, ax = plt.subplots(figsize=(max(4, cols * 0.7), max(4, rows * 0.7)), dpi=100)
    ax.set_facecolor('#0e1117')
    fig.patch.set_facecolor('#0e1117')
    
    # Colors matching Streamlit CSS
    C_WALL = '#212631'
    C_FLOOR = '#1f6b47'
    C_STATION = '#1e3a5f'
    C_AGENT = '#1d4ed8'
    C_FIRE = '#e65100'
    C_SPREAD = '#880e4f'
    C_EXT = '#374151'
    
    for r in range(rows):
        for c in range(cols):
            pos = (r, c)
            val = sim.grid[r][c]
            
            # Determine color and text
            if pos == sim.agent_pos:
                color = C_AGENT
                txt = "🤖"
            elif pos in sim.active_fires:
                color = C_SPREAD if hasattr(sim, 'spread_fires') and pos in sim.spread_fires else C_FIRE
                txt = "🔥"
            elif pos == sim.station_pos:
                color = C_STATION
                txt = "🏠"
            elif pos in sim.extinguished_history:
                color = C_EXT
                txt = str(val) if val > 0 else ""
            elif val == 0:
                color = C_WALL
                txt = ""
            else:
                color = C_FLOOR
                txt = str(val)
                
            rect = patches.Rectangle((c, rows - 1 - r), 1, 1, linewidth=1, edgecolor='#232936', facecolor=color)
            ax.add_patch(rect)
            if txt:
                ax.text(c + 0.5, rows - 1 - r + 0.5, txt, color='white', fontsize=12,
                        ha='center', va='center', fontweight='bold', fontname='Segoe UI Emoji')
                
    ax.set_xlim(0, cols)
    ax.set_ylim(0, rows)
    ax.set_xticks(range(cols))
    ax.set_yticks(range(rows))
    ax.set_xticklabels(range(cols), color='#a0aec0', fontsize=8)
    ax.set_yticklabels([str(rows - 1 - y) for y in range(rows)], color='#a0aec0', fontsize=8)
    ax.grid(color='#232936', linestyle=':', linewidth=0.5)
    
    path_len = len(getattr(sim, 'planned_path', []))
    stack_depth = len(getattr(sim, 'dfs_stack', []))
    title = f"{test_name} | Step {step_idx:02d} | Tick {sim.ticks:02d} | PathLen: {path_len} | Stack: {stack_depth}\nGoal: {sim.current_goal} | Action: {sim.last_action}"
    ax.set_title(title, color='white', fontsize=8, pad=10)
    plt.tight_layout()
    plt.savefig(out_path, facecolor=fig.get_facecolor(), edgecolor='none')
    plt.close(fig)

def run_evaluation(test_name, grid, station, fires, spread_rate, single_cost, splash_cost, max_steps=120):
    case_folder = os.path.join(SCREENSHOTS_DIR, test_name)
    shutil.rmtree(case_folder, ignore_errors=True)
    os.makedirs(case_folder, exist_ok=True)
    log_file = os.path.join(TESTING_DIR, f"{test_name}_log.txt")
    
    sim = FirefightingSimulation(
        grid=grid,
        station_pos=station,
        fire_sources=fires,
        fire_spread_interval=spread_rate,
        single_cost=single_cost,
        splash_cost=splash_cost
    )
    
    logs = []
    logs.append(f"==================================================")
    logs.append(f"TEST CASE: {test_name}")
    logs.append(f"Grid: {sim.rows}x{sim.cols} | Station: {station} | Fires: {fires}")
    logs.append(f"Spread Rate: {spread_rate}t | Single: {single_cost}t | Splash: {splash_cost}t")
    logs.append(f"==================================================\n")
    
    step_idx = 0
    # Save initial frame
    render_frame(sim, test_name, step_idx, os.path.join(case_folder, f"step_{step_idx:02d}.png"))
    logs.append(f"Step {step_idx:02d} | Tick {sim.ticks:02d} | Pos: {sim.agent_pos} | PlannedPathLen: {len(sim.planned_path)} | Active: {list(sim.active_fires.keys())} | Stack: {sim.dfs_stack} | Goal: {sim.current_goal} | Action: {sim.last_action}")
    
    stuck_counter = 0
    prev_pos = sim.agent_pos
    prev_active = set(sim.active_fires.keys())
    
    diagonal_moves_detected = []
    freeze_detected = False
    premature_completion_detected = False
    
    while not sim.is_completed and step_idx < max_steps:
        step_idx += 1
        pos_before = sim.agent_pos
        sim.step()
        
        # Check diagonal or teleport move:
        man_dist = manhattan_distance(pos_before, sim.agent_pos)
        if man_dist > 1:
            # Check if agent changed pos by > 1 in a single step (diagonal or jump)
            diagonal_moves_detected.append({
                'step': step_idx,
                'from': pos_before,
                'to': sim.agent_pos,
                'dist': man_dist,
                'action': sim.last_action
            })
            
        render_frame(sim, test_name, step_idx, os.path.join(case_folder, f"step_{step_idx:02d}.png"))
        path_len = len(getattr(sim, 'planned_path', []))
        log_line = f"Step {step_idx:02d} | Tick {sim.ticks:02d} | Pos: {sim.agent_pos} | Dist: {sim.total_distance} | PlannedPathLen: {path_len} | ActiveFires: {len(sim.active_fires)} {list(sim.active_fires.keys())} | SourcesLeft: {len(sim.fire_sources)} | Stack: {sim.dfs_stack} | Goal: {sim.current_goal} | Action: {sim.last_action}"
        logs.append(log_line)
        
        # Check if stuck
        curr_active = set(sim.active_fires.keys())
        if sim.agent_pos == pos_before and curr_active == prev_active and "Extinguish" not in sim.last_action:
            stuck_counter += 1
            if stuck_counter >= 5:
                freeze_detected = True
                logs.append(f">>> [ERROR] SIMULATION FROZEN / STUCK at step {step_idx}! Agent stationary at {sim.agent_pos} while fires remain: {curr_active}")
                break
        else:
            stuck_counter = 0
            
        prev_pos = sim.agent_pos
        prev_active = curr_active
        
    if sim.is_completed and (len(sim.active_fires) > 0 or len(sim.fire_sources) > 0):
        premature_completion_detected = True
        logs.append(f">>> [ERROR] PREMATURE COMPLETION! Sim claimed completed, but active_fires={list(sim.active_fires.keys())}, fire_sources={sim.fire_sources}")
        
    logs.append(f"\n--- SUMMARY FOR {test_name} ---")
    logs.append(f"Completed: {sim.is_completed}")
    logs.append(f"Total Steps Taken: {step_idx}")
    logs.append(f"Total Ticks: {sim.ticks}")
    logs.append(f"Total Distance: {sim.total_distance}")
    logs.append(f"Fires Extinguished: {sim.fires_extinguished_count}")
    logs.append(f"Freeze Detected: {freeze_detected}")
    logs.append(f"Premature Completion: {premature_completion_detected}")
    logs.append(f"Diagonal / Non-Orthogonal Jumps Count: {len(diagonal_moves_detected)}")
    for d in diagonal_moves_detected:
        logs.append(f"  * Step {d['step']}: Jump from {d['from']} to {d['to']} (Manhattan distance = {d['dist']}) during '{d['action']}'")
        
    with open(log_file, "w", encoding="utf-8") as f:
        f.write("\n".join(logs))
        
    print(f"Finished {test_name}: Steps={step_idx}, Ticks={sim.ticks}, Frozen={freeze_detected}, PrematureDone={premature_completion_detected}, DiagonalJumps={len(diagonal_moves_detected)}")
    return {
        'name': test_name,
        'completed': sim.is_completed,
        'steps': step_idx,
        'ticks': sim.ticks,
        'distance': sim.total_distance,
        'frozen': freeze_detected,
        'premature': premature_completion_detected,
        'diagonal_jumps': diagonal_moves_detected
    }

# ----------------- RUN TESTCASES -----------------
results = []

# 1. Preset 1: Small Office (6x6)
grid1 = [
    [0, 0, 0, 0, 0, 0],
    [0, 15, 2, 3, 2, 0],
    [0, 1, 0, 0, 4, 0],
    [0, 2, 1, 5, 3, 0],
    [0, 1, 8, 9, 2, 0],
    [0, 0, 0, 0, 0, 0]
]
fires1 = [(3, 4), (4, 2), (1, 3), (2, 4), (3, 1), (4, 3)]
results.append(run_evaluation("Preset_1_Small_Office", grid1, (1, 1), fires1, spread_rate=8, single_cost=2, splash_cost=4))

# 2. Preset 2: Multi-Room Lab (8x8)
grid2 = [
    [0, 0, 0, 0, 0, 0, 0, 0],
    [0, 15, 2, 2, 0, 4, 5, 0],
    [0, 1, 1, 2, 0, 3, 4, 0],
    [0, 0, 3, 0, 0, 2, 0, 0],
    [0, 5, 6, 7, 2, 1, 8, 0],
    [0, 4, 0, 0, 0, 0, 9, 0],
    [0, 3, 2, 1, 2, 8, 10, 0],
    [0, 0, 0, 0, 0, 0, 0, 0]
]
fires2 = [(4, 1), (6, 6), (2, 5), (4, 6), (5, 6), (6, 3)]
results.append(run_evaluation("Preset_2_Lab_8x8", grid2, (1, 1), fires2, spread_rate=10, single_cost=2, splash_cost=5))

# 3. Preset 3: Warehouse & Storage (10x10)
grid3 = [
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
fires3 = [(4, 4), (3, 6), (8, 2), (6, 3), (1, 8), (4, 6), (6, 5)]
results.append(run_evaluation("Preset_3_Warehouse_10x10", grid3, (1, 1), fires3, spread_rate=12, single_cost=2, splash_cost=6))

# 4. Edge Case: Blocked High-Priority Corridor (Reproduce Freezing Bug)
grid4 = [
    [15, 1, 1, 1, 1, 10],
    [0,  0, 0, 0, 0, 0]
]
fires4 = [(0, 3), (0, 5)]
results.append(run_evaluation("Edge_Case_Blocked_High_Priority", grid4, (0, 0), fires4, spread_rate=50, single_cost=2, splash_cost=4))

# 5. Edge Case: Diagonal Fire Step-In (Test Diagonal vs 4-Directional)
grid5 = [
    [15, 1, 1, 1],
    [1,  5, 1, 1],
    [1,  1, 1, 1]
]
fires5 = [(1, 1)]  # (1, 1) is diagonal to station (0, 0)
results.append(run_evaluation("Edge_Case_Diagonal_StepIn", grid5, (0, 0), fires5, spread_rate=50, single_cost=2, splash_cost=4))

# 6. Edge Case: Delayed Spawn Cutoff (Test Premature Finish with Pending Fires)
grid6 = [
    [15, 1, 1, 1],
    [1,  1, 1, 1]
]
fires6 = [(0, 2), (1, 3)]
results.append(run_evaluation("Edge_Case_Delayed_Spawn_Cutoff", grid6, (0, 0), fires6, spread_rate=50, single_cost=2, splash_cost=4))

print("\nAll evaluation tests completed! Logs and screenshots stored in:", TESTING_DIR)
