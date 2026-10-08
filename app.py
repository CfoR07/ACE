import streamlit as st
import time
import random
from functions import FirefightingSimulation

st.set_page_config(page_title="Autonomous Firefighting Agent", layout="wide")

# Custom Styling for Native Crisp Grid & Telemetry
st.markdown("""
<style>
    .reportview-container, .main, .block-container {
        background-color: #0e1117;
        color: #ffffff;
    }
    .grid-container {
        display: inline-block;
        background-color: #12161f;
        padding: 12px;
        border-radius: 12px;
        box-shadow: 0 8px 24px rgba(0,0,0,0.5);
        border: 1px solid #232936;
    }
    .grid-row {
        display: flex;
        gap: 6px;
        margin-bottom: 6px;
    }
    .grid-cell {
        width: 48px;
        height: 48px;
        border-radius: 8px;
        display: flex;
        align-items: center;
        justify-content: center;
        font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
        font-weight: 700;
        font-size: 20px;
        user-select: none;
        transition: transform 0.15s ease, background-color 0.2s ease;
        box-shadow: inset 0 0 0 1px rgba(255,255,255,0.06);
    }
    .grid-cell:hover {
        transform: scale(1.05);
    }
    /* Cell Types */
    .cell-wall {
        background-color: #212631;
        color: #4a5568;
        font-size: 13px;
    }
    .cell-floor {
        background-color: #1f6b47;
        color: #a7f3d0;
        font-size: 14px;
    }
    .cell-station {
        background-color: #1e3a5f;
        border: 2px solid #3b82f6;
    }
    .cell-agent {
        background-color: #1d4ed8;
        border: 2px solid #60a5fa;
        animation: pulse 1.5s infinite;
    }
    .cell-fire {
        background-color: #e65100;
        border: 2px solid #ff9800;
        animation: burn 1.2s infinite alternate;
    }
    .cell-fire-spread {
        background-color: #880e4f;
        border: 2px solid #e91e63;
        animation: burn 1.2s infinite alternate;
    }
    .cell-extinguished {
        background-color: #374151;
        color: #9ca3af;
        font-size: 13px;
    }
    @keyframes pulse {
        0% { box-shadow: 0 0 0 0 rgba(96, 165, 250, 0.7); }
        70% { box-shadow: 0 0 0 10px rgba(96, 165, 250, 0); }
        100% { box-shadow: 0 0 0 0 rgba(96, 165, 250, 0); }
    }
    @keyframes burn {
        0% { transform: scale(1); filter: brightness(1); }
        100% { transform: scale(1.04); filter: brightness(1.2); }
    }
    .status-card {
        background-color: #161b26;
        border-radius: 10px;
        padding: 16px;
        border: 1px solid #232936;
    }
    .metric-card {
        background-color: #161b26;
        border-radius: 10px;
        padding: 14px;
        border: 1px solid #232936;
        text-align: center;
    }
</style>
""", unsafe_allow_html=True)

# Preset configurations
PRESETS = {
    "Preset 1: Small Office (6x6)": {
        "grid": [
            [0, 0, 0, 0, 0, 0],
            [0, 15, 2, 3, 2, 0],
            [0, 1, 0, 0, 4, 0],
            [0, 2, 1, 5, 3, 0],
            [0, 1, 8, 9, 2, 0],
            [0, 0, 0, 0, 0, 0]
        ],
        "fires": "3,4; 4,2; 1,3; 2,4; 3,1; 4,3",
        "spread_rate": 8,
        "single_cost": 2,
        "splash_cost": 4
    },
    "Preset 2: Multi-Room Lab (8x8)": {
        "grid": [
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 15, 2, 2, 0, 4, 5, 0],
            [0, 1, 1, 2, 0, 3, 4, 0],
            [0, 0, 3, 0, 0, 2, 0, 0],
            [0, 5, 6, 7, 2, 1, 8, 0],
            [0, 4, 0, 0, 0, 0, 9, 0],
            [0, 3, 2, 1, 2, 8, 10, 0],
            [0, 0, 0, 0, 0, 0, 0, 0]
        ],
        "fires": "4,1; 6,6; 2,5; 4,6; 5,6; 6,3",
        "spread_rate": 10,
        "single_cost": 2,
        "splash_cost": 5
    },
    "Preset 3: Warehouse & Storage (10x10)": {
        "grid": [
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
        ],
        "fires": "4,4; 3,6; 8,2; 6,3; 1,8; 4,6; 6,5",
        "spread_rate": 12,
        "single_cost": 2,
        "splash_cost": 6
    }
}

# Sidebar: Controls & Configuration
st.sidebar.markdown("### Simulation Controls")
anim_speed = st.sidebar.slider("Step speed (seconds):", min_value=0.05, max_value=1.0, value=0.35, step=0.05)

ctrl_c1, ctrl_c2 = st.sidebar.columns(2)
start_run_btn = ctrl_c1.button("Start / Run", key="start_run")
reset_btn = ctrl_c2.button("Reset", key="reset_sim")
single_step_btn = st.sidebar.button("Single Step", key="single_step")

st.sidebar.markdown("---")
st.sidebar.markdown("### Custom Building & Fire Input")

preset_choice = st.sidebar.selectbox("Load Example Preset:", list(PRESETS.keys()), key="preset_choice_box")
default_vals = PRESETS[preset_choice]

# Check if preset changed to reset custom inputs
if "current_preset" not in st.session_state or st.session_state["current_preset"] != preset_choice:
    st.session_state["current_preset"] = preset_choice
    st.session_state["grid_area"] = "\n".join([" ".join(map(str, row)) for row in default_vals["grid"]])
    st.session_state["fires_input_field"] = default_vals["fires"]

with st.sidebar.expander("Edit Building Layout & Fires", expanded=False):
    st.caption("0 = Wall, 1-10 = Floor Priority, 15 = Base Station")
    grid_input = st.text_area("Grid Matrix:", key="grid_area", height=140)
    
    def _do_randomize_fires():
        grid_src = st.session_state.get("grid_area", "")
        lines = [l.strip().split() for l in grid_src.strip().split("\n") if l.strip()]
        walkable = []
        for r, row in enumerate(lines):
            for c, val in enumerate(row):
                if val not in ('0', '15'):
                    walkable.append(f"{r},{c}")
        if len(walkable) >= 4:
            picked = random.sample(walkable, min(6, len(walkable)))
            st.session_state["fires_input_field"] = "; ".join(picked)

    fires_input = st.text_input("Fire Coordinates (r,c; r,c):", key="fires_input_field")
    st.button("Randomize Fire Locations", on_click=_do_randomize_fires)

    single_cost = st.number_input("Single Attack Cost (ticks):", min_value=1, value=default_vals["single_cost"])
    splash_cost = st.number_input("Splash Attack Cost (ticks):", min_value=1, value=default_vals["splash_cost"])
    spread_rate = st.number_input("Fire Spread Interval (ticks):", min_value=1, value=default_vals["spread_rate"])

# Parse inputs
def parse_inputs():
    try:
        grid = []
        for line in grid_input.strip().split("\n"):
            line = line.strip()
            if not line:
                continue
            # If spaces are used: split on whitespace
            if " " in line:
                grid.append([int(x) for x in line.split()])
            else:
                # If typed as contiguous digits: parse character by character
                grid.append([int(ch) for ch in line])
                
        station = None
        for r in range(len(grid)):
            for c in range(len(grid[0])):
                if grid[r][c] == 15:
                    station = (r, c)
                    break
            if station:
                break
                
        fires = []
        for item in fires_input.split(";"):
            parts = item.strip().split(",")
            if len(parts) == 2:
                fires.append((int(parts[0]), int(parts[1])))
                
        if not station:
            return grid, None, fires, "No Base Station (value 15) found in the grid matrix! Please add a '15' block for the agent's station."
        if not fires:
            return grid, station, fires, "No fire coordinates specified! Please enter at least one fire position."
            
        return grid, station, fires, None
    except Exception as e:
        return None, None, None, f"Matrix parsing error: {e}"

grid, station, fire_sources, err = parse_inputs()

# Initialize Simulation in Session State
needs_reinit = (err is None and ("sim" not in st.session_state or reset_btn or 
                st.session_state.get("last_initialized_fires") != fires_input or 
                st.session_state.get("last_initialized_grid") != grid_input))

if needs_reinit:
    st.session_state.sim = FirefightingSimulation(
        grid=grid,
        station_pos=station,
        fire_sources=fire_sources,
        fire_spread_interval=spread_rate,
        single_cost=single_cost,
        splash_cost=splash_cost
    )
    st.session_state["last_initialized_fires"] = fires_input
    st.session_state["last_initialized_grid"] = grid_input
    st.session_state.is_running = False

sim = st.session_state.get("sim", None)

if err or sim is None:
    if err:
        st.error(f"Input Error: {err}")
    st.stop()

# Main Dashboard Layout
st.markdown("## Model-Based Firefighting Agent (A* Search)")

# Legend Bar
st.markdown("""
<div style="display: flex; gap: 20px; font-size: 13px; margin-bottom: 14px; color: #a0aec0; align-items: center; flex-wrap: wrap;">
    <span><span style="display:inline-block; width:12px; height:12px; background:#1f6b47; border-radius:3px; margin-right:4px;"></span> Floor (1-10 Priority)</span>
    <span><span style="display:inline-block; width:12px; height:12px; background:#212631; border-radius:3px; margin-right:4px;"></span> Wall (0)</span>
    <span><span style="display:inline-block; width:12px; height:12px; background:#e65100; border-radius:3px; margin-right:4px;"></span> Initial Fire</span>
    <span><span style="display:inline-block; width:12px; height:12px; background:#880e4f; border:1px solid #e91e63; border-radius:3px; margin-right:4px;"></span> Spread Fire</span>
    <span><span style="display:inline-block; width:12px; height:12px; background:#1d4ed8; border-radius:3px; margin-right:4px;"></span> Agent</span>
    <span><span style="display:inline-block; width:12px; height:12px; background:#1e3a5f; border:1px solid #3b82f6; border-radius:3px; margin-right:4px;"></span> Station</span>
</div>
""", unsafe_allow_html=True)

col_map, col_telemetry = st.columns([1.3, 1.0])

def generate_grid_html(sim):
    html = '<div class="grid-container">'
    for r in range(sim.rows):
        html += '<div class="grid-row">'
        for c in range(sim.cols):
            pos = (r, c)
            val = sim.grid[r][c]
            
            if pos == sim.agent_pos:
                content = '🤖'
                css = 'grid-cell cell-agent'
            elif pos in sim.active_fires:
                content = '🔥'
                if hasattr(sim, 'spread_fires') and pos in sim.spread_fires:
                    css = 'grid-cell cell-fire-spread'
                else:
                    css = 'grid-cell cell-fire'
            elif pos == sim.station_pos:
                content = '🏠'
                css = 'grid-cell cell-station'
            elif pos in sim.extinguished_history:
                content = str(val) if val > 0 else ''
                css = 'grid-cell cell-extinguished'
            elif val == 0:
                content = ''
                css = 'grid-cell cell-wall'
            else:
                # Dynamic green shade based on priority
                content = str(val)
                css = 'grid-cell cell-floor'
            
            html += f'<div class="{css}">{content}</div>'
        html += '</div>'
    html += '</div>'
    return html

with col_map:
    map_placeholder = st.empty()
    map_placeholder.markdown(generate_grid_html(sim), unsafe_allow_html=True)

with col_telemetry:
    st.markdown("### Agent Status & Telemetry")
    status_box = st.empty()
    goal_box = st.empty()
    path_len_box = st.empty()

    def update_telemetry_ui():
        status_box.markdown(f"**Agent status:** <span style='color: #48db9d; font-family: monospace;'>{getattr(sim, 'last_action', 'Standby')}</span>", unsafe_allow_html=True)
        goal_box.markdown(f"**Current Goal:** <span style='background: #1e3a8a; padding: 3px 8px; border-radius: 4px; font-size: 13px; font-family: monospace;'>{getattr(sim, 'current_goal', 'IDLE')}</span>", unsafe_allow_html=True)
        path_len = len(getattr(sim, 'planned_path', []))
        path_len_box.markdown(f"**Planned Path Length:** <span style='background: #374151; padding: 3px 8px; border-radius: 4px; font-size: 13px; font-family: monospace;'>{path_len}</span>", unsafe_allow_html=True)

    update_telemetry_ui()

    st.markdown("<br>", unsafe_allow_html=True)
    m_col1, m_col2 = st.columns(2)
    ext_card = m_col1.empty()
    steps_card = m_col2.empty()

    def update_cards():
        ext_card.markdown(f"""
        <div class="metric-card">
            <div style="font-size: 13px; color: #a0aec0;">Extinguished</div>
            <div style="font-size: 2.2rem; font-weight: bold; color: #ffffff;">{sim.fires_extinguished_count}</div>
        </div>
        """, unsafe_allow_html=True)
        steps_card.markdown(f"""
        <div class="metric-card">
            <div style="font-size: 13px; color: #a0aec0;">Sim Steps</div>
            <div style="font-size: 2.2rem; font-weight: bold; color: #ffffff;">{sim.ticks}</div>
        </div>
        """, unsafe_allow_html=True)

    update_cards()

    st.markdown("<br>", unsafe_allow_html=True)
    with st.expander("Agent Internal Model & Memory State", expanded=True):
        mem_placeholder = st.empty()
        def update_memory_state():
            rem_sources = len(sim.fire_sources)
            next_t = f"in {sim.next_spawn_tick - sim.ticks}t" if sim.next_spawn_tick and sim.next_spawn_tick > sim.ticks else "None pending"
            dfs_depth = len(getattr(sim, 'dfs_stack', []))
            
            # Formatted list of active fires coordinates currently burning
            active_coords_list = list(sim.active_fires.keys())
            active_str = ", ".join([f"({r},{c})" for r, c in active_coords_list]) if active_coords_list else "None (All Cleared)"
            
            reported_str = ", ".join([f"({r},{c})" for r, c in getattr(sim, 'reported_fires', [])]) if getattr(sim, 'reported_fires', None) else "None"
            cleared_info = f"{sim.fires_cleared_tick} ticks" if getattr(sim, 'fires_cleared_tick', None) is not None else "Active"
            mem_placeholder.markdown(f"""
            - **Current Active Fires ({len(active_coords_list)}):** `{active_str}`
            - **Agent Position:** `{sim.agent_pos}`
            - **Total Distance Navigated:** `{sim.total_distance}` blocks
            - **DFS Backtrack Stack Depth:** `{dfs_depth}`
            - **Time to Extinguish All Fires:** `{cleared_info}`
            - **Upcoming Unignited Fire Sources:** `{rem_sources}` ({next_t})
            - **Extinguished Blocks Retained:** `{len(sim.extinguished_history)}`
            """)
        update_memory_state()

# Handle Single Step execution
if single_step_btn:
    if not sim.is_completed:
        sim.step()
        map_placeholder.markdown(generate_grid_html(sim), unsafe_allow_html=True)
        update_telemetry_ui()
        update_cards()
        update_memory_state()
        st.rerun()

# Handle Start / Run Live Animation Loop
if start_run_btn:
    st.session_state.is_running = True
    max_steps = 300
    steps = 0
    while not sim.is_completed and st.session_state.is_running and steps < max_steps:
        sim.step()
        map_placeholder.markdown(generate_grid_html(sim), unsafe_allow_html=True)
        update_telemetry_ui()
        update_cards()
        update_memory_state()
        time.sleep(anim_speed)
        steps += 1
        if sim.is_completed:
            st.session_state.is_running = False
            break
    st.rerun()

if sim.is_completed:
    clear_time_str = f"**{sim.fires_cleared_tick} ticks**" if getattr(sim, 'fires_cleared_tick', None) is not None else f"**{sim.ticks} ticks**"
    st.success(f"Mission Accomplished! All fires put off in {clear_time_str}. Total mission time (including return to station): **{sim.ticks} ticks** | Total distance: **{sim.total_distance} blocks**.")
