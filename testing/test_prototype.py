import heapq
import random

DIRS_4 = [(-1, 0), (1, 0), (0, -1), (0, 1)]
DIRS_8 = [(-1, -1), (-1, 0), (-1, 1),
          (0, -1),           (0, 1),
          (1, -1),  (1, 0),  (1, 1)]

def manhattan_distance(p1, p2):
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

def astar_search(grid, start, goal, active_fires):
    if start == goal:
        return [start]
    rows, cols = len(grid), len(grid[0])
    open_set = []
    heapq.heappush(open_set, (0, 0, start, [start]))
    visited = {start: 0}

    while open_set:
        _, g, current, path = heapq.heappop(open_set)
        if current == goal:
            return path

        for dr, dc in DIRS_4:
            nr, nc = current[0] + dr, current[1] + dc
            neighbor = (nr, nc)
            if 0 <= nr < rows and 0 <= nc < cols:
                if grid[nr][nc] == 0:
                    continue
                if neighbor in active_fires:
                    continue
                new_g = g + 1
                if neighbor not in visited or new_g < visited[neighbor]:
                    visited[neighbor] = new_g
                    f = new_g + manhattan_distance(neighbor, goal)
                    heapq.heappush(open_set, (f, new_g, neighbor, path + [neighbor]))
    return None

def find_safe_stand_pos(grid, agent_pos, fire_pos, active_fires):
    rows, cols = len(grid), len(grid[0])
    candidates = []
    # Strict 4-orthogonal neighbors only
    for dr, dc in DIRS_4:
        nr, nc = fire_pos[0] + dr, fire_pos[1] + dc
        pos = (nr, nc)
        if 0 <= nr < rows and 0 <= nc < cols:
            if grid[nr][nc] != 0 and pos not in active_fires:
                path = astar_search(grid, agent_pos, pos, active_fires)
                if path is not None:
                    candidates.append((len(path), pos, path))
    if candidates:
        candidates.sort(key=lambda x: x[0])
        return candidates[0][1], candidates[0][2]
    return None, None

class FixedSim:
    def __init__(self, grid, station_pos, fire_sources, fire_spread_interval=8, single_cost=2, splash_cost=4):
        self.grid = [row[:] for row in grid]
        self.rows = len(grid)
        self.cols = len(grid[0])
        self.station_pos = station_pos
        self.agent_pos = station_pos
        self.reported_fires = list(fire_sources)
        self.fire_spread_interval = fire_spread_interval
        self.single_cost = single_cost
        self.splash_cost = splash_cost
        
        self.ticks = 0
        self.active_fires = {}
        self.spread_fires = set()
        self.extinguished_history = set()
        self.dfs_stack = []
        self.recently_extinguished = None
        self.fire_clusters = []
        self.next_cluster_id = 1
        
        self.total_distance = 0
        self.fires_extinguished_count = 0
        self.fires_cleared_tick = None
        self.response_times = []
        self.severity_at_extinguish = []
        self.logs = []
        self.last_action = "Standby"
        self.current_goal = "IDLE"
        self.planned_path = []
        self.is_completed = False

        for pos in self.reported_fires:
            self._ignite_fire(pos)

    def _ignite_fire(self, pos):
        if self.grid[pos[0]][pos[1]] == 0:
            return
        if pos not in self.active_fires:
            self.active_fires[pos] = self.ticks
            self._assign_to_cluster(pos)

    def _assign_to_cluster(self, pos):
        for cluster in self.fire_clusters:
            for f in cluster['fires']:
                if abs(pos[0] - f[0]) <= 1 and abs(pos[1] - f[1]) <= 1:
                    cluster['fires'].add(pos)
                    return
        c_id = self.next_cluster_id
        self.next_cluster_id += 1
        zone_pri = self.grid[pos[0]][pos[1]]
        new_cluster = {
            'id': c_id,
            'source': pos,
            'fires': {pos},
            'zone_priority': zone_pri,
            'spawn_tick': self.ticks
        }
        self.fire_clusters.append(new_cluster)

    def _update_clusters(self):
        for cluster in self.fire_clusters:
            cluster['fires'] = {p for p in cluster['fires'] if p in self.active_fires}
        self.fire_clusters = [c for c in self.fire_clusters if len(c['fires']) > 0]

    def _select_target_cluster(self):
        self._update_clusters()
        if not self.fire_clusters:
            return None

        best_reachable_cluster = None
        best_reachable_score = -float('inf')
        fallback_cluster = None
        fallback_score = -float('inf')

        for c in self.fire_clusters:
            dists = [manhattan_distance(self.agent_pos, f) for f in c['fires']]
            min_dist = min(dists) if dists else 1
            severity = len(c['fires'])
            score = (c['zone_priority'] * 3.0) + (severity * 2.0) - (min_dist * 1.0)
            
            if score > fallback_score:
                fallback_score = score
                fallback_cluster = c

            # Check if any fire in cluster has safe stand pos
            is_reachable = False
            for f in c['fires']:
                stand, path = find_safe_stand_pos(self.grid, self.agent_pos, f, self.active_fires)
                if stand is not None and path is not None:
                    is_reachable = True
                    break

            if is_reachable and score > best_reachable_score:
                best_reachable_score = score
                best_reachable_cluster = c

        if best_reachable_cluster is not None:
            return best_reachable_cluster

        # Dynamic fallback: find ANY reachable active fire in the entire building
        closest_fire = None
        shortest_len = float('inf')
        for f in self.active_fires.keys():
            stand, path = find_safe_stand_pos(self.grid, self.agent_pos, f, self.active_fires)
            if stand is not None and path is not None:
                if len(path) < shortest_len:
                    shortest_len = len(path)
                    closest_fire = f

        if closest_fire:
            for c in self.fire_clusters:
                if closest_fire in c['fires']:
                    return c

        return fallback_cluster

    def spread_fire(self):
        if not self.active_fires:
            return
        candidates = []
        for fire_pos in self.active_fires.keys():
            for dr, dc in DIRS_4:
                nr, nc = fire_pos[0] + dr, fire_pos[1] + dc
                if 0 <= nr < self.rows and 0 <= nc < self.cols:
                    if self.grid[nr][nc] != 0 and (nr, nc) not in self.active_fires:
                        if (nr, nc) != self.agent_pos and (nr, nc) != self.station_pos:
                            candidates.append((nr, nc))
        if candidates:
            unique_candidates = list(set(candidates))
            num_to_spread = min(random.choice([1, 2]), len(unique_candidates))
            chosen = random.sample(unique_candidates, num_to_spread)
            for pos in chosen:
                self.spread_fires.add(pos)
                self._ignite_fire(pos)

    def advance_time(self, duration_ticks):
        for _ in range(duration_ticks):
            self.ticks += 1
            if self.ticks % self.fire_spread_interval == 0 and self.active_fires:
                self.spread_fire()

    def get_fires_in_visibility(self):
        ar, ac = self.agent_pos
        visible = []
        for dr, dc in DIRS_8:
            nr, nc = ar + dr, ac + dc
            if (nr, nc) in self.active_fires:
                visible.append((nr, nc))
        return visible

    def perform_splash(self, fires_in_sight):
        cost = self.splash_cost
        for f in fires_in_sight:
            self._record_extinguish(f)
            self.extinguished_history.add(f)
            del self.active_fires[f]
            self.spread_fires.discard(f)
        
        closest = min(fires_in_sight, key=lambda f: manhattan_distance(self.agent_pos, f))
        self.recently_extinguished = closest
        self.advance_time(cost)
        if not self.active_fires and self.fires_cleared_tick is None:
            self.fires_cleared_tick = self.ticks
        self.last_action = f"Splash Extinguish (cleared {len(fires_in_sight)} fires)"
        self.planned_path = []

    def perform_single(self, target_fire):
        cost = self.single_cost
        self._record_extinguish(target_fire)
        self.extinguished_history.add(target_fire)
        self.recently_extinguished = target_fire
        del self.active_fires[target_fire]
        self.spread_fires.discard(target_fire)
        
        self.advance_time(cost)
        if not self.active_fires and self.fires_cleared_tick is None:
            self.fires_cleared_tick = self.ticks
        self.last_action = f"Single Extinguish at {target_fire}"
        self.planned_path = []

    def _record_extinguish(self, fire_pos):
        ignite_tick = self.active_fires.get(fire_pos, self.ticks)
        severity = self.ticks - ignite_tick
        self.response_times.append(severity)
        self.severity_at_extinguish.append(severity)
        self.fires_extinguished_count += 1

    def step(self):
        if self.is_completed:
            return

        # ---------------------------------------------------------------------
        # 1. MISSION COMPLETE: All fires extinguished -> Direct A* to station
        # ---------------------------------------------------------------------
        if not self.active_fires:
            self.dfs_stack = []
            self.recently_extinguished = None
            self.current_goal = "RETURN_STATION"
            if self.agent_pos == self.station_pos:
                self.is_completed = True
                self.planned_path = []
                self.last_action = "Safely docked at Base Station"
                self.logs.append(f"Tick {self.ticks}: Mission complete! Agent successfully returned to Base Station.")
                return
            else:
                path = astar_search(self.grid, self.agent_pos, self.station_pos, set())
                if path and len(path) > 1:
                    self.agent_pos = path[1]
                    self.planned_path = path[2:]
                    self.last_action = f"Returning to base ({self.agent_pos})"
                    self.total_distance += 1
                    self.advance_time(1)
                    return
                else:
                    self.last_action = "Path to base blocked"
                    self.advance_time(1)
                    return

        # ---------------------------------------------------------------------
        # 2. IMMEDIATE EXTINGUISH: Sensor sees active fires in 3x3 window
        # ---------------------------------------------------------------------
        visible_fires = self.get_fires_in_visibility()
        if visible_fires:
            self.planned_path = []
            if len(visible_fires) >= 2:
                self.perform_splash(visible_fires)
            else:
                self.perform_single(visible_fires[0])
            return

        # ---------------------------------------------------------------------
        # 3. LOCAL DFS STEP-IN: Enter extinguished block to inspect room
        # Strictly orthogonal movement via A* (never leap diagonally)
        # ---------------------------------------------------------------------
        if self.recently_extinguished is not None:
            target = self.recently_extinguished
            self.recently_extinguished = None
            path = astar_search(self.grid, self.agent_pos, target, self.active_fires)
            if path and len(path) > 1:
                self.dfs_stack.append(self.agent_pos)
                self.agent_pos = path[1]
                self.total_distance += 1
                self.advance_time(1)
                self.current_goal = f"DFS_INSPECT_{target}"
                self.last_action = f"Stepped into extinguished block {self.agent_pos} to inspect"
                if self.agent_pos != target:
                    self.recently_extinguished = target
                    self.planned_path = path[2:]
                else:
                    self.planned_path = []
                return

        # ---------------------------------------------------------------------
        # 4. LOCAL DFS BACKTRACK: Leaf node reached inside room
        # ---------------------------------------------------------------------
        if self.dfs_stack:
            previous_pos = self.dfs_stack.pop()
            self.agent_pos = previous_pos
            self.total_distance += 1
            self.advance_time(1)
            self.planned_path = []
            self.current_goal = "DFS_BACKTRACK"
            self.last_action = f"Cluster clear here. Backtracking to {previous_pos}"
            # If no fires visible here, clear stack so agent can A* to next cluster
            if not self.get_fires_in_visibility():
                self.dfs_stack = []
            return

        # ---------------------------------------------------------------------
        # 5. GLOBAL A* NAVIGATION: Move towards next highest-utility fire cluster
        # ---------------------------------------------------------------------
        target_cluster = self._select_target_cluster()
        if target_cluster:
            cluster_fires = sorted(target_cluster['fires'], key=lambda f: manhattan_distance(self.agent_pos, f))
            stand_pos, path, chosen_fire = None, None, None
            # Check every fire in cluster for reachability
            for f in cluster_fires:
                st, pa = find_safe_stand_pos(self.grid, self.agent_pos, f, self.active_fires)
                if st is not None and pa is not None:
                    stand_pos, path, chosen_fire = st, pa, f
                    break

            # Fallback across all active fires if this cluster is temporarily blocked
            if not stand_pos:
                for f in sorted(self.active_fires.keys(), key=lambda f: manhattan_distance(self.agent_pos, f)):
                    st, pa = find_safe_stand_pos(self.grid, self.agent_pos, f, self.active_fires)
                    if st is not None and pa is not None:
                        stand_pos, path, chosen_fire = st, pa, f
                        break

            if stand_pos and path and len(path) > 1:
                self.agent_pos = path[1]
                self.planned_path = path[2:]
                c_id = target_cluster['id']
                self.current_goal = f"EXTINGUISH_CLUSTER_{c_id}"
                self.last_action = f"Moving to {self.agent_pos}"
                self.total_distance += 1
                self.advance_time(1)
                return
            elif stand_pos and self.agent_pos == stand_pos:
                self.planned_path = []
                vis = self.get_fires_in_visibility()
                if vis:
                    if len(vis) >= 2:
                        self.perform_splash(vis)
                    else:
                        self.perform_single(vis[0])
                    return

if __name__ == "__main__":
    random.seed(42)
    grid = [
        [0, 0, 0, 0, 0, 0],
        [0, 15, 2, 3, 2, 0],
        [0, 1, 0, 0, 4, 0],
        [0, 2, 1, 5, 3, 0],
        [0, 1, 8, 9, 2, 0],
        [0, 0, 0, 0, 0, 0]
    ]
    fires = [(3, 4), (4, 2), (1, 3), (2, 4), (3, 1), (4, 3)]
    sim = FixedSim(grid, (1, 1), fires, fire_spread_interval=8, single_cost=2, splash_cost=4)

    print("=== TRACE OF FIXED SIM (PRESET 1) ===")
    for step in range(1, 40):
        if sim.is_completed:
            print(f"Mission complete at step {step}!")
            break
        p_before = sim.agent_pos
        sim.step()
        p_after = sim.agent_pos
        dr = p_after[0] - p_before[0]
        dc = p_after[1] - p_before[1]
        is_diag = (dr != 0 and dc != 0)
        d = manhattan_distance(p_before, p_after)
        diag_flag = " [DIAGONAL!]" if is_diag else ""
        jump_flag = f" [JUMP {d}!]" if d > 1 else ""
        print(f"Step {step:02d} | Tick {sim.ticks:02d} | Pos: {p_before} -> {p_after} | Action: {sim.last_action:<45} | Goal: {sim.current_goal:<20} | PathLen={len(sim.planned_path)}{diag_flag}{jump_flag}")
