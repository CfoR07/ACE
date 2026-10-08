import os
import sys
import heapq
import random
import matplotlib.pyplot as plt
import matplotlib.patches as patches

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

class AdvancedFireSim:
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
        self.fire_clusters = []
        self.next_cluster_id = 1
        
        # Support dynamic upcoming fire alarms:
        # If > 3 fires provided, first 3 ignite at start, rest queued in fire_sources
        if len(self.reported_fires) > 3:
            initial_fires = self.reported_fires[:3]
            self.fire_sources = list(self.reported_fires[3:])
            self.next_spawn_tick = self.ticks + fire_spread_interval
        else:
            initial_fires = list(self.reported_fires)
            self.fire_sources = []
            self.next_spawn_tick = None
        
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
        self.dfs_stack = []

        for pos in initial_fires:
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

        # Dynamic fallback across all active fires
        for f in sorted(self.active_fires.keys(), key=lambda f: manhattan_distance(self.agent_pos, f)):
            stand, path = find_safe_stand_pos(self.grid, self.agent_pos, f, self.active_fires)
            if stand is not None and path is not None:
                for c in self.fire_clusters:
                    if f in c['fires']:
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
            # Check dynamic upcoming fire alarms
            if self.fire_sources and self.ticks >= self.next_spawn_tick:
                new_fire = self.fire_sources.pop(0)
                self._ignite_fire(new_fire)
                self.logs.append(f"Tick {self.ticks}: Dynamic fire alarm reported at {new_fire}!")
                if self.fire_sources:
                    self.next_spawn_tick = self.ticks + self.fire_spread_interval
                else:
                    self.next_spawn_tick = None
            # Periodic fire spread
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
        self.advance_time(cost)
        if not self.active_fires and not self.fire_sources and self.fires_cleared_tick is None:
            self.fires_cleared_tick = self.ticks
        self.current_goal = "EXTINGUISH_AREA_SPLASH"
        self.last_action = f"Splash Extinguish (cleared {len(fires_in_sight)} fires)"
        self.planned_path = []

    def perform_single(self, target_fire):
        cost = self.single_cost
        self._record_extinguish(target_fire)
        self.extinguished_history.add(target_fire)
        del self.active_fires[target_fire]
        self.spread_fires.discard(target_fire)
        self.advance_time(cost)
        if not self.active_fires and not self.fire_sources and self.fires_cleared_tick is None:
            self.fires_cleared_tick = self.ticks
        self.current_goal = f"EXTINGUISH_FIRE_{target_fire}"
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
        # 1. IMMEDIATE SENSOR CHECK: Active fires in 3x3 visibility window
        # ---------------------------------------------------------------------
        visible_fires = self.get_fires_in_visibility()
        if visible_fires:
            if len(visible_fires) >= 2:
                self.perform_splash(visible_fires)
            else:
                self.perform_single(visible_fires[0])
            return

        # ---------------------------------------------------------------------
        # 2. MISSION COMPLETE / RETURN TO STATION CHECK: No active fires left
        # ---------------------------------------------------------------------
        if not self.active_fires:
            # If upcoming fires still pending, wait on standby at station
            if self.fire_sources:
                if self.agent_pos == self.station_pos:
                    self.current_goal = "STANDBY_AT_STATION"
                    self.last_action = "On standby at Base Station (waiting for next fire alarm)"
                    self.planned_path = []
                    self.advance_time(1)
                    return
                else:
                    path = astar_search(self.grid, self.agent_pos, self.station_pos, set())
                    if path and len(path) > 1:
                        self.agent_pos = path[1]
                        self.planned_path = path[2:]
                        self.current_goal = "RETURN_STATION"
                        self.last_action = f"Returning to standby ({self.agent_pos})"
                        self.total_distance += 1
                        self.advance_time(1)
                        return
            else:
                # All active and scheduled fires cleared!
                if self.agent_pos == self.station_pos:
                    self.is_completed = True
                    self.planned_path = []
                    self.current_goal = "RETURN_STATION"
                    self.last_action = "Safely docked at Base Station"
                    self.logs.append(f"Tick {self.ticks}: Mission complete! Agent successfully returned to Base Station.")
                    return
                else:
                    path = astar_search(self.grid, self.agent_pos, self.station_pos, set())
                    if path and len(path) > 1:
                        self.agent_pos = path[1]
                        self.planned_path = path[2:]
                        self.current_goal = "RETURN_STATION"
                        self.last_action = f"Returning to base ({self.agent_pos})"
                        self.total_distance += 1
                        self.advance_time(1)
                        return

        # ---------------------------------------------------------------------
        # 3. GLOBAL NAVIGATION TO HIGHEST-UTILITY ACTIVE FIRE
        # Direct A* path to next priority fire (never backtrack to empty cells)
        # ---------------------------------------------------------------------
        target_cluster = self._select_target_cluster()
        if target_cluster:
            cluster_fires = sorted(target_cluster['fires'], key=lambda f: manhattan_distance(self.agent_pos, f))
            stand_pos, path, chosen_fire = None, None, None
            for f in cluster_fires:
                st, pa = find_safe_stand_pos(self.grid, self.agent_pos, f, self.active_fires)
                if st is not None and pa is not None:
                    stand_pos, path, chosen_fire = st, pa, f
                    break

            if not stand_pos:
                for f in sorted(self.active_fires.keys(), key=lambda f: manhattan_distance(self.agent_pos, f)):
                    st, pa = find_safe_stand_pos(self.grid, self.agent_pos, f, self.active_fires)
                    if st is not None and pa is not None:
                        stand_pos, path, chosen_fire = st, pa, f
                        break

            if stand_pos and path and len(path) > 1:
                self.agent_pos = path[1]
                self.planned_path = path[2:]
                cid = target_cluster['id']
                pri = target_cluster['zone_priority']
                self.current_goal = f"NAVIGATE_TO_CLUSTER_{cid}"
                self.last_action = f"Moving to {self.agent_pos} (Target Fire {chosen_fire}, Pri {pri})"
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
