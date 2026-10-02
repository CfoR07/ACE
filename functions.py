import heapq
import math
import random

# Movement directions (4-directional)
DIRS_4 = [(-1, 0), (1, 0), (0, -1), (0, 1)]
# Visibility / 8-neighborhood
DIRS_8 = [(-1, -1), (-1, 0), (-1, 1),
          (0, -1),           (0, 1),
          (1, -1),  (1, 0),  (1, 1)]


def manhattan_distance(p1, p2):
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])


def astar_search(grid, start, goal, active_fires):
    """
    Finds shortest safe path from start to goal avoiding walls (0) and active fire cells.
    Returns list of coords from start to goal, or None if unreachable.
    """
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

            # Check bounds, walls, and active fire
            if 0 <= nr < rows and 0 <= nc < cols:
                if grid[nr][nc] == 0:
                    continue  # wall
                if neighbor in active_fires:
                    continue  # unsafe

                new_g = g + 1
                if neighbor not in visited or new_g < visited[neighbor]:
                    visited[neighbor] = new_g
                    f = new_g + manhattan_distance(neighbor, goal)
                    heapq.heappush(open_set, (f, new_g, neighbor, path + [neighbor]))

    return None


def find_safe_stand_pos(grid, agent_pos, fire_pos, active_fires):
    """
    Finds the best safe neighbor adjacent to or in 8-vis of target fire to stand and fight.
    Prefers cells with shortest path from current agent position.
    """
    rows, cols = len(grid), len(grid[0])
    candidates = []

    for dr, dc in DIRS_8:
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


class FirefightingSimulation:
    def __init__(self, grid, station_pos, fire_sources, fire_spread_interval, single_cost, splash_cost):
        self.grid = [row[:] for row in grid]
        self.rows = len(grid)
        self.cols = len(grid[0])
        self.station_pos = station_pos
        self.agent_pos = station_pos
        
        # Fire configuration
        self.fire_sources = list(fire_sources)  # list of potential spawn locations
        self.fire_spread_interval = fire_spread_interval  # ticks between spreads
        self.single_cost = single_cost
        self.splash_cost = splash_cost
        
        # State tracking
        self.ticks = 0
        self.active_fires = {}  # (r, c) -> ignition_tick
        self.extinguished_history = set()
        self.inspection_queue = []  # safe blocks to inspect after extinguish
        self.fire_clusters = []  # list of clusters: {'id', 'source', 'fires': set(), 'priority'}
        self.next_cluster_id = 1
        
        # Performance metrics
        self.total_distance = 0
        self.fires_extinguished_count = 0
        self.response_times = []  # ticks taken from fire ignition to put out
        self.severity_at_extinguish = []  # age in ticks of fire when put out
        self.logs = []
        self.last_action = "Agent scanning area / waiting for clear route."
        self.current_goal = "PATROL_UNKNOWN"
        self.planned_path = []
        self.is_completed = False
        
        # Max wait limit for distant fires (e.g. if next fire arrives after too long)
        dim = max(self.rows, self.cols)
        self.max_wait_limit = dim * 3  # reasonable cutoff threshold
        
        # Initial fires vs delayed fires:
        # If multiple fires provided, ignite first batch (up to 2 or 3) immediately at start
        total_initial = max(1, min(len(self.fire_sources), random.choice([1, 2, 3]) if len(self.fire_sources) >= 3 else 1))
        initial_batch = []
        for _ in range(total_initial):
            if self.fire_sources:
                initial_batch.append(self.fire_sources.pop(random.randrange(len(self.fire_sources))))
                
        for pos in initial_batch:
            self._ignite_fire(pos)
        self.logs.append(f"Tick {self.ticks}: Initial alert received for {len(initial_batch)} fire(s): {initial_batch}.")

        # Schedule remaining fires with realistic room-dimension-based interval
        avg_n = (self.rows + self.cols) / 2.0
        low = max(2, int(round((avg_n - 4) / 2.0)))
        high = max(low + 2, int(round((avg_n + 4) / 2.0)))
        self.spawn_interval_range = (low, high)
        
        if self.fire_sources:
            next_delay = random.randint(low, high)
            self.next_spawn_tick = self.ticks + next_delay
        else:
            self.next_spawn_tick = None

    def _ignite_fire(self, pos):
        if self.grid[pos[0]][pos[1]] == 0:
            return  # Walls cannot catch fire
        if pos not in self.active_fires:
            self.active_fires[pos] = self.ticks
            self._assign_to_cluster(pos)

    def _assign_to_cluster(self, pos):
        """
        Groups fire into clusters. If fire is within 3x3 of an existing cluster source,
        joins it; otherwise creates a new alert cluster.
        """
        for cluster in self.fire_clusters:
            src = cluster['source']
            if abs(pos[0] - src[0]) <= 1 and abs(pos[1] - src[1]) <= 1:
                cluster['fires'].add(pos)
                return
        
        # New cluster
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
        self.logs.append(f"Tick {self.ticks}: New fire cluster #{c_id} registered at {pos} (Zone Priority: {zone_pri}).")

    def _update_clusters(self):
        # Remove resolved fires from clusters and prune empty clusters
        for cluster in self.fire_clusters:
            cluster['fires'] = {p for p in cluster['fires'] if p in self.active_fires}
        self.fire_clusters = [c for c in self.fire_clusters if len(c['fires']) > 0]

    def _select_target_cluster(self):
        self._update_clusters()
        if not self.fire_clusters:
            return None

        # Priority Rule: Score = w1 * zone_priority + w2 * cluster_size - w3 * distance
        best_cluster = None
        best_score = -float('inf')

        for c in self.fire_clusters:
            # find min distance to any fire in cluster
            dists = [manhattan_distance(self.agent_pos, f) for f in c['fires']]
            min_dist = min(dists) if dists else 1
            severity = len(c['fires'])
            score = (c['zone_priority'] * 3.0) + (severity * 2.0) - (min_dist * 1.0)
            if score > best_score:
                best_score = score
                best_cluster = c

        return best_cluster

    def spread_fire(self):
        """Spreads fire to 1-2 adjacent walkable blocks (4-directional)."""
        new_burns = []
        for fire_pos in list(self.active_fires.keys()):
            candidates = []
            for dr, dc in DIRS_4:
                nr, nc = fire_pos[0] + dr, fire_pos[1] + dc
                if 0 <= nr < self.rows and 0 <= nc < self.cols:
                    if self.grid[nr][nc] != 0 and (nr, nc) not in self.active_fires:
                        # Agent safe condition: do not ignite agent cell directly under it
                        if (nr, nc) != self.agent_pos:
                            candidates.append((nr, nc))
            
            if candidates:
                num_to_spread = min(random.choice([1, 2]), len(candidates))
                chosen = random.sample(candidates, num_to_spread)
                new_burns.extend(chosen)
        
        for pos in new_burns:
            self._ignite_fire(pos)

    def advance_time(self, duration_ticks):
        """Advances clock by duration_ticks, triggering spawns and spreads."""
        for _ in range(duration_ticks):
            self.ticks += 1
            
            # Spawn random fire from pool if schedule arrives
            if self.next_spawn_tick is not None and self.ticks >= self.next_spawn_tick:
                if self.fire_sources:
                    pos = self.fire_sources.pop(random.randrange(len(self.fire_sources)))
                    self._ignite_fire(pos)
                    self.logs.append(f"Tick {self.ticks}: Alert! Fire burst ignited at {pos}.")
                    low, high = self.spawn_interval_range
                    self.next_spawn_tick = self.ticks + random.randint(low, high)
                else:
                    self.next_spawn_tick = None

            # Periodic fire spread
            if self.ticks % self.fire_spread_interval == 0 and self.active_fires:
                self.spread_fire()

    def get_fires_in_visibility(self):
        """Returns active fires in agent's 8-neighborhood."""
        ar, ac = self.agent_pos
        visible = []
        for dr, dc in DIRS_8:
            nr, nc = ar + dr, ac + dc
            if (nr, nc) in self.active_fires:
                visible.append((nr, nc))
        return visible

    def perform_splash(self, fires_in_sight):
        """Extinguishes all fires in 8-visibility range."""
        cost = self.splash_cost
        for f in fires_in_sight:
            self._record_extinguish(f)
            self.extinguished_history.add(f)
            self.inspection_queue.append(f)
            del self.active_fires[f]
        
        self.advance_time(cost)
        self.last_action = f"Splash Extinguish (cleared {len(fires_in_sight)} fires)"
        self.logs.append(f"Tick {self.ticks}: Agent used SPLASH (cost {cost}t) extinguishing {len(fires_in_sight)} fires.")

    def perform_single(self, target_fire):
        """Extinguishes single fire and schedules inspection traversal."""
        cost = self.single_cost
        self._record_extinguish(target_fire)
        self.extinguished_history.add(target_fire)
        self.inspection_queue.append(target_fire)
        del self.active_fires[target_fire]
        
        self.advance_time(cost)
        self.last_action = f"Single Extinguish at {target_fire}"
        self.logs.append(f"Tick {self.ticks}: Agent used SINGLE (cost {cost}t) on fire at {target_fire}.")

    def _record_extinguish(self, fire_pos):
        ignite_tick = self.active_fires.get(fire_pos, self.ticks)
        severity = self.ticks - ignite_tick
        self.response_times.append(severity)
        self.severity_at_extinguish.append(severity)
        self.fires_extinguished_count += 1

    def step(self):
        """Executes one simulation step for the agent."""
        if self.is_completed:
            return

        # 1. Check if fires in visibility can be extinguished
        visible_fires = self.get_fires_in_visibility()
        if visible_fires:
            k = len(visible_fires)
            # Cost comparison: Single cost is k * single_cost + k movement steps
            est_single_cost = (k * self.single_cost) + (2 * k + 1)
            
            if k >= 2 and self.splash_cost < est_single_cost:
                self.perform_splash(visible_fires)
            else:
                self.perform_single(visible_fires[0])
            return

        # 2. Check if there are active fires to approach
        target_cluster = self._select_target_cluster()
        if target_cluster:
            cluster_fires = sorted(target_cluster['fires'], key=lambda f: manhattan_distance(self.agent_pos, f))
            chosen_fire = cluster_fires[0]
            self.current_goal = f"EXTINGUISH_CLUSTER_{target_cluster['id']}"
            
            stand_pos, path = find_safe_stand_pos(self.grid, self.agent_pos, chosen_fire, self.active_fires)
            if stand_pos and path and len(path) > 1:
                self.planned_path = path[1:]
                prev = self.agent_pos
                self.agent_pos = path[1]
                self.last_action = f"Moving to {self.agent_pos}"
                self.total_distance += 1
                self.advance_time(1)
                return
            elif stand_pos and self.agent_pos == stand_pos:
                self.planned_path = []
                vis = self.get_fires_in_visibility()
                if vis:
                    self.perform_single(vis[0])
                    return

        # 3. If no active fires remain, inspect previously extinguished blocks if queued
        if not self.active_fires and self.inspection_queue:
            target_inspect = self.inspection_queue[0]
            self.current_goal = f"INSPECT_{target_inspect}"
            if self.agent_pos == target_inspect:
                self.inspection_queue.pop(0)
            else:
                path = astar_search(self.grid, self.agent_pos, target_inspect, self.active_fires)
                if path and len(path) > 1:
                    self.planned_path = path[1:]
                    prev = self.agent_pos
                    self.agent_pos = path[1]
                    self.last_action = f"Inspecting {target_inspect}"
                    self.total_distance += 1
                    self.advance_time(1)
                    return
                else:
                    self.inspection_queue.pop(0)

        # 4. If no fires and inspection done, return to base
        if not self.active_fires and not self.inspection_queue:
            self.current_goal = "RETURN_STATION"
            if self.agent_pos == self.station_pos:
                # Check if future fires exceed cutoff or are exhausted
                if not self.fire_sources:
                    self.is_completed = True
                    self.last_action = "Safely docked at Base Station"
                    self.logs.append(f"Tick {self.ticks}: Mission complete! Agent successfully returned to Base Station.")
                elif self.next_spawn_tick is not None and (self.next_spawn_tick - self.ticks) > self.max_wait_limit:
                    self.is_completed = True
                    self.last_action = "Safely docked at Base Station (Next fire exceeds wait limit)"
                    self.logs.append(f"Tick {self.ticks}: Cutoff limit reached! Next fire scheduled at tick {self.next_spawn_tick} exceeds wait threshold ({self.max_wait_limit}t). Mission ended.")
                else:
                    self.last_action = "On standby at Base Station"
                    self.advance_time(1)
                return
            else:
                path = astar_search(self.grid, self.agent_pos, self.station_pos, self.active_fires)
                if path and len(path) > 1:
                    self.planned_path = path[1:]
                    prev = self.agent_pos
                    self.agent_pos = path[1]
                    self.last_action = f"Returning to base ({self.agent_pos})"
                    self.total_distance += 1
                    self.advance_time(1)
                    return
                else:
                    self.logs.append(f"Tick {self.ticks}: Warning - Path back to station blocked!")
                    self.last_action = "Path to base blocked"
                    self.advance_time(1)
