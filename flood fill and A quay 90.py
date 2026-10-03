# micromouse_floodfill_astar.py
# Phase 1: Flood-Fill explore (unknown=open) to center
# Phase 2: A* shortest path (open-only) for speed-run
import API
import sys
from collections import deque
import heapq

# ------------- logging -------------
def log(msg):
    sys.stderr.write(str(msg) + "\n")
    sys.stderr.flush()

# ------------- directions -------------
# 0:N, 1:E, 2:S, 3:W
DIRS = [(0, 1), (1, 0), (0, -1), (-1, 0)]
def L(d):  return (d + 3) % 4
def R(d):  return (d + 1) % 4
def B(d):  return (d + 2) % 4

# Bật/tắt bít ngõ cụt để hạn chế quay lại (không bắt buộc cho speed-run)
USE_DEADEND_FILL = False

# ------------- map model -------------
class MazeMap:
    """walls[x][y][d] ∈ {True (wall), False (open), None (unknown)}"""
    def __init__(self, w, h):
        self.w, self.h = w, h
        self.walls = [[[None for _ in range(4)] for _ in range(h)] for _ in range(w)]
        # biên ngoài là tường
        for x in range(w):
            self.walls[x][0][2]  = True
            self.walls[x][h-1][0] = True
        for y in range(h):
            self.walls[0][y][3]  = True
            self.walls[w-1][y][1] = True

    def inb(self, x, y): return 0 <= x < self.w and 0 <= y < self.h

    def set_wall_pair(self, x, y, d, is_wall: bool):
        nx, ny = x + DIRS[d][0], y + DIRS[d][1]
        self.walls[x][y][d] = is_wall
        if self.inb(nx, ny):
            self.walls[nx][ny][B(d)] = is_wall

    # policies
    def is_open(self, x, y, d):            # chỉ đường đã biết mở
        return self.walls[x][y][d] is False

    def is_open_or_unknown(self, x, y, d): # cho pha explore
        v = self.walls[x][y][d]
        return v is False or v is None

# ------------- goals (center) -------------
def center_cells(w, h):
    cx = [w//2] if w % 2 == 1 else [w//2 - 1, w//2]
    cy = [h//2] if h % 2 == 1 else [h//2 - 1, h//2]
    return {(x, y) for x in cx for y in cy}

# ------------- flood-fill (BFS) distance -------------
def compute_distances(mm: MazeMap, goals: set, allow_unknown: bool):
    INF = 10**9
    dist = [[INF for _ in range(mm.h)] for _ in range(mm.w)]
    q = deque()
    for gx, gy in goals:
        dist[gx][gy] = 0
        q.append((gx, gy))
    while q:
        x, y = q.popleft()
        for d in range(4):
            nx, ny = x + DIRS[d][0], y + DIRS[d][1]
            if not mm.inb(nx, ny): continue
            ok = mm.is_open_or_unknown(nx, ny, B(d)) if allow_unknown else mm.is_open(nx, ny, B(d))
            if ok:
                nd = dist[x][y] + 1
                if nd < dist[nx][ny]:
                    dist[nx][ny] = nd
                    q.append((nx, ny))
    return dist

# ------------- A* shortest path (open-only) -------------
def astar_shortest_path(mm: MazeMap, start, goals: set):
    """
    A*: node = (x,y). Cost = số ô (1 mỗi cạnh). Heuristic = Manhattan min đến goal.
    Trả về danh sách hướng d (0..3) từ start đến goal gần nhất; [] nếu không có đường.
    """
    sx, sy = start
    if (sx, sy) in goals:
        return []

    # heuristic: khoảng cách Manhattan tới gần nhất trong goals
    def h(x, y):
        return min(abs(x - gx) + abs(y - gy) for gx, gy in goals)

    INF = 10**9
    g = [[INF for _ in range(mm.h)] for _ in range(mm.w)]
    parent = [[None for _ in range(mm.h)] for _ in range(mm.w)]  # store (px,py,dir_from_parent)
    pq = []  # (f, g, x, y)
    g[sx][sy] = 0
    heapq.heappush(pq, (h(sx, sy), 0, sx, sy))

    closed = [[False for _ in range(mm.h)] for _ in range(mm.w)]

    while pq:
        fcur, gcur, x, y = heapq.heappop(pq)
        if closed[x][y]: 
            continue
        closed[x][y] = True

        if (x, y) in goals:
            # reconstruct
            path_dirs = []
            cx, cy = x, y
            while (cx, cy) != (sx, sy):
                px, py, d = parent[cx][cy]
                path_dirs.append(d)  # direction from parent -> (cx,cy)
                cx, cy = px, py
            path_dirs.reverse()
            return path_dirs

        for d in range(4):
            if not mm.is_open(x, y, d): 
                continue
            nx, ny = x + DIRS[d][0], y + DIRS[d][1]
            if not mm.inb(nx, ny): 
                continue
            ng = gcur + 1
            if ng < g[nx][ny]:
                g[nx][ny] = ng
                parent[nx][ny] = (x, y, d)
                nf = ng + h(nx, ny)
                heapq.heappush(pq, (nf, ng, nx, ny))

    return []  # no path

# ------------- motion helpers -------------
def turn_to(cur, target):
    diff = (target - cur) % 4
    if diff == 0: return [], cur
    if diff == 1: return [API.turnRight], target
    if diff == 2: return [API.turnRight, API.turnRight], target
    return [API.turnLeft], target

def try_step():
    if not API.wallFront():
        API.moveForward(1)
        return True
    return False

# ------------- sensing & overlays -------------
def sense_here(mm: MazeMap, x, y, hd):
    mm.set_wall_pair(x, y, L(hd), API.wallLeft())
    mm.set_wall_pair(x, y, hd,    API.wallFront())
    mm.set_wall_pair(x, y, R(hd), API.wallRight())

def mark_overlay(x, y, dv):
    try:
        API.setText(x, y, str(dv if dv < 10**8 else "∞"))
        API.setColor(x, y, "G" if (x + y) % 2 == 0 else "g")
    except Exception:
        pass

# ------------- dead-end fill (optional) -------------
def dead_end_fill(mm: MazeMap, goals: set):
    q = deque()
    def enqueue_if_dead(x, y):
        known, open_dirs = 0, []
        for d in range(4):
            v = mm.walls[x][y][d]
            if v is not None:
                known += 1
                if v is False: open_dirs.append(d)
        if known == 4 and len(open_dirs) == 1 and (x, y) not in goals:
            q.append((x, y, open_dirs[0]))
    for x in range(mm.w):
        for y in range(mm.h):
            enqueue_if_dead(x, y)
    while q:
        x, y, od = q.popleft()
        known, open_dirs = 0, []
        for d in range(4):
            v = mm.walls[x][y][d]
            if v is not None:
                known += 1
                if v is False: open_dirs.append(d)
        if known == 4 and len(open_dirs) == 1 and (x, y) not in goals:
            mm.set_wall_pair(x, y, open_dirs[0], True)
            nx, ny = x + DIRS[od][0], y + DIRS[od][1]
            if mm.inb(nx, ny):
                enqueue_if_dead(nx, ny)

# ------------- greedy heading chooser (for explore) -------------
def choose_dir(mm: MazeMap, dist, x, y, hd, allow_unknown: bool):
    best, val = [], 10**9
    for d in range(4):
        ok = mm.is_open_or_unknown(x, y, d) if allow_unknown else mm.is_open(x, y, d)
        if not ok: continue
        nx, ny = x + DIRS[d][0], y + DIRS[d][1]
        if not mm.inb(nx, ny): continue
        v = dist[nx][ny]
        if v < val: val, best = v, [d]
        elif v == val: best.append(d)
    if not best: return None
    for pref in [hd, L(hd), R(hd), B(hd)]:
        if pref in best: return pref
    return best[0]

# ------------- execute a path of headings -------------
def exec_path(path_dirs, start_hd):
    hd = start_hd
    for d in path_dirs:
        turns, hd = turn_to(hd, d)
        for t in turns: t()
        if API.wallFront():
            return False
        API.moveForward(1)
    return True

# ------------- main -------------
def main():
    W, H = API.mazeWidth(), API.mazeHeight()
    goals = center_cells(W, H)
    log(f"FF+ASTAR | size={W}x{H} | goals={goals}")

    # Start state (giả định hướng Bắc)
    x, y, hd = 0, 0, 0
    mm = MazeMap(W, H)

    # ---------- Phase 1: EXPLORE ----------
    sense_here(mm, x, y, hd)
    if USE_DEADEND_FILL: dead_end_fill(mm, goals)

    steps = 0
    while True:
        if API.wasReset():
            API.ackReset(); log("Reset detected."); return
        steps += 1

        dist = compute_distances(mm, goals, allow_unknown=True)
        mark_overlay(x, y, dist[x][y])

        if (x, y) in goals:
            API.setColor(x, y, "R"); API.setText(x, y, "GOAL")
            log(f"Reached center in {steps} steps → SPEED RUN (A*).")
            break

        nd = choose_dir(mm, dist, x, y, hd, allow_unknown=True)
        if nd is None:
            log("No neighbor under exploration policy. Stop."); return

        turns, hd = turn_to(hd, nd)
        for t in turns: t()

        if not try_step():
            # mâu thuẫn → đánh dấu tường, cảm biến lại và tiếp tục
            mm.set_wall_pair(x, y, hd, True)
            sense_here(mm, x, y, hd)
            if USE_DEADEND_FILL: dead_end_fill(mm, goals)
            continue

        # xác nhận cạnh vừa đi là mở & cập nhật vị trí
        mm.set_wall_pair(x, y, hd, False)
        x += DIRS[hd][0]; y += DIRS[hd][1]
        sense_here(mm, x, y, hd)
        if USE_DEADEND_FILL: dead_end_fill(mm, goals)

    # ---------- Phase 2: SPEED-RUN (A*) ----------
    # 1) Tìm đường ngắn nhất (open-only) từ START → CENTER
    path_dirs = astar_shortest_path(mm, start=(0, 0), goals=goals)
    if not path_dirs:
        log("A*: no open-only path from start to center (map incomplete?).")
        return

    # 2) Quay về start (nếu simulator không tự reset vị trí)
    #    Dùng BFS open-only về (0,0)
    back_dist = compute_distances(mm, goals={(0, 0)}, allow_unknown=False)
    while (x, y) != (0, 0):
        nd = choose_dir(mm, back_dist, x, y, hd, allow_unknown=False)
        if nd is None:
            log("Cannot return to start for speed-run."); return
        turns, hd = turn_to(hd, nd)
        for t in turns: t()
        if not try_step(): 
            log("Blocked while returning to start."); return
        x += DIRS[hd][0]; y += DIRS[hd][1]

    # 3) Chạy speed-run theo path của A*
    ok = exec_path(path_dirs, start_hd=hd)
    log("Speed-run complete." if ok else "Speed-run interrupted by unexpected wall.")

if __name__ == "__main__":
    main()
