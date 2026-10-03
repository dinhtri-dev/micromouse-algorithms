# micromouse_ff_astar_fillet60_replan.py
# Phase 1: Flood-Fill (unknown=open) → center
# Return: go back to (0,0) & align heading=0
# Phase 2: A* (open-only) → per-cell speed-run with “60°-style” fillet + ONLINE REPLAN
import API
import sys
from collections import deque
import heapq

def log(msg):
    sys.stderr.write(str(msg) + "\n"); sys.stderr.flush()

# ---------------- directions ----------------
# 0:N, 1:E, 2:S, 3:W
DIRS = [(0,1),(1,0),(0,-1),(-1,0)]
def L(d):  return (d+3)%4
def R(d):  return (d+1)%4
def B(d):  return (d+2)%4

USE_DEADEND_FILL = False

# ---------------- map model ----------------
class MazeMap:
    """walls[x][y][d] ∈ {True (wall), False (open), None (unknown)}"""
    def __init__(self,w,h):
        self.w,self.h=w,h
        self.walls=[[[None for _ in range(4)] for _ in range(h)] for _ in range(w)]
        # viền ngoài = tường
        for x in range(w):
            self.walls[x][0][2]=True
            self.walls[x][h-1][0]=True
        for y in range(h):
            self.walls[0][y][3]=True
            self.walls[w-1][y][1]=True

    def inb(self,x,y): return 0<=x<self.w and 0<=y<self.h

    def set_wall_pair(self,x,y,d,is_wall):
        nx,ny=x+DIRS[d][0],y+DIRS[d][1]
        self.walls[x][y][d]=is_wall
        if self.inb(nx,ny):
            self.walls[nx][ny][B(d)] = is_wall

    def is_open(self,x,y,d):  # chỉ cạnh đã biết mở
        return self.walls[x][y][d] is False

    def is_open_or_unknown(self,x,y,d):  # cho khám phá
        v=self.walls[x][y][d]
        return (v is False) or (v is None)

# ---------------- helpers ----------------
def center_cells(w,h):
    cx=[w//2] if w%2==1 else [w//2-1,w//2]
    cy=[h//2] if h%2==1 else [h//2-1,h//2]
    return {(x,y) for x in cx for y in cy}

def compute_distances(mm,goals,allow_unknown):
    INF=10**9
    dist=[[INF for _ in range(mm.h)] for _ in range(mm.w)]
    q=deque()
    for gx,gy in goals:
        dist[gx][gy]=0; q.append((gx,gy))
    while q:
        x,y=q.popleft()
        for d in range(4):
            nx,ny=x+DIRS[d][0],y+DIRS[d][1]
            if not mm.inb(nx,ny): continue
            ok = mm.is_open_or_unknown(nx,ny,B(d)) if allow_unknown else mm.is_open(nx,ny,B(d))
            if ok:
                nd=dist[x][y]+1
                if nd<dist[nx][ny]:
                    dist[nx][ny]=nd; q.append((nx,ny))
    return dist

def astar_shortest_path(mm,start,goals):
    """Trả về list hướng (0..3) từ start → goal gần nhất; [] nếu không có đường."""
    sx,sy=start
    if (sx,sy) in goals: return []
    def h(x,y): return min(abs(x-gx)+abs(y-gy) for gx,gy in goals)
    INF=10**9
    g=[[INF for _ in range(mm.h)] for _ in range(mm.w)]
    parent=[[None for _ in range(mm.h)] for _ in range(mm.w)]  # (px,py,dir_from_parent)
    pq=[]; g[sx][sy]=0; heapq.heappush(pq,(h(sx,sy),0,sx,sy))
    closed=[[False for _ in range(mm.h)] for _ in range(mm.w)]
    while pq:
        fcur,gcur,x,y=heapq.heappop(pq)
        if closed[x][y]: continue
        closed[x][y]=True
        if (x,y) in goals:
            path=[]; cx,cy=x,y
            while (cx,cy)!=(sx,sy):
                px,py,dir_from_parent=parent[cx][cy]
                path.append(dir_from_parent)
                cx,cy=px,py
            path.reverse()
            return path
        for d in range(4):
            if not mm.is_open(x,y,d): continue
            nx,ny=x+DIRS[d][0],y+DIRS[d][1]
            if not mm.inb(nx,ny): continue
            ng=gcur+1
            if ng<g[nx][ny]:
                g[nx][ny]=ng; parent[nx][ny]=(x,y,d)
                heapq.heappush(pq,(ng+h(nx,ny),ng,nx,ny))
    return []

def turn_to(cur,target):
    diff=(target-cur)%4
    if diff==0: return [],cur
    if diff==1: return [API.turnRight],target
    if diff==2: return [API.turnRight,API.turnRight],target
    return [API.turnLeft],target

def try_step():
    if not API.wallFront():
        API.moveForward(1); return True
    return False

def sense_here(mm,x,y,hd):
    mm.set_wall_pair(x,y,L(hd),API.wallLeft())
    mm.set_wall_pair(x,y,hd,   API.wallFront())
    mm.set_wall_pair(x,y,R(hd),API.wallRight())

def mark_overlay(x,y,v):
    try:
        API.setText(x,y,str(v if v<10**8 else "∞"))
        API.setColor(x,y,"G" if (x+y)%2==0 else "g")
    except Exception: pass

def dead_end_fill(mm,goals):
    q=deque()
    def enqueue(x,y):
        known=0; opens=[]
        for d in range(4):
            v=mm.walls[x][y][d]
            if v is not None:
                known+=1
                if v is False: opens.append(d)
        if known==4 and len(opens)==1 and (x,y) not in goals:
            q.append((x,y,opens[0]))
    for x in range(mm.w):
        for y in range(mm.h): enqueue(x,y)
    while q:
        x,y,od=q.popleft()
        known=0; opens=[]
        for d in range(4):
            v=mm.walls[x][y][d]
            if v is not None:
                known+=1
                if v is False: opens.append(d)
        if known==4 and len(opens)==1 and (x,y) not in goals:
            mm.set_wall_pair(x,y,opens[0],True)
            nx,ny=x+DIRS[opens[0]][0],y+DIRS[opens[0]][1]
            if mm.inb(nx,ny): enqueue(nx,ny)

def choose_dir(mm,dist,x,y,hd,allow_unknown):
    best=[]; val=10**9
    for d in range(4):
        ok = mm.is_open_or_unknown(x,y,d) if allow_unknown else mm.is_open(x,y,d)
        if not ok: continue
        nx,ny=x+DIRS[d][0],y+DIRS[d][1]
        if not mm.inb(nx,ny): continue
        v=dist[nx][ny]
        if v<val: val,best=v,[d]
        elif v==val: best.append(d)
    if not best: return None
    for pref in [hd,L(hd),R(hd),B(hd)]:
        if pref in best: return pref
    return best[0]

# ---------------- return to start & align ----------------
def return_to_start_and_align(mm, x, y, hd, target_hd=0):
    while (x, y) != (0, 0):
        dist = compute_distances(mm, goals={(0,0)}, allow_unknown=False)
        nd = choose_dir(mm, dist, x, y, hd, allow_unknown=False)
        if nd is None:
            dist2 = compute_distances(mm, goals={(0,0)}, allow_unknown=True)
            nd = choose_dir(mm, dist2, x, y, hd, allow_unknown=True)
            if nd is None:
                log("No way to return to start."); break
        turns, hd = turn_to(hd, nd)
        for t in turns: t()
        if not try_step():
            mm.set_wall_pair(x, y, hd, True)
            sense_here(mm, x, y, hd)
            continue
        mm.set_wall_pair(x, y, hd, False)
        x += DIRS[hd][0]; y += DIRS[hd][1]
        sense_here(mm, x, y, hd)
    turns, hd = turn_to(hd, target_hd)
    for t in turns: t()
    return x, y, hd

# ---------------- SPEED-RUN w/ per-cell replan & fillet ----------------
def speed_run_replan(mm, x, y, hd, goals, max_iters=10000):
    iters = 0
    while (x, y) not in goals and iters < max_iters:
        iters += 1
        path = astar_shortest_path(mm, start=(x, y), goals=goals)
        if not path:
            log("A*: no open-only path from current pose.")
            return False

        d = path[0]

        # same heading → advance one cell
        if d == hd:
            if API.wallFront():
                mm.set_wall_pair(x, y, hd, True)   # map correction
                sense_here(mm, x, y, hd)
                continue  # replan
            API.moveForward(1)
            mm.set_wall_pair(x, y, hd, False)
            x += DIRS[hd][0]; y += DIRS[hd][1]
            sense_here(mm, x, y, hd)
            continue

        # orthogonal turn → try fillet “60° style”
        if (d % 2) != (hd % 2):
            right = ((hd + 1) % 4 == d)

            # enter fillet: 45° toward target
            if right:
                API.turnRight45()
            else:
                API.turnLeft45()
            # check clearance for the first half-step
            if API.wallFront(1):
                # rollback and do a normal 90° + 1 cell
                if right:  API.turnLeft45()
                else:      API.turnRight45()
                turns, hd = turn_to(hd, d)
                for t in turns: t()
                if API.wallFront():
                    mm.set_wall_pair(x, y, hd, True); sense_here(mm, x, y, hd); continue
                API.moveForward(1)
                mm.set_wall_pair(x, y, hd, False)
                x += DIRS[hd][0]; y += DIRS[hd][1]
                sense_here(mm, x, y, hd)
                continue

            # commit first half forward
            API.moveForwardHalf(1)

            # exit fillet back to cardinal target
            if right:
                API.turnLeft45()
            else:
                API.turnRight45()
            hd = d

            # final half to cell center; if blocked, mark wall and rollback
            if API.wallFront(1):
                # This means target cell is not free → mark wall and back off 45° to be safe
                # (we're already on cardinal hd=d)
                mm.set_wall_pair(x, y, hd, True)
                # optional micro-fix: step back half if your sim supports; here just re-sense
                sense_here(mm, x, y, hd)
                # undo last 45 so we’re cardinal-aligned for next plan
                # (we're already aligned; nothing else to undo)
                continue  # replan from same cell

            API.moveForwardHalf(1)
            mm.set_wall_pair(x, y, hd, False)
            x += DIRS[hd][0]; y += DIRS[hd][1]
            sense_here(mm, x, y, hd)
            continue

        # 180° (hiếm): quay 180 rồi đi 1 ô
        turns, hd = turn_to(hd, d)
        for t in turns: t()
        if API.wallFront():
            mm.set_wall_pair(x, y, hd, True); sense_here(mm, x, y, hd); continue
        API.moveForward(1)
        mm.set_wall_pair(x, y, hd, False)
        x += DIRS[hd][0]; y += DIRS[hd][1]
        sense_here(mm, x, y, hd)

    return (x, y) in goals

# ---------------- main ----------------
def main():
    W,H=API.mazeWidth(),API.mazeHeight()
    goals=center_cells(W,H)
    log(f"FF + ReturnStart + A* Per-Cell Fillet | {W}x{H} goals={goals}")

    x,y,hd=0,0,0
    mm=MazeMap(W,H)

    # -------- Phase 1: Explore (unknown=open) --------
    sense_here(mm,x,y,hd)
    if USE_DEADEND_FILL: dead_end_fill(mm,goals)

    steps=0
    while True:
        if API.wasReset(): API.ackReset(); return
        steps+=1
        dist=compute_distances(mm,goals,allow_unknown=True)
        mark_overlay(x,y,dist[x][y])

        if (x,y) in goals:
            API.setColor(x,y,"R"); API.setText(x,y,"GOAL")
            log(f"Reached center in {steps} steps.")
            break

        nd=choose_dir(mm,dist,x,y,hd,allow_unknown=True)
        if nd is None: log("No neighbor (explore)."); return

        turns,hd=turn_to(hd,nd)
        for t in turns: t()

        if not try_step():
            mm.set_wall_pair(x,y,hd,True)
            sense_here(mm,x,y,hd)
            if USE_DEADEND_FILL: dead_end_fill(mm,goals)
            continue

        mm.set_wall_pair(x,y,hd,False)
        x+=DIRS[hd][0]; y+=DIRS[hd][1]
        sense_here(mm,x,y,hd)
        if USE_DEADEND_FILL: dead_end_fill(mm,goals)

    # -------- Return to (0,0) & align North --------
    x, y, hd = return_to_start_and_align(mm, x, y, hd, target_hd=0)
    if (x, y) != (0, 0):
        log("Could not return to start reliably. Abort.")
        return

    # -------- Phase 2: Per-cell A* speed-run (fillet + replan) --------
    ok = speed_run_replan(mm, x=0, y=0, hd=hd, goals=goals)
    log("Speed-run complete." if ok else "Speed-run aborted.")
if __name__=="__main__":
    main()
