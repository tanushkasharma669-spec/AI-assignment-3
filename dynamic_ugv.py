import random
import heapq
import time
import math
from typing import List, Tuple, Dict, Optional

class Node:
    def __init__(self, x, y, cost, heuristic, parent=None):
        self.x = x
        self.y = y
        self.cost = cost
        self.heuristic = heuristic
        self.total_cost = cost + heuristic
        self.parent = parent

    def __lt__(self, other):
        return self.total_cost < other.total_cost


class UGVGrid:
    def __init__(self, width=70, height=70, density_level='medium',
                 start=(0,0), goal=(69,69)):
        self.width = width
        self.height = height
        self.grid = [[0]*width for _ in range(height)]
        self.start = start
        self.original_start = start
        self.goal = goal
        self.generate_obstacles(density_level)

    def generate_obstacles(self, level):
        density = {'low':0.1,'medium':0.2,'high':0.3}.get(level,0.2)
        count = int(self.width*self.height*density)

        placed = 0
        while placed < count:
            x = random.randint(0,self.width-1)
            y = random.randint(0,self.height-1)

            if (x,y) in [self.start,self.goal]:
                continue

            if self.grid[y][x] == 0:
                self.grid[y][x] = 1
                placed += 1

    def update_dynamic_obstacles(self, prob=0.02):
        for _ in range(int(self.width*self.height*prob)):
            x = random.randint(0,self.width-1)
            y = random.randint(0,self.height-1)

            if (x,y) in [self.start,self.goal]:
                continue

            self.grid[y][x] = 1 - self.grid[y][x]

    def heuristic(self,a,b):
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

    def get_neighbors(self,x,y):
        dirs = [(0,1),(1,0),(0,-1),(-1,0),(1,1),(-1,-1),(1,-1),(-1,1)]
        res = []

        for dx,dy in dirs:
            nx,ny = x+dx,y+dy
            if 0<=nx<self.width and 0<=ny<self.height:
                if self.grid[ny][nx]==0:
                    cost = 1 if dx==0 or dy==0 else math.sqrt(2)
                    res.append(((nx,ny),cost))
        return res

    def print_grid(self,path=None):
        path_set = set(path) if path else set()
        for y in range(self.height):
            row=""
            for x in range(self.width):
                if (x,y)==self.original_start: row+="S"
                elif (x,y)==self.goal: row+="G"
                elif (x,y) in path_set: row+="*"
                elif self.grid[y][x]==1: row+="#"
                else: row+="."
            print(row)
        print()


def astar_search(env):
    start_time = time.time()

    start = Node(env.start[0],env.start[1],0,env.heuristic(env.start,env.goal))
    open_set = []
    heapq.heappush(open_set,start)

    best = {env.start:0}
    nodes_expanded = 0

    while open_set:
        curr = heapq.heappop(open_set)

        if (curr.x,curr.y)==env.goal:
            path=[]
            while curr:
                path.append((curr.x,curr.y))
                curr=curr.parent
            path.reverse()
            return path,nodes_expanded,time.time()-start_time

        nodes_expanded+=1

        for (nx,ny),cost in env.get_neighbors(curr.x,curr.y):
            new_cost = best[(curr.x,curr.y)] + cost

            if (nx,ny) not in best or new_cost<best[(nx,ny)]:
                best[(nx,ny)] = new_cost
                h = env.heuristic((nx,ny),env.goal)
                heapq.heappush(open_set,Node(nx,ny,new_cost,h,curr))

    return None,nodes_expanded,time.time()-start_time


def dynamic_astar_navigation(env):
    current = env.start
    full_path=[current]

    total_nodes=0
    total_time=0

    while current!=env.goal:
        env.start=current
        path,nodes,t = astar_search(env)

        total_nodes+=nodes
        total_time+=t

        if not path:
            return None,total_nodes,total_time

        if len(path)<2: break

        next_step = path[1]

        env.update_dynamic_obstacles()

        if env.grid[next_step[1]][next_step[0]]==1:
            print(" Path blocked → Replanning...")
            continue

        current = next_step
        full_path.append(current)

    return full_path,total_nodes,total_time


def get_coord(label,w,h):
    while True:
        try:
            x=int(input(f"{label} X (0-{w-1}): "))
            y=int(input(f"{label} Y (0-{h-1}): "))
            if 0<=x<w and 0<=y<h:
                return (x,y)
        except:
            pass
        print("Invalid input")


def main():
    print("Dynamic UGV Navigation (A*)")

    start = get_coord("Start",70,70)
    goal = get_coord("Goal",70,70)

    env = UGVGrid(70,70,'medium',start,goal)

    path,nodes,exec_time = dynamic_astar_navigation(env)

    if path:
        path_length = sum(
            1 if abs(path[i][0]-path[i-1][0]) + abs(path[i][1]-path[i-1][1]) == 1
            else math.sqrt(2)
            for i in range(1,len(path))
        )

        straight_line = math.sqrt((goal[0]-start[0])**2 + (goal[1]-start[1])**2)
        detour_factor = path_length/straight_line if straight_line>0 else 1
        efficiency = (straight_line/path_length*100) if path_length>0 else 0

        print("Path Found!")
        print("Path :")
        print(f"Start: {path[:5]} ... End: {path[-5:]}")
        env.print_grid(path)

        print(f"-- Measures of Effectiveness --")
        print(f"Path Length        : {path_length:.2f} units")
        print(f"Straight-Line Dist : {straight_line:.2f} units")
        print(f"Detour Factor      : {detour_factor:.3f}x")
        print(f"Path Efficiency    : {efficiency:.1f}%")
        print(f"Nodes Expanded     : {nodes}")
        print(f"Execution Time     : {exec_time:.5f} seconds")

    else:
        print("No path found! Dense obstacles blocked the goal.")
        env.print_grid()

        print(f"-- Measures of Effectiveness --")
        print(f"Nodes Expanded (searched before failing): {nodes}")
        print(f"Execution Time : {exec_time:.5f} seconds")


if __name__ == "__main__":
    main()