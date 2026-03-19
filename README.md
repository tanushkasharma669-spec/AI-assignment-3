# AI Assignments 3

This project implements core AI search algorithms for shortest path computation and UGV navigation in static and dynamic environments.

---

## 1: Dijkstra’s Algorithm (Uniform Cost Search)

Computes the shortest path between Indian cities using real road distances.

- Graph represented using adjacency lists  
- Data loaded from CSV file (`edge_list.csv`)  
- Supports user input for source and destination  

**Key Concepts**
- Uniform Cost Search  
- Graph Traversal  
- Priority Queue (Min Heap)  

**Files**
- `dijkstra_india.py`  
- `edge_list.csv`  

**Example Output**  
Path: Delhi → Agra → Jaipur  
Total Distance: 450 km  

---

## 2: UGV Navigation with Static Obstacles

Simulates a UGV navigating a **70×70 grid** with fixed obstacles using A*.

- Random obstacle generation  
- Density levels: Low, Medium, High  
- 8-directional movement  
- Optimal path using A*  

**Measures of Effectiveness**
- Path Length  
- Straight-Line Distance  
- Detour Factor  
- Path Efficiency  
- Nodes Expanded  
- Execution Time  

**File**
- `Static_UGV.py`  

---

## 3: UGV Navigation with Dynamic Obstacles

Extends static navigation to a dynamic environment.

- Obstacles change during execution  
- Environment is partially unknown  
- Uses **Repeated A*** (online replanning)

**Approach**
- Plan path using A*  
- Move step-by-step  
- Replan if path is blocked  

**Features**
- Dynamic obstacle updates  
- Real-time replanning  
- Grid-based visualization  

**File**
- `Dynamic_UGV.py`  

---

## How to Run

1. Dijkstra (Cities)
python dijkstra.py

2. Static UGV
python static_ugv.py

3. Dynamic UGV
python dynamic_ugv.py

## Algorithms Used

Dijkstra’s Algorithm (Uniform Cost Search)

A* Search Algorithm

Repeated A* (Dynamic Replanning)
