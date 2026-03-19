import heapq
import csv
from typing import Set, Dict, List, Tuple, Optional

#undirected graph
class Graph:
    def __init__(self) -> None:
        self.nodes: Set[str] = set()
        self.edges: Dict[str, List[str]] = {}
        self.distances: Dict[Tuple[str, str], int] = {}

    def add_edge(self, from_node: str, to_node: str, distance: int) -> None:
        for node in (from_node, to_node):
            if node not in self.nodes:
                self.nodes.add(node)
                self.edges[node] = []

        self.edges[from_node].append(to_node)
        self.edges[to_node].append(from_node)    
        self.distances[(from_node, to_node)] = distance
        self.distances[(to_node, from_node)] = distance

# LOAD CSV  (Origin, Destination, Distance)
def load_graph(filename: str) -> Graph:
    graph = Graph()
    with open(filename, newline='', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            origin      = row['Origin'].strip()
            destination = row['Destination'].strip()
            distance    = int(row['Distance'].strip())
            graph.add_edge(origin, destination, distance)
    print(f"Loaded {len(graph.nodes)} cities from '{filename}'.")
    return graph

# Dijkstra's Algorithm
def dijkstra(graph: Graph, start: str, goal: str) -> Tuple[Optional[List[str]], float]:
    if start not in graph.nodes or goal not in graph.nodes:
        return None, float('inf')

    # Priority queue: (cumulative_distance, node)
    pq: List[Tuple[float, str]] = [(0.0, start)]

    shortest: Dict[str, float] = {node: float('inf') for node in graph.nodes}
    shortest[start] = 0.0

    previous: Dict[str, Optional[str]] = {node: None for node in graph.nodes}
    visited:  Set[str] = set()

    while pq:
        current_dist, current_node = heapq.heappop(pq)

        if current_node == goal:
            # Reconstruct path
            path: List[str] = []
            node: Optional[str] = current_node
            while node is not None:
                path.insert(0, node)
                node = previous[node]
            return path, current_dist

        if current_node in visited:
            continue
        visited.add(current_node)

        for neighbour in graph.edges[current_node]:
            if neighbour in visited:
                continue
            new_dist = current_dist + graph.distances[(current_node, neighbour)]
            if new_dist < shortest[neighbour]:
                shortest[neighbour] = new_dist
                previous[neighbour] = current_node
                heapq.heappush(pq, (new_dist, neighbour))

    return None, float('inf')   # no path found

def main():
    graph = load_graph("edge_list.csv")

    cities = sorted(graph.nodes)

    print("\n--- Dijkstra's Algorithm (Uniform Cost Search) ---")
    print("Available cities:")
    for i, city in enumerate(cities, 1):
        print(f"  [{i:2}] {city}")
    print("-" * 50)

    while True:
        print("\nType city names exactly as shown above.")
        src = input("Source city      (or 'quit'): ").strip()
        if src.lower() == 'quit':
            break
        if src not in graph.nodes:
            print(f"  '{src}' not found. Check spelling.")
            continue

        dst = input("Destination city             : ").strip()
        if dst not in graph.nodes:
            print(f"  '{dst}' not found. Check spelling.")
            continue

        path, distance = dijkstra(graph, src, dst)

        if path:
            print(f"\n  Path : {' -> '.join(path)}")
            print(f"  Total: {int(distance)} km")
        else:
            print(f"\n  No path found between {src} and {dst}.")

    print("Goodbye!")

if __name__ == "__main__":
    main()