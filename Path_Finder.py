import heapq
import matplotlib.pyplot as plt

import networkx as nx

class Graph:
    def __init__(self):
        self.edges = {}

    def add_edge(self, u, v, weight):
        if u not in self.edges:
            self.edges[u] = []
        self.edges[u].append((v, weight))

        # For undirected graph, add the reverse edge
        if v not in self.edges:
            self.edges[v] = []
        self.edges[v].append((u, weight))

    def dijkstra(self, start):
        # Intialize distances and priority queue
        distances = {node: float('infinity') for node in self.edges}

        distances[start] = 0
        priority_queue = [(0, start)]
        previous_nodes = {node: None for node in self.edges}

        while priority_queue:
            current_distance, current_node = heapq.heappop(priority_queue)

            # Nodes can only be added once to the queue
            if current_distance > distances[current_node]:
                continue
            for neighbour, weight in self.edges[current_node]:
                distance = current_distance + weight

                # Only consider this new path if it's better
                if distance < distances[neighbour]:
                    distances[neighbour] = distance
                    previous_nodes[neighbour] = current_node

                    heapq.heappush(priority_queue, (distance, neighbour))

        return distances, previous_nodes

    def shortest_path(self, start, end):
        distances, previous_nodes = self.dijkstra(start)
        path = []
        current_node = end

        while current_node is not None:
            path.append(current_node)
            current_node = previous_nodes[current_node]

        path.reverse()
        return path, distances[end]

    def visualize_graph(self):
        G = nx.Graph()
        for u in self.edges:
            for v, weight in self.edges[u]:
                G.add_edge(u, v, weight=weight)

        pos = nx.spring_layout(G)
        nx.draw(G, pos, with_labels=True, node_size=700, node_color='lightblue')
        edge_labels = nx.get_edge_attributes(G, 'weight')

        nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)

        plt.show()


if __name__ == "__main__":
    g = Graph()
    g.add_edge('A', 'B', 1)
    g.add_edge('A', 'C', 2)
    g.add_edge('B', 'C', 1)
    g.add_edge('B', 'D', 5)
    g.add_edge('C', 'D', 1)

    start_node = 'A'
    end_node = 'C'

    path, distance = g.shortest_path(start_node, end_node)

    joined_path = ' ---> '.join(path)

    print(f'Shortest path from {start_node} to {end_node}: {joined_path} with distance {distance}')

    # print('Shortest path from ' + start_node + ' to ' + end_node + ':' + ' ---> '.join(path) + ' with distance ' + distance)

    g.visualize_graph()