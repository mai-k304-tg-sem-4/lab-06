from collections import deque

class Graph:
    def __init__(self, edges_matrix):
        self.edges_matrix = edges_matrix

    def printSolution(self, distances, start_vertex):
        INF = float('inf')
        print("Вершина  Расстояние")
        for node in range(len(self.edges_matrix)):
            if node != start_vertex and distances[node] != INF:
                print(start_vertex + 1, "->", node + 1, " = ", distances[node])

    def bellman_ford(self, start_vertex):
        INF = float('inf')
        matrix = self.edges_matrix
        n = len(matrix)
        distances = [INF] * n
        distances[start_vertex] = 0
        for i in range(n - 1):
            for u in range(n):
                for v in range(n):
                    if matrix[u][v] != '0':
                        weight = int(matrix[u][v])
                        if distances[u] + weight < distances[v]:
                            distances[v] = distances[u] + weight
        for u in range(n):
            for v in range(n):
                if matrix[u][v] != '0':
                    weight = int(matrix[u][v])
                    if distances[u] + weight < distances[v]:
                        print("Граф содержит отрицательный цикл")
                        return
        self.printSolution(distances, start_vertex)

    def dijkstra(self, start_vertex):
        INF = float('inf')
        dist = [INF] * len(self.edges_matrix)
        dist[start_vertex] = 0
        sptSet = [False] * len(self.edges_matrix)
        for cout in range(len(self.edges_matrix)):
            u = self.minDistance(dist, sptSet)
            sptSet[u] = True
            for v in range(len(self.edges_matrix)):
                if (int(self.edges_matrix[u][v]) > 0 and sptSet[v] == False and dist[v] > dist[u] + int(
                        self.edges_matrix[u][v])):
                    dist[v] = dist[u] + int(self.edges_matrix[u][v])
            for u in range(len(self.edges_matrix)):
                for v in range(len(self.edges_matrix)):
                    if self.edges_matrix[u][v] != '0':
                        weight = int(self.edges_matrix[u][v])
                        if dist[u] != INF and dist[u] + weight < dist[v]:
                            print("Граф содержит отрицательный цикл")
                            return
        self.printSolution(dist, start_vertex)

    def levit(self, start_vertex):
        distance = [float('inf')] * len(self.edges_matrix)
        distance[start_vertex] = 0
        queue = deque([start_vertex])
        in_queue = [False] * len(self.edges_matrix)
        in_queue[start_vertex] = True
        predecessors = [None] * len(self.edges_matrix)
        count_iteration = 0
        while queue:
            count_iteration += 1
            if count_iteration > len(self.edges_matrix):
                print("Граф содержит отрицательный цикл")
                return
            u = queue.popleft()
            in_queue[u] = False
            for v in range(len(self.edges_matrix)):
                if self.edges_matrix[u][v] != '0':
                    weight = int(self.edges_matrix[u][v])
                    if distance[u] + weight < distance[v]:
                        distance[v] = distance[u] + weight
                        predecessors[v] = u
                        if not in_queue[v]:
                            queue.append(v)
                            in_queue[v] = True
        self.printSolution(distance, start_vertex)

    def minDistance(self, dist, sptSet):
        min = float('inf')
        min_index = 0
        for v in range(len(self.edges_matrix)):
            if dist[v] < min and sptSet[v] == False:
                min = dist[v]
                min_index = v
        return min_index


edges_matrix = []
file = input()
with open(file, "r") as file:
    for line in file:
        edges_matrix.append(line.strip().split(' '))

instance = Graph(edges_matrix)

print("Введите начальную вершину")
start = int(input())
start -= 1

instance.bellman_ford(start)
instance.dijkstra(start)
instance.levit(start)
