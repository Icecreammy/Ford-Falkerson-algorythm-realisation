import random
import time

class Graph:
    def __init__(self, vertices):
        self.V = vertices
        self.graph = [[0] * vertices for i in range(vertices)]

    def add_edge(self, u, v, capacity):
        self.graph[u][v] = capacity

    def dfs(self, u, t, flow, visited, parent):
        visited[u] = True
        if u == t:
            return flow
        for v in range(self.V):
            if not visited[v] and self.graph[u][v] > 0:
                parent[v] = u
                new_flow = min(flow, self.graph[u][v])
                result = self.dfs(v, t, new_flow, visited, parent)
                if result > 0:
                    return result
        return 0

    def ford_fulkerson(self, source, sink):
        max_flow = 0
        parent = [-1] * self.V
        visited = [False] * self.V

        while True:
            visited = [False] * self.V
            flow = self.dfs(source, sink, float('inf'), visited, parent)
            if flow == 0:
                break
            max_flow += flow
            v = sink
            while v != source:
                u = parent[v]
                self.graph[u][v] -= flow
                self.graph[v][u] += flow
                v = u

        return max_flow


def generate_random_graph(vertices, density=0.3, min_capacity=1, max_capacity=10, allow_cycles=True, allow_fractions=False):
    """
    Генерация случайного графа с заданными параметрами.
    vertices: Количество вершин.
    density: Плотность графа (вероятность наличия ребра).
    min_capacity: Минимальная пропускная способность ребра.
    max_capacity: Максимальная пропускная способность ребра.
    allow_cycles: Разрешить циклы в графе.
    allow_fractions: Разрешить дробные пропускные способности.
    """
    graph = Graph(vertices)
    for u in range(vertices):
        for v in range(vertices):
            if u != v and random.random() < density:
                # Генерация пропускной способности
                if allow_fractions:
                    capacity = random.uniform(min_capacity, max_capacity)
                else:
                    capacity = random.randint(min_capacity, max_capacity)

                # Добавление ребра
                graph.add_edge(u, v, capacity)

                # Добавление обратного ребра для циклов
                if allow_cycles and random.random() < 0.5:
                    if allow_fractions:
                        reverse_capacity = random.uniform(min_capacity, max_capacity)
                    else:
                        reverse_capacity = random.randint(min_capacity, max_capacity)
                    graph.add_edge(v, u, reverse_capacity)
    return graph


def test_algorithm_efficiency(vertices_list, density=0.3, allow_cycles=True, allow_fractions=False):
    results = []
    for vertices in vertices_list:
        graph = generate_random_graph(vertices, density, allow_cycles=allow_cycles, allow_fractions=allow_fractions)
        source = 0
        sink = vertices - 1

        start_time = time.time()
        max_flow = graph.ford_fulkerson(source, sink)
        end_time = time.time()

        elapsed_time = end_time - start_time
        results.append((vertices, elapsed_time, max_flow))
        print(f"Vertices: {results[-1][0]}, Time: {results[-1][1]:.6f} seconds, Max Flow: {results[-1][2]}")

    return results


def main1():
    # Параметры графа
    vertices = 6  # Количество вершин
    source = 0    # Исток
    sink = vertices - 1  # Сток

    # Генерация случайного графа с циклами и дробными значениями
    graph = generate_random_graph(vertices, density=0.3, min_capacity=1, max_capacity=10, allow_cycles=False, allow_fractions=False)

    # Вывод графа
    print("Случайный граф (матрица смежности):")
    for row in graph.graph:
        print([round(val, 2) for val in row])  # Округление для удобства чтения

    # Вычисление максимального потока
    max_flow = graph.ford_fulkerson(source, sink)
    print(f"\nМаксимальный поток из истока {source} в сток {sink}: {max_flow}")
    print(f"\nВремя выполнения программы {time.process_time()}")


def main2():
    # Список количеств вершин для тестирования
    vertices_list = [25, 50, 75, 100, 125, 150, 175, 200, 225, 250, 275, 300]

    # Тестирование алгоритма
    print("Тестирование алгоритма Форда-Фалкерсона:")
    results = test_algorithm_efficiency(vertices_list, density=0.5, allow_cycles=True, allow_fractions=True)

    # Вывод результатов
    print("\nРезультаты тестирования:")
    for vertices, time_taken, max_flow in results:
        print(f"Vertices: {vertices}, Time: {time_taken:.6f} seconds, Max Flow: {max_flow}")


if __name__ == "__main__":
    main2()
