"""
Making a weighted graph and implementing a dijskstra algorithm
"""

import networkx as nx
import matplotlib.pyplot as plt


w_graph = nx.Graph()
w_graph.add_nodes_from(['S', 'A', 'B', 'C', 'D',])
w_graph.add_weighted_edges_from([
    ('S', 'A', 20),
    ('S', 'D', 10),
    ('A', 'B', 20),
    ('A', 'C', 50),
    ('A', 'D', 20),
    ('B', 'C', 10),
    ('C', 'D', 50)
    ])

def get_unvisited_nodes(graph:nx.Graph):
    '''
    A Function that returns a list of nodes

    Parameters:
    -----------
    g : nx.Graph
        a graph made with networkx library
    '''
    unvisited_nodes = []
    for nodes in graph.nodes():
        unvisited_nodes.append(nodes)
    return unvisited_nodes

def init_costs(nodes:list, start:str) -> dict:
    '''
    fuction to create a cost dictionary

    parameters
    ----------
    nodes : list
        list of all nodes

    start : str
        name of the start node
    '''
    costs = {}
    for node in nodes:
        if node == start:
            costs[node] = (0, start)
        else:
            costs[node] =  (float('inf'), None)
    return costs


def set_costs(graph:nx.Graph, start:str) -> dict:
    '''
    A Function that returns a dict of tuples conating:
    node : (cost, successor)

    Parameters:
    -----------
    graph : nx.Graph
        weighted Graph from networkx library

    start: str
        Name of the start node

    '''
    nodes = get_unvisited_nodes(graph)
    costs = init_costs(nodes, start)
    cache = init_costs(nodes, start)

    while cache:
        current_position = min(cache, key=cache.get)
        cache.pop(current_position)
        nodes.remove(current_position)
        neighbors = graph.neighbors(current_position)
        for neighbor in neighbors:
            if current_position == start:
                costs[neighbor] = (graph[start][neighbor]['weight'], start)
            else:
                sum_cost = graph[current_position][neighbor]['weight'] + costs[current_position][0]
                if sum_cost < costs[neighbor][0]:
                    costs[neighbor] = (sum_cost, current_position)
    return costs

def dijskrta(graph: nx.Graph, start: str, goal: str):
    '''
    Dijskrta algorithm

    Parameters
    ----------
    graph : nx.Graph
        weighted graph from networkx library

    '''
    cost_dict = set_costs(graph, start)
    route = ""
    route_cache = goal

    while route_cache != start:
        if cost_dict[route_cache][1] != goal:
            route += cost_dict[route_cache][1] + " "
            route_cache = cost_dict[route_cache][1]
    route = goal + ' ' + route
    route = route[::-1]
    route = route[1:]
    route = route.replace(' ', ' ▶ ')
    print(
        f'The shortest path from {start} to {goal} is:\n'
        f'{route}\n'
        f'Costs: {cost_dict[goal][0]}'
    )

while True:
    possible_nodes = get_unvisited_nodes(w_graph)
    point_of_origin = input(f'Choose a point of origin\n{possible_nodes}\n')
    if point_of_origin not in possible_nodes:
        print('Please choose a point of origin from the given list\n')
    else:
        break
while True:
    possible_goals = get_unvisited_nodes(w_graph)
    goal_line = input(f'Choose a point of origin\n{possible_goals}\n')
    if goal_line not in possible_goals:
        print('Please choose a goal from the given list\n')
    else:
        break

dijskrta(w_graph, point_of_origin, goal_line)


pos = nx.planar_layout(w_graph)
nx.draw_networkx(w_graph, pos)
labels =nx.get_edge_attributes(w_graph, 'weight')
nx.draw_networkx_edge_labels(w_graph, pos, edge_labels=labels)
path = nx.shortest_path(w_graph, source=point_of_origin, target=goal_line)
path_edges = list(zip(path,path[1:]))
nx.draw_networkx_nodes(w_graph, pos, nodelist=path, node_color='r')
nx.draw_networkx_edges(w_graph, pos, edgelist=path_edges, edge_color='r')
plt.axis('equal')
plt.show()
