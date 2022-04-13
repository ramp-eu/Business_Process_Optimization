from copy import copy, deepcopy

def StatesfromList2Graph(LX):
    ii=0
    LX_short=dict()
    for xx in LX:
        x=''
        for yy in xx:
            x = x + str(yy)
        LX_short.update({ii:x})
        ii += 1
    return LX_short


def GraphWITHAdjacency(LX_short, adj_matrix):
    graph = {}
    for r in range(len(adj_matrix)):
        g = {}
        for x in range(len(adj_matrix)):
            start = LX_short[r]
            if (adj_matrix[r][x] != 0):
                nextstation = LX_short[x]
                #print(start, nextstation, {nextstation:adj_matrix[r][x]})
                g.update({nextstation:adj_matrix[r][x]})
        graph.update({start:g})
    return graph


def dijkstra(graph, start, goal):
    VERBOSE=False
    shortest_path = {}
    pred = {}
    unseenNodes = deepcopy(graph)
    infinity = 7777777
    path = []

    for node in unseenNodes:
        shortest_path[ node ] = infinity
    shortest_path[ start ] = 0.
    #print(shortest_path)

    while unseenNodes:
        minNode = None
        if VERBOSE: print("\n=============================\n PY_dijkstra: graph=",graph,"\n====================")
        if VERBOSE: print("\n PY_dijkstra: Strt - Goal (%s - %s)=")%(start,goal)
        for node in unseenNodes:
            if minNode is None:
                minNode = node
            elif shortest_path[node] < shortest_path[minNode]:
                minNode = node
            if VERBOSE: print("PY_dijkstra: Setting minNode for node >>",node, "<<   temp  minnode=",minNode)

        for childNode, weight in graph[minNode].items():
            if VERBOSE: print("vvvvvvvvvvvvvvvvvvvvvvvvvvv")
            if VERBOSE: print('   PY_dijkstra: Checking minNode >>',minNode,'<<   graph[minNode]=',graph[minNode])
            #if VERBOSE: print("PY_dijkstra:         graph[minNode] >>",graph[minNode])
            #if VERBOSE: print("PY_dijkstra: graph[minNode].items() >>",graph[minNode].items())
            if VERBOSE: print('   PY_dijkstra: Checking for >> weight + shortest_path[minNode] < shortest_path[childNode]')
            if VERBOSE: print('   PY_dijkstra: Checking for >>',weight,' +  ',shortest_path[minNode],'  <  ',shortest_path[childNode])
            if weight + shortest_path[minNode] < shortest_path[childNode]:
                shortest_path[ childNode ] = weight + shortest_path[minNode]
                pred[childNode] = minNode
                if VERBOSE: print('      >PY_dijkstra:  shortest_path[',childNode,'] = ',shortest_path[ childNode ])
                if VERBOSE: print('      >PY_dijkstra:           pred[',childNode,'] = ',pred[childNode])
                #if VERBOSE: print("PY_dijkstra:                childNode  >>",childNode)
                #if VERBOSE: print("PY_dijkstra: shortest_path[ childNode ] >>",shortest_path[ childNode ])
                #if VERBOSE: print("PY_dijkstra:            pred[childNode] >>",pred[childNode])
        unseenNodes.pop(minNode)

    currentNode = goal
    while currentNode != start:
        try:
            path.insert(0, currentNode)
            currentNode = pred[currentNode]
        except KeyError:
            print('Path not reachable')
            break
    path.insert(0, start)
    print('..................',shortest_path[goal])
    if shortest_path[goal] != infinity:
        print('Shortest path is ' + str(shortest_path[goal]))
        print('The path is ' + str(path))
    return path, shortest_path[goal]
