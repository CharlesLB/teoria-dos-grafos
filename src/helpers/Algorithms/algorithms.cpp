#include "algorithms.hpp"

#include <algorithm>
#include <iostream>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <stack>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "../../lib/Edge/edge.hpp"
#include "../../lib/Graph/graph.hpp"
#include "../../lib/Node/node.hpp"

vector<Node*> getOpenNeighborhoodNodesByNode(Node* node) {
    vector<Node*> openNeighborhoodNodes;
    unordered_set<Node*> visitedNodes;

    if (node == nullptr) {
        return openNeighborhoodNodes;
    }

    Edge* edge = node->getFirstEdge();

    while (edge != nullptr) {
        Node* neighborNode = edge->getTail();

        if (neighborNode != nullptr && visitedNodes.find(neighborNode) == visitedNodes.end()) {
            openNeighborhoodNodes.push_back(neighborNode);
            visitedNodes.insert(neighborNode);
        }

        edge = edge->getNextEdge();
    }

    return openNeighborhoodNodes;
}

vector<Node*> getClosedNeighborhoodNodesByNode(Node* node) {
    vector<Node*> openNeighborhoodNodes = getOpenNeighborhoodNodesByNode(node);
    vector<Node*> closedNeighborhoodNodes;

    closedNeighborhoodNodes.push_back(node);

    for (Node* openNeighborhoodNode : openNeighborhoodNodes) {
        closedNeighborhoodNodes.push_back(openNeighborhoodNode);
    }

    return closedNeighborhoodNodes;
}

vector<Node*> getArticulationNodesInGraph(Graph* graph) {
    vector<Node*> articulationNodes;
    unordered_map<Node*, bool> visited;
    unordered_map<Node*, int> discoveryTime;
    unordered_map<Node*, int> low;
    unordered_map<Node*, Node*> parentMap;
    unordered_map<Node*, bool> articulationPoint;
    int time = 0;

    Node* startNode = graph->getFirstNode();
    while (startNode != nullptr) {
        if (!visited[startNode]) {
            DFSArticulationPoints(startNode, -1, visited, discoveryTime, low, parentMap, articulationPoint, time, articulationNodes);
        }
        startNode = startNode->getNextNode();
    }

    return articulationNodes;
}

void DFSArticulationPoints(Node* node, int parent, unordered_map<Node*, bool>& visited,
                           unordered_map<Node*, int>& discoveryTime,
                           unordered_map<Node*, int>& low,
                           unordered_map<Node*, Node*>& parentMap,
                           unordered_map<Node*, bool>& articulationPoint,
                           int& time, vector<Node*>& articulationNodes) {
    visited[node] = true;
    discoveryTime[node] = low[node] = ++time;
    int children = 0;

    for (Edge* edge : node->getEdges()) {
        Node* adjacent = edge->getTail();

        if (!visited[adjacent]) {
            children++;
            parentMap[adjacent] = node;
            DFSArticulationPoints(adjacent, node->getId(), visited, discoveryTime, low, parentMap, articulationPoint, time, articulationNodes);

            low[node] = min(low[node], low[adjacent]);

            if (parent == -1 && children > 1) {
                articulationPoint[node] = true;
            }

            if (parent != -1 && low[adjacent] >= discoveryTime[node]) {
                articulationPoint[node] = true;
            }
        } else if (adjacent != parentMap[node]) {
            low[node] = min(low[node], discoveryTime[adjacent]);
        }
    }

    if (articulationPoint[node] && find(articulationNodes.begin(), articulationNodes.end(), node) == articulationNodes.end()) {
        articulationNodes.push_back(node);
    }
}

vector<Edge*> getBridgeEdgesInGraph(Graph* graph) {
    vector<Edge*> bridgeEdges;
    unordered_map<Node*, bool> visited;
    unordered_map<Node*, int> discoveryTime;
    unordered_map<Node*, int> low;
    int time = 0;

    Node* startNode = graph->getFirstNode();
    while (startNode != nullptr) {
        if (!visited[startNode]) {
            DFSBridgeEdges(startNode, -1, visited, discoveryTime, low, time, bridgeEdges);
        }
        startNode = startNode->getNextNode();
    }

    return bridgeEdges;
}

void DFSBridgeEdges(Node* node, int parent, unordered_map<Node*, bool>& visited,
                    unordered_map<Node*, int>& discoveryTime,
                    unordered_map<Node*, int>& low,
                    int& time, vector<Edge*>& bridgeEdges) {
    visited[node] = true;
    discoveryTime[node] = low[node] = ++time;

    for (Edge* edge : node->getEdges()) {
        Node* adjacent = edge->getTail();

        if (!visited[adjacent]) {
            DFSBridgeEdges(adjacent, node->getId(), visited, discoveryTime, low, time, bridgeEdges);

            low[node] = min(low[node], low[adjacent]);

            if (low[adjacent] > discoveryTime[node]) {
                bridgeEdges.push_back(edge);
            }
        } else if (adjacent->getId() != parent) {
            low[node] = min(low[node], discoveryTime[adjacent]);
        }
    }
}

struct NodeDistance {
    Node* node;
    int distance;

    NodeDistance(Node* n, int d) : node(n), distance(d) {}

    bool operator>(const NodeDistance& other) const {
        return distance > other.distance;
    }
};

Graph* getMinimumPathAndCostByDijkstra(Graph* graph, Node* sourceNode, Node* targetNode) {
    if (graph == nullptr || sourceNode == nullptr || targetNode == nullptr) {
        return nullptr;
    }

    unordered_map<Node*, int> distances;
    unordered_map<Node*, Node*> previousNodes;
    priority_queue<pair<int, Node*>, vector<pair<int, Node*>>, greater<>> priorityQueue;

    for (Edge* edge : graph->getEdges()) {
        distances[edge->getHead()] = numeric_limits<int>::max();
        distances[edge->getTail()] = numeric_limits<int>::max();
    }
    distances[sourceNode] = 0;

    priorityQueue.push({0, sourceNode});

    while (!priorityQueue.empty()) {
        Node* currentNode = priorityQueue.top().second;
        priorityQueue.pop();

        if (currentNode == targetNode) {
            break;
        }

        for (Edge* edge = currentNode->getFirstEdge(); edge != nullptr; edge = edge->getNextEdge()) {
            Node* neighborNode = edge->getTail();
            int newDist = distances[currentNode] + edge->getWeight();

            if (newDist < distances[neighborNode]) {
                distances[neighborNode] = newDist;
                previousNodes[neighborNode] = currentNode;
                priorityQueue.push({newDist, neighborNode});
            }
        }
    }

    Graph* minimumPathGraph = new Graph(graph->isDirected(), graph->isWeightedEdges(), graph->isWeightedNodes());
    Node* currentNode = targetNode;

    while (currentNode != nullptr) {
        Node* previousNode = previousNodes[currentNode];
        if (previousNode != nullptr) {
            Edge* edge = graph->findEdgeByNodes(previousNode, currentNode);
            minimumPathGraph->createEdge(minimumPathGraph->createOrUpdateNode(previousNode->getId(), previousNode->getWeight()),
                                         minimumPathGraph->createOrUpdateNode(currentNode->getId(), currentNode->getWeight()),
                                         edge->getWeight());
        }
        currentNode = previousNode;
    }

    return minimumPathGraph;
}

Graph* getMinimumPathAndCostByFloyd(Graph* graph, Node* sourceNode, Node* targetNode) {
    int numNodes = graph->getNumNodes();

    if (numNodes == 0) {
        return nullptr;
    }

    vector<vector<int>> dist(numNodes, vector<int>(numNodes, numeric_limits<int>::max()));
    vector<vector<Node*>> next(numNodes, vector<Node*>(numNodes, nullptr));

    for (Edge* edge : graph->getEdges()) {
        if (edge == nullptr || edge->getHead() == nullptr || edge->getTail() == nullptr) {
            continue;
        }
        int u = edge->getHead()->getId();
        int v = edge->getTail()->getId();

        if (u < 0 || u >= numNodes || v < 0 || v >= numNodes) {
            continue;
        }

        int weight = edge->getWeight();

        dist[u][v] = weight;
        next[u][v] = edge->getTail();

        if (!graph->isDirected()) {
            dist[v][u] = weight;
            next[v][u] = edge->getHead();
        }
    }

    for (int i = 0; i < numNodes; ++i) {
        dist[i][i] = 0;
        next[i][i] = graph->findNodeById(i);
    }

    for (int k = 0; k < numNodes; ++k) {
        for (int i = 0; i < numNodes; ++i) {
            for (int j = 0; j < numNodes; ++j) {
                if (dist[i][k] != numeric_limits<int>::max() && dist[k][j] != numeric_limits<int>::max()) {
                    if (dist[i][j] > dist[i][k] + dist[k][j]) {
                        dist[i][j] = dist[i][k] + dist[k][j];
                        next[i][j] = next[i][k];
                    }
                }
            }
        }
    }

    int sourceId = sourceNode->getId();
    int targetId = targetNode->getId();

    if (sourceId < 0 || sourceId >= numNodes || targetId < 0 || targetId >= numNodes) {
        return nullptr;
    }

    if (next[sourceId][targetId] == nullptr) {
        return nullptr;
    }

    Graph* minimumPathGraph = new Graph(graph->isDirected(), graph->isWeightedEdges(), graph->isWeightedNodes());
    Node* currentNode = sourceNode;

    while (currentNode != targetNode) {
        Node* nextNode = next[currentNode->getId()][targetNode->getId()];
        Edge* edge = graph->findEdgeByNodes(currentNode, nextNode);
        minimumPathGraph->createEdge(
            minimumPathGraph->createOrUpdateNode(currentNode->getId(), currentNode->getWeight()),
            minimumPathGraph->createOrUpdateNode(nextNode->getId(), nextNode->getWeight()),
            edge->getWeight());
        currentNode = nextNode;
    }

    return minimumPathGraph;
}

Graph* getComplementGraph(Graph* graph) {
    if (graph == nullptr || graph->getNumNodes() == 0) {
        cout << "Grafo inválido ou vazio." << endl;
        return nullptr;
    }

    Graph* complementGraph = new Graph(graph->isDirected(), graph->isWeightedEdges(), graph->isWeightedNodes());

    Node* currentNode = graph->getFirstNode();
    while (currentNode != nullptr) {
        complementGraph->createOrUpdateNode(currentNode->getId(), currentNode->getWeight());
        currentNode = currentNode->getNextNode();
    }

    Node* nodeU = graph->getFirstNode();
    while (nodeU != nullptr) {
        Node* nodeV = nodeU->getNextNode();
        while (nodeV != nullptr) {
            if (graph->findEdgeByNodes(nodeU, nodeV) == nullptr) {
                complementGraph->createEdge(
                    complementGraph->findNodeById(nodeU->getId()),
                    complementGraph->findNodeById(nodeV->getId()),
                    1);
                if (!graph->isDirected()) {
                    complementGraph->createEdge(
                        complementGraph->findNodeById(nodeV->getId()),
                        complementGraph->findNodeById(nodeU->getId()),
                        1);
                }
            }
            nodeV = nodeV->getNextNode();
        }
        nodeU = nodeU->getNextNode();
    }

    return complementGraph;
};

void printStronglyConnectedComponents(Graph* graph) {
    stack<Node*> Stack;
    unordered_map<Node*, bool> visited;

    Node* startNode = graph->getFirstNode();
    while (startNode != nullptr) {
        if (!visited[startNode]) {
            fillOrder(startNode, Stack, visited);
        }
        startNode = startNode->getNextNode();
    }

    Graph* transposedGraph = getTranspose(graph);

    visited.clear();

    while (!Stack.empty()) {
        Node* node = Stack.top();
        Stack.pop();

        if (!visited[node]) {
            vector<Node*> component;
            DFSUtil(node, visited, component);

            cout << "Strongly Connected Component: ";
            for (Node* n : component) {
                cout << n->getId() << " ";
            }
            cout << endl;
        }
    }

    delete transposedGraph;
}

void fillOrder(Node* node, stack<Node*>& Stack, unordered_map<Node*, bool>& visited) {
    visited[node] = true;

    for (Edge* edge : node->getEdges()) {
        Node* adjacent = edge->getTail();
        if (!visited[adjacent]) {
            fillOrder(adjacent, Stack, visited);
        }
    }

    Stack.push(node);
}

void DFSUtil(Node* node, unordered_map<Node*, bool>& visited, vector<Node*>& component) {
    visited[node] = true;
    component.push_back(node);

    for (Edge* edge : node->getEdges()) {
        Node* adjacent = edge->getTail();
        if (!visited[adjacent]) {
            DFSUtil(adjacent, visited, component);
        }
    }
}

Graph* getTranspose(Graph* graph) {
    Graph* transposedGraph = new Graph(graph->isDirected(), graph->isWeightedEdges(), graph->isWeightedNodes());

    Node* node = graph->getFirstNode();
    while (node != nullptr) {
        transposedGraph->createOrUpdateNode(node->getId(), node->getWeight());
        node = node->getNextNode();
    }

    node = graph->getFirstNode();
    while (node != nullptr) {
        for (Edge* edge : node->getEdges()) {
            Node* head = edge->getHead();
            Node* tail = edge->getTail();
            transposedGraph->createEdge(transposedGraph->findNodeById(tail->getId()),
                                        transposedGraph->findNodeById(head->getId()),
                                        edge->getWeight());
        }
        node = node->getNextNode();
    }

    return transposedGraph;
}

void depthFirstSearch(Node* node, unordered_set<Node*>& visited) {
    if (visited.find(node) != visited.end()) {
        return;
    }

    visited.insert(node);

    for (Edge* edge = node->getFirstEdge(); edge != nullptr; edge = edge->getNextEdge()) {
        Node* neighbor = (node == edge->getHead()) ? edge->getTail() : edge->getHead();
        depthFirstSearch(neighbor, visited);
    }
}

vector<Node*> getDirectTransitiveClosureByNode(Node* node) {
    vector<Node*> directTransitiveClosure;
    unordered_set<Node*> visited;

    depthFirstSearch(node, visited);

    for (Node* visitedNode : visited) {
        directTransitiveClosure.push_back(visitedNode);
    }

    return directTransitiveClosure;
}

vector<Node*> getIndirectTransitiveClosureByNode(Graph* graph, Node* node) {
    vector<Node*> indirectTransitiveClosure;
    Node* firstNode = graph->getFirstNode();

    if (firstNode == nullptr) {
        return indirectTransitiveClosure;
    }

    for (Node* currentNode = firstNode; currentNode != nullptr; currentNode = currentNode->getNextNode()) {
        if (currentNode == node) {
            continue;
        }

        vector<Node*> directTransitiveClosure = getDirectTransitiveClosureByNode(currentNode);

        if (find(directTransitiveClosure.begin(), directTransitiveClosure.end(), node) != directTransitiveClosure.end()) {
            indirectTransitiveClosure.push_back(currentNode);
        }
    }

    return indirectTransitiveClosure;
}

Graph* createInducedSubgraph(Graph* graph, const vector<Node*>& selectedNodes) {
    Graph* subgraph = new Graph(graph->isDirected(), graph->isWeightedEdges(), graph->isWeightedNodes());

    for (Node* node : selectedNodes) {
        subgraph->createOrUpdateNode(node->getId(), node->getWeight());
    }

    for (Node* node : selectedNodes) {
        for (Edge* edge : node->getEdges()) {
            Node* head = edge->getHead();
            Node* tail = edge->getTail();

            if (find(selectedNodes.begin(), selectedNodes.end(), head) != selectedNodes.end() &&
                find(selectedNodes.begin(), selectedNodes.end(), tail) != selectedNodes.end()) {
                subgraph->createEdge(
                    subgraph->createOrUpdateNode(head->getId(), head->getWeight()),
                    subgraph->createOrUpdateNode(tail->getId(), tail->getWeight()),
                    edge->getWeight());
            }
        }
    }

    return subgraph;
}

GraphMetrics getGraphMetricsInGraph(Graph* graph) {
    int numNodes = graph->getNumNodes();

    if (graph == nullptr || numNodes == 0) {
        cerr << "Grafo inválido ou vazio." << endl;
        return {-1, -1, {}, {}};
    }

    vector<vector<int>> dist(numNodes + 1, vector<int>(numNodes + 1, numeric_limits<int>::max()));

    for (Edge* edge : graph->getEdges()) {
        if (edge == nullptr || edge->getHead() == nullptr || edge->getTail() == nullptr) {
            continue;
        }
        int u = edge->getHead()->getId();
        int v = edge->getTail()->getId();
        int weight = edge->getWeight();

        if (u < 0 || u > numNodes || v < 0 || v > numNodes) {
            continue;
        }

        dist[u][v] = weight;
        if (!graph->isDirected()) {
            dist[v][u] = weight;
        }
    }

    for (int i = 1; i <= numNodes; ++i) {
        dist[i][i] = 0;
    }

    for (int k = 1; k <= numNodes; ++k) {
        for (int i = 1; i <= numNodes; ++i) {
            for (int j = 1; j <= numNodes; ++j) {
                if (dist[i][k] != numeric_limits<int>::max() && dist[k][j] != numeric_limits<int>::max()) {
                    dist[i][j] = min(dist[i][j], dist[i][k] + dist[k][j]);
                }
            }
        }
    }

    int diameter = 0;
    int radius = numeric_limits<int>::max();
    vector<int> eccentricities(numNodes + 1, 0);

    for (int i = 1; i <= numNodes; ++i) {
        int maxDistance = 0;
        for (int j = 1; j <= numNodes; ++j) {
            if (dist[i][j] != numeric_limits<int>::max()) {
                maxDistance = max(maxDistance, dist[i][j]);
            }
        }
        eccentricities[i] = maxDistance;
        if (maxDistance < radius) {
            radius = maxDistance;
        }
        if (maxDistance > diameter) {
            diameter = maxDistance;
        }
    }

    vector<int> center;
    vector<int> periphery;

    for (int i = 1; i <= numNodes; ++i) {
        if (eccentricities[i] == radius) {
            center.push_back(i);
        }
        if (eccentricities[i] == diameter) {
            periphery.push_back(i);
        }
    }

    return {diameter, radius, center, periphery};
}

int find(int parent[], int i) {
    if (parent[i] == i)
        return i;
    return parent[i] = find(parent, parent[i]);
}

void unionSets(int parent[], int rank[], int x, int y) {
    int rootX = find(parent, x);
    int rootY = find(parent, y);

    if (rootX != rootY) {
        if (rank[rootX] > rank[rootY]) {
            parent[rootY] = rootX;
        } else if (rank[rootX] < rank[rootY]) {
            parent[rootX] = rootY;
        } else {
            parent[rootY] = rootX;
            rank[rootX]++;
        }
    }
}

Graph* getMinimumSpanningTreeByKruskal(Graph* originalGraph, vector<Node*>& nodeVector) {
    Graph* mstGraph = new Graph(originalGraph->isDirected(), originalGraph->isWeightedEdges(), originalGraph->isWeightedNodes());

    map<int, int> nodeIndex;
    int index = 0;

    for (Node* node : nodeVector) {
        nodeIndex[node->getId()] = index++;
        mstGraph->createOrUpdateNode(node->getId(), node->getWeight());
    }

    vector<Edge*> edgeList;

    for (Edge* edge : originalGraph->getEdges()) {
        if (nodeIndex.find(edge->getHead()->getId()) != nodeIndex.end() &&
            nodeIndex.find(edge->getTail()->getId()) != nodeIndex.end()) {
            edgeList.push_back(edge);
        }
    }

    sort(edgeList.begin(), edgeList.end(), [](Edge* e1, Edge* e2) {
        return e1->getWeight() < e2->getWeight();
    });

    vector<int> parent(nodeIndex.size());
    vector<int> rank(nodeIndex.size(), 0);

    for (int i = 0; i < nodeIndex.size(); i++) {
        parent[i] = i;
    }

    for (Edge* edge : edgeList) {
        int u = nodeIndex[edge->getHead()->getId()];
        int v = nodeIndex[edge->getTail()->getId()];

        if (find(parent.data(), u) != find(parent.data(), v)) {
            mstGraph->createEdge(edge->getHead(), edge->getTail(), edge->getWeight());
            unionSets(parent.data(), rank.data(), u, v);
        }
    }

    return mstGraph;
}

struct CompareEdge {
    bool operator()(Edge* e1, Edge* e2) {
        return e1->getWeight() > e2->getWeight();
    }
};

Graph* getMinimumSpanningTreeByPrim(Graph* originalGraph, vector<Node*>& nodeVector) {
    Graph* mstGraph = new Graph(originalGraph->isDirected(), originalGraph->isWeightedEdges(), originalGraph->isWeightedNodes());

    if (nodeVector.empty()) return mstGraph;

    map<int, Node*> nodeMap;
    for (Node* node : nodeVector) {
        nodeMap[node->getId()] = node;
        mstGraph->createOrUpdateNode(node->getId(), node->getWeight());
    }

    priority_queue<Edge*, vector<Edge*>, CompareEdge> edgeQueue;

    set<int> visited;

    Node* startNode = nodeVector[0];
    visited.insert(startNode->getId());

    for (Edge* edge : startNode->getEdges()) {
        if (nodeMap.find(edge->getTail()->getId()) != nodeMap.end()) {
            edgeQueue.push(edge);
        }
    }

    while (!edgeQueue.empty()) {
        Edge* minEdge = edgeQueue.top();
        edgeQueue.pop();

        Node* u = minEdge->getHead();
        Node* v = minEdge->getTail();

        if (visited.find(v->getId()) != visited.end()) {
            continue;
        }

        mstGraph->createEdge(u, v, minEdge->getWeight());

        visited.insert(v->getId());

        for (Edge* edge : v->getEdges()) {
            if (visited.find(edge->getTail()->getId()) == visited.end() &&
                nodeMap.find(edge->getTail()->getId()) != nodeMap.end()) {
                edgeQueue.push(edge);
            }
        }
    }

    return mstGraph;
}

void dfsHelper(Node* node, std::unordered_set<int>& visited, std::vector<Edge*>& treeEdges, std::vector<Edge*>& backEdges) {
    visited.insert(node->getId());

    for (Edge* edge : node->getEdges()) {
        Node* adjacent = edge->getTail();

        if (visited.find(adjacent->getId()) == visited.end()) {
            treeEdges.push_back(edge);
            dfsHelper(adjacent, visited, treeEdges, backEdges);
        } else {
            backEdges.push_back(edge);
        }
    }
}

void printDepthFirstSearchTree(Graph* graph, Node* startNode) {
    if (!startNode) {
        std::cout << "Node not found." << std::endl;
        return;
    }

    std::unordered_set<int> visited;
    std::vector<Edge*> treeEdges;
    std::vector<Edge*> backEdges;

    dfsHelper(startNode, visited, treeEdges, backEdges);

    std::cout << "DFS Tree Edges:" << std::endl;
    for (Edge* edge : treeEdges) {
        std::cout << edge->getHead()->getId() << " -> " << edge->getTail()->getId() << std::endl;
    }

    std::cout << "Back Edges:" << std::endl;
    for (Edge* edge : backEdges) {
        std::cout << edge->getHead()->getId() << " -> " << edge->getTail()->getId() << std::endl;
    }
}
