#include <iostream>
#include <limits>
#include <map>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "../../lib/Edge/edge.hpp"
#include "../../lib/Graph/graph.hpp"
#include "../../lib/Node/node.hpp"
#include "../../utils/graph/graph.hpp"

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
    return articulationNodes;
}

vector<Edge*> getBridgeEdgesInGraph(Graph* graph) {
    vector<Edge*> bridgeEdges;
    return bridgeEdges;
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

    std::unordered_map<Node*, int> distances;
    for (Node* node = graph->getFirstNode(); node != nullptr; node = node->getNextNode()) {
        distances[node] = (node == sourceNode) ? 0 : std::numeric_limits<int>::max();
    }

    std::priority_queue<NodeDistance, std::vector<NodeDistance>, std::greater<NodeDistance>> pq;
    pq.push(NodeDistance(sourceNode, 0));

    while (!pq.empty()) {
        Node* currentNode = pq.top().node;
        int currentDistance = pq.top().distance;
        pq.pop();

        if (currentNode == targetNode) {
            break;
        }

        for (Edge* edge = currentNode->getFirstEdge(); edge != nullptr; edge = edge->getNextEdge()) {
            Node* neighbor = (currentNode == edge->getHead()) ? edge->getTail() : edge->getHead();
            int weight = edge->getWeight();

            if (distances[currentNode] + weight < distances[neighbor]) {
                distances[neighbor] = distances[currentNode] + weight;
                pq.push(NodeDistance(neighbor, distances[neighbor]));
            }
        }
    }

    Graph* shortestPathGraph = new Graph(graph->isDirected(), graph->isWeightedEdges(), graph->isWeightedNodes());

    Node* currentNode = targetNode;
    while (currentNode != sourceNode) {
        int shortestDistance = distances[currentNode];

        Edge* shortestEdge = nullptr;
        for (Edge* edge = currentNode->getFirstEdge(); edge != nullptr; edge = edge->getNextEdge()) {
            Node* neighbor = (currentNode == edge->getHead()) ? edge->getTail() : edge->getHead();
            int weight = edge->getWeight();

            if (distances[neighbor] + weight == shortestDistance) {
                shortestEdge = edge;
                break;
            }
        }

        if (shortestEdge) {
            Node* neighborNode = (currentNode == shortestEdge->getHead()) ? shortestEdge->getTail() : shortestEdge->getHead();
            shortestPathGraph->createOrUpdateNode(currentNode->getId(), currentNode->getWeight());
            shortestPathGraph->createOrUpdateNode(neighborNode->getId(), neighborNode->getWeight());
            shortestPathGraph->createEdge(currentNode, neighborNode, shortestEdge->getWeight());
            currentNode = neighborNode;
            continue;
        }

        break;
    }

    return shortestPathGraph;
}

Graph* getMinimumPathAndCostByBellmanFord(Graph* graph, Node* sourceNode, Node* targetNode) {
    if (graph == nullptr || sourceNode == nullptr || targetNode == nullptr) {
        return nullptr;
    }

    std::vector<Node*> nodes;
    for (Node* node = graph->getFirstNode(); node != nullptr; node = node->getNextNode()) {
        nodes.push_back(node);
    }

    // Step 1: Initialize distances to all nodes as infinity except the source node, which is 0.
    std::unordered_map<Node*, int> distances;
    for (Node* node : nodes) {
        distances[node] = (node == sourceNode) ? 0 : std::numeric_limits<int>::max();
    }

    // Step 2: Relax edges iteratively V-1 times (where V is the number of nodes)
    for (size_t i = 1; i < nodes.size(); ++i) {
        for (Edge* edge : graph->getEdges()) {
            Node* u = edge->getHead();
            Node* v = edge->getTail();
            int weight = edge->getWeight();

            if (distances[u] != std::numeric_limits<int>::max() && distances[u] + weight < distances[v]) {
                distances[v] = distances[u] + weight;
            }
        }
    }

    // Step 3: Check for negative weight cycles
    for (Edge* edge : graph->getEdges()) {
        Node* u = edge->getHead();
        Node* v = edge->getTail();
        int weight = edge->getWeight();

        if (distances[u] != std::numeric_limits<int>::max() && distances[u] + weight < distances[v]) {
            return nullptr;
        }
    }

    // Step 4: Create a new Graph to store the shortest path from sourceNode to targetNode.
    Graph* shortestPathGraph = new Graph(graph->isDirected(), graph->isWeightedEdges(), graph->isWeightedNodes());

    Node* currentNode = targetNode;
    while (currentNode != sourceNode) {
        int shortestDistance = distances[currentNode];

        Edge* shortestEdge = nullptr;
        for (Edge* edge = currentNode->getFirstEdge(); edge != nullptr; edge = edge->getNextEdge()) {
            Node* neighbor = (currentNode == edge->getHead()) ? edge->getTail() : edge->getHead();
            int weight = edge->getWeight();

            if (distances[neighbor] + weight == shortestDistance) {
                shortestEdge = edge;
                break;
            }
        }

        if (shortestEdge) {
            Node* neighborNode = (currentNode == shortestEdge->getHead()) ? shortestEdge->getTail() : shortestEdge->getHead();
            shortestPathGraph->createOrUpdateNode(currentNode->getId(), currentNode->getWeight());
            shortestPathGraph->createOrUpdateNode(neighborNode->getId(), neighborNode->getWeight());
            shortestPathGraph->createEdge(currentNode, neighborNode, shortestEdge->getWeight());
            currentNode = neighborNode;
            continue;
        }

        break;
    }

    return shortestPathGraph;
}

Graph* getMinimumPathAndCostByFloyd(Graph* graph, Node* sourceNode, Node* targetNode) {
    if (graph == nullptr || sourceNode == nullptr || targetNode == nullptr) {
        return nullptr;
    }

    // Step 1: Initialize the distance matrix.
    int numNodes = graph->getNumNodes();
    std::vector<std::vector<int>> distance(numNodes, std::vector<int>(numNodes, std::numeric_limits<int>::max()));

    for (Node* node = graph->getFirstNode(); node != nullptr; node = node->getNextNode()) {
        distance[node->getId()][node->getId()] = 0;
    }

    for (Edge* edge : graph->getEdges()) {
        Node* head = edge->getHead();
        Node* tail = edge->getTail();
        int weight = edge->getWeight();
        distance[head->getId()][tail->getId()] = weight;
    }

    // Step 2: Calculate the shortest paths between all pairs of nodes.
    for (int k = 0; k < numNodes; ++k) {
        for (int i = 0; i < numNodes; ++i) {
            for (int j = 0; j < numNodes; ++j) {
                if (distance[i][k] != std::numeric_limits<int>::max() &&
                    distance[k][j] != std::numeric_limits<int>::max() &&
                    distance[i][k] + distance[k][j] < distance[i][j]) {
                    distance[i][j] = distance[i][k] + distance[k][j];
                }
            }
        }
    }

    // Step 3: Extract the shortest path from sourceNode to targetNode.
    int shortestDistance = distance[sourceNode->getId()][targetNode->getId()];
    if (shortestDistance == std::numeric_limits<int>::max()) {
        return nullptr;
    }

    Graph* shortestPathGraph = new Graph(graph->isDirected(), graph->isWeightedEdges(), graph->isWeightedNodes());

    Node* currentNode = targetNode;
    while (currentNode != sourceNode) {
        for (Node* node = graph->getFirstNode(); node != nullptr; node = node->getNextNode()) {
            int weight = distance[sourceNode->getId()][node->getId()] +
                         distance[node->getId()][targetNode->getId()];

            if (weight == shortestDistance) {
                shortestPathGraph->createOrUpdateNode(node->getId(), node->getWeight());
                shortestDistance -= distance[node->getId()][targetNode->getId()];
                currentNode = node;
                break;
            }
        }
    }

    return shortestPathGraph;
}