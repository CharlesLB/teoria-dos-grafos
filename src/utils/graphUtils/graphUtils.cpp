#include "./graphUtils.hpp"

#include <sys/stat.h>
#include <sys/types.h>

#include <string>
#include <unordered_map>

#include "../../lib/Edge/edge.hpp"
#include "../../lib/Graph/graph.hpp"
#include "../../lib/Node/node.hpp"

using namespace std;

Graph* createGraphCopy(Graph* graph) {
    Graph* copiedGraph = new Graph(graph->isDirected(), graph->isWeightedEdges(), graph->isWeightedNodes());

    unordered_map<Node*, Node*> nodeMap;

    Node* originalNode = graph->getFirstNode();

    while (originalNode != nullptr) {
        int id = originalNode->getId();
        int weight = originalNode->getWeight();

        Node* copiedNode = copiedGraph->createOrUpdateNode(id, weight);

        nodeMap[originalNode] = copiedNode;

        originalNode = originalNode->getNextNode();
    }

    for (Edge* originalEdge : graph->getEdges()) {
        Node* originalHead = originalEdge->getHead();
        Node* originalTail = originalEdge->getTail();

        Node* copiedHead = nodeMap[originalHead];
        Node* copiedTail = nodeMap[originalTail];

        int weight = originalEdge->getWeight();

        copiedGraph->createEdge(copiedHead, copiedTail, weight);
    }

    return copiedGraph;
}
