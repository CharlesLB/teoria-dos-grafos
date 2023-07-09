// mkdir __tests__;
// g++ -std=c++11 ./src/tests/crud.spec.cpp -o ./__tests__/crud.exe ; ./__tests__/crud.exe

#include <dirent.h>
#include <stdlib.h>
#include <string.h>

#include <iostream>
#include <sstream>

#include "../core/Writer/writer.cpp"
#include "../lib/Edge/edge.cpp"
#include "../lib/Graph/graph.cpp"
#include "../lib/Node/node.cpp"
#include "../utils/tests/tests.cpp"

string name = "CRUD";

int main() {
    describe(name);

    Graph* graph = new Graph(1, 1, 1);

    // Create nodes
    it("Create nodes");

    Node* node1 = graph->createOrUpdateNode(1, 10);
    Node* node2 = graph->createOrUpdateNode(2, 20);
    Node* node3 = graph->createOrUpdateNode(3, 30);

    expect(graph->getNumNodes(), 3, "Should have 3 nodes");
    expect(graph->getNumEdges(), 0, "Should have 0 edges");

    expect(graph->getFirstNode()->getId(), 3, "First node should be node 3");
    expect(graph->getFirstNode()->getNextNode()->getId(), 2, "Second node should be node 2");
    expect(graph->getFirstNode()->getNextNode()->getNextNode()->getId(), 1, "Third node should be node 1");

    // Create edges
    it("Create edges");

    Edge* edge1 = graph->createOrUpdateEdge(node1, node2, 10);
    Edge* edge2 = graph->createOrUpdateEdge(node1, node3, 10);
    Edge* edge3 = graph->createOrUpdateEdge(node2, node3, 20);
    Edge* edge4 = graph->createOrUpdateEdge(node3, node1, 30);

    expect(graph->getNumNodes(), 3, "Should have 3 nodes");
    expect(graph->getNumEdges(), 4, "Should have 4 edges");

    expect(graph->getEdges()[0]->getWeight(), 10, "First edge should have weight 10");
    expect(graph->getEdges()[1]->getWeight(), 10, "Second edge should have weight 10");
    expect(graph->getEdges()[2]->getWeight(), 20, "Third edge should have weight 20");
    expect(graph->getEdges()[3]->getWeight(), 30, "Fourth edge should have weight 30");

    expect(graph->getFirstNode()->getDegreeIn(), 2, "First node should have degree in 2");
    expect(graph->getFirstNode()->getDegreeOut(), 1, "First node should have degree out 1");
    expect(graph->getFirstNode()->getNextNode()->getDegreeIn(), 1, "Second node should have degree in 1");
    expect(graph->getFirstNode()->getNextNode()->getDegreeOut(), 1, "Second node should have degree out 1");
    expect(graph->getFirstNode()->getNextNode()->getNextNode()->getDegreeIn(), 1, "Third node should have degree in 1");
    expect(graph->getFirstNode()->getNextNode()->getNextNode()->getDegreeOut(), 2, "Third node should have degree out 2");

    // Update nodes
    it("Update nodes");

    Node* node4 = graph->createOrUpdateNode(1, 40);
    Node* node5 = graph->createOrUpdateNode(2, 50);
    Node* node6 = graph->createOrUpdateNode(3, 60);

    expect(graph->getNumNodes(), 3, "Should have 3 nodes");
    expect(graph->getNumEdges(), 4, "Should have 4 edges");

    // Update edges
    it("Update edges");

    Edge* edge5 = graph->createOrUpdateEdge(node1, node2, 40);
    Edge* edge6 = graph->createOrUpdateEdge(node1, node3, 40);
    Edge* edge7 = graph->createOrUpdateEdge(node2, node3, 50);
    Edge* edge8 = graph->createOrUpdateEdge(node3, node1, 30);

    expect(graph->getNumNodes(), 3, "Should have 3 nodes");
    expect(graph->getNumEdges(), 4, "Should have 4 edges");

    expect(graph->getEdges()[0]->getWeight(), 40, "First edge should have weight 40");
    expect(graph->getEdges()[1]->getWeight(), 40, "Second edge should have weight 40");
    expect(graph->getEdges()[2]->getWeight(), 50, "Third edge should have weight 50");
    expect(graph->getEdges()[3]->getWeight(), 30, "Fourth edge should have weight 30");
    expect(graph->getFirstNode()->getDegreeIn(), 2, "First node should have degree in 2");
    expect(graph->getFirstNode()->getDegreeOut(), 1, "First node should have degree out 1");
    expect(graph->getFirstNode()->getNextNode()->getDegreeIn(), 1, "Second node should have degree in 1");
    expect(graph->getFirstNode()->getNextNode()->getDegreeOut(), 1, "Second node should have degree out 1");
    expect(graph->getFirstNode()->getNextNode()->getNextNode()->getDegreeIn(), 1, "Third node should have degree in 1");
    expect(graph->getFirstNode()->getNextNode()->getNextNode()->getDegreeOut(), 2, "Third node should have degree out 2");

    // delete edges

    it("Delete edges");

    graph->deleteEdge(edge1);

    expect(graph->getNumNodes(), 3, "Should have 3 nodes");
    expect(graph->getNumEdges(), 3, "Should have 3 edges");

    expect(graph->getFirstNode()->getDegreeIn(), 2, "First node should have degree in 2");
    expect(graph->getFirstNode()->getDegreeOut(), 1, "First node should have degree out 1");
    expect(graph->getFirstNode()->getNextNode()->getDegreeIn(), 0, "Second node should have degree in 0");
    expect(graph->getFirstNode()->getNextNode()->getDegreeOut(), 1, "Second node should have degree out 1");
    expect(graph->getFirstNode()->getNextNode()->getNextNode()->getDegreeIn(), 1, "Third node should have degree in 1");
    expect(graph->getFirstNode()->getNextNode()->getNextNode()->getDegreeOut(), 1, "Third node should have degree out 1");

    // Writer::printGraph(graph);
    return 0;
}
