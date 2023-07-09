// mkdir __tests__;
// g++ -std=c++11 ./src/tests/validators.spec.cpp -o ./__tests__/crud.exe ; ./__tests__/crud.exe

#include "../helpers/Validators/validators.cpp"

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

string name = "Validators ";

void testCheckGraphIsKRegularByK() {
    it("Test checkGraphIsKRegularByK");

    Graph* graph = new Graph(false, true, true);

    Node* node1 = graph->createOrUpdateNode(1, 10);
    Node* node2 = graph->createOrUpdateNode(2, 20);
    Node* node3 = graph->createOrUpdateNode(3, 30);
    Node* node4 = graph->createOrUpdateNode(4, 40);
    Node* node5 = graph->createOrUpdateNode(5, 50);

    graph->createOrUpdateEdge(node1, node2, 10);
    graph->createOrUpdateEdge(node2, node3, 20);
    graph->createOrUpdateEdge(node3, node4, 30);
    graph->createOrUpdateEdge(node4, node5, 40);
    graph->createOrUpdateEdge(node5, node1, 50);

    expect(checkGraphIsKRegularByK(graph, 2), true, "Should be 2-regular");
    expect(checkGraphIsKRegularByK(graph, 3), false, "Should not be 3-regular");
    expect(checkGraphIsKRegularByK(graph, 4), false, "Should not be 4-regular");
}

void testCheckGraphIsTrivial() {
    it("Test checkGraphIsTrivial");

    Graph* graph = new Graph(false, true, true);

    Node* node1 = graph->createOrUpdateNode(1, 10);

    expect(checkGraphIsTrivial(graph), true, "Should be trivial");

    Node* node2 = graph->createOrUpdateNode(2, 20);
    Node* node3 = graph->createOrUpdateNode(3, 30);
    graph->createOrUpdateEdge(node1, node2, 10);
    graph->createOrUpdateEdge(node2, node3, 20);

    expect(checkGraphIsTrivial(graph), false, "Should not be trivial");
}

void testCheckGraphIsNull() {
    it("Test checkGraphIsNull");

    Graph* graph = new Graph(false, true, true);

    expect(checkGraphIsNull(graph), true, "Should be null");

    Node* node1 = graph->createOrUpdateNode(1, 10);
    Node* node2 = graph->createOrUpdateNode(2, 20);
    graph->createOrUpdateEdge(node1, node2, 10);

    expect(checkGraphIsNull(graph), false, "Should not be null");
}

void testCheckGraphIsComplete() {
    it("Test checkGraphIsComplete");

    Graph* graph = new Graph(false, true, true);

    Node* node1 = graph->createOrUpdateNode(1, 10);
    Node* node2 = graph->createOrUpdateNode(2, 20);
    Node* node3 = graph->createOrUpdateNode(3, 30);

    expect(checkGraphIsComplete(graph), false, "Should not be complete");

    graph->createOrUpdateEdge(node1, node2, 10);
    graph->createOrUpdateEdge(node1, node3, 20);
    graph->createOrUpdateEdge(node2, node3, 30);

    expect(checkGraphIsComplete(graph), true, "Should be complete");
}

void testCheckGraphIsBipartite() {
    it("Test checkGraphIsBipartite");

    Graph* graph = new Graph(false, true, true);

    Node* node1 = graph->createOrUpdateNode(1, 10);
    Node* node2 = graph->createOrUpdateNode(2, 20);
    Node* node3 = graph->createOrUpdateNode(3, 30);
    Node* node4 = graph->createOrUpdateNode(4, 40);

    graph->createOrUpdateEdge(node1, node3, 10);
    graph->createOrUpdateEdge(node2, node4, 20);

    expect(checkGraphIsBipartite(graph), true, "Should be bipartite");

    graph->createOrUpdateEdge(node1, node2, 30);
    graph->createOrUpdateEdge(node1, node4, 30);

    expect(checkGraphIsBipartite(graph), false, "Should not be bipartite");
}

void testEurelianUndirectedGraph() {
    it("Test eurelian undirected graph");

    Graph* graph = new Graph(false, false, false);

    Node* node1 = graph->createOrUpdateNode(1, 10);
    Node* node2 = graph->createOrUpdateNode(2, 20);
    Node* node3 = graph->createOrUpdateNode(3, 30);
    Edge* edge1 = graph->createOrUpdateEdge(node1, node2, 10);
    graph->createOrUpdateEdge(node2, node3, 20);
    graph->createOrUpdateEdge(node3, node1, 30);

    expect(checkGraphIsEulerian(graph), true, "Should be eurelian");

    graph->deleteEdge(edge1);

    expect(checkGraphIsEulerian(graph), false, "Should not be eurelian");
}

void testEurelianDirectedGraph() {
    it("Test eurelian directed graph");

    Graph* graph = new Graph(false, false, false);

    Node* node1 = graph->createOrUpdateNode(1, 10);
    Node* node2 = graph->createOrUpdateNode(2, 20);
    Node* node3 = graph->createOrUpdateNode(3, 30);
    Edge* edge1 = graph->createOrUpdateEdge(node1, node2, 10);
    graph->createOrUpdateEdge(node2, node3, 20);
    graph->createOrUpdateEdge(node3, node1, 30);

    expect(checkGraphIsEulerian(graph), true, "Should be eurelian");

    graph->deleteEdge(edge1);

    expect(checkGraphIsEulerian(graph), false, "Should not be eurelian");
}

int main() {
    describe(name);

    testCheckGraphIsKRegularByK();
    testCheckGraphIsTrivial();
    testCheckGraphIsNull();
    testCheckGraphIsComplete();
    testCheckGraphIsBipartite();
    testEurelianDirectedGraph();
    testEurelianUndirectedGraph();

    return 0;
}
