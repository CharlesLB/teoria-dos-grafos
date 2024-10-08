#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <string>
#include <vector>

#include "../../lib/Graph/graph.hpp"

using namespace std;

class Graph;
class Controller {
   public:
    static void processOperation(char* argv[], Graph* graph);
    static void processOperationAMPL(char* argv[], Graph* graph, int numClusters);

   private:
    static void exitSystem();
    static void printGraphToFile(Graph* graph, string outputPath);
    static void nodeAndEdgeInsertionDeletion(Graph* graph);
    static void getNodeDegree(Graph* graph);
    static void checkGraphRegular(Graph* graph);
    static void getGraphOrder(Graph* graph);
    static void isGraphTrivial(Graph* graph);
    static void isGraphNull(Graph* graph);
    static void showOpenNeighborhood(Graph* graph);
    static void showClosedNeighborhood(Graph* graph);
    static void checkMultigraph(Graph* graph);
    static void checkCompleteGraph(Graph* graph);
    static void checkBipartiteGraph(Graph* graph);
    static void getMinimumPathAndCost(Graph* graph);
    static void getGraphDegree(Graph* graph);
    static void getDirectTransitiveClosure(Graph* graph);
    static void getIndirectTransitiveClosure(Graph* graph);
    static void getDegreeSequence(Graph* graph);
    static void getInducedSubgraph(Graph* graph);
    static void getGraphComplement(Graph* graph);
    static void getStronglyConnectedComponents(Graph* graph);
    static void checkEulerianGraph(Graph* graph);
    static void getArticulationNodes(Graph* graph);
    static void getBridgeEdges(Graph* graph);
    static void getGraphMetrics(Graph* graph);
    static void getMinimumSpanningTreeKruscal(Graph* graph);
    static void getMinimumSpanningTreePrim(Graph* graph);
    static void getDepthFirstSearchTree(Graph* graph);

    // MGGPP
    static void runGreedyAlgorithm(Graph* graph, int numClusters);
    static void runGRASPAlgorithm(Graph* graph, int numClusters);
    static void runReactiveGRASPAlgorithm(Graph* graph, int numClusters);
};

#endif