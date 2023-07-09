#include "./controller.hpp"

#include <dirent.h>
#include <stdlib.h>
#include <string.h>

#include <iostream>
#include <sstream>

#include "../../helpers/Validators/validators.hpp"
#include "../../lib/Edge/edge.hpp"
#include "../../lib/Graph/graph.hpp"
#include "../../lib/Node/node.hpp"
#include "../Manager/manager.hpp"
#include "../Reader/reader.hpp"
#include "../Writer/writer.hpp"

using namespace std;

void Controller::processOperation(char* argv[], bool hasWeightedNode, bool hasWeightedEdge, bool isDirected, Graph* graph) {
    char option;

    string options[] = {
        "Impressão do grafo em um arquivo",
        "Inclusão e exclusão de nó e de aresta",
        "Retornar o grau de um dado nó informado pelo usuário (caso seja um dígrafo, informar grau de entrada e de saída)",
        "Verificar a k-regularidade do grafo (k informado pelo usuário)",
        "Informar a ordem do grafo",
        "Informar se o grafo é trivial",
        "Informar se o grafo é nulo",
        "Mostrar a vizinhança aberta de um dado nó informado pelo usuário",
        "Mostrar a vizinhança fechada de um dado nó informado pelo usuário",
        "Verificar se o grafo é um multigrafo",
        "Verificar se o grafo é completo",
        "Verificar se o grafo é bipartido",
        "Dados dois vértices informados pelo usuário, mostrar o caminho mínimo e seu custo entre esses vértices (utilizar o algoritmo de Dijkstra ou de Floyd, conforme o usuário solicite)",
        "Retornar o grau do grafo",
        "Retornar o fecho transitivo direto de um dado nó informado pelo usuário",
        "Retornar o fecho transitivo indireto de um dado nó informado pelo usuário",
        "Apresentar a sequência de graus",
        "Apresentar o subgrafo induzido por um dado conjunto de vértices informado pelo usuário",
        "Apresentar o complementar do grafo",
        "Para digrafos, apresentar as componentes fortemente conexas",
        "Verificar se o grafo é eulerianos",
        "Apresentar os nós de articulação",
        "Apresentar as arestas ponte",
        "Apresentar o raio, o diâmetro, o centro e a periferia do grafo",
        "Apresentar a AGM do grafo ou, para grafos desconexos, as florestas de custo mínimo",
        "Apresentar o caminho mínimo entre dois vértices usando o algoritmo de Dijkstra ou de Floyd (escolha do usuário)", "Exit", ""};

    while (true) {
        Writer::printMenu(options);
        option = Reader::readChar();
        bool invalidOption = false;

        switch (option) {
            case '0':
                exitSystem();
                break;
            case 'a':
                printGraphToFile(graph, argv[2]);
                break;

            case 'b':
                nodeAndEdgeInsertionDeletion(graph);
                break;

            case 'c':
                getNodeDegree(graph);
                break;

            case 'd':
                checkGraphRegular(graph);
                break;

            case 'e':
                getGraphOrder(graph);
                break;

            case 'f':
                isGraphTrivial(graph);
                break;

            case 'g':
                isGraphNull(graph);
                break;

            case 'h':
                showOpenNeighborhood(graph);
                break;

            case 'i':
                showClosedNeighborhood(graph);
                break;

            case 'j':
                checkMultigraph(graph);
                break;

            case 'k':
                checkCompleteGraph(graph);
                break;

            case 'l':
                checkBipartiteGraph(graph);
                break;

            case 'm':
                getMinimumPathAndCost(graph);
                break;

            case 'n':
                getGraphDegree(graph);
                break;

            case 'o':
                getDirectTransitiveClosure(graph);
                break;

            case 'p':
                getIndirectTransitiveClosure(graph);
                break;

            case 'q':
                getDegreeSequence(graph);
                break;

            case 'r':
                getInducedSubgraph(graph);
                break;

            case 's':
                getGraphComplement(graph);
                break;

            case 't':
                getStronglyConnectedComponents(graph);
                break;

            case 'u':
                checkEulerianGraph(graph);
                break;

            case 'v':
                getArticulationNodes(graph);
                break;

            case 'w':
                getBridgeEdges(graph);
                break;

            case 'x':
                getGraphMetrics(graph);
                break;

            case 'y':
                getMinimumSpanningTree(graph);
                break;

            case 'z':
                getMinimumPath(graph);
                break;

            default:
                invalidOption = true;
                cout << "Invalid option\n";
                break;
        }

        if (invalidOption) continue;
        Reader::continueConfirmation();
    }
}

void Controller::exitSystem() {
    cout << "Good Bye!\n";
    exit(0);
}

void Controller::printGraphToFile(Graph* graph, string outputPath) {
    Writer::printGraph(graph);
}

void Controller::nodeAndEdgeInsertionDeletion(Graph* graph) {
    Manager::processOperation(graph);
}

void Controller::getNodeDegree(Graph* graph) {
    Manager::getNodeDegree(graph);
}

void Controller::checkGraphRegular(Graph* graph) {
    // TODO
}

void Controller::getGraphOrder(Graph* graph) {
    // TODO
}

void Controller::isGraphTrivial(Graph* graph) {
    bool isTrivial = checkGraphIsTrivial(graph);

    if (isTrivial) {
        cout << "The graph is trivial\n";
        return;
    }

    cout << "The graph is not trivial\n";
}

void Controller::isGraphNull(Graph* graph) {
    bool isNull = checkGraphIsNull(graph);

    if (isNull) {
        cout << "The graph is null\n";
        return;
    }

    cout << "The graph is not null\n";
}

void Controller::showOpenNeighborhood(Graph* graph) {
    // TODO
}

void Controller::showClosedNeighborhood(Graph* graph) {
    // TODO
}

void Controller::checkMultigraph(Graph* graph) {
    bool isMultigraph = checkGraphIsMultigraph(graph);

    if (isMultigraph) {
        cout << "The graph is multigraph\n";
        return;
    }

    cout << "The graph is not multigraph\n";
}

void Controller::checkCompleteGraph(Graph* graph) {
    bool isComplete = checkGraphIsComplete(graph);

    if (isComplete) {
        cout << "The graph is complete\n";
        return;
    }

    cout << "The graph is not complete\n";
}

void Controller::checkBipartiteGraph(Graph* graph) {
    bool isBipartite = checkGraphIsBipartite(graph);

    if (isBipartite) {
        cout << "The graph is bipartite\n";
        return;
    }

    cout << "The graph is not bipartite\n";
}

void Controller::getMinimumPathAndCost(Graph* graph) {
    // TODO
}

void Controller::getGraphDegree(Graph* graph) {
    // TODO
}

void Controller::getDirectTransitiveClosure(Graph* graph) {
    // TODO
}

void Controller::getIndirectTransitiveClosure(Graph* graph) {
    // TODO
}

void Controller::getDegreeSequence(Graph* graph) {
    // TODO
}

void Controller::getInducedSubgraph(Graph* graph) {
    // TODO
}

void Controller::getGraphComplement(Graph* graph) {
    // TODO
}

void Controller::getStronglyConnectedComponents(Graph* graph) {
    // TODO
}

void Controller::checkEulerianGraph(Graph* graph) {
    bool isEulerian = checkGraphIsEulerian(graph);

    if (isEulerian) {
        cout << "The graph is eulerian\n";
        return;
    }

    cout << "The graph is not eulerian\n";
}

void Controller::getArticulationNodes(Graph* graph) {
    // TODO
}

void Controller::getBridgeEdges(Graph* graph) {
    // TODO
}

void Controller::getGraphMetrics(Graph* graph) {
    // TODO
}

void Controller::getMinimumSpanningTree(Graph* graph) {
    // TODO
}

void Controller::getMinimumPath(Graph* graph) {
    // TODO
}
