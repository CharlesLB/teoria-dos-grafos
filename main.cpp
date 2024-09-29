#include "./src/core/Main/main.cpp"

#include <dirent.h>
#include <stdlib.h>
#include <string.h>

#include <iostream>
#include <sstream>

#include "./src/core/Controller/controller.cpp"
#include "./src/core/Manager/manager.cpp"
#include "./src/core/Reader/reader.cpp"
#include "./src/core/Writer/writer.cpp"
#include "./src/helpers/Algorithms/algorithms.cpp"
#include "./src/helpers/MGGPPAlgorithms/MGGPPAlgorithms.cpp"
#include "./src/helpers/Validators/validators.cpp"
#include "./src/lib/Edge/edge.cpp"
#include "./src/lib/Graph/graph.cpp"
#include "./src/lib/Node/node.cpp"
#include "./src/utils/filesystem/filesystem.cpp"

using namespace std;

int main(int argc, char* argv[]) {
    Main::main(argc, argv);
    return 0;
}
