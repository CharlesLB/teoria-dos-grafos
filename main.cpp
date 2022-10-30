#include <fstream>
#include <iostream>

#include "./lib/reader.cpp"
#include "./lib/writer.cpp"

using namespace std;

int main(int argc, char* argv[]) {
    string path;

    if (argc != 2) {
        cout << "Especifique o caminho da entrada com: " << argv[0] << " <input_folder>" << endl;
        return 1;
    }

    path = argv[1];

    return 0;
}
