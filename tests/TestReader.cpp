#include <dirent.h>
#include <stdlib.h>
#include <string.h>

#include <iostream>
#include <sstream>

#include "../lib/edge.cpp"
#include "../lib/graph.cpp"
#include "../lib/node.cpp"
#include "../lib/reader.cpp"
#include "../lib/writer.cpp"

using namespace std;

void testInteger() {
    cout << "Integer Test" << endl;
    cout << "Type a integer number: ";
    int i = Reader::integer();
    cout << "You typed: " << i << endl;
}

int main(int argc, char* argv[]) {
    testInteger();
}
