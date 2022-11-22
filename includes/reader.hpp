#ifndef READER_H
#define READER_H

#include <string>
#include <vector>

using namespace std;

class Reader {
   public:
    static int integer();

    static Graph graph(string filename);

   private:
};

#endif