#ifndef MAIN_H
#define MAIN_H

#include <string>
#include <vector>

using namespace std;

class Main {
   public:
    static int main(int argc, char* argv[]);

   private:
    static int DATProcess(int argc, char* argv[]);
    static int AMPLProcess(int argc, char* argv[]);
};

#endif