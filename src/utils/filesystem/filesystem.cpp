#include "./filesystem.hpp"

#include <sys/stat.h>
#include <sys/types.h>

#include <string>

struct stat info;
using namespace std;

bool existsFolder(string folderPath) {
    return (stat(folderPath.c_str(), &info) == 0 && info.st_mode & S_IFDIR);
}

bool existsFile(string filePath) {
    return (stat(filePath.c_str(), &info) == 0 && info.st_mode & S_IFREG);
}

void mkdirSync(string folderPath) {
    mkdir(folderPath.c_str(), 0777);
}
