#ifndef TESTS_H
#define TESTS_H

#include <iostream>
#include <string>

using namespace std;

bool existsFolder(string folderPath);

bool existsFile(string filePath);

void mkdirSync(string folderPath);

string getRawFileName(string fileName);

#endif