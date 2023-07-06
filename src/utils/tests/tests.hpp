#ifndef TESTS_H
#define TESTS_H

#include <iostream>
#include <string>

template <typename T>
void expect(const T& value1, const T& value2, const std::string& testName);

void describe(const std::string& description);

void it(const std::string& description);

#endif