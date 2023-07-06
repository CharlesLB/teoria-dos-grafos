#include <iostream>

template <typename T>
void expect(const T& received, const T& expected, const std::string& testName) {
    if (received == expected) {
        std::cout << testName << ": \x1B[32mPASSED\033[0m\t\t" << std::endl;
        return;
    }

    std::cout << testName << ": "
              << "\033[1;31mFAILED | Expect <" << expected << "> received <" << received << ">\033[0m"
              << std::endl;
}

void describe(const std::string& description) {
    std::cout << std::endl;
    std::cout << "Running: " << description << " Test" << std::endl;
}

void it(const std::string& description) {
    std::cout << std::endl
              << "// " << description << std::endl
              << std::endl;
}