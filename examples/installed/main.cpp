#include <uvpack/version.hpp>
#include <iostream>
int installed_pack_check();
int main() {
    if (const int failure=installed_pack_check()) return failure;
    std::cout << uvpack::metadata::json << '\n';
}
