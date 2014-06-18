#include <iostream>

#include <boost/thread.hpp>

void print_world() {
    std::cout << "world!" << std::endl;
}

void world() {
    boost::thread t(print_world);
    t.join();
}
