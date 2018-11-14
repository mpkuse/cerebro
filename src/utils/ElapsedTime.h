#pragma once

#include <chrono>
#include <iostream>
#include <iomanip>

class ElapsedTime
{
public:
    ElapsedTime() {
        // start timer
        this->begin = std::chrono::steady_clock::now();
    }

    void tic() {
        // start timer
        this->begin = std::chrono::steady_clock::now();
    }


    int toc_milli() {
        std::chrono::steady_clock::time_point end= std::chrono::steady_clock::now();
        return (int) std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
    }

    int toc_micro() {
        std::chrono::steady_clock::time_point end= std::chrono::steady_clock::now();
        return (int) std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
    }

    int toc( ) {
        std::chrono::steady_clock::time_point end= std::chrono::steady_clock::now();
        return (int) std::chrono::duration_cast<std::chrono::seconds>(end - begin).count();
    }



private:
    std::chrono::steady_clock::time_point begin;

};
