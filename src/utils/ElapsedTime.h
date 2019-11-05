#pragma once

#include <chrono>
#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>

#if 0 //old removal
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
#endif


class ElapsedTime
{
public:
    ElapsedTime() {
        // start timer
        this->begin = std::chrono::steady_clock::now();
    }

    ElapsedTime( const std::string msg ) {
        // start timer
        this->begin = std::chrono::steady_clock::now();
        this->msg_string = msg;
    }


    void tic() {
        // start timer
        this->begin = std::chrono::steady_clock::now();
    }

    void tic( const std::string msg ) {
        // start timer
        this->begin = std::chrono::steady_clock::now();
        this->msg_string = msg;
    }



    int toc_milli() {
        std::chrono::steady_clock::time_point end= std::chrono::steady_clock::now();
        return (int) std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
    }

    int toc_micro() {
        std::chrono::steady_clock::time_point end= std::chrono::steady_clock::now();
        return (int) std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
    }

    int toc_sec( ) {
        std::chrono::steady_clock::time_point end= std::chrono::steady_clock::now();
        return (int) std::chrono::duration_cast<std::chrono::seconds>(end - begin).count();
    }

    std::string toc()
    {
        auto ms = toc_milli();
        return ":" + msg_string + ": ms=" + std::to_string(ms) + " ";
    }


private:
    std::chrono::steady_clock::time_point begin;
    std::string msg_string = "";

};

class DateAndTime
{
public:
    static std::string current_date_and_time()
    {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);

        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X");
        return ss.str();
    }

    // TODO Can make more functions later to get hr, min, date etc. Not priority.

};
