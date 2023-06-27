#pragma once

#include <ctime>
#include <iostream>
#include <string>
#include <cstdlib>
#include <chrono>
#include <fstream>

class TicToc
{
public:
    TicToc( bool _disp ) // display 
    {
        tic();
    }

    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    double toc()
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        double elapsed_ms = elapsed_seconds.count() * 1000;

        return elapsed_ms;    
    }

private:  
    std::chrono::time_point<std::chrono::system_clock> start, end;
};
