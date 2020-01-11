#pragma once

#include <iostream>
#include <thread>

class Thread
{
public:
    bool isRunning = true;

    // Runs the thread
    virtual void start();
    virtual void stop();

protected:
    std::thread thread;

    // function should exit if stopFlag is true
    bool stopFlag = false;

    ~Thread();

    // Defines the code for the thread to execute
    virtual void run() = 0;
};
