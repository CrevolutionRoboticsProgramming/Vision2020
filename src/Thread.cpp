#include "Thread.hpp"

Thread::~Thread()
{
    stop();
}

void Thread::start()
{
    isRunning = true;
    stopFlag = false;
    thread = std::thread{&Thread::run, this};
}

void Thread::stop()
{
    if (isRunning)
    {
        stopFlag = true;

        while (!thread.joinable())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds{250});
        }

        thread.join();

        isRunning = false;
    }
}
