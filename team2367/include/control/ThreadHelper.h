#pragma once
#include <thread>

class ThreadHelper
{
public:
    ThreadHelper() : m_thread_handle(){}; // NOLINT
    ~ThreadHelper()
    {
        m_stop_thread = true;
        if (m_thread_handle.joinable())
        {
            m_thread_handle.join();
        }
    }

    void start()
    {
        m_stop_thread = false;
        m_thread_handle = std::thread(&ThreadHelper::runHelper, this);
    }

    void stop()
    {
        m_stop_thread = true;
        join();
    }

    void join()
    {
        if (m_thread_handle.joinable())
        {
            m_thread_handle.join();
        }
    }

    [[nodiscard]] bool isStopped() const
    {
        return m_stop_thread;
    }

private:
    bool m_stop_thread{false};
    std::thread m_thread_handle;
    virtual void run() = 0;
    virtual void init() = 0;

    void runHelper()
    {
        init();
        while (!m_stop_thread)
        {
            run();
        }
    }
};