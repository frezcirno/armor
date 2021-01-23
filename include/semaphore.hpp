//
// Created by 刘雍熙 on 2020-02-01.
// 信号量
//


#ifndef ATTACK2020_SEMAPHORE_HPP
#define ATTACK2020_SEMAPHORE_HPP

#include <thread>
#include <condition_variable>
#include <mutex>
#include <atomic>

#include <chrono>

using namespace std::chrono_literals;

namespace armor {
    void thread_sleep_ms(long ms) {
        std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    }

    void thread_sleep_s(long s) {
        std::this_thread::sleep_for(std::chrono::seconds(s));
    }

    void thread_sleep_us(long us) {
        std::this_thread::sleep_for(std::chrono::microseconds(us));
    }

    class Semaphore {
    private:
        std::condition_variable m_condVar;
        std::atomic_bool m_isWakeUp;
        std::atomic_bool m_isWillExit;
    public:
        std::mutex mutex;

        explicit Semaphore() : m_isWakeUp(false), m_isWillExit(false) {}
        /**
         * 在tiemout指定的时间内若被唤醒或将退出则执行func
         * @param timeout 等候时间
         * @param func 判别函数
         * @return true/false
         */
        template<typename _Rep, typename _Period>
        bool wait_for(const std::chrono::duration<_Rep, _Period> &timeout, const std::function<void()> &func) {
            std::unique_lock<std::mutex> lock(mutex);
            if (m_condVar.wait_for(lock, timeout,
                                   [&]() -> bool {
                                       return m_isWakeUp.load() || m_isWillExit.load();
                                   })) {
                func();
                lock.unlock();
                m_isWakeUp.exchange(false);
                return true;
            } else {
                std::cout << "[Semaphore] timeout" << std::endl;
                lock.unlock();
                m_isWakeUp.exchange(false);
                return false;
            }
        }

        /**
         * 尝试唤醒, 若wait尚在执行, 则不会执行任何操作, 也不会唤醒wait
         * @param func
         */
        void signal_try(const std::function<void()> &func) {
            std::unique_lock<std::mutex> lock(mutex, std::try_to_lock);
            if (lock.owns_lock()) {
                func();
                lock.unlock();
                m_isWakeUp.exchange(true);
                m_condVar.notify_one();
            }
        }

        /**
         * 同步唤醒, 阻塞到 wait 接收到本次唤醒
         * @param func
         */
        void signal_sync(const std::function<void()> &func) {
            std::unique_lock<std::mutex> lock(mutex);
            func();
            lock.unlock();
            m_isWakeUp.exchange(true);
            m_condVar.notify_one();
        };

        /**
         * 简单的唤醒 wait
         */
        void signal() {
            m_isWakeUp.exchange(true);
            m_condVar.notify_one();
        };
        /**
         * 退出
         */
        void quit() {
            m_isWillExit.exchange(true);
            m_condVar.notify_all();
        }
    };
}
#endif //ATTACK2020_SEMAPHORE_HPP
