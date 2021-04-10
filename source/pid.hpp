#pragma once
#include <atomic>
#include <cassert>
#include <cstdint>
#include <mutex>

/* PID控制算法 */
class PID {
  private:
    double m_p, m_i, m_d;
    double m_sum;
    double lastError;
    int64_t lastTimeStamp;
    std::atomic_bool isFirst;
    std::mutex m_mutex;

    double limit(double in, double low, double up) { return in > up ? up : (in < low ? low : in); }

  public:
    explicit PID() : m_p(0), m_i(0), m_d(0), m_sum(0), lastError(0), lastTimeStamp(0), isFirst(true) {}

    /**
     *
     * @param p 比例
     * @param i 积分
     * @param d 微分
     */
    void init(double p, double i, double d) {
        m_p = (p);
        m_i = (i);
        m_d = (d);
    }

    /**
     * @param error 
     * @param timestamp 时间戳
     * func 对ryaw进行pid控制修正
     * @return out
     */
    double calc(double error, int64_t timeStamp) {
        std::lock_guard<std::mutex> lockGuard(m_mutex);  //互斥锁
        double out = error;
        if (!isFirst) {
            if (lastError * error < 0)
                m_sum = 0;
            m_sum += error;
            m_sum = limit(m_sum, -500, 500);                      //msum>=-500
            double dt = (timeStamp - lastTimeStamp) / 1000000.0;  //s
            dt = limit(dt, 0, 0.010);                             //保证ts>=0
            assert(dt > 0);
            out = error * m_p + m_sum * m_i * dt + (error - lastError) * m_d / dt;
        } else {
            isFirst = false;
            m_sum = 0;
        }
        lastError = error;
        lastTimeStamp = timeStamp;
        return limit(out, -5, 5);  //out>=-5
    }

    bool clear() {
        return isFirst.exchange(true);
    }
};
