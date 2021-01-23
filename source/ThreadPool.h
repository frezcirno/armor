#ifndef THREAD_POOL_H
#define THREAD_POOL_H

#include <vector>
#include <queue>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <future>
#include <functional>
#include <stdexcept>

/*作为封装库直接使用*/

class ThreadPool {
public:
    ThreadPool(size_t);
    template<class F, class... Args>
    auto enqueue(F&& f, Args&&... args) 
        -> std::future<typename std::result_of<F(Args...)>::type>;
    ~ThreadPool();
private:
    std::vector< std::thread > workers;/* 工作线程 */
    std::queue< std::function<void()> > tasks;/* 任务队列*/
    
    
    std::mutex queue_mutex;/* 保持队列同步的锁 */
    std::condition_variable condition;/* 结束线程条件 */
    bool stop;/* 线程池是否结束运行 */
};
 
/**
     * 构造函数
     * 在线程池内创建threads个线程
     * @param threads 线程数量
     */
inline ThreadPool::ThreadPool(size_t threads)
    :   stop(false)
{
    /* 创建threads个工作线程 */
    /* 所有工作线程都应被阻塞 */
    for(size_t i = 0;i<threads;++i)
        workers.emplace_back(
            [this]
            {
                for(;;)
                {
                    std::function<void()> task;

                    {
                        std::unique_lock<std::mutex> lock(this->queue_mutex);
                        /* 谓语为假时阻塞线程 */
                        this->condition.wait(lock,
                            [this]{ return this->stop || !this->tasks.empty(); });/* 谓语 */
                        /* 防止出错，初始时stop=false，tasks为空 */
                        if(this->stop && this->tasks.empty())
                            return;
                        
                        task = std::move(this->tasks.front());
                        this->tasks.pop();
                    }

                    task();
                }
            }
        );
}

/**
     * 任务入队
     */
template<class F, class... Args>
auto ThreadPool::enqueue(F&& f, Args&&... args) 
    -> std::future<typename std::result_of<F(Args...)>::type>
{
    using return_type = typename std::result_of<F(Args...)>::type;
    /* 接受任务 */
    auto task = std::make_shared< std::packaged_task<return_type()> >(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...)
        );
        
    std::future<return_type> res = task->get_future();
    {
        std::unique_lock<std::mutex> lock(queue_mutex);
        /* 防止线程池结束工作时仍在入队 */
        if(stop)
            throw std::runtime_error("enqueue on stopped ThreadPool");
        /* 任务入队 */
        tasks.emplace([task](){ (*task)(); });
    }
    /* 随机唤醒一个等待的工作线程，开始工作 */
    condition.notify_one();

    return res;
}

/**
     * 析构函数
     * join所有线程
     */
inline ThreadPool::~ThreadPool()
{
    {
        std::unique_lock<std::mutex> lock(queue_mutex);
        stop = true;
    }
    condition.notify_all();
    for(std::thread &worker: workers)
        worker.join();
}

#endif
