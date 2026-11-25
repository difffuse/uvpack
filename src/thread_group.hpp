#pragma once
#include <exception>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

namespace uvpack {
// Own worker lifetimes even if creating a later thread throws. Worker failures
// are rethrown on the caller after joining; they never escape a thread entry.
class ThreadGroup {
public:
    ThreadGroup()=default;
    ThreadGroup(const ThreadGroup&)=delete;
    ThreadGroup& operator=(const ThreadGroup&)=delete;
    ~ThreadGroup(){join_all();}
    template<class Function,class... Args>
    void emplace_back(Function&& function,Args&&... args) {
        auto work=std::bind(std::forward<Function>(function),std::forward<Args>(args)...);
        threads_.emplace_back([this,work=std::move(work)]() mutable {
            try {work();}
            catch(...) {
                std::lock_guard<std::mutex> lock(mutex_);
                if(!failure_)failure_=std::current_exception();
            }
        });
    }
    void join() {
        join_all();
        if(failure_)std::rethrow_exception(failure_);
    }
private:
    void join_all(){for(auto& thread:threads_)if(thread.joinable())thread.join();}
    std::vector<std::thread> threads_;
    std::mutex mutex_;
    std::exception_ptr failure_;
};
}
