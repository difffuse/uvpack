#pragma once
#include "uvpack/cancellation.hpp"
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <mutex>
#include <optional>
#include <thread>

namespace uvpack::cli {
// The signal adapter belongs to this CLI process, never to the native library.
inline std::atomic<bool> interrupted{false};
static_assert(std::atomic<bool>::is_always_lock_free,"signal adapter needs lock-free atomic stores");

class RequestCancellation {
public:
    RequestCancellation(CancellationToken token,std::optional<std::filesystem::path> path)
        :token_(std::move(token)),path_(std::move(path)) {
        poll(); // A pre-existing signal must prevent the first packing attempt.
        if(token_.requested())return;
        worker_=std::thread([this] {
            std::unique_lock<std::mutex> lock(mutex_);
            while(!wake_.wait_for(lock,std::chrono::milliseconds(10),[this]{return stop_;})) {
                lock.unlock();
                try {poll();}
                catch(const std::exception& ex) {
                    lock.lock();error_=ex.what();token_.request();return;
                }
                lock.lock();
                if(token_.requested())return;
            }
        });
    }
    RequestCancellation(const RequestCancellation&)=delete;
    RequestCancellation& operator=(const RequestCancellation&)=delete;
    ~RequestCancellation(){stop();}
    void finish() {
        stop();
        if(!error_.empty())throw std::runtime_error(error_);
        poll();
    }
private:
    void poll() {
        if(interrupted.load(std::memory_order_relaxed))token_.request();
        if(!path_)return;
        std::error_code error;
        const auto status=std::filesystem::status(*path_,error);
        if(error==std::errc::no_such_file_or_directory)return;
        if(error)throw std::runtime_error("cannot inspect cancel file: "+error.message());
        if(std::filesystem::is_directory(status))throw std::runtime_error("cancel-file path is a directory");
        if(std::filesystem::exists(status))token_.request();
    }
    void stop() {
        {std::lock_guard<std::mutex> lock(mutex_);stop_=true;}
        wake_.notify_all();
        if(worker_.joinable())worker_.join();
    }
    CancellationToken token_;
    std::optional<std::filesystem::path> path_;
    std::mutex mutex_;
    std::condition_variable wake_;
    bool stop_=false;
    std::string error_;
    std::thread worker_;
};
}
