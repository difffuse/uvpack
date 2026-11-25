#pragma once
#include <atomic>
#include <memory>
#include <stdexcept>

namespace uvpack {
// Copies share one request's state. A newly constructed token starts uncancelled;
// cancellation is permanent, so use a new token for the next request.
class CancellationToken {
public:
    CancellationToken():state_(std::make_shared<std::atomic<bool>>(false)) {}
    // A moved-from handle is empty and remains safe to query or signal.
    void request() const noexcept {if(state_)state_->store(true,std::memory_order_relaxed);}
    bool requested() const noexcept {return state_ && state_->load(std::memory_order_relaxed);}
    void throw_if_requested() const {
        if(requested())throw std::runtime_error("packing cancelled");
    }
private:
    std::shared_ptr<std::atomic<bool>> state_;
};
}
