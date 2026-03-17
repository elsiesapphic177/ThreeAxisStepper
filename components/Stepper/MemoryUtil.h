#pragma once
#include <stdint.h>
#include <stddef.h>
#include "esp_attr.h"
#include <new>
#include <utility>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"


// pow of 2 check (Compile-time check)
#define IS_POWER_OF_2(x) ((x) != 0 && (((x) & ((x) - 1)) == 0))

/**
 * @brief Optimized lock-free ring buffer for ESP32 Task-to-ISR communication
 * @tparam T Data type (recommended to be simple types like int, float, or small structs)
 * @tparam Size Buffer size (must be a power of 2, e.g., 64, 128, 256)
 */
template <typename T, size_t Size>
class FastISRQueue {
    static_assert(IS_POWER_OF_2(Size), "Queue size must be a power of 2!");

public:
    FastISRQueue() : head_(0), tail_(0) {}

    /**
     * @brief [Task side] Push data into the queue
     * @return true on success, false if the queue is full
     */
    inline bool IRAM_ATTR push(const T& data) {
        uint32_t h = head_;
        uint32_t next_h = (h + 1) & Mask;

        if (next_h != tail_) {
            buffer_[h] = data;
            
            // Write Barrier: Ensure data is written before updating head
            __asm__ __volatile__ ("memw" ::: "memory");
            
            head_ = next_h;
            return true;
        }
        return false;
    }

    /**
     * @brief [ISR side] Read a single data item
     * @param out_data Output data container
     * @return true on success, false if the queue is empty
     */
    inline bool IRAM_ATTR pop(T& out_data) {
        uint32_t t = tail_;
        
        if (t != head_) {
            out_data = buffer_[t];
            
            uint32_t next_t = (t + 1) & Mask;
            
            // Read Barrier: Ensure data is read before updating tail
            __asm__ __volatile__ ("memw" ::: "memory");
            
            tail_ = next_t;
            return true;
        }
        return false;
    }

    /**
     * @brief [ISR side] Efficiently process all accumulated data
     * use Lambda function to process, reducing the overhead of frequently updating tail in ISR
     */
    template <typename Func>
    inline void IRAM_ATTR consume_all(Func func) {
        uint32_t t = tail_;
        uint32_t h = head_; // Cache head to avoid repeatedly reading volatile in the loop

        while (t != h) {
            // Call user-provided processing function
            func(buffer_[t]);
            
            t = (t + 1) & Mask;
        }

        // After batch processing, update tail once
        if (t != tail_) {
            __asm__ __volatile__ ("memw" ::: "memory");
            tail_ = t;
        }
    }
    
    inline void IRAM_ATTR clear() {
        head_ = 0;
        __asm__ __volatile__ ("memw" ::: "memory");
        tail_ = 0;
        __asm__ __volatile__ ("memw" ::: "memory");
    }

    // Check queue status (for debugging or low-priority logic only)
    inline bool isEmpty() const { return head_ == tail_; }
    inline bool isFull() const { return ((head_ + 1) & Mask) == tail_; }
    inline size_t available() const { return (head_ - tail_) & Mask; }

private:
    static const uint32_t Mask = Size - 1;

    // Use alignas to prevent False Sharing
    // Ensure head and tail are not on the same Cache Line (32 bytes)
    struct alignas(32) {
        volatile uint32_t head_;
    };
    
    struct alignas(32) {
        volatile uint32_t tail_;
    };

    // Actual storage
    volatile T buffer_[Size];
    
    // To simplify, we directly declare members here because we used struct alignas above.
    // The compiler will automatically handle padding.
    // Note: In some compiler behaviors, for strict alignment,
    // manual padding declaration might be required, but alignas is usually sufficient.
};

//Single Consumer, single producer fixed-size slot queue. 
template <typename BaseClass,size_t MaxObjSize, size_t QueueDepth>
class FixedSlotQueue {
private:
    // Define slot, enforce 8-byte alignment
    struct Slot {
        alignas(8) uint8_t storage[MaxObjSize];
    };

    Slot slots_[QueueDepth];
    size_t head_ = 0;
    size_t tail_ = 0;
    size_t count_ = 0;
    bool acquired_ = false;        // true when tail slot is held by a Handle
    SemaphoreHandle_t mutex_;
    SemaphoreHandle_t slotsSem_;   // counting semaphore: tracks available slots

public:
    FixedSlotQueue() {
        mutex_ = xSemaphoreCreateMutex();
        slotsSem_ = xSemaphoreCreateCounting(QueueDepth, QueueDepth);
    }

    ~FixedSlotQueue() {
        // clear remaining objects
        while (count_ > 0) {
            BaseClass* obj = reinterpret_cast<BaseClass*>(slots_[tail_].storage);
            obj->~BaseClass();
            tail_ = (tail_ + 1) % QueueDepth;
            count_--;
        }
        vSemaphoreDelete(mutex_);
        vSemaphoreDelete(slotsSem_);
    }

    // -----------------------------------------------------
    // Producer API
    // -----------------------------------------------------
    
    /**
     * @brief Push an object into the queue (Producer)
     * Blocks if the queue is full until a slot becomes available (or timeout).
     * @param profile  Pointer to the object to enqueue
     * @param ticksToWait  Max ticks to wait for a free slot (default: portMAX_DELAY = block forever)
     * @return true on success, false on timeout or oversized object
     */
    bool push(const BaseClass* profile, TickType_t ticksToWait = portMAX_DELAY) {
        if (profile->getSize() > MaxObjSize) return false;

        // 1. Block here until a slot is available (or timeout)
        if (xSemaphoreTake(slotsSem_, ticksToWait) != pdTRUE) return false;

        // 2. Quick lock to copy data and update head
        if (xSemaphoreTake(mutex_, ticksToWait) != pdTRUE) {
            xSemaphoreGive(slotsSem_);  // rollback
            return false;
        }

        void* dest = slots_[head_].storage;
        profile->copyTo(dest);

        head_ = (head_ + 1) % QueueDepth;
        count_++;

        xSemaphoreGive(mutex_);
        return true;
    }

    // -----------------------------------------------------
    // Consumer API (Zero-Copy)
    // -----------------------------------------------------

    /**
     * @brief Smart pointer Handle, used for automatic management of slot release
     */
    class Handle {
        friend class FixedSlotQueue;
        FixedSlotQueue* queue_;
        BaseClass* profile_;

        // Private constructor, can only be created by Queue
        Handle(FixedSlotQueue* q, BaseClass* p) : queue_(q), profile_(p) {}

    public:
        Handle() : queue_(nullptr), profile_(nullptr) {}
        // Support move semantics, disable copy
        Handle(Handle&& other) : queue_(other.queue_), profile_(other.profile_) {
            other.queue_ = nullptr;
            other.profile_ = nullptr;
        }
        Handle& operator=(Handle&& other) {
            if (this != &other) {
                // Release currently held slot
                if (queue_ && profile_) {
                    queue_->release();
                }
                // Take over new resources
                queue_ = other.queue_;
                profile_ = other.profile_;
                other.queue_ = nullptr;
                other.profile_ = nullptr;
            }
            return *this;
        }
        Handle(const Handle&) = delete;
        Handle& operator=(const Handle&) = delete;

        // Destructor automatically releases the slot
        ~Handle() {
            if (queue_ && profile_) {
                queue_->release();
            }
        }

        // Use like a pointer
        BaseClass* operator->() { return profile_; }
        BaseClass& operator*() { return *profile_; }
        
        // Check if valid
        bool isValid() const { return profile_ != nullptr; }
        void reset(){
            if(queue_ && profile_){
                queue_->release();
                queue_ = nullptr;
                profile_ = nullptr;
            }
        }
    };

    /**
     * @brief Acquire the current work item (Consumer)
     * @return Handle object. If the Queue is empty, Handle.isValid() will be false
     */
    Handle acquire() {
        BaseClass* ptr = nullptr;

        // 1. Quick lock: only to read the pointer
        if (xSemaphoreTake(mutex_, portMAX_DELAY) == pdTRUE) {
            if (count_ > 0 && !acquired_) {
                // Get the pointer at the current tail, but *do not* move the tail
                ptr = reinterpret_cast<BaseClass*>(slots_[tail_].storage);
                acquired_ = true;
            }
            xSemaphoreGive(mutex_);
        }
        
        // Return Handle (if ptr is nullptr, it means no work)
        return Handle(ptr ? this : nullptr, ptr);
    }

    void clear() {
        if (xSemaphoreTake(mutex_, portMAX_DELAY) == pdTRUE) {
            // Release all remaining objects and restore slot semaphore
            while (count_ > 0) {
                BaseClass* obj = reinterpret_cast<BaseClass*>(slots_[tail_].storage);
                obj->~BaseClass();
                tail_ = (tail_ + 1) % QueueDepth;
                count_--;
                xSemaphoreGive(slotsSem_);
            }
            acquired_ = false;
            xSemaphoreGive(mutex_);
        }
    }

    // Status query
    size_t count() {
        size_t c = 0;
        if (xSemaphoreTake(mutex_, portMAX_DELAY) == pdTRUE) {
            c = count_;
            xSemaphoreGive(mutex_);
        }
        return c;
    }

private:
    /**
     * @brief Release a slot
     */
    void release() {
        bool expected = false;
        if (xSemaphoreTake(mutex_, portMAX_DELAY) == pdTRUE) {
            if (count_ > 0 && acquired_) {
                // 1. Manually call the destructor
                BaseClass* obj = reinterpret_cast<BaseClass*>(slots_[tail_].storage);
                obj->~BaseClass();

                // 2. Move the tail, officially releasing the space for the Producer
                tail_ = (tail_ + 1) % QueueDepth;
                count_--;
                acquired_ = false;

                // 3. Signal that a slot is now free — wakes up blocked push()
                expected = true;
            }
            xSemaphoreGive(mutex_);
        }
        if(expected){
            xSemaphoreGive(slotsSem_);
        }
    }
};

/**
 * @brief Single-consumer, single-producer fixed-depth queue using std::move.
 *
 * Unlike FixedSlotQueue (designed for polymorphic types with virtual copyTo),
 * MoveQueue works with a known concrete type T and transfers objects via
 * std::move, which is safe for types that own heap resources (pointers, etc.).
 *
 * Producer blocks when full; consumer uses a zero-copy Handle (RAII) that
 * destroys the object and frees the slot on release.
 *
 * @tparam T          Concrete element type (must be move-constructible)
 * @tparam QueueDepth Number of slots
 */
template <typename T, size_t QueueDepth>
class MoveQueue {
private:
    // Each slot holds properly aligned, uninitialized storage for one T.
    struct Slot {
        alignas(T) uint8_t storage[sizeof(T)];
    };

    Slot slots_[QueueDepth];
    size_t head_  = 0;
    size_t tail_  = 0;
    size_t count_ = 0;
    bool acquired_ = false;        // true when tail slot is held by a Handle
    SemaphoreHandle_t mutex_;
    SemaphoreHandle_t slotsSem_;   // counting semaphore: tracks available slots

    T*       ptrAt(size_t i)       { return reinterpret_cast<T*>(slots_[i].storage); }
    const T* ptrAt(size_t i) const { return reinterpret_cast<const T*>(slots_[i].storage); }

public:
    MoveQueue() {
        mutex_    = xSemaphoreCreateMutex();
        slotsSem_ = xSemaphoreCreateCounting(QueueDepth, QueueDepth);
    }

    ~MoveQueue() {
        // Destroy any remaining objects
        while (count_ > 0) {
            ptrAt(tail_)->~T();
            tail_ = (tail_ + 1) % QueueDepth;
            count_--;
        }
        vSemaphoreDelete(mutex_);
        vSemaphoreDelete(slotsSem_);
    }

    // Non-copyable, non-movable
    MoveQueue(const MoveQueue&)            = delete;
    MoveQueue& operator=(const MoveQueue&) = delete;

    // ---------------------------------------------------------
    // Producer API
    // ---------------------------------------------------------

    /**
     * @brief Move-construct an object into the next free slot.
     * Blocks if the queue is full until a slot becomes available (or timeout).
     * @param obj           Rvalue reference to the object to enqueue
     * @param ticksToWait   Max ticks to wait for a free slot (default: block forever)
     * @return true on success, false on timeout
     */
    bool push(T&& obj, TickType_t ticksToWait = portMAX_DELAY) {
        // 1. Block until a slot is available (or timeout)
        if (xSemaphoreTake(slotsSem_, ticksToWait) != pdTRUE) return false;

        // 2. Lock to move-construct into the slot and update head
        if (xSemaphoreTake(mutex_, ticksToWait) != pdTRUE) {
            xSemaphoreGive(slotsSem_);  // rollback
            return false;
        }

        // Placement-new with move constructor — no memcpy, no shallow copy
        new (slots_[head_].storage) T(std::move(obj));

        head_ = (head_ + 1) % QueueDepth;
        count_++;

        xSemaphoreGive(mutex_);
        return true;
    }

    /**
     * @brief Construct an object in-place inside the next free slot.
     * @tparam Args Constructor argument types
     * @param ticksToWait   Max ticks to wait for a free slot
     * @param args          Arguments forwarded to T's constructor
     * @return true on success, false on timeout
     */
    template <typename... Args>
    bool emplace(TickType_t ticksToWait, Args&&... args) {
        if (xSemaphoreTake(slotsSem_, ticksToWait) != pdTRUE) return false;

        if (xSemaphoreTake(mutex_, ticksToWait) != pdTRUE) {
            xSemaphoreGive(slotsSem_);
            return false;
        }

        new (slots_[head_].storage) T(std::forward<Args>(args)...);

        head_ = (head_ + 1) % QueueDepth;
        count_++;

        xSemaphoreGive(mutex_);
        return true;
    }

    // ---------------------------------------------------------
    // Consumer API (Zero-Copy with RAII Handle)
    // ---------------------------------------------------------

    /**
     * @brief RAII handle that grants exclusive access to the front element.
     * On destruction (or reset), the element is destroyed and the slot freed.
     */
    class Handle {
        friend class MoveQueue;
        MoveQueue* queue_;
        T*         ptr_;

        Handle(MoveQueue* q, T* p) : queue_(q), ptr_(p) {}

    public:
        Handle() : queue_(nullptr), ptr_(nullptr) {}

        // Move-only
        Handle(Handle&& other) : queue_(other.queue_), ptr_(other.ptr_) {
            other.queue_ = nullptr;
            other.ptr_   = nullptr;
        }
        Handle& operator=(Handle&& other) {
            if (this != &other) {
                if (queue_ && ptr_) queue_->release();
                queue_ = other.queue_;
                ptr_   = other.ptr_;
                other.queue_ = nullptr;
                other.ptr_   = nullptr;
            }
            return *this;
        }
        Handle(const Handle&)            = delete;
        Handle& operator=(const Handle&) = delete;

        ~Handle() {
            if (queue_ && ptr_) queue_->release();
        }

        T* operator->()             { return ptr_; }
        const T* operator->() const { return ptr_; }
        T& operator*()              { return *ptr_; }
        const T& operator*() const  { return *ptr_; }

        bool isValid() const { return ptr_ != nullptr; }

        void reset() {
            if (queue_ && ptr_) {
                queue_->release();
                queue_ = nullptr;
                ptr_   = nullptr;
            }
        }
    };

    /**
     * @brief Acquire the front element for consumption (zero-copy).
     * @return Handle — use isValid() to check if the queue was non-empty.
     */
    Handle acquire() {
        T* ptr = nullptr;
        if (xSemaphoreTake(mutex_, portMAX_DELAY) == pdTRUE) {
            if (count_ > 0 && !acquired_) {
                ptr = ptrAt(tail_);
                acquired_ = true;
            }
            xSemaphoreGive(mutex_);
        }
        return Handle(ptr ? this : nullptr, ptr);
    }

    /**
     * @brief Remove the most recently pushed element (de-push).
     * Safe even when a Handle holds the front element — the back is a
     * different slot unless count==1, in which case pop_back is refused.
     * @return true if an element was removed, false if nothing to pop
     */
    bool pop_back() {
        bool freed = false;
        if (xSemaphoreTake(mutex_, portMAX_DELAY) == pdTRUE) {
            // Must have at least one element that is NOT the acquired front
            if (count_ > 0 && !(count_ == 1 && acquired_)) {
                head_ = (head_ + QueueDepth - 1) % QueueDepth;
                ptrAt(head_)->~T();
                count_--;
                freed = true;
            }
            xSemaphoreGive(mutex_);
        }
        if (freed) {
            xSemaphoreGive(slotsSem_);
        }
        return freed;
    }

    /**
     * @brief Discard all queued elements, destroying each one properly.
     */
    void clear() {
        if (xSemaphoreTake(mutex_, portMAX_DELAY) == pdTRUE) {
            while (count_ > 0) {
                ptrAt(tail_)->~T();
                tail_ = (tail_ + 1) % QueueDepth;
                count_--;
                xSemaphoreGive(slotsSem_);
            }
            acquired_ = false;
            xSemaphoreGive(mutex_);
        }
    }

    // Status query
    size_t count() {
        size_t c = 0;
        if (xSemaphoreTake(mutex_, portMAX_DELAY) == pdTRUE) {
            c = count_;
            xSemaphoreGive(mutex_);
        }
        return c;
    }

private:
    /**
     * @brief Release the currently acquired slot (called by Handle).
     */
    void release() {
        bool freed = false;
        if (xSemaphoreTake(mutex_, portMAX_DELAY) == pdTRUE) {
            if (count_ > 0 && acquired_) {
                ptrAt(tail_)->~T();
                tail_ = (tail_ + 1) % QueueDepth;
                count_--;
                acquired_ = false;
                freed = true;
            }
            xSemaphoreGive(mutex_);
        }
        if (freed) {
            xSemaphoreGive(slotsSem_);
        }
    }
};
