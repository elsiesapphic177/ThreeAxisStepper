#pragma once
#include <stdint.h>
#include <stddef.h>
#include <cstddef>
#include <new>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <utility>
#include <type_traits>
#include <tuple>
#include <initializer_list>

/**
 * @brief Fixed-size string class for ESP32 firmware — drop-in replacement for std::string
 * @tparam MaxLen Maximum number of characters (excluding null terminator)
 *
 * Advantages over std::string on embedded targets:
 * - Zero heap allocation (fully stack / static)
 * - Deterministic memory footprint
 * - No memory fragmentation
 * - Faster construction / destruction (no malloc / free)
 * - Interoperable across different capacities via templates
 */
template<size_t MaxLen>
class FixedString {
    static_assert(MaxLen > 0, "FixedString capacity must be > 0");

private:
    char   buf_[MaxLen + 1];   // +1 for null terminator
    size_t len_;

public:
    static constexpr size_t npos = static_cast<size_t>(-1);

    // =====================================================================
    //  Constructors
    // =====================================================================
    FixedString() : len_(0) { buf_[0] = '\0'; }

    FixedString(const char* str) {
        if (str) {
            len_ = strlen(str);
            if (len_ > MaxLen) len_ = MaxLen;
            memcpy(buf_, str, len_);
        } else {
            len_ = 0;
        }
        buf_[len_] = '\0';
    }

    FixedString(const char* str, size_t n) {
        if (str && n > 0) {
            len_ = (n > MaxLen) ? MaxLen : n;
            memcpy(buf_, str, len_);
        } else {
            len_ = 0;
        }
        buf_[len_] = '\0';
    }

    FixedString(size_t n, char c) {
        len_ = (n > MaxLen) ? MaxLen : n;
        memset(buf_, c, len_);
        buf_[len_] = '\0';
    }

    // Copy (same capacity)
    FixedString(const FixedString& other) : len_(other.len_) {
        memcpy(buf_, other.buf_, len_ + 1);
    }

    // Move (same capacity — for fixed buffers move == copy, but we clear the source)
    FixedString(FixedString&& other) : len_(other.len_) {
        memcpy(buf_, other.buf_, len_ + 1);
        other.len_ = 0;
        other.buf_[0] = '\0';
    }

    // Cross-capacity copy
    template<size_t OtherLen>
    FixedString(const FixedString<OtherLen>& other) {
        len_ = other.length();
        if (len_ > MaxLen) len_ = MaxLen;
        memcpy(buf_, other.c_str(), len_);
        buf_[len_] = '\0';
    }

    // =====================================================================
    //  Assignment
    // =====================================================================
    FixedString& operator=(const char* str) {
        if (str) {
            len_ = strlen(str);
            if (len_ > MaxLen) len_ = MaxLen;
            memcpy(buf_, str, len_);
        } else {
            len_ = 0;
        }
        buf_[len_] = '\0';
        return *this;
    }

    FixedString& operator=(const FixedString& other) {
        if (this != &other) {
            len_ = other.len_;
            memcpy(buf_, other.buf_, len_ + 1);
        }
        return *this;
    }

    FixedString& operator=(FixedString&& other) {
        if (this != &other) {
            len_ = other.len_;
            memcpy(buf_, other.buf_, len_ + 1);
            other.len_ = 0;
            other.buf_[0] = '\0';
        }
        return *this;
    }

    template<size_t OtherLen>
    FixedString& operator=(const FixedString<OtherLen>& other) {
        len_ = other.length();
        if (len_ > MaxLen) len_ = MaxLen;
        memcpy(buf_, other.c_str(), len_);
        buf_[len_] = '\0';
        return *this;
    }

    FixedString& operator=(char c) {
        buf_[0] = c;
        buf_[1] = '\0';
        len_ = 1;
        return *this;
    }

    // =====================================================================
    //  Element access
    // =====================================================================
    const char* c_str()    const { return buf_; }
    const char* data()     const { return buf_; }
    char*       data()           { return buf_; }
    size_t      length()   const { return len_; }
    size_t      size()     const { return len_; }
    constexpr size_t max_size() const { return MaxLen; }
    constexpr size_t capacity() const { return MaxLen; }
    bool        empty()    const { return len_ == 0; }
    bool        full()     const { return len_ >= MaxLen; }
    size_t      remaining()const { return MaxLen - len_; }

    char&       operator[](size_t i)       { return buf_[i]; }
    const char& operator[](size_t i) const { return buf_[i]; }

    char&       front()       { return buf_[0]; }
    const char& front() const { return buf_[0]; }
    char&       back()        { return buf_[len_ - 1]; }
    const char& back()  const { return buf_[len_ - 1]; }

    // =====================================================================
    //  Iterators (range-for compatible)
    // =====================================================================
    char*       begin()        { return buf_; }
    const char* begin()  const { return buf_; }
    char*       end()          { return buf_ + len_; }
    const char* end()    const { return buf_ + len_; }
    const char* cbegin() const { return buf_; }
    const char* cend()   const { return buf_ + len_; }

    // =====================================================================
    //  Modifiers
    // =====================================================================
    FixedString& append(const char* str) {
        if (str) {
            size_t slen  = strlen(str);
            size_t avail = MaxLen - len_;
            size_t n     = (slen < avail) ? slen : avail;
            memcpy(buf_ + len_, str, n);
            len_ += n;
            buf_[len_] = '\0';
        }
        return *this;
    }

    FixedString& append(const char* str, size_t n) {
        if (str && n > 0) {
            size_t avail = MaxLen - len_;
            if (n > avail) n = avail;
            memcpy(buf_ + len_, str, n);
            len_ += n;
            buf_[len_] = '\0';
        }
        return *this;
    }

    FixedString& append(size_t n, char c) {
        size_t avail = MaxLen - len_;
        if (n > avail) n = avail;
        memset(buf_ + len_, c, n);
        len_ += n;
        buf_[len_] = '\0';
        return *this;
    }

    FixedString& append(char c) {
        if (len_ < MaxLen) {
            buf_[len_++] = c;
            buf_[len_] = '\0';
        }
        return *this;
    }

    template<size_t OtherLen>
    FixedString& append(const FixedString<OtherLen>& other) {
        return append(other.c_str(), other.length());
    }

    FixedString& operator+=(const char* str) { return append(str); }
    FixedString& operator+=(char c)          { return append(c); }

    template<size_t OtherLen>
    FixedString& operator+=(const FixedString<OtherLen>& other) {
        return append(other);
    }

    void push_back(char c) { append(c); }

    void pop_back() {
        if (len_ > 0) {
            buf_[--len_] = '\0';
        }
    }

    void clear() {
        len_ = 0;
        buf_[0] = '\0';
    }

    void truncate(size_t newLen) {
        if (newLen < len_) {
            len_ = newLen;
            buf_[len_] = '\0';
        }
    }

    /// Erase @p n characters starting at @p pos
    FixedString& erase(size_t pos = 0, size_t n = npos) {
        if (pos >= len_) return *this;
        if (n == npos || pos + n >= len_) {
            len_ = pos;
        } else {
            memmove(buf_ + pos, buf_ + pos + n, len_ - pos - n);
            len_ -= n;
        }
        buf_[len_] = '\0';
        return *this;
    }

    /// Insert @p str at @p pos
    FixedString& insert(size_t pos, const char* str) {
        if (!str || pos > len_) return *this;
        size_t slen  = strlen(str);
        size_t avail = MaxLen - len_;
        if (slen > avail) slen = avail;
        if (slen == 0) return *this;
        memmove(buf_ + pos + slen, buf_ + pos, len_ - pos);
        memcpy(buf_ + pos, str, slen);
        len_ += slen;
        buf_[len_] = '\0';
        return *this;
    }

    /// Replace [pos, pos+n) with @p str
    FixedString& replace(size_t pos, size_t n, const char* str) {
        erase(pos, n);
        insert(pos, str);
        return *this;
    }

    // =====================================================================
    //  Comparison
    // =====================================================================
    int compare(const char* str) const {
        return strcmp(buf_, str ? str : "");
    }

    template<size_t OtherLen>
    int compare(const FixedString<OtherLen>& other) const {
        return strcmp(buf_, other.c_str());
    }

    bool operator==(const char* s) const { return compare(s) == 0; }
    bool operator!=(const char* s) const { return compare(s) != 0; }
    bool operator< (const char* s) const { return compare(s) <  0; }
    bool operator> (const char* s) const { return compare(s) >  0; }
    bool operator<=(const char* s) const { return compare(s) <= 0; }
    bool operator>=(const char* s) const { return compare(s) >= 0; }

    template<size_t O> bool operator==(const FixedString<O>& o) const { return compare(o) == 0; }
    template<size_t O> bool operator!=(const FixedString<O>& o) const { return compare(o) != 0; }
    template<size_t O> bool operator< (const FixedString<O>& o) const { return compare(o) <  0; }
    template<size_t O> bool operator> (const FixedString<O>& o) const { return compare(o) >  0; }
    template<size_t O> bool operator<=(const FixedString<O>& o) const { return compare(o) <= 0; }
    template<size_t O> bool operator>=(const FixedString<O>& o) const { return compare(o) >= 0; }

    // Allow  "hello" == fixedStr
    friend bool operator==(const char* lhs, const FixedString& rhs) { return rhs == lhs; }
    friend bool operator!=(const char* lhs, const FixedString& rhs) { return rhs != lhs; }

    // =====================================================================
    //  Search
    // =====================================================================
    size_t find(char c, size_t from = 0) const {
        for (size_t i = from; i < len_; ++i)
            if (buf_[i] == c) return i;
        return npos;
    }

    size_t find(const char* str, size_t from = 0) const {
        if (!str || from >= len_) return npos;
        const char* p = strstr(buf_ + from, str);
        return p ? static_cast<size_t>(p - buf_) : npos;
    }

    size_t rfind(char c) const {
        for (size_t i = len_; i > 0; --i)
            if (buf_[i - 1] == c) return i - 1;
        return npos;
    }

    bool contains(const char* str) const { return find(str) != npos; }
    bool contains(char c)          const { return find(c)   != npos; }

    bool startsWith(const char* prefix) const {
        if (!prefix) return true;
        size_t plen = strlen(prefix);
        return len_ >= plen && memcmp(buf_, prefix, plen) == 0;
    }

    bool endsWith(const char* suffix) const {
        if (!suffix) return true;
        size_t slen = strlen(suffix);
        return len_ >= slen && memcmp(buf_ + len_ - slen, suffix, slen) == 0;
    }

    // =====================================================================
    //  Substring
    // =====================================================================
    FixedString substr(size_t pos, size_t n = npos) const {
        if (pos >= len_) return FixedString();
        size_t actual = (n == npos || pos + n > len_) ? (len_ - pos) : n;
        return FixedString(buf_ + pos, actual);
    }

    // =====================================================================
    //  printf-style formatting
    // =====================================================================

    /// Create a FixedString from a printf format.  e.g. FixedString<64>::format("x=%d", 42)
    static FixedString format(const char* fmt, ...) __attribute__((format(printf, 1, 2))) {
        FixedString result;
        va_list args;
        va_start(args, fmt);
        int written = vsnprintf(result.buf_, MaxLen + 1, fmt, args);
        va_end(args);
        result.len_ = (written > 0)
                      ? (static_cast<size_t>(written) > MaxLen ? MaxLen : static_cast<size_t>(written))
                      : 0;
        return result;
    }

    /// Append printf-formatted text.  Returns chars that *would* have been written (like snprintf).
    int appendFormat(const char* fmt, ...) __attribute__((format(printf, 2, 3))) {
        va_list args;
        va_start(args, fmt);
        size_t avail   = MaxLen - len_;
        int    written = vsnprintf(buf_ + len_, avail + 1, fmt, args);
        va_end(args);
        if (written > 0) {
            size_t added = (static_cast<size_t>(written) > avail) ? avail : static_cast<size_t>(written);
            len_ += added;
        }
        return written;
    }

    // =====================================================================
    //  Numeric conversions
    // =====================================================================
    int    toInt   (int base = 10) const { return static_cast<int>(strtol(buf_, nullptr, base)); }
    long   toLong  (int base = 10) const { return strtol(buf_, nullptr, base); }
    float  toFloat ()              const { return strtof(buf_, nullptr); }
    double toDouble()              const { return strtod(buf_, nullptr); }

    static FixedString fromInt  (int val)                        { return format("%d", val); }
    static FixedString fromFloat(float val, int decimals = 2)    { return format("%.*f", decimals, static_cast<double>(val)); }

    // =====================================================================
    //  Implicit conversion — convenient for ESP-IDF C APIs (printf, ESP_LOGI …)
    // =====================================================================
    operator const char*() const { return buf_; }
};

template<size_t N>
FixedString<N> splitString(FixedString<N>& stringToSplit, const char* symbol){
    int pos = stringToSplit.find(symbol);
    FixedString<N> out;
    if(pos >= 0){
        out = stringToSplit.substr(0, pos);
        stringToSplit = stringToSplit.substr(pos + strlen(symbol));
    }else{
        out = stringToSplit;
        stringToSplit = "";
    }
    return out;
}

template<size_t N>
FixedString<N> splitString(FixedString<N>& stringToSplit){
    return splitString(stringToSplit, ",");
}

template<size_t N>
void removeHeadBlackspaceString(FixedString<N>& stringToRemove){
    if(stringToRemove.front() == '\0' || stringToRemove.front() != ' ')
        return;
    for(int i = 1; i < (int)stringToRemove.length(); i++){
        if(stringToRemove[i] != ' '){
            stringToRemove = stringToRemove.substr(i);
            return;
        }
    }
}

template<size_t N>
void removeBackBlackspaceString(FixedString<N>& stringToRemove){
    while(stringToRemove.length() > 0 && stringToRemove.back() == ' '){
        stringToRemove = stringToRemove.substr(0, stringToRemove.length() - 1);
    }
}

// =========================================================================
//  Free concatenation operators
// =========================================================================
template<size_t L1, size_t L2>
FixedString<(L1 > L2 ? L1 : L2)> operator+(const FixedString<L1>& a, const FixedString<L2>& b) {
    FixedString<(L1 > L2 ? L1 : L2)> result(a);
    result.append(b);
    return result;
}

template<size_t N>
FixedString<N> operator+(const FixedString<N>& a, const char* b) {
    FixedString<N> result(a);
    result.append(b);
    return result;
}

template<size_t N>
FixedString<N> operator+(const char* a, const FixedString<N>& b) {
    FixedString<N> result(a);
    result.append(b);
    return result;
}

// =========================================================================
//  Common aliases
// =========================================================================
using String16  = FixedString<16>;
using String20  = FixedString<20>;
using String32  = FixedString<32>;
using String64  = FixedString<64>;
using String128 = FixedString<128>;
using String256 = FixedString<256>;
using String512 = FixedString<512>;

// =========================================================================
//  FixedFunction — heap-free replacement for std::function
// =========================================================================

// Primary template (undefined) — only the partial specialization below is used.
template<typename Signature, size_t BufSize = 64>
class FixedFunction;

// Trait to detect any FixedFunction instantiation
template<typename T> struct is_fixed_function : std::false_type {};
template<typename Sig, size_t B> struct is_fixed_function<FixedFunction<Sig, B>> : std::true_type {};

/**
 * @brief Fixed-size, heap-free replacement for std::function
 * @tparam Ret    Return type
 * @tparam Args   Argument types
 * @tparam BufSize  Size of internal storage in bytes (default 64)
 *
 * Stores any callable (lambda, functor, function pointer) whose size ≤ BufSize
 * inside an inline buffer — zero heap allocation.  If the callable is too large,
 * compilation fails with a clear static_assert message.
 *
 * Usage is identical to std::function:
 * @code
 *   FixedFunction<void(int)> fn = [&](int x) { total += x; };
 *   fn(42);
 *
 *   // Explicit buffer size for large captures:
 *   FixedFunction<void(), 64> big = [arr]() { process(arr); };
 * @endcode
 */
template<typename Ret, typename... Args, size_t BufSize>
class FixedFunction<Ret(Args...), BufSize> {
    // -----------------------------------------------------------------
    //  Type-erased operations table (one per concrete callable type)
    // -----------------------------------------------------------------
    struct Ops {
        Ret  (*invoke)(void* storage, Args... args);
        void (*destroy)(void* storage);
        void (*copyConstruct)(void* dst, const void* src);
        void (*moveConstruct)(void* dst, void* src);
    };

    // Generate an Ops table for a concrete callable type F
    template<typename F>
    static const Ops* opsFor() {
        static const Ops ops = {
            // invoke
            [](void* storage, Args... args) -> Ret {
                return (*static_cast<F*>(storage))(static_cast<Args&&>(args)...);
            },
            // destroy
            [](void* storage) {
                static_cast<F*>(storage)->~F();
            },
            // copy-construct
            [](void* dst, const void* src) {
                new (dst) F(*static_cast<const F*>(src));
            },
            // move-construct
            [](void* dst, void* src) {
                new (dst) F(static_cast<F&&>(*static_cast<F*>(src)));
            }
        };
        return &ops;
    }

    // -----------------------------------------------------------------
    //  Data members
    // -----------------------------------------------------------------
    alignas(alignof(std::max_align_t)) char storage_[BufSize];
    const Ops* ops_;

    // Allow cross-size FixedFunction with same signature to access internals
    template<typename, size_t> friend class FixedFunction;

public:
    // =================================================================
    //  Constructors
    // =================================================================

    /// Empty (null) function
    FixedFunction() : ops_(nullptr) {}
    FixedFunction(decltype(nullptr)) : ops_(nullptr) {}

    /// Construct from any callable (lambda, functor, function pointer)
    template<typename F,
             typename = typename std::enable_if<
                 !is_fixed_function<typename std::decay<F>::type>::value
             >::type>
    FixedFunction(F&& f) {
        using Callable = typename std::decay<F>::type;
        static_assert(std::is_invocable_r_v<Ret, Callable, Args...>,
            "FixedFunction: callable signature mismatch — "
            "the lambda/functor cannot be called with the declared argument types, "
            "or its return type is incompatible.");
        static_assert(sizeof(Callable) <= BufSize,
            "FixedFunction: callable is too large for the inline buffer. "
            "Increase BufSize template parameter.");
        static_assert(alignof(Callable) <= alignof(std::max_align_t),
            "FixedFunction: callable alignment exceeds buffer alignment.");

        new (storage_) Callable(static_cast<F&&>(f));
        ops_ = opsFor<Callable>();
    }

    /// Copy constructor
    FixedFunction(const FixedFunction& other) : ops_(other.ops_) {
        if (ops_) {
            ops_->copyConstruct(storage_, other.storage_);
        }
    }

    /// Move constructor
    FixedFunction(FixedFunction&& other) : ops_(other.ops_) {
        if (ops_) {
            ops_->moveConstruct(storage_, other.storage_);
            other.ops_->destroy(other.storage_);
            other.ops_ = nullptr;
        }
    }

    /// Cross-size copy constructor (from smaller or equal buffer)
    template<size_t OtherBuf,
             typename = std::enable_if_t<(OtherBuf != BufSize) && (OtherBuf <= BufSize)>>
    FixedFunction(const FixedFunction<Ret(Args...), OtherBuf>& other)
        : ops_(reinterpret_cast<const Ops*>(other.ops_)) {
        if (ops_) {
            ops_->copyConstruct(storage_, other.storage_);
        }
    }

    /// Cross-size move constructor (from smaller or equal buffer)
    template<size_t OtherBuf,
             typename = std::enable_if_t<(OtherBuf != BufSize) && (OtherBuf <= BufSize)>>
    FixedFunction(FixedFunction<Ret(Args...), OtherBuf>&& other)
        : ops_(reinterpret_cast<const Ops*>(other.ops_)) {
        if (ops_) {
            ops_->moveConstruct(storage_, other.storage_);
            // Destroy source through its own ops pointer
            other.ops_->destroy(other.storage_);
            other.ops_ = nullptr;
        }
    }

    /// Destructor
    ~FixedFunction() {
        if (ops_) {
            ops_->destroy(storage_);
        }
    }

    // =================================================================
    //  Assignment
    // =================================================================

    FixedFunction& operator=(const FixedFunction& other) {
        if (this != &other) {
            if (ops_) ops_->destroy(storage_);
            ops_ = other.ops_;
            if (ops_) ops_->copyConstruct(storage_, other.storage_);
        }
        return *this;
    }

    FixedFunction& operator=(FixedFunction&& other) {
        if (this != &other) {
            if (ops_) ops_->destroy(storage_);
            ops_ = other.ops_;
            if (ops_) {
                ops_->moveConstruct(storage_, other.storage_);
                other.ops_->destroy(other.storage_);
                other.ops_ = nullptr;
            }
        }
        return *this;
    }

    FixedFunction& operator=(decltype(nullptr)) {
        if (ops_) {
            ops_->destroy(storage_);
            ops_ = nullptr;
        }
        return *this;
    }

    /// Cross-size copy assignment (from smaller or equal buffer)
    template<size_t OtherBuf,
             typename = std::enable_if_t<(OtherBuf != BufSize) && (OtherBuf <= BufSize)>>
    FixedFunction& operator=(const FixedFunction<Ret(Args...), OtherBuf>& other) {
        if (ops_) ops_->destroy(storage_);
        ops_ = reinterpret_cast<const Ops*>(other.ops_);
        if (ops_) ops_->copyConstruct(storage_, other.storage_);
        return *this;
    }

    /// Cross-size move assignment (from smaller or equal buffer)
    template<size_t OtherBuf,
             typename = std::enable_if_t<(OtherBuf != BufSize) && (OtherBuf <= BufSize)>>
    FixedFunction& operator=(FixedFunction<Ret(Args...), OtherBuf>&& other) {
        if (ops_) ops_->destroy(storage_);
        ops_ = reinterpret_cast<const Ops*>(other.ops_);
        if (ops_) {
            ops_->moveConstruct(storage_, other.storage_);
            other.ops_->destroy(other.storage_);
            other.ops_ = nullptr;
        }
        return *this;
    }

    /// Assign from any callable
    template<typename F,
             typename = typename std::enable_if<
                 !is_fixed_function<typename std::decay<F>::type>::value
             >::type>
    FixedFunction& operator=(F&& f) {
        if (ops_) ops_->destroy(storage_);

        using Callable = typename std::decay<F>::type;
        static_assert(std::is_invocable_r_v<Ret, Callable, Args...>,
            "FixedFunction: callable signature mismatch — "
            "the lambda/functor cannot be called with the declared argument types, "
            "or its return type is incompatible.");
        static_assert(sizeof(Callable) <= BufSize,
            "FixedFunction: callable is too large for the inline buffer.");
        static_assert(alignof(Callable) <= alignof(std::max_align_t),
            "FixedFunction: callable alignment exceeds buffer alignment.");

        new (storage_) Callable(static_cast<F&&>(f));
        ops_ = opsFor<Callable>();
        return *this;
    }

    // =================================================================
    //  Invocation
    // =================================================================

    Ret operator()(Args... args) const {
        // If empty, just return default-constructed Ret (or nothing for void)
        if (!ops_) return Ret();
        return ops_->invoke(const_cast<void*>(static_cast<const void*>(storage_)),
                            static_cast<Args&&>(args)...);
    }

    // =================================================================
    //  Bind — produce a FixedFunction<Ret()> with a *larger* output buffer
    // =================================================================
    //
    // The Binder stores:  Ops* + bound args + callable[BufSize]
    // so the output buffer must be at least that big.  The output size
    // is computed automatically and checked at compile time.
    //
    // Helper: compute the minimum output BufSize for bind().
    // Layout of Binder: { Ops*, BoundData, alignas(max_align_t) callable[BufSize] }
    // We round the header up to max_align_t, then add BufSize for the callable.
    template<typename BoundData>
    static constexpr size_t bindBufSize() {
        constexpr size_t maxA  = alignof(std::max_align_t);
        constexpr size_t hdr   = sizeof(const Ops*) + sizeof(BoundData);
        constexpr size_t pad   = (hdr + maxA - 1) / maxA * maxA;
        return pad + BufSize;
    }

    /// Bind a single argument.  Returns FixedFunction<Ret(), OutBuf>.
    /// OutBuf defaults to the exact size needed; override to a larger value if desired.
    template <typename BoundArg,
              size_t OutBuf = bindBufSize<std::decay_t<BoundArg>>(),
              typename = std::enable_if_t<(sizeof...(Args) == 1) && (sizeof(BoundArg) >= 0)>>
    FixedFunction<Ret(), OutBuf> bind(BoundArg&& arg) && {
        using BoundArgT = std::decay_t<BoundArg>;

        struct Binder {
            const Ops* ops;
            BoundArgT arg;
            alignas(std::max_align_t) char callable[BufSize];

            Binder(FixedFunction&& src, BoundArgT&& boundArg)
                : ops(src.ops_), arg(static_cast<BoundArgT&&>(boundArg)) {
                if (ops) {
                    ops->moveConstruct(callable, src.storage_);
                    ops->destroy(src.storage_);
                    src.ops_ = nullptr;
                }
            }

            Binder(const Binder& o)
                : ops(o.ops), arg(o.arg) {
                if (ops) ops->copyConstruct(callable, o.callable);
            }

            Binder(Binder&& o) noexcept
                : ops(o.ops), arg(static_cast<BoundArgT&&>(o.arg)) {
                if (ops) {
                    ops->moveConstruct(callable, o.callable);
                    o.ops->destroy(o.callable);
                    o.ops = nullptr;
                }
            }

            ~Binder() {
                if (ops) ops->destroy(callable);
            }

            Ret operator()() const {
                if (!ops) {
                    if constexpr (!std::is_void_v<Ret>) return Ret();
                    return;
                }
                if constexpr (std::is_void_v<Ret>) {
                    ops->invoke(const_cast<void*>(static_cast<const void*>(callable)), arg);
                } else {
                    return ops->invoke(const_cast<void*>(static_cast<const void*>(callable)), arg);
                }
            }
        };

        static_assert(sizeof(Binder) <= OutBuf,
            "FixedFunction::bind(): output buffer too small for Binder. "
            "Increase OutBuf template parameter.");

        return FixedFunction<Ret(), OutBuf>(Binder{static_cast<FixedFunction&&>(*this),
                                                   static_cast<BoundArgT&&>(arg)});
    }

    /// Bind all arguments (variadic).  Returns FixedFunction<Ret(), OutBuf>.
    template <typename... BoundArgs,
              size_t OutBuf = bindBufSize<std::tuple<std::decay_t<BoundArgs>...>>(),
              typename = std::enable_if_t<(sizeof...(Args) > 1)
                                       && (sizeof...(BoundArgs) == sizeof...(Args))
                                       && (sizeof...(BoundArgs) >= 0)>>
    FixedFunction<Ret(), OutBuf> bind(BoundArgs&&... args) && {
        using BoundTuple = std::tuple<std::decay_t<BoundArgs>...>;

        struct Binder {
            const Ops* ops;
            BoundTuple boundArgs;
            alignas(std::max_align_t) char callable[BufSize];

            Binder(FixedFunction&& src, BoundArgs&&... inArgs)
                : ops(src.ops_), boundArgs(static_cast<BoundArgs&&>(inArgs)...) {
                if (ops) {
                    ops->moveConstruct(callable, src.storage_);
                    ops->destroy(src.storage_);
                    src.ops_ = nullptr;
                }
            }

            Binder(const Binder& o)
                : ops(o.ops), boundArgs(o.boundArgs) {
                if (ops) ops->copyConstruct(callable, o.callable);
            }

            Binder(Binder&& o) noexcept
                : ops(o.ops), boundArgs(static_cast<BoundTuple&&>(o.boundArgs)) {
                if (ops) {
                    ops->moveConstruct(callable, o.callable);
                    o.ops->destroy(o.callable);
                    o.ops = nullptr;
                }
            }

            ~Binder() {
                if (ops) ops->destroy(callable);
            }

            Ret operator()() const {
                if (!ops) {
                    if constexpr (!std::is_void_v<Ret>) return Ret();
                    return;
                }
                if constexpr (std::is_void_v<Ret>) {
                    std::apply([&](auto&&... unpacked) {
                        ops->invoke(const_cast<void*>(static_cast<const void*>(callable)),
                                    static_cast<decltype(unpacked)&&>(unpacked)...);
                    }, boundArgs);
                } else {
                    return std::apply([&](auto&&... unpacked) -> Ret {
                        return ops->invoke(const_cast<void*>(static_cast<const void*>(callable)),
                                            static_cast<decltype(unpacked)&&>(unpacked)...);
                    }, boundArgs);
                }
            }
        };

        static_assert(sizeof(Binder) <= OutBuf,
            "FixedFunction::bind(): output buffer too small for Binder. "
            "Increase OutBuf template parameter.");

        return FixedFunction<Ret(), OutBuf>(Binder{static_cast<FixedFunction&&>(*this),
                                                   static_cast<BoundArgs&&>(args)...});
    }

    // =================================================================
    //  State query
    // =================================================================

    explicit operator bool() const { return ops_ != nullptr; }

    bool operator==(decltype(nullptr)) const { return ops_ == nullptr; }
    bool operator!=(decltype(nullptr)) const { return ops_ != nullptr; }

    /// Reset to empty
    void reset() {
        if (ops_) {
            ops_->destroy(storage_);
            ops_ = nullptr;
        }
    }

    /// Swap with another FixedFunction
    void swap(FixedFunction& other) {
        FixedFunction tmp(static_cast<FixedFunction&&>(other));
        other = static_cast<FixedFunction&&>(*this);
        *this = static_cast<FixedFunction&&>(tmp);
    }

    /// Maximum size of callable this instance can hold
    static constexpr size_t bufferSize() { return BufSize; }
};

// -----------------------------------------------------------------------------
// Free-function helpers
// -----------------------------------------------------------------------------

/// Bind a FixedFunction into a zero-argument FixedFunction.
///   auto g = bind(f, args...);   // output size computed automatically
/// The output buffer is large enough to hold the original callable + bound args.
/// If `f` is expensive to copy, call with std::move(f) to move it.
template <typename Ret, typename... Args, size_t BufSize, typename... BoundArgs>
auto bind(FixedFunction<Ret(Args...), BufSize> f, BoundArgs&&... args) {
    return std::move(f).bind(std::forward<BoundArgs>(args)...);
}

template<typename Obj, size_t Length>
class RingBuffer {
private:
    Obj objects[Length];
    int head{0};   // index of next write
    int tail{0};   // index of next read
    int count_{0};

public:
    RingBuffer() = default;

    RingBuffer(std::initializer_list<Obj> init) {
        for (auto& v : init) push(v);
    }

    // Push to back (copy)
    void push(const Obj& obj) {
        objects[head] = obj;
        head = (head + 1) % Length;
        if (count_ < (int)Length) {
            ++count_;
        } else {
            tail = (tail + 1) % Length; // overwrite oldest
        }
    }

    // Push to back (move)
    void push(Obj&& obj) {
        objects[head] = std::move(obj);
        head = (head + 1) % Length;
        if (count_ < (int)Length) {
            ++count_;
        } else {
            tail = (tail + 1) % Length; // overwrite oldest
        }
    }

    // Pop from front (moves out of the slot)
    Obj pop_front() {
        // boundary check first
        assert(count_ > 0 && "RingBuffer underflow on popFront");
        Obj obj = std::move(objects[tail]);
        tail = (tail + 1) % Length;
        --count_;
        return obj;
    }

    // Pop from back (moves out of slot)
    Obj pop_back() {
        // boundary check first
        assert(count_ > 0 && "RingBuffer underflow on popBack");
        head = (head - 1 + Length) % Length;
        Obj obj = std::move(objects[head]);
        --count_;
        return obj;
    }

    // Element access by index (0 = oldest)
    Obj& operator[](size_t i) {
        assert(i < (size_t)count_ && "RingBuffer index out of range");
        return objects[(tail + i) % Length];
    }

    const Obj& operator[](size_t i) const {
        assert(i < (size_t)count_ && "RingBuffer index out of range");
        return objects[(tail + i) % Length];
    }

    // Front / back access
    Obj& front()             { return objects[tail]; }
    const Obj& front() const { return objects[tail]; }
    Obj& back()              { return objects[(head - 1 + Length) % Length]; }
    const Obj& back() const  { return objects[(head - 1 + Length) % Length]; }

    // Size queries
    size_t size()     const { return count_; }
    size_t capacity() const { return Length; }
    bool   empty()    const { return count_ == 0; }
    bool   full()     const { return count_ == (int)Length; }

    // true when there is room to push without overwriting
    bool available() const { return count_ < (int)Length; }

    // Clear all elements
    void clear() {
        head = 0;
        tail = 0;
        count_ = 0;
    }
};

