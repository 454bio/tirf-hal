#pragma once

// Utilities to make it easier to write idiomatic C++ when calling C libraries.

#include <functional>
#include <string>
#include <system_error>

// Converts rc-style errors into exceptions.
inline void CHECK_RC(int rc, const std::string &hint)
{
    if (rc < 0)
    {
        throw std::system_error(std::make_error_code(static_cast<std::errc>(rc)), hint);
    }
}

// Ensures that deleter_ is called on any scope exit, including normal exit, break/continue, gotos, exceptions, etc.
// Useful to clean up C handles no matter what, especially when this object is defined right after the handle is initially acquired.
class CleanupHelper
{
public:
    CleanupHelper(const std::function<void()> &deleter) : deleter_(deleter) {}

    // It is highly unlikely that you actually want this to run more than once.
    CleanupHelper(const CleanupHelper &) = delete;
    CleanupHelper &operator=(const CleanupHelper &) = delete;

    // ... but moving should be okay.
    CleanupHelper(CleanupHelper &&) = default;
    CleanupHelper &operator=(CleanupHelper &&) = default;

    ~CleanupHelper()
    {
        if (deleter_)
        {
            deleter_();
        }
    }

private:
    std::function<void()> deleter_;
};
