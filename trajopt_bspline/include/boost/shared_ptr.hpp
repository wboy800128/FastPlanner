// Minimal shim to satisfy generated headers that include <boost/shared_ptr.hpp>
#ifndef BOOST_SHARED_PTR_SHIM_H
#define BOOST_SHARED_PTR_SHIM_H

#include <memory>

namespace boost {

template <typename T>
using shared_ptr = std::shared_ptr<T>;

template <typename T, typename... Args>
inline shared_ptr<T> make_shared(Args&&... args) {
    return std::make_shared<T>(std::forward<Args>(args)...);
}

} // namespace boost

#endif // BOOST_SHARED_PTR_SHIM_H
