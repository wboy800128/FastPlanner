// Minimal stub of matplotlibcpp to allow building without Python/Matplotlib
#ifndef MATPLOTLIBCPP_STUB_H
#define MATPLOTLIBCPP_STUB_H

#include <vector>
#include <map>
#include <string>

namespace matplotlibcpp {

inline void plot(const std::vector<double>& x, const std::vector<double>& y, const std::map<std::string, std::string>& /*kwargs*/ = {}) {}

inline void scatter_colored(const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& /*color*/, double /*s*/, const std::map<std::string, std::string>& /*kwargs*/ = {}) {}

inline void legend() {}
inline void show() {}

} // namespace matplotlibcpp

#endif // MATPLOTLIBCPP_STUB_H
