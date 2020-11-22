#ifndef GRAPHSFM_UTIL_STRINGPRINTF_H_
#define GRAPHSFM_UTIL_STRINGPRINTF_H_

#include <cstdarg>
#include <string>

namespace DAGSfM {

#if (defined(__GNUC__) || defined(__clang__))
// Tell the compiler to do printf format string checking if the compiler
// supports it; see the 'format' attribute in
// <http://gcc.gnu.org/onlinedocs/gcc-4.3.0/gcc/Function-Attributes.html>.
//
// N.B.: As the GCC manual states, "[s]ince non-static C++ methods
// have an implicit 'this' argument, the arguments of such methods
// should be counted from two, not one."
#define GRAPHSFM_PRINTF_ATTRIBUTE(string_index, first_to_check) \
  __attribute__((__format__(__printf__, string_index, first_to_check)))
#define GRAPHSFM_SCANF_ATTRIBUTE(string_index, first_to_check) \
  __attribute__((__format__(__scanf__, string_index, first_to_check)))
#else
#define GRAPHSFM_PRINTF_ATTRIBUTE(string_index, first_to_check)
#endif

// Return a C++ string.
extern std::string StringPrintf(const char* format, ...)
    // Tell the compiler to do printf format string checking.
    GRAPHSFM_PRINTF_ATTRIBUTE(1, 2);

}  // namespace DAGSfM

#endif  // GRAPHSFM_UTIL_STRINGPRINTF_H_
