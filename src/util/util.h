#ifndef GRAPHSFM_UTIL_UTIL_H
#define GRAPHSFM_UTIL_UTIL_H

#include "util/map_util.h"

namespace DAGSfM {
typedef unsigned char uchar;

// A macro to disallow the copy constructor and operator= functions
// This should be used in the private: declarations for a class
#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
  TypeName(const TypeName&);               \
  void operator=(const TypeName&)

// Determines the array size an array a
#define GRAPHSFM_ARRAYSIZE(a)   \
  ((sizeof(a) / sizeof(*(a))) / \
   static_cast<size_t>(!(sizeof(a) % sizeof(*(a)))))

// Deletes all pointers in a container.
template <class ForwardIterator>
void STLDeleteContainerPointers(ForwardIterator begin, ForwardIterator end) {
  while (begin != end) {
    ForwardIterator temp = begin;
    ++begin;
    delete *temp;
  }
}

// Deletes all pointers in an STL container (anything that has a begin() and
// end() function)
template <class T>
void STLDeleteElements(T* container) {
  if (!container) return;
  STLDeleteContainerPointers(container->begin(), container->end());
  container->clear();
}

// Find the intersection of two (unordered) containers. This replicates the
// functionality of std::set_intersection for unordered containers that cannot
// be sorted.
template <typename InputContainer1, typename InputContainer2,
          typename OutputContainer = InputContainer1>
void ContainerIntersection(const InputContainer1& in1,
                           const InputContainer2& in2, OutputContainer* out) {
  // Always iterate over the smaller container.
  if (in2.size() < in1.size()) {
    return ContainerIntersection(in2, in1, out);
  }

  // Loop over all elements and add common elements to the output container.
  for (const auto& entry : in1) {
    if (ContainsKey(in2, entry)) {
      out->insert(entry);
    }
  }
}

}  // namespace DAGSfM

#endif  // GRAPHSFM_UTIL_UTIL_H
