#ifndef _SRC_MAPREDUCE_MAPPER_H_
#define _SRC_MAPREDUCE_MAPPER_H_

namespace GraphSfM {

class Mapper {
 public:
  virtual void Map(const void* input) = 0;
};

}  // namespace GraphSfM

#endif