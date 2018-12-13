#ifndef GRACLUS_H
#define GRACLUS_H


#include <metis.h>

// class Graclus
// {
//     static idxtype* normalizedCut(char* filename, int nparts, int& clusterNum);    
// };
typedef struct graclus
{
  idxtype* part;
  int clusterNum;
}Graclus;
Graclus normalizedCut(char* filename, int nparts);

#endif