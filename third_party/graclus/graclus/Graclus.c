// extern "C"
// {
//   //#include <metis.h>
//   //#include "io.h"
// }
#include "Graclus.h"


int boundary_points = 0;
int spectral_initialization = 0;
int cutType = 0; //cut type, default is normalized cut
int memory_saving = 0; // forbid using local search or empty cluster removing

/*************************************************************************
* multi-level weighted kernel k-means main function
**************************************************************************/
Graclus normalizedCut(char* filename, int nparts)
{
  Graclus ncData;
  int options[11];
  idxtype *part;  // cluster result stored in array part
  float rubvec[MAXNCON], lbvec[MAXNCON];
  GraphType graph;
  int numflag = 0, wgtflag = 0, edgecut, chain_length = 0;
  int no_args = 1, levels = 0;


  no_args = 0;
  
  if (nparts < 2) 
  {
    printf("The number of partitions should be greater than 1!\n");
    exit(0);
  }

  ReadGraph(&graph, filename, &wgtflag);
  if (graph.nvtxs <= 0) 
  {
    puts("Empty graph. Nothing to do.\n");
    exit(0);
  }

	levels = amax((graph.nvtxs)/(40*log2_metis(nparts)), 20*(nparts));
  
  // if(graph.ncon > 1)
  //   printf("  Balancing Constraints: %d\n", graph.ncon);

  part = idxmalloc(graph.nvtxs, "main: part");
  options[0] = 0;

  // printf("#Clusters: %d\n", nparts);
  if (graph.ncon == 1) 
  {
    MLKKM_PartGraphKway(&graph.nvtxs, graph.xadj, graph.adjncy, graph.vwgt, graph.adjwgt, 
			  &wgtflag, &numflag, &nparts, &chain_length, options, &edgecut, part, levels);
  }
  else 
  {
    for (int i = 0; i < graph.ncon; i++)
      rubvec[i] = HORIZONTAL_IMBALANCE;
  }

  ComputePartitionBalance(&graph, nparts, part, lbvec);
  ComputeNCut(&graph, part, nparts);
  
  //for(int i = 0; i < graph.nvtxs; i++) printf("%d\n", part[i]);
  //int clusterNum = graph.nvtxs;

  ncData.part = part;
  ncData.clusterNum = graph.nvtxs;

  //GKfree((void **) &graph.xadj, (void **) &graph.adjncy, (void **) &graph.vwgt, (void **) &graph.adjwgt, (void **) &part, LTERM);
  GKfree((void **) &graph.xadj, (void **) &graph.adjncy, (void **) &graph.vwgt, (void **) &graph.adjwgt, LTERM);  

  return ncData;
}

/*************************************************************************
* multi-level weighted kernel k-means main function
**************************************************************************/
// int main(int argc, char ** agrv)
// {
//   // int* clusterNum = (int*)malloc(sizeof(int*));
//   char filename[256];
//   int nparts = 1;
//   puts("Please input filename and cluster number: ");
//   scanf("%s%d", filename, &nparts);
//   Graclus ncdata = normalizedCut(filename, nparts);
//   for(int i = 0; i < ncdata.clusterNum; i++) printf("%d \n", ncdata.part[i]);
//   return 0;
// }  


