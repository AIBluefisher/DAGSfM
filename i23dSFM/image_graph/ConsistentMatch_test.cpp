#include "ConsistentMatchGraph.hpp"

#include <iostream>
using namespace std;


int main(int argc, char* argv[])
{
    ImageGraph imageGraph;
    // TODO: Construct image graph

    // TODO: Compute the motion map of consistent match graph

    ConsitentMatchGraph consistentMG;

    size_t rejectThreshold = 50;    // not determined yet
    size_t inlierThreshold = 40;    // by default
    consistentMG.OnlineMST(g, rejectThreshold, inlierThreshold);

    double discreThreshold = 0.5;   // not determined yet
    consistentMG.StrongTripletExpansion(discreThreshold);

    size_t communityScale = 30;     // not determined yet
    double loopDiscreThreshold = 0.5;// not determined yet
    consistentMG.ComponentMerging(communityScale, loopDiscreThreshold);
}