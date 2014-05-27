#include <cstdio>
#include <cstdlib>
#include "pruned_highway_labeling.h"

using namespace std;

int main(int argc, char *argv[]) {
    if (argc != 3) {
        fprintf(stderr, "Usage: construct GRAPH LABEL\n");
        return 0;
    }
    
    PrunedHighwayLabeling phl;
    phl.ConstructLabel(argv[1]);
    phl.Statistics();
    phl.StoreLabel(argv[2]);
    
    return 0;
}
