#include <cstdio>
#include <cstdlib>
#include "pruned_highway_labeling.h"

using namespace std;

int main(int argc, char *argv[]) {
    if (argc != 2) {
        fprintf(stderr, "Usage: query LABEL\n");
        return 0;
    }
    
    PrunedHighwayLabeling phl;
    phl.LoadLabel(argv[1]);
    phl.Statistics();
    
    for (int v, w; scanf("%d %d", &v, &w) != EOF; ) {
        printf("%d\n", phl.Query(v, w));
    }
    
    return 0;
}
