#include <cstdio>
#include <cstdlib>
#include <vector>
#include <sys/time.h>
#include "pruned_highway_labeling.h"

using namespace std;

const int NumQueries = 1000000;

double GetTime(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec + tv.tv_usec * 1e-6;
}

int main(int argc, char *argv[]) {
    if (argc != 2) {
        fprintf(stderr, "Usage: benchmark GRAPH\n");
        return 0;
    }
    
    PrunedHighwayLabeling phl;
    phl.ConstructLabel(argv[1]);
    phl.Statistics();
    
    int V = phl.NumVertices();
    vector <int> v(NumQueries);
    vector <int> w(NumQueries);
    for (int i = 0; i < NumQueries; i++) {
        v[i] = rand() % V;
        w[i] = rand() % V;
    }
    
    double start_time = GetTime();
    for (int i = 0; i < NumQueries; i++) phl.Query(v[i], w[i]);
    double end_time = GetTime();
    printf("Query Time : %lf usec\n", (end_time - start_time) * 1e6 / NumQueries);
    
    return 0;
}
