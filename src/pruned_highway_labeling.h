#ifndef __PRUNED_HIGHWAY_LABELING_H__
#define __PRUNED_HIGHWAY_LABELING_H__

#include <cstdio>
#include <cstdlib>
#include <vector>
#include <queue>
#include <algorithm>
#include <malloc.h>
#include <xmmintrin.h>
#include <sys/time.h>

class PrunedHighwayLabeling {
    public:
    
    PrunedHighwayLabeling() : V(0), label(NULL), load_time(0), construct_time(0) {}
    ~PrunedHighwayLabeling() { Free(); };
    
    void ConstructLabel(const char *file);
    void LoadLabel(const char *file);
    void StoreLabel(const char *file);
    inline int Query(int v, int w);
    
    void Free(void);
    
    int NumVertices(void) { return V; }
    void Statistics(void);
    
    private:
    
    static const int LEVEL = 4;
    static const int INF = 1000000000;
    static const unsigned GUARD = 0xFFFFFFFFU;
    static const unsigned PATH_MASK = 0xFFFFFC00U;
    static const unsigned NUM_MASK = 0x000003FFU;
    
    struct road {
        int from;
        int to;
        int time;
        int dist;
    };
    
    struct edge {
        int to;
        int time;
        int level;
    };
    
    struct label_t {
        int time;
        unsigned *path;
        int *cost;
    } __attribute__((aligned(64)));
    
    int V;
    label_t *label;
    std::vector <int> contract;
    double load_time, construct_time;
    
    double GetTime(void) {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        return tv.tv_sec + tv.tv_usec * 1e-6;
    }
    
    int GetLevel(int time, int dist) {
        if (time == 0) return 0;
        if (dist > time * 0.9) return 0;
        if (dist > time * 0.7) return 1;
        if (dist > time * 0.5) return 2;
        return 3;
    }
    
    inline void ChangeMin(int &now, int next) { if (next < now) now = next; }
    
    inline bool Prune(std::pair<std::vector <unsigned>, std::vector <std::pair<int, int> > > &lv, std::pair<std::vector <unsigned>, std::vector <std::pair<int, int> > > &lw, int time);
};

inline bool PrunedHighwayLabeling::Prune(std::pair<std::vector <unsigned>, std::vector <std::pair<int, int> > > &lv, std::pair<std::vector <unsigned>, std::vector <std::pair<int, int> > > &lw, int time) {
    unsigned vfnum = 0;
    unsigned vsnum = 0;
    unsigned wfnum = 0;
    unsigned wsnum = 0;
    while (true) {
        if (vfnum == lv.first.size() || wfnum == lw.first.size()) return false;
        if ((lv.first[vfnum] & PATH_MASK) == (lw.first[wfnum] & PATH_MASK)) {
            for (unsigned i = 0, j = 0; ; ) {
                if (lv.second[vsnum + i].first == lw.second[wsnum + j].first) {
                    if (lv.second[vsnum + i].second + lw.second[wsnum + j].second <= time) return true;
                    i++;
                    j++;
                    if (i == (lv.first[vfnum] & NUM_MASK) || j == (lw.first[wfnum] & NUM_MASK)) break;
                } else if (lv.second[vsnum + i].first < lw.second[wsnum + j].first) {
                    if (lv.second[vsnum + i].second + lw.second[wsnum + j].second + lw.second[wsnum + j].first - lv.second[vsnum + i].first <= time) return true;
                    i++;
                    if (i == (lv.first[vfnum] & NUM_MASK)) break;
                } else {
                    if (lv.second[vsnum + i].second + lw.second[wsnum + j].second + lv.second[vsnum + i].first - lw.second[wsnum + j].first <= time) return true;
                    j++;
                    if (j == (lw.first[wfnum] & NUM_MASK)) break;
                }
            }
            vsnum += lv.first[vfnum] & NUM_MASK;
            vfnum++;
            wsnum += lw.first[wfnum] & NUM_MASK;
            wfnum++;
        } else if ((lv.first[vfnum] & PATH_MASK) < (lw.first[wfnum] & PATH_MASK)) {
            vsnum += lv.first[vfnum] & NUM_MASK;
            vfnum++;
        } else {
            wsnum += lw.first[wfnum] & NUM_MASK;
            wfnum++;
        }
    }
}

void PrunedHighwayLabeling::ConstructLabel(const char *file) {
    Free();
    
    // read graph
    // each line should contain two vertices, travel time and geometrical length
    // treat a graph as an undirected graph
    load_time = -GetTime();
    std::vector <std::vector <edge> > graph;
    {
        std::vector <road> edges;
        FILE *in = fopen(file, "r");
        if (in == NULL) return;
        for (int from, to, time, dist; fscanf(in, "%d %d %d %d", &from, &to, &time, &dist) != EOF; ) {
            V = std::max(V, from + 1);
            V = std::max(V, to + 1);
            edges.push_back((road){from, to, time, dist});
        }
        fclose(in);
        
        graph.resize(V);
        for (size_t i = 0; i < edges.size(); i++) {
            int from = edges[i].from;
            int to = edges[i].to;
            int time = edges[i].time;
            int dist = edges[i].dist;
            int level = GetLevel(time, dist);
            graph[from].push_back((edge){to, time, level});
            graph[to].push_back((edge){from, time, level});
        }
    }
    load_time += GetTime();
    
    // allocate memory for labels
    label = (label_t *)memalign(64, sizeof(label_t) * V);
    if (label == NULL) {
        V = 0;
        return;
    }
    for (int v = 0; v < V; v++) {
        label[v].time = 0;
        label[v].path = NULL;
        label[v].cost = NULL;
    }
    
    // contract degree one vertices and order vertices
    construct_time = -GetTime();
    std::vector <std::vector <int> > order(LEVEL);
    {
        // contract degree one vertices
        contract.resize(V, -1);
        for (int v = 0; v < V; v++) {
            if (graph[v].size() == 1) contract[v] = graph[v][0].to;
        }
        for (int v = 0; v < V; v++) {
            if (contract[v] != -1) {
                int to = graph[v][0].to;
                for (std::vector <edge>::iterator it = graph[to].begin(); ; it++) {
                    if (it->to == v) {
                        graph[to].erase(it);
                        break;
                    }
                }
                label[v].time = graph[v][0].time;
                std::vector <edge>().swap(graph[v]);
            }
        }
        
        // order vertices
        std::vector <int> vertex_level(V, LEVEL);
        std::vector <std::vector <std::pair<double, int> > > rank(LEVEL);
        for (int v = 0; v < V; v++) {
            if (contract[v] == -1) {
                for (size_t i = 0; i < graph[v].size(); i++) {
                    vertex_level[v] = std::min(vertex_level[v], graph[v][i].level);
                }
                for (int i = vertex_level[v]; i < LEVEL; i++) rank[i].push_back(std::make_pair((double)rand() / RAND_MAX, v));
            }
        }
        for (int i = 0; i < LEVEL; i++) std::sort(rank[i].begin(), rank[i].end());
        for (int i = 0; i < LEVEL; i++) {
            if (i < LEVEL - 1) {
                for (size_t j = 0; j * 1000 < rank[i].size(); j++) order[i].push_back(rank[i][j].second);
            } else {
                for (size_t j = 0; j < rank[i].size(); j++) order[i].push_back(rank[i][j].second);
            }
        }
    }
    
    // pruned highway labeling
    // select path and add labels for the path
    {
        unsigned path_num = 0;
        std::vector <bool> used(V, false);
        std::vector <int> cost(V, INF);
        std::vector <int> dist(V);
        std::vector <int> parent(V, -1);
        std::vector <int> child(V, 0);
        std::vector <int> check(V);
        std::vector <int> visit(V);
        std::vector <long long> last(V, -1);
        std::vector <std::pair<std::vector <unsigned>, std::vector <std::pair<int, int> > > > tmp_label(V);
        std::priority_queue <std::pair<int, int>, std::vector <std::pair<int, int> >, std::greater <std::pair<int, int> > > path_que;
        std::priority_queue <std::pair<int, std::pair<int, int> >, std::vector <std::pair<int, std::pair<int, int> > >, std::greater <std::pair<int, std::pair<int, int> > > > label_que;
        
        for (int i = 0; i < LEVEL; i++) {
            for (size_t j = 0; j < order[i].size(); j++) {
                // select path
                int v = order[i][j];
                if (used[v]) continue;
                int check_num = 0;
                int visit_num = 0;
                cost[v] = 0;
                check[check_num++] = v;
                path_que.push(std::make_pair(0, v));
                while (!path_que.empty()) {
                    int time = path_que.top().first;
                    int now = path_que.top().second;
                    path_que.pop();
                    if (cost[now] < time) continue;
                    visit[visit_num++] = now;
                    for (size_t k = 0; k < graph[now].size(); k++) {
                        int w = graph[now][k].to;
                        if (used[w]) continue;
                        if (cost[w] > time + graph[now][k].time) {
                            if (cost[w] == INF) check[check_num++] = w;
                            cost[w] = time + graph[now][k].time;
                            parent[w] = now;
                            if (graph[now][k].level <= i) {
                                child[w] = 1;
                            } else {
                                child[w] = 0;
                            }
                            if (Prune(tmp_label[v], tmp_label[w], cost[w])) continue;
                            path_que.push(std::make_pair(cost[w], w));
                        }
                    }
                }
                for (int k = visit_num - 1; k > 0; k--) child[parent[visit[k]]] += child[visit[k]];
                int end = v;
                while (true) {
                    int next = -1, max_child = 0;
                    for (size_t k = 0; k < graph[end].size(); k++) {
                        int w = graph[end][k].to;
                        if (parent[w] == end && child[w] > max_child) {
                            next = w;
                            max_child = child[w];
                        }
                    }
                    if (next == -1) break;
                    end = next;
                }
                
                //add labels
                if (end != v) {
                    int child_num = 0;
                    while (end != -1) {
                        if (i == LEVEL - 1 || child_num * 20 <= child[end] * 19) {
                            used[end] = true;
                            label_que.push(std::make_pair(cost[end], std::make_pair(end, end)));
                        }
                        child_num = child[end];
                        end = parent[end];
                    }
                    long long num = (long long)path_num << 22;
                    while (!label_que.empty()) {
                        int time = label_que.top().first;
                        int now = label_que.top().second.first;
                        int start = label_que.top().second.second;
                        label_que.pop();
                        if (last[now] >= num + cost[start]) continue;
                        if ((last[now] >> 32 << 32) == num) {
                            if (dist[now] + cost[start] * 2 <= time) continue;
                            if (Prune(tmp_label[start], tmp_label[now], time - cost[start])) continue;
                        } else {
                            if (!used[now] && Prune(tmp_label[start], tmp_label[now], time - cost[start])) continue;
                        }
                        last[now] = num + cost[start];
                        dist[now] = time - cost[start] * 2;
                        if (tmp_label[now].first.size() == 0 || (tmp_label[now].first.back() & PATH_MASK) != path_num) {
                            tmp_label[now].first.push_back(path_num + 1);
                        } else {
                            tmp_label[now].first.back()++;
                        }
                        tmp_label[now].second.push_back(std::make_pair(cost[start], time - cost[start]));
                        for (size_t k = 0; k < graph[now].size(); k++) {
                            int w = graph[now][k].to;
                            if (!used[w] && last[w] < num + cost[start]) label_que.push(std::make_pair(time + graph[now][k].time, std::make_pair(w, start)));
                        }
                    }
                    path_num += 1 << 10;
                }
                
                for (int k = 0; k < check_num; k++) {
                    cost[check[k]] = INF;
                    parent[check[k]] = -1;
                    child[check[k]] = 0;
                }
            }
        }
        
        // convert labels
        for (int v = 0; v < V; v++) {
            if (contract[v] == -1) {
                label[v].path = (unsigned *)memalign(64, sizeof(unsigned) * (tmp_label[v].first.size() + 1));
                label[v].cost = (int *)memalign(64, sizeof(int) * tmp_label[v].second.size() * 2);
                if (label[v].path == NULL || label[v].cost == NULL) {
                    Free();
                    return;
                }
                for (size_t i = 0; i < tmp_label[v].first.size(); i++) label[v].path[i] = (tmp_label[v].first[i] & PATH_MASK) + (tmp_label[v].first[i] & NUM_MASK) * 2;
                label[v].path[tmp_label[v].first.size()] = GUARD;
                for (size_t i = 0; i < tmp_label[v].second.size(); i++) {
                    label[v].cost[i * 2] = tmp_label[v].second[i].first;
                    label[v].cost[i * 2 + 1] = tmp_label[v].second[i].second;
                }
                std::vector <unsigned>().swap(tmp_label[v].first);
                std::vector <std::pair<int, int> >().swap(tmp_label[v].second);
            }
        }
        for (int v = 0; v < V; v++) {
            if (contract[v] != -1) {
                label[v].path = label[contract[v]].path;
                label[v].cost = label[contract[v]].cost;
            }
        }
    }
    construct_time += GetTime();
}

void PrunedHighwayLabeling::LoadLabel(const char *file) {
    Free();
    
    FILE *in = fopen(file, "rb");
    if (in == NULL) return;
    fread(&V, sizeof(int), 1, in);
    contract.resize(V);
    fread(&contract[0], sizeof(int), V, in);
    label = (label_t *)memalign(64, sizeof(label_t) * V);
    if (label == NULL) {
        V = 0;
        return;
    }
    for (int v = 0; v < V; v++) {
        label[v].time = 0;
        label[v].path = NULL;
        label[v].cost = NULL;
    }
    for (int v = 0; v < V; v++) {
        if (contract[v] != -1) {
            fread(&label[v].time, sizeof(int), 1, in);
        } else {
            int fnum, snum;
            fread(&fnum, sizeof(int), 1, in);
            fread(&snum, sizeof(int), 1, in);
            label[v].path = (unsigned *)memalign(64, sizeof(unsigned) * fnum);
            label[v].cost = (int *)memalign(64, sizeof(int) * snum);
            if (label[v].path == NULL || label[v].cost == NULL) {
                Free();
                return;
            }
            fread(label[v].path, sizeof(unsigned), fnum, in);
            fread(label[v].cost, sizeof(int), snum, in);
        }
    }
    for (int v = 0; v < V; v++) {
        if (contract[v] != -1) {
            label[v].path = label[contract[v]].path;
            label[v].cost = label[contract[v]].cost;
        }
    }
    fclose(in);
}

void PrunedHighwayLabeling::StoreLabel(const char *file) {
    FILE *out = fopen(file, "wb");
    if (out == NULL) return;
    fwrite(&V, sizeof(int), 1, out);
    fwrite(&contract[0], sizeof(int), V, out);
    for (int v = 0; v < V; v++) {
        if (contract[v] != -1) {
            fwrite(&label[v].time, sizeof(int), 1, out);
        } else {
            int fnum = 1, snum = 0;
            for (int i = 0; ; i++) {
                if (label[v].path[i] == GUARD) break;
                fnum++;
                snum += label[v].path[i] & NUM_MASK;
            }
            fwrite(&fnum, sizeof(int), 1, out);
            fwrite(&snum, sizeof(int), 1, out);
            fwrite(label[v].path, sizeof(unsigned), fnum, out);
            fwrite(label[v].cost, sizeof(int), snum, out);
        }
    }
    fclose(out);
}

inline int PrunedHighwayLabeling::Query(int v, int w) {
    if (v == w) return 0;
    if (contract[v] == w || contract[w] == v || (contract[v] != -1 && contract[v] == contract[w])) return label[v].time + label[w].time;
    int time = INF;
    unsigned *vf = label[v].path;
    int *vs = label[v].cost;
    unsigned *wf = label[w].path;
    int *ws = label[w].cost;
    
    _mm_prefetch(vf, _MM_HINT_T0);
    _mm_prefetch(vs, _MM_HINT_T0);
    _mm_prefetch(wf, _MM_HINT_T0);
    _mm_prefetch(ws, _MM_HINT_T0);
    
    while (true) {
        if ((*vf & PATH_MASK) == (*wf & PATH_MASK)) {
            if (*vf == GUARD) return time + label[v].time + label[w].time;
            for (unsigned i = 0, j = 0; ; ) {
                if (*(vs + i) == *(ws + j)) {
                    ChangeMin(time, *(vs + i + 1) + *(ws + j + 1));
                    i += 2;
                    j += 2;
                    if (i == (*vf & NUM_MASK) || j == (*wf & NUM_MASK)) break;
                } else if (*(vs + i) < *(ws + j)) {
                    ChangeMin(time, *(vs + i + 1) + *(ws + j + 1) + *(ws + j) - *(vs + i));
                    i += 2;
                    if (i == (*vf & NUM_MASK)) break;
                } else {
                    ChangeMin(time, *(vs + i + 1) + *(ws + j + 1) + *(vs + i) - *(ws + j));
                    j += 2;
                    if (j == (*wf & NUM_MASK)) break;
                }
            }
            vs += *vf & NUM_MASK;
            vf++;
            ws += *wf & NUM_MASK;
            wf++;
        } else if ((*vf & PATH_MASK) < (*wf & PATH_MASK)) {
            if (*wf == GUARD) return time + label[v].time + label[w].time;
            vs += *vf & NUM_MASK;
            vf++;
        } else {
            if (*vf == GUARD) return time + label[v].time + label[w].time;
            ws += *wf & NUM_MASK;
            wf++;
        }
    }
}

void PrunedHighwayLabeling::Free(void) {
    for (int v = 0; v < V; v++) {
        if (contract[v] == -1) {
            free(label[v].path);
            free(label[v].cost);
            label[v].path = NULL;
            label[v].cost = NULL;
        }
    }
    free(label);
    V = 0;
    label = NULL;
}

void PrunedHighwayLabeling::Statistics(void) {
    long long sum_label = 0, sum_memory = 0;
    
    for (int v = 0; v < V; v++) {
        if (contract[v] != -1) continue;
        for (int i = 0; ; i++) {
            if (label[v].path[i] == GUARD) break;
            sum_label += (label[v].path[i] & NUM_MASK) / 2;
            sum_memory += sizeof(unsigned);
        }
        sum_memory += sizeof(unsigned);
    }
    sum_memory += sizeof(int) * sum_label * 2;
    
    printf("Load Time : %lf sec\n", load_time);
    printf("Construct Time : %lf sec\n", construct_time);
    printf("Average Label Size : %lld\n", sum_label / V);
    printf("Memory Size : %lld byte\n", sum_memory);
}

#endif
