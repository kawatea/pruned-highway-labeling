Pruned Highway Labeling
=======================

Pruned Highway Labeling is a shortest-path distance querying algorithm for road networks.

Usage
-----

    $ make
    $ bin/construct graph_file label_file
    $ bin/query label_file

* Execute `make` to build programs.
* Execute `construct` to construct labels for a graph.
* Execute `query` to query distance between two vertices.
* In a graph file, each line should contain two vertices, travel time and geometrical length (see `sample_graph.tsv`).
* Vertices should be described by integers starting from zero.

Please see `pruned_highway_labeling.h` and `benchmark.cpp` for detailed information.

References
----------
* Takuya Akiba, Yoichi Iwata, Ken-ichi Kawarabayashi, and Yuki Kawata, **[Fast Shortest-path Distance Queries on Road Networks by Pruned Highway Labeling](http://epubs.siam.org/doi/pdf/10.1137/1.9781611973198.14)**. In ALENEX 2014.