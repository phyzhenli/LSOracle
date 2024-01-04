/* LSOracle: A learning based Oracle for Logic Synthesis

 * MIT License
 * Copyright 2019 Laboratory for Nano Integrated Systems (LNIS)
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#pragma once

#include <algorithm>
#include <cstdint>
#include <unordered_map>
#include <vector>
#include <set>
#include <cassert>
#include <queue>
#include <mockturtle/mockturtle.hpp>
#include <kitty/kitty.hpp>
#include <libkahypar.h>

#include "algorithms/partitioning/partition_view.hpp"
#include "algorithms/partitioning/structure_partition.hpp"
#include "algorithms/partitioning/hyperg.hpp"
#include "kahypar_config.hpp"
#include "utility.hpp"

#include "dagP.h"

namespace oracle
{
template <typename network>
class kahypar_partitioner
{
    using Ntk = mockturtle::names_view<network>;
public:
    kahypar_partitioner(Ntk &ntk, int part_num, std::string p_strategy, std::string config_direc = "",
                      kahypar_hypernode_weight_t *hypernode_weights = nullptr,
                        kahypar_hyperedge_weight_t *hyperedge_weights = nullptr,
                        double imbalance = 0.9): ntk(ntk), part_num(part_num), node_partition(ntk)
    {
        // mockturtle::incomplete_node_map<kahypar_hyperedge_id_t, Ntk> initial_partitions;
        static_assert(mockturtle::is_network_type_v<Ntk>, "Ntk is not a network type");
        static_assert(mockturtle::has_set_visited_v<Ntk>,
                      "Ntk does not implement the set_visited method");
        static_assert(mockturtle::has_visited_v<Ntk>,
                      "Ntk does not implement the visited method");
        static_assert(mockturtle::has_get_node_v<Ntk>,
                      "Ntk does not implement the get_node method");
        static_assert(mockturtle::has_get_constant_v<Ntk>,
                      "Ntk does not implement the get_constant method");
        static_assert(mockturtle::has_is_constant_v<Ntk>,
                      "Ntk does not implement the is_constant method");
        static_assert(mockturtle::has_make_signal_v<Ntk>,
                      "Ntk does not implement the make_signal method");


        std::cout << "num_partitions: " << part_num << "\n";

        if (part_num == 1) {
            ntk.foreach_node([&](auto n) { node_partition[n] = 0; });
            return;
        }

        /******************
        Generate HyperGraph
        ******************/
        hypergraph<Ntk> t(ntk);
        if (p_strategy == "kahypar" or p_strategy == "dagP")
            t.get_hypergraph(ntk);
        if (p_strategy == "mffc_kahypar" or p_strategy == "mffc_dagP")
            t.get_mffc_hypergraph();

        // t.dump();
        if (p_strategy == "dagP") {
            int nbParts = part_num;
            t.write_idot();
            MLGP_option opt;
            dgraph G;
            dagP_init_parameters (&opt, 2);
            dagP_init_filename(&opt, "aig.dot");
            dagP_opt_reallocUBLB (&opt, nbParts);
            opt.runs = 5;
            opt.use_binary_input = 0;
            // fix error: In topsortPart, not every nodes are sorted: to = 0, nbpart = 2
            // see https://github.com/GT-TDAlab/dagP/issues/5
            opt.conpar = 0;
            opt.inipart = 11;
            dagP_read_graph ("aig.dot", &G, &opt);
            idxType *parts = (idxType*) calloc((G.nVrtx+1), sizeof(idxType));
            if (parts == NULL)
                printf("Could not allocate `parts` array.\n");
            ecType x = dagP_partition_from_dgraph(&G, &opt, parts);

            for(idxType i=1; i<= G.nVrtx; ++i){
                node_partition[i-1] = parts[i];
            }

            // printf ("edge cut: %d\n", (int) x);
            free(parts);
            dagP_free_option(&opt);
            dagP_free_graph(&G);
            return;
        }

        if (p_strategy == "kahypar" or p_strategy == "mffc_kahypar") {
            uint32_t kahyp_num_hyperedges = 0;
            uint32_t kahyp_num_vertices = 0;
            uint32_t kahyp_num_indeces_hyper = 0;
            unsigned long kahyp_num_sets = 0;
            std::vector<uint32_t> kahypar_connections;
            std::vector<unsigned long> kahyp_set_indeces;

            t.return_hyperedges(kahypar_connections);
            kahyp_num_hyperedges = t.get_num_edges();
            if (p_strategy == "kahypar") kahyp_num_vertices = t.get_num_vertices();
            if (p_strategy == "mffc_kahypar") kahyp_num_vertices = t.get_num_edges();
            kahyp_num_indeces_hyper = t.get_num_indeces();
            kahyp_num_sets = t.get_num_sets();
            t.get_indeces(kahyp_set_indeces);

            /******************
            Partition with kahypar
            ******************/
            //configures kahypar
            kahypar_context_t* context = kahypar_context_new();

            std::cout << "Using config file " << config_direc << std::endl;
            kahypar_configure_context_from_file(context, config_direc.c_str());

            //set number of hyperedges and vertices. These variables are defined by the hyperG command
            const kahypar_hyperedge_id_t num_hyperedges = kahyp_num_hyperedges;
            const kahypar_hypernode_id_t num_vertices = kahyp_num_vertices;

            //set edges with different weights
            if (p_strategy == "kahypar") {
                if (hyperedge_weights == nullptr) {
                    hyperedge_weights = new kahypar_hyperedge_weight_t[kahyp_num_hyperedges];
                    std::vector<std::vector<uint32_t>> hyperedges = t.get_hyperedges();
                    mockturtle::depth_view depth_ntk{ntk};
                    for ( int i = 0; i < hyperedges.size(); i++ ) {
                        if ( depth_ntk.is_on_critical_path(hyperedges[i][0]) )
                            hyperedge_weights[i] = 1000;
                        else
                            hyperedge_weights[i] = hyperedges[i].size();
                    }
                }
            }

            if (p_strategy == "mffc_kahypar") {
                hypernode_weights = new kahypar_hypernode_weight_t [kahyp_num_vertices];
                for (uint32_t i = 0; i < kahyp_num_vertices; i++) {
                    mockturtle::mffc_view mffc{ ntk, t.get_ntk_node(i) };
                    hypernode_weights[i] = mffc.size()-mffc.num_pis()-1;
                    // cout << hypernode_weights[i] << " ";
                }
            }

            //vector with indeces where each set starts
            std::unique_ptr<size_t[]> hyperedge_indices = std::make_unique<size_t[]>
                (kahyp_num_sets + 1);

            for (int j = 0; j < kahyp_num_sets + 1; j++) {
                hyperedge_indices[j] = kahyp_set_indeces[j];
            }

            std::unique_ptr<kahypar_hyperedge_id_t[]> hyperedges =
                std::make_unique<kahypar_hyperedge_id_t[]>(kahyp_num_indeces_hyper);

            for (int i = 0; i < kahyp_num_indeces_hyper; i++) {
                hyperedges[i] = kahypar_connections[i];
            }

            const kahypar_partition_id_t k = part_num;

            kahypar_hyperedge_weight_t objective = 0;

            std::vector<kahypar_partition_id_t> partition(num_vertices, -1);
            kahypar_hypergraph_t *hypergraph = kahypar_create_hypergraph(k,
                                                                        num_vertices,
                                                                        num_hyperedges,
                                                                        hyperedge_indices.get(),
                                                                        hyperedges.get(),
                                                                        hyperedge_weights,
                                                                        hypernode_weights);
            kahypar_partition_hypergraph(hypergraph, k, imbalance, &objective, context,
                                        partition.data());
            if (p_strategy == "kahypar") {
                for (int i = 0; i < num_vertices; i++) {
                    node_partition[i] = partition[i];
                }
            }

            // if (p_strategy == "mffc_kahypar") {
            //     for (uint32_t i = 0; i < num_vertices; i++) {
            //         cout << partition[i] << " ";
            //     }
            // }
            // cout << endl;

            if (p_strategy == "mffc_kahypar") {
                for (uint32_t i = 0; i < num_vertices; i++) {
                    if (i == 0)
                        node_partition[i] = partition[i];
                    else {
                        // cout << "ntk node: " << t.get_ntk_node(i) << ", ";
                        mockturtle::mffc_view mffc{ ntk, t.get_ntk_node(i) };
                        mffc.foreach_gate([&]( auto const& n, auto ii ) {
                            (void)ii;
                            node_partition[n] = partition[i];
                            // cout << "node " << n << " is part " << partition[i] << ", ";
                        });
                    }
                }
            }

            kahypar_context_free(context);
        }
    }

    partition_manager_junior<network> partition_manager() {
        return partition_manager_junior<network>(ntk, node_partition, part_num);
    }

private:
    int part_num = 0;
    mockturtle::node_map<int, mockturtle::names_view<network>> node_partition;
    mockturtle::names_view<network> ntk;
};
};
