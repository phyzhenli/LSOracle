/* mockturtle: C++ logic network library
 * Copyright (C) 2018-2021  EPFL
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

/*!
  \file write_gblif.hpp
  \brief Write networks to gate blif format

  \author Zhen Li
*/

#pragma once

#include <array>
#include <fstream>
#include <iostream>
#include <string>

#include <lorina/verilog.hpp>
#include <fmt/format.h>

#include "../traits.hpp"
#include "../utils/node_map.hpp"
#include "../utils/string_utils.hpp"
#include "../views/binding_view.hpp"
#include "../views/topo_view.hpp"

namespace mockturtle
{

using namespace std::string_literals;

// namespace detail
// {

// template<class Ntk>
// std::vector<std::pair<bool, std::string>>
// format_fanin( Ntk const& ntk, node<Ntk> const& n, node_map<std::string, Ntk>& node_names )
// {
//   std::vector<std::pair<bool, std::string>> children;
//   ntk.foreach_fanin( n, [&]( auto const& f ) {
//       children.emplace_back( std::make_pair( ntk.is_complemented( f ), node_names[f] ) );
//     });
//   return children;
// }

// } // namespace detail

struct write_gblif_params
{
  std::string module_name = "top";
  std::vector<std::pair<std::string, uint32_t>> input_names;
  std::vector<std::pair<std::string, uint32_t>> output_names;
};

/*! \brief Writes network in gate blif format into output stream
 *
 * An overloaded variant exists that writes the network into a file.
 *
 * **Required network functions:**
 * - `num_pis`
 * - `num_pos`
 * - `foreach_pi`
 * - `foreach_node`
 * - `foreach_fanin`
 * - `get_node`
 * - `get_constant`
 * - `is_constant`
 * - `is_pi`
 * - `is_and`
 * - `is_or`
 * - `is_xor`
 * - `is_xor3`
 * - `is_maj`
 * - `node_to_index`
 *
 * \param ntk Network
 * \param os Output stream
 */
template<class Ntk>
void write_gblif( Ntk const& ntk, std::ostream& os, write_gblif_params const& ps = {} )
{
  static_assert( is_network_type_v<Ntk>, "Ntk is not a network type" );
  static_assert( has_fanin_size_v<Ntk>, "Ntk does not implement the fanin_size method" );
  static_assert( has_foreach_fanin_v<Ntk>, "Ntk does not implement the foreach_fanin method" );
  static_assert( has_foreach_pi_v<Ntk>, "Ntk does not implement the foreach_pi method" );
  static_assert( has_foreach_po_v<Ntk>, "Ntk does not implement the foreach_po method" );
  static_assert( has_is_constant_v<Ntk>, "Ntk does not implement the is_constant method" );
  static_assert( has_is_pi_v<Ntk>, "Ntk does not implement the is_pi method" );
  static_assert( has_get_node_v<Ntk>, "Ntk does not implement the get_node method" );
  static_assert( has_num_pis_v<Ntk>, "Ntk does not implement the num_pis method" );
  static_assert( has_num_pos_v<Ntk>, "Ntk does not implement the num_pos method" );
  static_assert( has_node_to_index_v<Ntk>, "Ntk does not implement the node_to_index method" );
  static_assert( has_node_function_v<Ntk>, "Ntk does not implement the node_function method" );

  assert( ntk.is_combinational() && "Network has to be combinational" );
  
  topo_view topo_ntk{ntk};

  lorina::verilog_writer writer( os );

  // xs for node_names
  std::vector<std::string> xs, inputs;
  if ( ps.input_names.empty() )
  {
    if constexpr ( has_has_name_v<Ntk> && has_get_name_v<Ntk> )
    {
      ntk.foreach_pi( [&]( auto const& i, uint32_t index ){
        if ( ntk.has_name( ntk.make_signal( i ) ) )
        {
          xs.emplace_back( ntk.get_name( ntk.make_signal( i ) ) );
        }
        else
        {
          xs.emplace_back( fmt::format( "x{}", index ) );
        }
      });
    }
    else
    {
      for ( auto i = 0u; i < ntk.num_pis(); ++i )
      {
        xs.emplace_back( fmt::format( "x{}", i ) );
      }
    }
    inputs = xs;
  }
  else
  {
    uint32_t ctr{0u};
    for ( auto const& [name, width] : ps.input_names )
    {
      inputs.emplace_back( name );
      ctr += width;
      for ( auto i = 0u; i < width; ++i )
      {
        xs.emplace_back( fmt::format( "{}[{}]", name, i ) );
      }
    }
    if ( ctr != ntk.num_pis() )
    {
      std::cerr << "[e] input names do not partition all inputs\n";
    }
  }

  std::vector<std::string> ys, outputs;
  if ( ps.output_names.empty() )
  {
    if constexpr ( has_has_output_name_v<Ntk> && has_get_output_name_v<Ntk> )
    {
      ntk.foreach_po( [&]( auto const& o, uint32_t index ){
        if ( ntk.has_output_name( index ) )
        {
          ys.emplace_back( ntk.get_output_name( index ) );
        }
        else
        {
          ys.emplace_back( fmt::format( "y{}", index ) );
        }
      });
    }
    else
    {
      for ( auto i = 0u; i < ntk.num_pos(); ++i )
      {
        ys.emplace_back( fmt::format( "y{}", i ) );
      }
    }
    outputs = ys;
  }
  else
  {
    uint32_t ctr{0u};
    for ( auto const& [name, width] : ps.output_names )
    {
      outputs.emplace_back( name );
      ctr += width;
      for ( auto i = 0u; i < width; ++i )
      {
        ys.emplace_back( fmt::format( "{}[{}]", name, i ) );
      }
    }
    if ( ctr != ntk.num_pos() )
    {
      std::cerr << "[e] output names do not partition all outputs\n";
    }
  }


  /* write model */
  if constexpr ( has_get_network_name_v<Ntk>) {
      if (ntk.get_network_name() == "") {
	  os << ".model top\n";
      } else {
	  os << ".model " << ntk.get_network_name() << "\n";
      }
  } else {
      os << ".model top\n";
  }

  /* write inputs */
  if ( topo_ntk.num_pis() > 0u )
  {
    os << ".inputs ";
    topo_ntk.foreach_ci( [&]( auto const& n, auto index )
    {
      if ( ( ( index + 1 ) <= topo_ntk.num_cis() - topo_ntk.num_latches() ) )
      {
        if constexpr ( has_has_name_v<Ntk> && has_get_name_v<Ntk> )
        {
          signal<Ntk> const s = topo_ntk.make_signal( topo_ntk.node_to_index( n ) );
          std::string const name = topo_ntk.has_name( s ) ? topo_ntk.get_name( s ) : fmt::format( "pi{}", topo_ntk.get_node( s ) );
          os << name << ' ';
        }
        else
        {
          os << fmt::format( "pi{} ", topo_ntk.node_to_index( n ) );
        }
      }
    } );
    os << "\n";
  }

  /* write outputs */
  if ( topo_ntk.num_pos() > 0u )
  {
    os << ".outputs ";
    topo_ntk.foreach_co( [&]( auto const& f, auto index )
    {
      (void)f;
      if( index < topo_ntk.num_cos() - topo_ntk.num_latches() )
      {
        if constexpr ( has_has_output_name_v<Ntk> && has_get_output_name_v<Ntk> )
        {
          std::string const output_name = topo_ntk.has_output_name( index ) ? topo_ntk.get_output_name( index ) : fmt::format( "po{}", index );
          os << output_name << ' ';
        }
        else
        {
          os << fmt::format( "po{} ", index );
        }
      }
    } );
    os << "\n";
  }

  /* write constants */
  os << ".gate ZERO   Y=new_n0\n";

  if ( topo_ntk.get_constant( false ) != topo_ntk.get_constant( true ) )
    os << ".gate ONE    Y=new_n1\n";






  /* write nodes */
  node_map<std::string, Ntk> node_names( ntk );
  // node_names[ntk.get_constant( false )] = "1'b0";
  node_names[ntk.get_constant( false )] = "new_n0";
  if ( ntk.get_node( ntk.get_constant( false ) ) != ntk.get_node( ntk.get_constant( true ) ) ) {
    // node_names[ntk.get_constant( true )] = "1'b1";
    node_names[ntk.get_constant( true )] = "new_n1";
  }

  ntk.foreach_pi( [&]( auto const& n, auto i ) {
    node_names[n] = xs[i];
  } );

  topo_view ntk_topo{ntk};

  ntk_topo.foreach_node( [&]( auto const& n ) {
    if ( ntk.is_constant( n ) || ntk.is_pi( n ) )
      return true;

    /* assign a name */
    node_names[n] = fmt::format( "n{}", ntk.node_to_index( n ) );

    if constexpr ( has_is_buf_v<Ntk> )
    {
      if ( ntk.is_buf( n ) )
      {
        auto const fanin = detail::format_fanin<Ntk>( ntk, n, node_names );
        assert( fanin.size() == 1 );
        std::vector<std::pair<std::string,std::string>> args;
        if ( fanin[0].first ) /* input negated */
        {
          args.emplace_back( std::make_pair( "i", fanin[0].second ) );
          args.emplace_back( std::make_pair( "o", node_names[n] ) );
          writer.on_module_instantiation( "inverter", {}, "inv_" + node_names[n], args );
        }
        else
        {
          args.emplace_back( std::make_pair( "i", fanin[0].second ) );
          args.emplace_back( std::make_pair( "o", node_names[n] ) );
          writer.on_module_instantiation( "buffer", {}, "buf_" + node_names[n], args );
        }
        return true;
      }
    }

    if ( ntk.is_and( n ) )
    {
      // writer.on_assign( node_names[n], detail::format_fanin<Ntk>( ntk, n, node_names ), "&" );
      std::vector<std::pair<bool, std::string>> const ins = detail::format_fanin<Ntk>( ntk, n, node_names );
      if (!ins[0u].first && !ins[1u].first)
        os << fmt::format( ".gate AND    A={} B={} Y={}\n", ins[0u].second, ins[1u].second, node_names[n] );
      if (!ins[0u].first && ins[1u].first)
        os << fmt::format( ".gate ANDNOT A={} B={} Y={}\n", ins[0u].second, ins[1u].second, node_names[n] );
      if (ins[0u].first && !ins[1u].first)
        os << fmt::format( ".gate ANDNOT A={} B={} Y={}\n", ins[1u].second, ins[0u].second, node_names[n] );
      if (ins[0u].first && ins[1u].first)
        os << fmt::format( ".gate NOR    A={} B={} Y={}\n", ins[0u].second, ins[1u].second, node_names[n] );
    }
    else if ( ntk.is_or( n ) )
    {
      writer.on_assign( node_names[n], detail::format_fanin<Ntk>( ntk, n, node_names ), "|" );
    }
    else if ( ntk.is_xor( n ) || ntk.is_xor3( n ) )
    {
      // writer.on_assign( node_names[n], detail::format_fanin<Ntk>( ntk, n, node_names ), "^" );
      std::vector<std::pair<bool, std::string>> const ins = detail::format_fanin<Ntk>( ntk, n, node_names );
      for ( auto i = 0u; i < ins.size(); ++i ) {
        if ( ins.at( i ).first )
          os << fmt::format( ".gate NOT    A={} Y={}\n", ins.at(i).second, node_names[n]+"_"+ins.at(i).second+"_inv" );
      }
      if ( ntk.is_xor( n ) )
        os << fmt::format( ".gate XOR    A={} B={} Y={}\n",
                        ins.at( 0 ).first ? node_names[n]+"_"+ins.at( 0 ).second+"_inv" : ins.at( 0 ).second,
                        ins.at( 1 ).first ? node_names[n]+"_"+ins.at( 1 ).second+"_inv" : ins.at( 1 ).second,
                        node_names[n] );
      if ( ntk.is_xor3( n ) )
        os << fmt::format( ".gate XOR3   A={} B={} C={} Y={}\n",
                        ins.at( 0 ).first ? node_names[n]+"_"+ins.at( 0 ).second+"_inv" : ins.at( 0 ).second,
                        ins.at( 1 ).first ? node_names[n]+"_"+ins.at( 1 ).second+"_inv" : ins.at( 1 ).second,
                        ins.at( 2 ).first ? node_names[n]+"_"+ins.at( 2 ).second+"_inv" : ins.at( 2 ).second,
                        node_names[n] );
    }
    else if ( ntk.is_maj( n ) )
    {
      std::array<signal<Ntk>, 3> children;
      ntk.foreach_fanin( n, [&]( auto const& f, auto i ) { children[i] = f; } );

      if ( ntk.is_constant( ntk.get_node( children[0u] ) ) )
      {
        std::vector<std::pair<bool, std::string>> vs;
        vs.emplace_back( std::make_pair( ntk.is_complemented( children[1u] ), node_names[ntk.get_node( children[1u] )] ) );
        vs.emplace_back( std::make_pair( ntk.is_complemented( children[2u] ), node_names[ntk.get_node( children[2u] )] ) );

        if ( ntk.is_complemented( children[0u] ) )
        {
          // or
          // writer.on_assign( node_names[n], {vs[0u], vs[1u]}, "|" );
          if (!vs[0u].first && !vs[1u].first)
            os << fmt::format( ".gate OR     A={} B={} Y={}\n", vs[0u].second, vs[1u].second, node_names[n] );
          if (!vs[0u].first && vs[1u].first)
            os << fmt::format( ".gate ORNOT  A={} B={} Y={}\n", vs[0u].second, vs[1u].second, node_names[n] );
          if (vs[0u].first && !vs[1u].first)
            os << fmt::format( ".gate ORNOT  A={} B={} Y={}\n", vs[1u].second, vs[0u].second, node_names[n] );
          if (vs[0u].first && vs[1u].first)
            os << fmt::format( ".gate NAND   A={} B={} Y={}\n", vs[0u].second, vs[1u].second, node_names[n] );
        }
        else
        {
          // and
          // writer.on_assign( node_names[n], {vs[0u], vs[1u]}, "&" );
          if (!vs[0u].first && !vs[1u].first)
            os << fmt::format( ".gate AND    A={} B={} Y={}\n", vs[0u].second, vs[1u].second, node_names[n] );
          if (!vs[0u].first && vs[1u].first)
            os << fmt::format( ".gate ANDNOT A={} B={} Y={}\n", vs[0u].second, vs[1u].second, node_names[n] );
          if (vs[0u].first && !vs[1u].first)
            os << fmt::format( ".gate ANDNOT A={} B={} Y={}\n", vs[1u].second, vs[0u].second, node_names[n] );
          if (vs[0u].first && vs[1u].first)
            os << fmt::format( ".gate NOR    A={} B={} Y={}\n", vs[0u].second, vs[1u].second, node_names[n] );
        }
      }
      else
      {
        // writer.on_assign_maj3( node_names[n], detail::format_fanin<Ntk>( ntk, n, node_names ) );
        std::vector<std::pair<bool, std::string>> const ins = detail::format_fanin<Ntk>( ntk, n, node_names );
        for ( auto i = 0u; i < ins.size(); ++i ) {
          if ( ins.at( i ).first )
            os << fmt::format( ".gate NOT    A={} Y={}\n", ins.at(i).second, node_names[n]+"_"+ins.at(i).second+"_inv" );
        }
        os << fmt::format( ".gate MAJ3   A={} B={} C={} Y={}\n",
                        ins.at( 0 ).first ? node_names[n]+"_"+ins.at( 0 ).second+"_inv" : ins.at( 0 ).second,
                        ins.at( 1 ).first ? node_names[n]+"_"+ins.at( 1 ).second+"_inv" : ins.at( 1 ).second,
                        ins.at( 2 ).first ? node_names[n]+"_"+ins.at( 2 ).second+"_inv" : ins.at( 2 ).second,
                        node_names[n] );
      }
    }
    else
    {
      if constexpr ( has_is_nary_and_v<Ntk> )
      {
        if ( ntk.is_nary_and( n ) )
        {
          writer.on_assign( node_names[n], detail::format_fanin<Ntk>( ntk, n, node_names ), "&" );
          return true;
        }
      }
      if constexpr ( has_is_nary_or_v<Ntk> )
      {
        if ( ntk.is_nary_or( n ) )
        {
          writer.on_assign( node_names[n], detail::format_fanin<Ntk>( ntk, n, node_names ), "|" );
          return true;
        }
      }
      if constexpr ( has_is_nary_xor_v<Ntk> )
      {
        if ( ntk.is_nary_xor( n ) )
        {
          writer.on_assign( node_names[n], detail::format_fanin<Ntk>( ntk, n, node_names ), "^" );
          return true;
        }
      }
      writer.on_assign_unknown_gate( node_names[n] );
    }

    return true;
  } );

  ntk.foreach_po( [&]( auto const& f, auto i ) {
    // writer.on_assign_po( ys[i], std::make_pair( ntk.is_complemented( f ), node_names[f] ) );
    if (ntk.is_complemented( f )) {
      os << fmt::format( ".gate NOT    A={} Y={}\n", node_names[f], ys[i]+"_"+node_names[f]+"_inv" );
      os << fmt::format( ".gate BUF    A={} Y={}\n", ys[i]+"_"+node_names[f]+"_inv", ys[i] );
    }
    else
      os << fmt::format( ".gate BUF    A={} Y={}\n", node_names[f], ys[i] );
  } );

  os << ".end\n";
  os << std::flush;
}


/*! \brief Writes network in structural Verilog format into a file
 *
 * **Required network functions:**
 * - `num_pis`
 * - `num_pos`
 * - `foreach_pi`
 * - `foreach_node`
 * - `foreach_fanin`
 * - `get_node`
 * - `get_constant`
 * - `is_constant`
 * - `is_pi`
 * - `is_and`
 * - `is_or`
 * - `is_xor`
 * - `is_xor3`
 * - `is_maj`
 * - `node_to_index`
 *
 * \param ntk Network
 * \param filename Filename
 */
template<class Ntk>
void write_gblif( Ntk const& ntk, std::string const& filename, write_gblif_params const& ps = {} )
{
  std::ofstream os( filename.c_str(), std::ofstream::out );
  write_gblif( ntk, os, ps );
  os.close();
}

} /* namespace mockturtle */