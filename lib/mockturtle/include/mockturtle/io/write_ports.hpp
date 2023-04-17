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
  \file write_ports.hpp
  \brief Write networks inputs and outputs

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

/*! \brief Writes network ports into output stream
 *
 * **Required network functions:**
 * - `num_pis`
 * - `num_pos`
 * - `foreach_pi`
 *
 * \param ntk Network
 * \param os Output stream
 */
template<class Ntk>
void write_ports( Ntk const& ntk, std::ostream& os )
{
  static_assert( is_network_type_v<Ntk>, "Ntk is not a network type" );
  static_assert( has_num_pis_v<Ntk>, "Ntk does not implement the num_pis method" );
  static_assert( has_num_pos_v<Ntk>, "Ntk does not implement the num_pos method" );
  static_assert( has_foreach_pi_v<Ntk>, "Ntk does not implement the foreach_pi method" );
  static_assert( has_foreach_node_v<Ntk>, "Ntk does not implement the foreach_node method" );
  static_assert( has_foreach_fanin_v<Ntk>, "Ntk does not implement the foreach_fanin method" );
  static_assert( has_get_node_v<Ntk>, "Ntk does not implement the get_node method" );
  static_assert( has_get_constant_v<Ntk>, "Ntk does not implement the get_constant method" );
  static_assert( has_is_constant_v<Ntk>, "Ntk does not implement the is_constant method" );
  static_assert( has_is_pi_v<Ntk>, "Ntk does not implement the is_pi method" );
  static_assert( has_is_and_v<Ntk>, "Ntk does not implement the is_and method" );
  static_assert( has_is_or_v<Ntk>, "Ntk does not implement the is_or method" );
  static_assert( has_is_xor_v<Ntk>, "Ntk does not implement the is_xor method" );
  static_assert( has_is_xor3_v<Ntk>, "Ntk does not implement the is_xor3 method" );
  static_assert( has_is_maj_v<Ntk>, "Ntk does not implement the is_maj method" );
  static_assert( has_node_to_index_v<Ntk>, "Ntk does not implement the node_to_index method" );

  assert( ntk.is_combinational() && "Network has to be combinational" );

  std::vector<std::string> ports;

  if constexpr ( has_has_name_v<Ntk> && has_get_name_v<Ntk> )
  {
    ntk.foreach_pi( [&]( auto const& i, uint32_t index ){
      if ( ntk.has_name( ntk.make_signal( i ) ) )
      {
        ports.emplace_back( ntk.get_name( ntk.make_signal( i ) ) );
      }
      else
      {
        ports.emplace_back( fmt::format( "x{}", index ) );
      }
    });
  }
  else
  {
    for ( auto i = 0u; i < ntk.num_pis(); ++i )
    {
      ports.emplace_back( fmt::format( "x{}", i ) );
    }
  }

  if constexpr ( has_has_output_name_v<Ntk> && has_get_output_name_v<Ntk> )
  {
    ntk.foreach_po( [&]( auto const& o, uint32_t index ){
      if ( ntk.has_output_name( index ) )
      {
        ports.emplace_back( ntk.get_output_name( index ) );
      }
      else
      {
        ports.emplace_back( fmt::format( "y{}", index ) );
      }
    });
  }
  else
  {
    for ( auto i = 0u; i < ntk.num_pos(); ++i )
    {
      ports.emplace_back( fmt::format( "y{}", i ) );
    }
  }

  os << fmt::format( "{}", fmt::join( ports, " " ) );
}

/*! \brief Writes network ports
 *
 * **Required network functions:**
 * - `num_pis`
 * - `num_pos`
 * - `foreach_pi`
 *
 * \param ntk Network
 * \param filename Filename
 */
template<class Ntk>
void write_ports( Ntk const& ntk, std::string const& filename )
{
  std::ofstream os( filename.c_str(), std::ofstream::out );
  write_ports( ntk, os );
  os.close();
}

} /* namespace mockturtle */