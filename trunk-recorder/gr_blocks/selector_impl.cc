/* -*- c++ -*- */
/*
 * Copyright 2019 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * GNU Radio is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * GNU Radio is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include "selector_impl.h"
#include <gnuradio/io_signature.h>
#include <stdexcept>
#include <string.h>

namespace gr {
namespace blocks {

selector::sptr
selector::make(size_t itemsize, unsigned int input_index, unsigned int output_index) {
  return gnuradio::get_initial_sptr(
      new selector_impl(itemsize, input_index, output_index));
}

selector_impl::selector_impl(size_t itemsize,
                             unsigned int input_index,
                             unsigned int output_index)
    : block("selector",
            io_signature::make(1, -1, itemsize),
            io_signature::make(1, -1, itemsize)),
      d_itemsize(itemsize),
      d_enabled(true),
      d_input_index(input_index),
      d_output_index(output_index),
      d_num_inputs(0),
      d_got_samples(true),
      d_num_outputs(0) {

  d_enabled_output_ports = std::vector<bool>(d_max_port, false);
  // TODO: add message ports for input_index and output_index
}

selector_impl::~selector_impl() {}

bool selector_impl::got_samples() {
  bool current_got_samples = d_got_samples;
  d_got_samples = false;
  return current_got_samples;
}

void selector_impl::set_input_index(unsigned int input_index) {
  gr::thread::scoped_lock l(d_mutex);

  if (input_index < 0)
    throw std::out_of_range("input_index must be >= 0");

  if (input_index < d_num_inputs)
    d_input_index = input_index;
  else
    throw std::out_of_range("input_index must be < ninputs");
}

// This enables a single output port and disables all others
void selector_impl::set_output_index(unsigned int output_index) {
  gr::thread::scoped_lock l(d_mutex);

  if (output_index < 0)
    throw std::out_of_range("input_index must be >= 0");

  if (output_index < d_max_port)
    d_output_index = output_index;
  else
    throw std::out_of_range("output_index must be < noutputs");

  for (unsigned int out_idx = 0; out_idx < d_max_port; out_idx++) {
    if (output_index == out_idx) {
      d_enabled_output_ports[out_idx] = true;
    } else {
      d_enabled_output_ports[out_idx] = false;
    }
  }
}

void selector_impl::forecast(int noutput_items, gr_vector_int &ninput_items_required) {
  unsigned ninputs = ninput_items_required.size();
  for (unsigned i = 0; i < ninputs; i++) {
    ninput_items_required[i] = noutput_items;
  }
}

bool selector_impl::check_topology(int ninputs, int noutputs) {
  if ((int)d_input_index < ninputs && (int)d_output_index < noutputs) {
    d_num_inputs = (unsigned int)ninputs;
    d_num_outputs = (unsigned int)noutputs;
    return true;
  } else {
    GR_LOG_WARN(d_logger,
                "check_topology: Input or Output index greater than number of ports");
    return false;
  }
}

void selector_impl::set_port_enabled(unsigned int port, bool enabled) {
  if (port >= d_max_port) {
    BOOST_LOG_TRIVIAL(info) << "disable_output_port() - Port: " << port << " is greater than max port of: " << d_max_port;
    return;
  }

  gr::thread::scoped_lock l(d_mutex);
  d_enabled_output_ports[port] = enabled;
}

bool selector_impl::is_port_enabled(unsigned int port) {
  if (port >= d_max_port) {
    BOOST_LOG_TRIVIAL(info) << "disable_output_port() - Port: " << port << " is greater than max port of: " << d_max_port;
    return false;
  }

  gr::thread::scoped_lock l(d_mutex);
  return d_enabled_output_ports[port];
}

int selector_impl::general_work(int noutput_items,
                                gr_vector_int &ninput_items,
                                gr_vector_const_void_star &input_items,
                                gr_vector_void_star &output_items) {
  const uint8_t **in = (const uint8_t **)&input_items[0];
  uint8_t **out = (uint8_t **)&output_items[0];

  gr::thread::scoped_lock l(d_mutex);

  if (output_items.size() > 0) {
    d_got_samples = true;
  }

  for (size_t out_idx = 0; out_idx < output_items.size(); out_idx++) {
    if (d_enabled_output_ports[out_idx]) {
      std::copy(in[d_input_index],
                in[d_input_index] + noutput_items * d_itemsize,
                out[out_idx]);
      produce(out_idx, noutput_items);
    }
  }

  /*
    std::copy(in[d_input_index],
              in[d_input_index] + noutput_items * d_itemsize,
              out[d_output_index]);
    produce(d_output_index, noutput_items);*/

  consume_each(noutput_items);
  return WORK_CALLED_PRODUCE;
}

} /* namespace blocks */
} /* namespace gr */
