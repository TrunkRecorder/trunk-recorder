// smartnet_decode.h
/* -*- c++ -*- */
/*
 * Copyright 2004 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * GNU Radio is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
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
#ifndef smartnet_decode_H
#define smartnet_decode_H

#include <gnuradio/msg_queue.h>
#include <gnuradio/sync_block.h>

class smartnet_decode;

/*
 * We use boost::shared_ptr's instead of raw pointers for all access
 * to gr_blocks (and many other data structures).  The shared_ptr gets
 * us transparent reference counting, which greatly simplifies storage
 * management issues.  This is especially helpful in our hybrid
 * C++ / Python system.
 *
 * See http://www.boost.org/libs/smart_ptr/smart_ptr.htm
 *
 * As a convention, the _sptr suffix indicates a boost::shared_ptr
 */

#if GNURADIO_VERSION < 0x030900
typedef boost::shared_ptr<smartnet_decode> smartnet_decode_sptr;
#else
typedef std::shared_ptr<smartnet_decode> smartnet_decode_sptr;
#endif
/*!
 * \brief Return a shared_ptr to a new instance of smartnet_decode.
 *
 * To avoid accidental use of raw pointers, smartnet_decode's
 * constructor is private.  smartnet_make_decode is the public
 * interface for creating new instances.
 */
smartnet_decode_sptr smartnet_make_decode(gr::msg_queue::sptr queue, int sys_num);

/*!
 * \brief unstuff a packed stream of bits.
 * \ingroup block
 *
 *
 * This uses the preferred technique: subclassing gr_block.
 */
class smartnet_decode : public gr::sync_block {
private:
  // The friend declaration allows smartnet_make_decode to
  // access the private constructor.
  gr::msg_queue::sptr d_queue;
  int sys_num;

  friend smartnet_decode_sptr smartnet_make_decode(gr::msg_queue::sptr queue, int sys_num);

  smartnet_decode(gr::msg_queue::sptr queue, int sys_num); // private constructor

public:
  ~smartnet_decode(); // public destructor

  // Where all the action really happens

  int work(int noutput_items,

           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);

  /*	void forecast (int noutput_items,
                       gr_vector_int &ninput_items_required);*/
};

#endif /* smartnet_decode_H */
