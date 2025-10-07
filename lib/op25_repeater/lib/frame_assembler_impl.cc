/* -*- c++ -*- */
/* 
 * Copyright 2010, 2011, 2012, 2013, 2014, 2015, 2016, 2017 Max H. Parke KA1RBI 
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "frame_assembler_impl.h"
#include "rx_sync.h"
#include "rx_smartnet.h"
#include "rx_subchannel.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <vector>
#include <sys/time.h>

namespace gr {
    namespace op25_repeater {

        void frame_assembler_impl::set_xormask(const char* p) {
            if (d_sync)
                d_sync->set_xormask(p);
        }

        void frame_assembler_impl::set_nac(int nac) {
            if (d_sync)
                d_sync->set_nac(nac);
        }

        void frame_assembler_impl::set_slotid(int slotid) {
            if (d_sync)
                d_sync->set_slot_mask(slotid);
        }

        void frame_assembler_impl::set_slotkey(int key) {
            if (d_sync)
                d_sync->set_slot_key(key);
        }

        void frame_assembler_impl::sync_reset() {
            if (d_sync)
                d_sync->sync_reset();
        }

        void frame_assembler_impl::crypt_reset() {
            if (d_sync)
                d_sync->crypt_reset();
        }

        void frame_assembler_impl::crypt_key(uint16_t keyid, uint8_t algid, const std::vector<uint8_t> &key) {
            if (d_sync) {
                if (d_debug >= 10) {
                    std::string k_str = uint8_vector_to_hex_string(key);
                    fprintf(stderr, "%s frame_assembler_impl::crypt_key: setting keyid(0x%x), algid(0x%x), key(0x%s)\n", logts.get(d_msgq_id), keyid, algid, k_str.c_str());
                }
                d_sync->crypt_key(keyid, algid, key);
            }
        }

        void frame_assembler_impl::set_debug(int debug) {
            if (d_sync)
                d_sync->set_debug(debug);
        }

        frame_assembler::sptr
            frame_assembler::make(const char* options, int debug, int msgq_id, gr::msg_queue::sptr queue, bool d_soft_vocoder)
            {
                return gnuradio::get_initial_sptr
                    (new frame_assembler_impl(options, debug, msgq_id, queue, d_soft_vocoder));
            }

        /*
         * Our public destructor
         */
        frame_assembler_impl::~frame_assembler_impl()
        {
            if (d_sync)
                delete d_sync;
        }

        static const int MIN_IN = 1;	// mininum number of input streams
        static const int MAX_IN = 1;	// maximum number of input streams

        /*
         * The private constructor
         */
        frame_assembler_impl::frame_assembler_impl(const char* options, int debug, int msgq_id, gr::msg_queue::sptr queue, bool d_soft_vocoder)
            : gr::block("frame_assembler",
                    gr::io_signature::make (MIN_IN, MAX_IN, sizeof (char)),
	                gr::io_signature::make ( 2, 2, sizeof(int16_t))),
	
            d_msgq_id(msgq_id),
            d_msg_queue(queue),
            output_queue(),
            d_sync(NULL)
        {
            if (strcasecmp(options, "smartnet") == 0)
                d_sync = new rx_smartnet(options, logts, debug, msgq_id, queue);
            else if (strcasecmp(options, "subchannel") == 0)
                d_sync = new rx_subchannel(options, logts, debug, msgq_id, queue);
            else
                d_sync = new rx_sync(options, logts, debug, msgq_id, queue, output_queue, d_soft_vocoder);
        }

        int 
            frame_assembler_impl::general_work (int noutput_items,
                    gr_vector_int &ninput_items,
                    gr_vector_const_void_star &input_items,
                    gr_vector_void_star &output_items)
            {

                const uint8_t *in = (const uint8_t *) input_items[0];

                if (d_sync) {
                    for (int i=0; i<ninput_items[0]; i++) {
                        d_sync->rx_sym(in[i]);
                    }
                }
        
        int amt_produce = 0;

        
        
/*
          
        if (amt_produce > (int)output_queue.size()) {
          amt_produce = output_queue.size();
        }
*/  
        produce(0, output_queue[0].size());
        produce(1, output_queue[1].size());

                if ((output_queue[0].size() > 0) || ( output_queue[1].size() > 0)) {
        //BOOST_LOG_TRIVIAL(info) << "DMR Frame Assembler - Amt Prod: " << amt_produce << " output_queue 0: " << output_queue[0].size() << " output_queue 1: " << output_queue[1].size() <<" noutput_items: " <<  noutput_items;
        }
        for (int slot_id = 0; slot_id < 2; slot_id++) {
        int16_t *out = (int16_t *)output_items[slot_id];
        int src_id = d_sync->get_src_id(slot_id);
        std::pair<bool,long> terminated = d_sync->get_terminated(slot_id);
        if ((src_id != -1) && (src_id != 0)) {
            BOOST_LOG_TRIVIAL(info) << "DMR Frame Assembler - sending src: " << src_id;
            add_item_tag(0, nitems_written(0), pmt::intern("src_id"), pmt::from_long(src_id), pmt::intern(name()));
          }
          /*
        if (terminated) {
            add_item_tag(0, nitems_written(0), pmt::intern("terminate"), pmt::from_long(1), pmt::intern(name()));
        }*/
          for (int i = 0; i < output_queue[slot_id].size(); i++) {
              //BOOST_LOG_TRIVIAL(info) << output_queue[slot_id][i];
            out[i] = output_queue[slot_id][i];
          }
          output_queue[slot_id].clear();
          //output_queue[slot_id].erase(output_queue[slot_id].begin(), output_queue[slot_id].begin() + output_queue[slot_id].size());
        }

        //BOOST_LOG_TRIVIAL(info) << "DMR Frame Assembler - Amt Prod: " << amt_produce << " output_items 0: " << len(output_items[0]) << " output_items 1: " << len(output_items[1]) <<" noutput_items: " <<  noutput_items;
        
        consume_each(ninput_items[0]);
        // Tell runtime system how many output items we produced.
        return WORK_CALLED_PRODUCE;

        }

    } /* namespace op25_repeater */
} /* namespace gr */
