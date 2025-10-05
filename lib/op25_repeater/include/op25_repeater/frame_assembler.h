/* -*- c++ -*- */
/* 
 * Copyright 2017 Max H. Parke KA1RBI
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


#ifndef INCLUDED_OP25_REPEATER_FRAME_ASSEMBLER_H
#define INCLUDED_OP25_REPEATER_FRAME_ASSEMBLER_H

#include <op25_repeater/api.h>
#include <gnuradio/block.h>
#include <gnuradio/msg_queue.h>

namespace gr {
    namespace op25_repeater {

        /*!
         * \brief <+description of block+>
         * \ingroup op25_repeater
         *
         */
        class OP25_REPEATER_API frame_assembler : virtual public gr::block
        {
            public:
                #if GNURADIO_VERSION < 0x030900
                typedef boost::shared_ptr<frame_assembler> sptr;
                #else
                typedef std::shared_ptr<frame_assembler> sptr;
                #endif
                /*!
                 * \brief Return a shared_ptr to a new instance of op25_repeater::frame_assembler.
                 *
                 * To avoid accidental use of raw pointers, op25_repeater::frame_assembler's
                 * constructor is in a private implementation
                 * class. op25_repeater::frame_assembler::make is the public interface for
                 * creating new instances.
                 */
                static sptr make(const char* options, int debug, int msgq_id, gr::msg_queue::sptr queue, bool d_soft_vocoder);
                virtual void set_xormask(const char*p) {}
                virtual void set_nac(int nac) {}
                virtual void set_slotid(int slotid) {}
                virtual void set_slotkey(int key) {}
                virtual void set_debug(int debug) {}
                virtual void sync_reset(void) {}
                virtual void crypt_reset(void) {}
                virtual void crypt_key(uint16_t keyid, uint8_t algid, const std::vector<uint8_t> &key) {}
        };

    } // namespace op25_repeater
} // namespace gr

#endif /* INCLUDED_OP25_REPEATER_FRAME_ASSEMBLER_H */

