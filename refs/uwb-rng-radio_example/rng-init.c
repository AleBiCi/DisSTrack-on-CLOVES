/*
 * Copyright (c) 2022, University of Trento.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "contiki.h"
#include "lib/random.h"
#include "net/rime/rime.h"
#include "leds.h"
#include "net/netstack.h"
#include <stdio.h>
#include "dw1000.h"
#include "core/net/linkaddr.h"
#include "rng-support.h"
/*---------------------------------------------------------------------------*/
PROCESS(ranging_process, "Ranging process");
AUTOSTART_PROCESSES(&ranging_process);
/*---------------------------------------------------------------------------*/
#define RANGING_INTERVAL (CLOCK_SECOND / 2) // Period after which the initiator starts a new ranging round
#define RANGING_TIMEOUT (5000) /* Maximum time the initiator can wait for a ranging reply, in ~us (UWB microseconds).
                                * NB: This timeout has been set deliberately large, to let you play with different RESP_DELAY at the responder.
                                * If you want to increase RESP_DELAY further, remember to increase this timeout as well!
                                */
/*---------------------------------------------------------------------------*/
linkaddr_t init = {{0x13, 0x9a}}; // Node 1
#define NUM_DEST 4
linkaddr_t resp_list[NUM_DEST] = {
  {{0x19, 0x15}}, // Node 2
  {{0x11, 0x0c}}, // Node 3
  {{0x18, 0x33}}, // Node 7
  {{0x15, 0x95}}  // Node 36
};
linkaddr_t resp;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(ranging_process, ev, data)
{
  static struct etimer et; // Event timer
  static sstwr_init_msg_t init_msg; // Init ranging message (i.e., the first message in the TWR exchange, sent by the initiator)
  static sstwr_resp_msg_t resp_msg; // Resp ranging message (i.e., the second message in the TWR exchange, sent by the responder)
  static uint8_t seqn = 0; // Sequence number
  static uint8_t ret; // To check the TX/RX status

  PROCESS_BEGIN();

  printf("I am %02x%02x\n",
    linkaddr_node_addr.u8[0],
    linkaddr_node_addr.u8[1]
  );

  /* Initialize the radio */
  radio_init();

  /* Keep ranging (in round robin with the variious responders) */
  while(1) {
    seqn ++;

    /* Choose the destination (round robin) */
    linkaddr_copy(&resp, &resp_list[seqn % NUM_DEST]);

    /* Wait for RANGING_INTERVAL */
    etimer_set(&et, RANGING_INTERVAL);
    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER && etimer_expired(&et));
    printf("[%u] ranging with %02x%02x ...\n", seqn, resp.u8[0], resp.u8[1]);

    /* Prepare the packet header, setting the surce and destination addresses and the sequence number */
    fill_ieee_hdr(&init_msg.hdr, &linkaddr_node_addr, &resp, seqn);

    /* Send the packet */
    ret = start_tx(
      &init_msg, sizeof(init_msg), // Content
      DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED, // Mode
      RANGING_TIMEOUT, // Maximum RX time after TX (relevant only if DWT_RESPONSE_EXPECTED is set)
      0 // TX time, not set because we are asking the radio to TX immediately (DWT_START_TX_IMMEDIATE)
    );

    /* If the transmission failed, abort the ranging round and restart the procedure */
    if (!ret) {
      radio_reset();
      printf("[%u] fail (TX err)\n", seqn);
      continue;
    }

    /* Wait for a response */
    ret = wait_rx();

    /* If the reception failed, abort the ranging round and restart the procedure */
    if (!ret) {
      radio_reset();
      printf("[%u] fail (RX err)\n", seqn);
      continue;
    }

    /* Read the ranging reply */
    ret = read_rx_data(&resp_msg, sizeof(resp_msg));

    /* If the message could not be read, abort and restart the procedure */
    if (!ret) {
      radio_reset();
      printf("[%u] fail (bad size)\n", seqn);
      continue;
    }

    /* Get the linkaddr information (source, destination) in the resp packet */
    linkaddr_t resp_src, resp_dst;
    resp_src.u8[0] = resp_msg.hdr.src[1];
    resp_src.u8[1] = resp_msg.hdr.src[0];
    resp_dst.u8[0] = resp_msg.hdr.dst[1];
    resp_dst.u8[1] = resp_msg.hdr.dst[0];

    /* Check that the addresses match the initiator's expectations */
    if(linkaddr_cmp(&resp_src, &resp) && linkaddr_cmp(&resp_dst, &linkaddr_node_addr)) {

      /* Get init TX and resp RX timestamps */
      uint64_t init_tx_ts, resp_rx_ts;
      init_tx_ts = get_tx_timestamp(); // Time at which the initiator sent the init message
      resp_rx_ts = get_rx_timestamp(); // Time at which the initiator received the resp message

      /* TO-DO 1: Get init RX and resp TX timestamps, embedded in the resp message (resp_msg)! 
       * TIP: Find and exploit the appropriate function in rng-support.h / rng-support.c.
       */
      uint64_t init_rx_ts, resp_tx_ts;
      resp_msg_get_timestamp(&resp_msg.init_rx_ts[0], &init_rx_ts);
      resp_msg_get_timestamp(&resp_msg.resp_tx_ts[0], &resp_tx_ts);
      

      /* TO-DO 2: Compute the time of flight!
       * Note: 
       * - Timestamps are expressed in DWT_TIME_UNITS (~15.65e-12 s), see contiki-uwb/dev/dw1000/decadriver/deca_device_api.h 
       *   TIP: Consider whether this is the correct unit of measurement to express the time of flight.
       * - Remember that the clock can overflow between the two timestamp acquisitions;
       *   you *have to* handle this condition properly 
       *   TIP: check DWT_VALUES in rng-support.h
       * Message scheme:
       * INIT   init_tx_ts <---- t_one -----> resp_rx_ts
       *             \                            /
       *              \                          /
       * RESP     init_rx_ts <-- t_two ---> resp_tx_ts
       */
      int64_t t_one, t_two;
      double tof;
      t_one = (resp_rx_ts - init_tx_ts) % DWT_VALUES; /* Alternative approach:
                                                       * if (resp_rx_ts >= init_tx_ts) (i.e., the clock didn't overflow)
                                                       * t_one = resp_rx_ts - init_tx_ts;
                                                       * else
                                                       * t_one = DWT_VALUES - init_tx_ts + resp_rx_ts;
                                                       */
      t_two = (resp_tx_ts - init_rx_ts) % DWT_VALUES;
      tof = ((t_one - t_two) / 2.0) * DWT_TIME_UNITS;

      /* TO-DO 3: Based on the time of flight, compute the actual distance!
       * TIP: You can get the speed of light in air from the macro SPEED_OF_LIGHT (m/s) in rng-support.h
       */
      double dist;
      uint64_t dist_mm;  // In millimeters
      dist = tof * SPEED_OF_LIGHT;
      dist_mm = (uint64_t)(dist * 1000);

      printf("RANGING OK [%02x:%02x->%02x:%02x] %llu mm\n",
        linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
        resp.u8[0], resp.u8[1],
        dist_mm);
    }
    else {
      printf("[%u] fail [%02x%02x->%02x%02x]\n",
        seqn, resp_src.u8[0], resp_src.u8[1], resp_dst.u8[0], resp_dst.u8[1]);
      continue;
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
