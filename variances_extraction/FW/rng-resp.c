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
PROCESS(ranging_resp_process, "Ranging responder process");
AUTOSTART_PROCESSES(&ranging_resp_process);
/*---------------------------------------------------------------------------*/
/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 µs and 1 µs = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME (65536)
#define RESP_DELAY (500) // Time to wait before sending the ranging reply (resp message) in ~us
#define UWB_RESP_DELAY (RESP_DELAY * UUS_TO_DWT_TIME)
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(ranging_resp_process, ev, data)
{
  static sstwr_init_msg_t init_msg; // Init ranging message (i.e., the first message in the TWR exchange, sent by the initiator)
  static sstwr_resp_msg_t resp_msg; // Resp ranging message (i.e., the second message in the TWR exchange, sent by the responder)
  static uint8_t ret; // To check the TX/RX status

  PROCESS_BEGIN();

  printf("I am %02x%02x\n",
    linkaddr_node_addr.u8[0],
    linkaddr_node_addr.u8[1]
  );

  /* Initialize the radio */
  radio_init();

  while(1) {

    /* Start the receiver immediately */
    start_rx(NO_RX_TIMEOUT);
    printf("RX enabled\n");

    /* Wait for an init message */
    ret = wait_rx();

    /* If the reception failed, abort and restart the procedure */
    if (!ret) {
      radio_reset();
      printf("RX fail\n");
      continue;
    }

    /* Read the ranging init message */
    ret = read_rx_data(&init_msg, sizeof(init_msg));

    /* If the message could not be read, abort and restart the procedure */
    if (!ret) {
      radio_reset();
      printf("RX wrong size\n");
      continue;
    }

    /* Extract the packet source and destination addresses from the received message */
    linkaddr_t init_src, init_dst;
    init_src.u8[0] = init_msg.hdr.src[1];
    init_src.u8[1] = init_msg.hdr.src[0];
    init_dst.u8[0] = init_msg.hdr.dst[1];
    init_dst.u8[1] = init_msg.hdr.dst[0];

    /* Check if the destination is this node.
     * If so, send a resp message to complete the two-way ranging exchange */
    if(linkaddr_cmp(&init_dst, &linkaddr_node_addr)) {

      /* TO-DO 1: Get the init RX timestamp and *compute* the resp TX time to schedule the resp transmission 
       * TIP: Remember that DW1000 timestamps are 40 bits long (DWT_VALUES in rng-support.h) */
      uint64_t init_rx_ts, resp_tx_ts;
      init_rx_ts = get_rx_timestamp();
      resp_tx_ts = (init_rx_ts + UWB_RESP_DELAY) % DWT_VALUES;

      /* Set the source and destination addresses and the ranging sequence number in resp message */
      fill_ieee_hdr(&resp_msg.hdr, &linkaddr_node_addr, &init_src, init_msg.hdr.seqn);

      /* Predict the *actual* TX timestamp BEFORE sending */
      uint64_t predicted_tx_ts = predict_tx_timestamp(resp_tx_ts);

      /* TO-DO 2: Write the timestamps in the resp message.
       * TIP: Find and exploit the appropriate function in rng-support.h / rng-support.c.
       */
      resp_msg_set_timestamp(&resp_msg.init_rx_ts[0], init_rx_ts);
      resp_msg_set_timestamp(&resp_msg.resp_tx_ts[0], predicted_tx_ts);

      /* TO-DO 3: Send the packet and wait for TX confirmation
       * TIP: Use ret = start_tx(...);
       */
      ret = start_tx(
        &resp_msg, sizeof(resp_msg),
        DWT_START_TX_DELAYED,
        0,
        resp_tx_ts
      );

      /* Check the outcome of the transmission */
      if(ret) {
        printf("[%u] RESP %02x:%02x at %llu (ts: %llu->%llu)\n",
          init_msg.hdr.seqn, resp_msg.hdr.dst[1], resp_msg.hdr.dst[0],
          get_tx_timestamp(), init_rx_ts, predicted_tx_ts);
      }
      else {
        printf("[%u] RESP fail %02x:%02x\n",
          init_msg.hdr.seqn, resp_msg.hdr.dst[1], resp_msg.hdr.dst[0]);
      }
    }
    else {
      printf("RX wrong dest %02x:%02x\n", init_msg.hdr.dst[1], init_msg.hdr.dst[0]);
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
