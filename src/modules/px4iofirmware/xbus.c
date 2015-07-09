/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file dsm.c
 *
 * Serial protocol decoder for the JR XBus protocol.
 * 
 * Uses XBus Mode B which is the Multiplex SRXL protocol, details here
 * http://www.multiplex-rc.de/en/service/downloads/interface-descriptions.html?eID=dam_frontend_push&docID=4233
 *
 * Decodes into the global PPM buffer and updates accordingly.
 */

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>

#include <drivers/drv_hrt.h>

#include "px4io.h"

#define XBUS_NUM_CHANNELS	12		/**<Number of XBus channels*/
#define XBUS_PACKET_LENGTH     3 + XBUS_NUM_CHANNELS*2

static int bytesReceived = 0;       /**<Number of bytes received since start character */
static uint8_t xbus_buffer[XBUS_PACKET_LENGTH];

uint16_t CRC16(uint16_t crc, uint8_t value);

uint16_t CRC16(uint16_t crc, uint8_t value)
{
    uint8_t i;
    crc = crc ^ (int16_t)value<<8;
    for(i = 0; i < 8; i++) {
        if(crc & 0x8000)
            crc = crc << 1^0x1021;
        else
            crc = crc << 1;
    }
    return crc;
}

/**
 * Called periodically to check for input data from the XBus UART
 * 
 * 
 *
 * @param[in] bytes received through DSM serial port
 * @param[in] number of bytes received through DSM serial port
 * @param[out] values pointer to per channel array of decoded values
 * @param[out] num_values pointer to number of raw channel values returned

 * @return true=decoded raw channel values updated, false=no update
 */
bool
xbus_input(uint8_t *bytes, uint16_t num_bytes, uint16_t *values, uint16_t *num_values)
{
    /* If we have no new data, return here */
    if (num_bytes < 1)
        return false;
    
    /* Copy new bytes in to our buffer */
    memcpy(xbus_buffer + bytesReceived, bytes, num_bytes);
    
    /* If start character is not correct, reset everything */
    if (xbus_buffer[0] != 0xA1) {
        memset(xbus_buffer, '\0', XBUS_PACKET_LENGTH);
        bytesReceived = 0;
        return false;
    }
    
    /* Increment buffer length */
    bytesReceived += num_bytes;
    
    /* If we don't have a full packet, return here */
    if (bytesReceived < XBUS_PACKET_LENGTH)
        return false;
    
    /* Reset counter ready for next packet */
    bytesReceived = 0;
    
    /* Check CRC of packet */
    uint16_t crc_calc = 0;
    for (int i=0; i< XBUS_PACKET_LENGTH-2; i++) {
        crc_calc = CRC16(crc_calc, xbus_buffer[i]);
    }
    uint16_t crc_buffer = ((uint16_t)(xbus_buffer[XBUS_PACKET_LENGTH - 2]))<<8 | (uint16_t)(xbus_buffer[XBUS_PACKET_LENGTH - 1]);
    if (crc_calc != crc_buffer)
        return false;
    
    
    /* Apply this channel mapping to get correct order 
     *  JR XBus Mode B order is:
     *      - Aileron
     *      - Elevator
     *      - Rudder
     *      - Aux 2 (Flap)
     *      - Throttle
     *      - Aux 1 (Gear)
     *      - Aux 3
     *      - Aux 4
     *      - etc...
     **/
    uint8_t chMap[] = {0, 1, 3, 5, 2, 4, 6, 7, 8, 9, 10, 11};
    *num_values = XBUS_NUM_CHANNELS;
    
    /* Channel scaling is linear between
     * - 0x000: 800us
     * - 0xfff: 2200us
     **/
    for (int channel = 0; channel<XBUS_NUM_CHANNELS; channel++) {
        uint32_t value = (((uint16_t)xbus_buffer[2*channel+1])<<8) | ((uint16_t)xbus_buffer[2*channel+2]);
        double dblVal = value*(2200.0-800.0)/4095.0 + 800.0;
        values[chMap[channel]] = (uint16_t)dblVal;
    }
    return true;
}
