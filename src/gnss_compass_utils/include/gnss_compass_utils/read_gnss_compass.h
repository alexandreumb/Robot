/******************************************************************************/
/*                                                                            */
/*                Advanced Navigation Packet Protocol Library                 */
/*              C Language Dynamic GNSS Compass SDK, Version 7.0              */
/*                    Copyright 2023, Advanced Navigation                     */
/*                                                                            */
/******************************************************************************/
/*
 * Copyright (C) 2023 Advanced Navigation
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */


#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <time.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#define _WIN32_WINNT 0x0501
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#endif

#include "an_packet_protocol.h"
#include "ins_packets.h"

#define RADIANS_TO_DEGREES (180.0/M_PI)

static unsigned char request_all_configuration[] = { 0xE2, 0x01, 0x10, 0x9A, 0x73, 0xB6, 0xB4, 0xB5, 0xB8, 0xB9, 0xBA, 0xBC, 0xBD, 0xC0, 0xC2, 0xC3, 0xC4, 0x03, 0xC6, 0x45, 0xC7 };
int socket_fd = -1;

int transmit(const unsigned char* data, int length);
int receive(unsigned char* data, int length);
int an_packet_transmit(an_packet_t* an_packet);
void set_filter_options();


int transmit(const unsigned char* data, int length)
{
#if _WIN32
	return send(socket_fd, (char *) data, length, 0);
#else
	return write(socket_fd, data, length);
#endif
}

int receive(unsigned char* data, int length)
{
#if _WIN32
	return recv(socket_fd, (char *) data, length, 0);
#else
	return read(socket_fd, data, length);
#endif
}

int an_packet_transmit(an_packet_t* an_packet)
{
	an_packet_encode(an_packet);
	return transmit(an_packet_pointer(an_packet), an_packet_size(an_packet));
}

/*
 * This is an example of sending a configuration packet to GNSS Compass.
 *
 * 1. First declare the structure for the packet, in this case filter_options_packet_t.
 * 2. Set all the fields of the packet structure
 * 3. Encode the packet structure into an an_packet_t using the appropriate helper function
 * 4. Send the packet
 * 5. Free the packet
 */
void set_filter_options()
{
	an_packet_t* an_packet;
	filter_options_packet_t filter_options_packet;

	/* initialise the structure by setting all the fields to zero */
	memset(&filter_options_packet, 0, sizeof(filter_options_packet_t));

	filter_options_packet.permanent = TRUE;
	filter_options_packet.vehicle_type = vehicle_type_car;
	filter_options_packet.internal_gnss_enabled = TRUE;
	filter_options_packet.atmospheric_altitude_enabled = TRUE;
	filter_options_packet.velocity_heading_enabled = TRUE;
	filter_options_packet.reversing_detection_enabled = TRUE;
	filter_options_packet.motion_analysis_enabled = TRUE;

	an_packet = encode_filter_options_packet(&filter_options_packet);

	an_packet_transmit(an_packet);

	an_packet_free(&an_packet);
}
