/***************************************************************************//**
 *   @file   no_os_lf256fifo.c
 *   @brief  SPSC lock-free fifo of fixed size (256), specialized for UART.
 *   @author Darius Berghe (darius.berghe@analog.com)
********************************************************************************
 *   @copyright
 * Copyright 2020(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
#include <errno.h>
#include "no_os_lf256fifo.h"

struct lf256fifo {
	uint8_t * data; // pointer to memory area where the buffer will be stored
	uint8_t ffilled; // the index where the data starts (\note Read it as 'first-filled-slot')
	uint8_t fempty; // the index where empty/non-used area starts (\note Read it as 'first-empty-slot')
};

int lf256fifo_init(struct lf256fifo **fifo)
{
	if (fifo == NULL)
		return -EINVAL;

	struct lf256fifo *b = calloc(1, sizeof(struct lf256fifo));
	if (b == NULL)
		return -ENOMEM;

	b->data = calloc(1, 256);
	if (b->data == NULL) {
		free(b);
		return -ENOMEM;
	}

	*fifo = b;

	return 0;
}

bool lf256fifo_is_full(struct lf256fifo *fifo)
{
	return (fifo->fempty + 1) ==
	       fifo->ffilled; // intended overflow at 256 (data size is 256)
}

bool lf256fifo_is_empty(struct lf256fifo *fifo)
{
	return fifo->fempty == fifo->ffilled;
}

int lf256fifo_read(struct lf256fifo * fifo, uint8_t *c)
{
	if (lf256fifo_is_empty(fifo))
		return -1; // buffer empty

	*c = fifo->data[fifo->ffilled];
	fifo->ffilled++; // intended overflow at 256 (data size is 256)

	return 0;
}

int lf256fifo_write(struct lf256fifo *fifo, uint8_t c)
{
	if (lf256fifo_is_full(fifo))
		return -1; // buffer full

	fifo->data[fifo->fempty] = c;
	fifo->fempty++; // intended overflow at 256 (data size is 256)

	return 0; // return success
}

void lf256fifo_flush(struct lf256fifo *fifo)
{
	fifo->ffilled = fifo->fempty;
}
