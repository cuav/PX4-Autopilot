/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
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
 * @file hello_main.cpp
 * Example for Linux
 *
 * @author Mark Charlebois <charlebm@gmail.com>
 */

#include "hello_example.h"

#include <px4_platform_common/app.h>
#include <px4_platform_common/init.h>
#include <stdio.h>


static uint32_t sram4_buf[2] locate_data(".sram4");

uint32_t read_address_data(uint32_t address)
{
	return *(uint32_t *)address;
}

int PX4_MAIN(int argc, char **argv)
{
	px4::init(argc, argv, "hello");

	printf("hello\n");
	//HelloExample hello;
	//hello.main();

	printf("/************** test start **************/\n");

	printf("address %x val: %x \n", &sram4_buf[0], sram4_buf[0]);
	printf("address %x val: %x \n", &sram4_buf[1], sram4_buf[1]);
	uint32_t dd = 0x9876;

	uint32_t address = 0x38000000;

	for (uint32_t i = 0; i < 1024 / 4; i++) {
		if (i % 16 == 0) {
			printf("\n0x%x :", address);
		}

		uint32_t val = read_address_data(address);
		printf("%08x ", val);
		address += 4;
	}

	printf("\n");

	sram4_buf[0] = 1222;
	sram4_buf[1] = 4567;

	printf("address %x val: %d \n", &sram4_buf[0], sram4_buf[0]);
	printf("address %x val: %d \n", &sram4_buf[1], sram4_buf[1]);

	printf("dd address %x val: %x \n", &dd, dd);

	printf("/************** test end **************/\n");

	printf("goodbye\n");
	return 0;
}
