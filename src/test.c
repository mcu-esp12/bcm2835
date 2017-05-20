// Test program for bcm2835 library
// You can only expect this to run correctly
// as root on Raspberry Pi hardware, but it will compile and run with little effect
// on other hardware
//
// Author: Mike McCauley
// Copyright (C) 2011-2013 Mike McCauley
// $Id: test.c,v 1.3 2012/11/29 01:39:33 mikem Exp $

#include <bcm2835.h>
#include <stdio.h>

int main(int argc, char **argv)
{
    if (geteuid() == 0)
    {
	if (!bcm2835_init())
	    return 1;
	if (!bcm2835_close())
	    return 1;
    }
    else
    {
	fprintf(stderr, "****You need to be root to properly run this test program\n");
    }
    return 0;
}
