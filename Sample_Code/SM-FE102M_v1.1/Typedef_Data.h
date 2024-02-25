// Typedef_Data.h

#ifndef _TYPEDEF_DATA_h
#define _TYPEDEF_DATA_h

#include "Arduino.h"

typedef union {
	float	fl;
	uint8_t   Byte[8];
	struct {
		uint32_t	Long0;
		uint32_t	Long1;
	};
	struct {
		uint16_t	Word0;
		uint16_t	Word1;
		uint16_t	Word2;
		uint16_t	Word3;
	};
	struct {
		uint8_t	    Byte0;
		uint8_t	    Byte1;
		uint8_t	    Byte2;
		uint8_t	    Byte3;
		uint8_t	    Byte4;
		uint8_t	    Byte5;
		uint8_t	    Byte6;
		uint8_t	    Byte7;
	};
} t_LONG_LONG;

typedef union {
	long	l;
	uint32_t  uDWord;
	uint8_t   Byte[4];
	struct {
		uint16_t	Word0;
		uint16_t	Word1;
	};
	struct {
		uint8_t	Byte0;
		uint8_t	Byte1;
		uint8_t	Byte2;
		uint8_t	Byte3;
	};
} t_LONG;

typedef union {
	int		    i;
	uint16_t	uWord;
	struct {
		uint8_t	Byte0;
		uint8_t	Byte1;
	};
} t_SHORT;

typedef union {
	uint8_t	b;
	struct {
		uint8_t bit0 : 1;
		uint8_t bit1 : 1;
		uint8_t bit2 : 1;
		uint8_t bit3 : 1;
		uint8_t bit4 : 1;
		uint8_t bit5 : 1;
		uint8_t bit6 : 1;
		uint8_t bit7 : 1;
	};
} t_CHARUNION;


#endif

