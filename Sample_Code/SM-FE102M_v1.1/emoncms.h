/*
   -------------------------------------------------------------------
   EmonESP Serial to Emoncms gateway
   -------------------------------------------------------------------
   Adaptation of Chris Howells OpenEVSE ESP Wifi
   by Trystan Lea, Glyn Hudson, OpenEnergyMonitor

   Modified to use with the CircuitSetup.us Split Phase Energy Meter by jdeglavina

   All adaptation GNU General Public License as below.

   -------------------------------------------------------------------

   This file is part of OpenEnergyMonitor.org project.
   EmonESP is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3, or (at your option)
   any later version.
   EmonESP is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with EmonESP; see the file COPYING.  If not, write to the
   Free Software Foundation, Inc., 59 Temple Place - Suite 330,
   Boston, MA 02111-1307, USA.
*/

#ifndef _EMONESP_EMONCMS_H
#define _EMONESP_EMONCMS_H

#include <Arduino.h>

// -------------------------------------------------------------------
// Commutication with EmonCMS
// -------------------------------------------------------------------

extern boolean emoncms_connected;
extern unsigned long packets_sent;
extern unsigned long packets_success;

// -------------------------------------------------------------------
// Publish values to EmonCMS
//
// data: a comma seperated list of name:value pairs to send
// -------------------------------------------------------------------
void emoncms_publish(const char * data);

#endif // _EMONESP_EMONCMS_H
