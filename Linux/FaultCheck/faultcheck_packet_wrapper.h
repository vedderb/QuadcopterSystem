/*
    Copyright 2014 Benjamin Vedder	benjamin.vedder@sp.se

    This file is part of FaultCheck.

    FaultCheck is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    FaultCheck is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with FaultCheck.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef FAULTCHECK_PACKET_WRAPPER_H
#define FAULTCHECK_PACKET_WRAPPER_H

#include "faultcheck_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void faultcheck_packet_addPacket(const char* identifier, const char* data, int len);
int faultcheck_packet_getPacket(const char* identifier, char *data, int *len);
void faultcheck_packet_addFaultCorruptionBitFlip(const char *identifier, int byteIndex, int bitToFlip);
void faultcheck_packet_addFaultDrop(const char *identifier, int numPackets);
void faultcheck_packet_addFaultRepeat(const char *identifier, int numPackets);
void faultcheck_packet_setTriggerOnceAfterIterations(const char *identifier, unsigned long iterations);
void faultcheck_packet_setTriggerAfterIterations(const char *identifier, unsigned long iterations);
void faultcheck_packet_setDurationAfterTrigger(const char *identifier, int iterations);
void faultcheck_packet_removeAllFaultsIdentifier(const char *identifier);
void faultcheck_packet_removeAllFaults();

#ifdef __cplusplus
}
#endif

#endif // FAULTCHECK_PACKET_WRAPPER_H
