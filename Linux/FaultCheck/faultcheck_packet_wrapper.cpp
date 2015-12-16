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

#include "faultcheck_packet_wrapper.h"
#include "packettool.h"

static PacketTool packetTool;

void faultcheck_packet_addPacket(const char* identifier, const char *data, int len) {
    packetTool.addPacket(identifier, data, len);
}

int faultcheck_packet_getPacket(const char* identifier, char *data, int *len) {
    return packetTool.getPacket(identifier, data, len);
}

void faultcheck_packet_addFaultCorruptionBitFlip(const char* identifier, int byteIndex, int bitToFlip) {
    packetTool.addFaultCorruptionBitFlip(identifier, byteIndex, bitToFlip);
}

void faultcheck_packet_addFaultDrop(const char *identifier, int numPackets) {
    packetTool.addFaultDrop(identifier, numPackets);
}

void faultcheck_packet_addFaultRepeat(const char *identifier, int numPackets) {
    packetTool.addFaultRepeat(identifier, numPackets);
}

void faultcheck_packet_setTriggerOnceAfterIterations(const char* identifier, unsigned long iterations) {
    Trigger *trg = packetTool.trigger(identifier);

    if (trg) {
        trg->setOnceAfterItr(iterations);
    }
}

void faultcheck_packet_setTriggerAfterIterations(const char *identifier, unsigned long iterations) {
    Trigger *trg = packetTool.trigger(identifier);

    if (trg) {
        trg->setAfterItr(iterations);
    }
}

void faultcheck_packet_setDurationAfterTrigger(const char *identifier, int iterations) {
    Trigger *trg = packetTool.trigger(identifier);

    if (trg) {
        trg->setDuration(iterations);
    }
}

void faultcheck_packet_removeAllFaultsIdentifier(const char *identifier)
{
    packetTool.removeAllFaultsIdentifier(identifier);
}

void faultcheck_packet_removeAllFaults()
{
    packetTool.removeAllFaults();
}
