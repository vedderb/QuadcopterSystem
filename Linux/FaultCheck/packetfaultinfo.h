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

#ifndef PACKETFAULTINFO_H
#define PACKETFAULTINFO_H

#include "faultcheck_types.h"
#include "trigger.h"
#include "faultcheck.h"

class PacketFaultInfo
{
public:
    PacketFaultInfo();

    PACKET_FAULT_TYPE mType;
    int mPacketsToDrop;
    int mPacketsToRepeat;
    Trigger mTrigger;

    // Corruption
    FaultCheck mFaultCheck;
    int mCorruptionByte;

};

#endif // PACKETFAULTINFO_H
