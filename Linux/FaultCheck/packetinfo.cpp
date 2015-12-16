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

#include "packetinfo.h"

PacketInfo::PacketInfo()
{
    mLastFaultIndex = 0;
}

PacketFaultInfo *PacketInfo::getNextFault()
{
    if (mLastFaultIndex >= mFaults.size()) {
        return 0;
    }

    PacketFaultInfo *info = &mFaults[mLastFaultIndex++];
    info->mTrigger.update();

    return info;
}

PacketFaultInfo *PacketInfo::getLatestAddedFault()
{
    PacketFaultInfo *retVal = 0;

    if (mFaults.size() > 0) {
        retVal = &mFaults[mFaults.size() - 1];
    }

    return retVal;
}

void PacketInfo::resetFaultGetter()
{
    mLastFaultIndex = 0;
}
