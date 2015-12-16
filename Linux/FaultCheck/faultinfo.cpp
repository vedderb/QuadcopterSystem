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

#include "faultinfo.h"

FaultInfo::FaultInfo()
{
    mType = FAULT_NONE;
    mIsActive = false;
    mBitToFlip = 0;
    mLastInt = 0;
    mLastDouble = 0;
}

QDataStream &operator<<(QDataStream &out, const FaultInfo &faultinfo) {
    out << faultinfo.mBitToFlip
        << faultinfo.mIsActive
        << faultinfo.mLastDouble
        << faultinfo.mLastInt
        << faultinfo.mScalingFactor
        << faultinfo.mAdditionConstant
// TODO!        << faultinfo.mTrigger
        << static_cast<int>(faultinfo.mType);

    return out;
}

QDataStream &operator>>(QDataStream &in, FaultInfo &faultinfo) {
    int type;

    in >> faultinfo.mBitToFlip
       >> faultinfo.mIsActive
       >> faultinfo.mLastDouble
       >> faultinfo.mLastInt
       >> faultinfo.mScalingFactor
       >> faultinfo.mAdditionConstant
// TODO!       >> trigger
       >> type;

    faultinfo.mType = static_cast<FAULT_TYPE>(type);

    return in;
}
