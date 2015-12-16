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

#ifndef FAULTINFO_H
#define FAULTINFO_H

#include "faultcheck_types.h"
#include <QDataStream>
#include "trigger.h"

class FaultInfo
{
public:
    FaultInfo();

    FAULT_TYPE mType;
    Trigger mTrigger;
    bool mIsActive;
    int mBitToFlip;
    int mLastInt;
    double mLastDouble;
    double mScalingFactor;
    double mAdditionConstant;
};

QDataStream &operator<<(QDataStream &out, const FaultInfo &faultinfo);
QDataStream &operator>>(QDataStream &in, FaultInfo &painting);

#endif // FAULTINFO_H
