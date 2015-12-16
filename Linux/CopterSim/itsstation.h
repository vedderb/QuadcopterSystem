/*
    Copyright 2014 Benjamin Vedder	benjamin.vedder@sp.se

    This file is part of CopterSim.

    CopterSim is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    CopterSim is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with CopterSim.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef ITSSTATION_H
#define ITSSTATION_H

#include "datatypes.h"
#include <QMap>

/**
 * Note that this class is written so that it can be adapted to C easily.
 */

class ItsStation
{
public:
    ItsStation();
    void runIteration(unsigned long dt_l, const POS_STATE_t &pos);
    int getMsg(uint8_t *buffer);
    void processMsg(uint8_t *buffer);
    bool getOverride(CONTROL_INPUT_t *input);
    void syncGlobaltime(uint64_t time);
    void setId(int id);
    int getId();
    const QMap<int, LDM_ELEMENT_t> &getLdm();
    bool hasCollision();
    void addLineSegment(int id, const SEGMENT_t &segment);
    void removeAllLineSegments();

private:
    uint64_t mGlobalTime;
    int mId;
    POS_STATE_t mPos;
    CONTROL_INPUT_t mOverrideControl;
    int mDoOverride;
    QMap<int, LDM_ELEMENT_t> mLdm;
    bool mHasCollision;

};

#endif // ITSSTATION_H
