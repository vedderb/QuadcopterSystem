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

#ifndef COPTERMODEL_H
#define COPTERMODEL_H

#include "datatypes.h"
#include "itsstation.h"
#include <QHash>

class CopterModel
{
public:
    // Functions
    CopterModel(double px = 0.0, double py = 0.0, int id = 0);
    void setPos(POS_STATE_t state);
    const POS_STATE_t &getActualPos();
    const POS_STATE_t &getPerceivedPos();
    void runIteration(unsigned long dt_l);
    void setControlInput(CONTROL_INPUT_t &input);
    double getActualDistToCopter(CopterModel &copter);
    void syncGlobaltime(uint64_t time);
    uint64_t getGlobalTime();
    void correct_pos_anchor(ANCHOR_t &anchor);
    void correct_altitude(double altitude);

    // Public variables
    ItsStation mItsStation;
    POS_CORR_PARAM_t mCorrParam;

private:
    // Variables
    POS_STATE_t mActualPos;
    POS_STATE_t mPerceivedPos;
    CONTROL_INPUT_t mControlInput;
    uint64_t mGlobalTime;
    uint64_t mCorrLastTime;
    QHash<int, ANCHOR_t> mAnchors;

    // Functions
    void updateOrientation(POS_STATE_t &state, CONTROL_INPUT_t input, unsigned long dt_l);
    void updatePosition(POS_STATE_t &state, unsigned long dt_l);
    void updatePositionRK(POS_STATE_t &state, unsigned long dt_l);

};

#endif // COPTERMODEL_H
