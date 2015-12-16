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

#ifndef TRIGGER_H
#define TRIGGER_H

#include "faultcheck_types.h"

class Trigger
{
public:
    Trigger();
    void update();
    void setPermanent();
    void setOnceAfterItr(unsigned long itr);
    void setAfterItr(unsigned long itr);
    void setRandom(double prob);
    void setDuration(int iterations);
    bool isTriggered();

private:
    TRIGGER_TYPE mType;
    bool mIsActive;
    bool mIsTriggered;
    int mIterationsUnilActivation;
    int mDuration;
    double mRandProb;

};

#endif // TRIGGER_H
