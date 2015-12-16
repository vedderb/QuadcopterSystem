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

#include "trigger.h"
#include "utils.h"

Trigger::Trigger()
{
    mType = TRG_PERMANENT;
    mIsActive = false;
    mIsTriggered = false;
    mIterationsUnilActivation = -1;
    mDuration = -1;
    mRandProb = 0.5;
}

void Trigger::update()
{
    mIsTriggered = false;

    if (mDuration != 0) {
        switch(mType) {
        case TRG_PERMANENT:
            mIsTriggered = true;
            break;

        case TRG_AFTER_ITR:
            if (mIterationsUnilActivation > 0) {
                mIterationsUnilActivation--;
            } else if (mIterationsUnilActivation == 0) {
                mIsTriggered = true;
            }
            break;

        case TRG_RANDOM:
            if (utils::randBool(mRandProb)) {
                mIsTriggered = true;
            }
            break;

        case TRG_RANDOM_ONCE:
            if (utils::randBool(mRandProb) && mIterationsUnilActivation > 0) {
                mIsTriggered = true;
                mIterationsUnilActivation = -1;
            }
            break;
        }
    }

    if (mIsTriggered && mDuration > 0) {
        mDuration--;
    }
}

void Trigger::setPermanent()
{
    mType = TRG_PERMANENT;
    mDuration = -1;
}

void Trigger::setOnceAfterItr(unsigned long itr)
{
    mType = TRG_AFTER_ITR;
    mIterationsUnilActivation = itr;
    mDuration = 1;
}

void Trigger::setAfterItr(unsigned long itr)
{
    mType = TRG_AFTER_ITR;
    mIterationsUnilActivation = itr;
    mDuration = -1;
}

void Trigger::setRandom(double prob)
{
    mType = TRG_RANDOM;
    mRandProb = prob;
    mDuration = -1;
}

void Trigger::setDuration(int iterations)
{
    mDuration = iterations;
}

bool Trigger::isTriggered()
{
    return mIsTriggered;
}
