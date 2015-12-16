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

#include "faultcheck.h"
#include <QTime>
#include <QDebug>
#include "utils.h"

FaultCheck::FaultCheck()
{
    mStoredFaults.clear();
    mLastIdentifierIndex = 0;

    QTime now = QTime::currentTime();
    qsrand(now.msec());
}

FaultCheck::~FaultCheck()
{

}

void FaultCheck::addFaultBitFlip(const QString &identifier, int bitToFlip)
{
    FaultInfo info;
    info.mIsActive = true;
    info.mType = FAULT_BITFLIP;
    info.mBitToFlip = bitToFlip;
    addFault(identifier, info);
}

void FaultCheck::addFaultBitFlipRandom(const QString &identifier)
{
    FaultInfo info;
    info.mIsActive = true;
    info.mType = FAULT_BITFLIP_RANDOM;
    addFault(identifier, info);
}

void FaultCheck::addFaultAmplification(const QString &identifier, double factor)
{
    FaultInfo info;
    info.mIsActive = true;
    info.mType = FAULT_AMPLIFICATION;
    info.mScalingFactor = factor;
    addFault(identifier, info);
}

void FaultCheck::addFaultOffset(const QString &identifier, double constant)
{
    FaultInfo info;
    info.mIsActive = true;
    info.mType = FAULT_OFFSET;
    info.mAdditionConstant = constant;
    addFault(identifier, info);
}

void FaultCheck::addFaultSetTo(const QString &identifier, double value)
{
    FaultInfo info;
    info.mIsActive = true;
    info.mType = FAULT_SET_TO;
    info.mLastDouble = value;
    addFault(identifier, info);
}

/**
 * @brief FaultCheck::trigger
 * Get the trigger object for a fault
 * @param identifier
 * The identifier for the fault
 * @return
 * The last added fault for this identifier, or
 * 0 if no fault has been added.
 */
Trigger *FaultCheck::trigger(const QString &identifier)
{
    FaultInfo *info = getLatestAddedFault(identifier);

    if (info) {
        return &(info->mTrigger);
    }

    return NULL;
}

void FaultCheck::activateFault(const QString &identifier, bool isActive)
{
    if (mStoredFaults.contains(identifier)) {
        QList<FaultInfo> &infoVector = mStoredFaults[identifier];

        for (int i = 0; i < infoVector.size(); ++i) {
            infoVector[i].mIsActive = isActive;
        }
    }
}

void FaultCheck::removeAllFaultsIdentifier(const QString &identifier)
{
    mStoredFaults.remove(identifier);
}

void FaultCheck::removeAllFaults()
{
    mStoredFaults.clear();
}

void FaultCheck::injectFaultInt(const QString &identifier, int *integer)
{
    int ind;
    FaultInfo *info;
    bool isStuck = false;

    resetFaultGetter();
    info = getNextFault(identifier);

    // Apply all faults for this identifier
    while (info) {
        if (info->mTrigger.isTriggered()) {
            if (info->mType == FAULT_STUCK_TO_CURRENT) {
                isStuck = true;
            }

            if (!isStuck) {
                info->mLastInt = *integer;
            }

            switch(info->mType) {
            case FAULT_NONE:
                // Do nothing
                break;

            case FAULT_BITFLIP:
                *integer ^= 1 << info->mBitToFlip;
                break;

            case FAULT_BITFLIP_RANDOM:
                ind = utils::randInt(0, 31);
                *integer ^= 1 << ind;
                break;

            case FAULT_STUCK_TO_CURRENT:
                *integer = info->mLastInt;
                break;

            case FAULT_AMPLIFICATION:
                *integer = (int)((double)*integer * info->mScalingFactor);
                break;

            case FAULT_OFFSET:
                *integer = (int)((double)*integer + info->mAdditionConstant);
                break;

            case FAULT_SET_TO:
                *integer = (int)info->mLastDouble;
                break;
            }
        }

        info = getNextFault(identifier);
    }
}

void FaultCheck::injectFaultDouble(const QString &identifier, double *number)
{
    int ind;
    FaultInfo *info;
    bool isStuck = false;

    union {
        double db;
        long long lng;
    };

    db = *number;

    resetFaultGetter();
    info = getNextFault(identifier);

    // Apply all faults for this identifier
    while (info) {
        if (info->mTrigger.isTriggered()) {
            if (info->mType == FAULT_STUCK_TO_CURRENT) {
                isStuck = true;
            }

            if (info->mType == FAULT_SET_TO) {
                isStuck = true;
            }

            if (!isStuck) {
                info->mLastDouble = db;
            }

            switch(info->mType) {
            case FAULT_NONE:
                // Do nothing
                break;

            case FAULT_BITFLIP:
                lng ^= 1 << info->mBitToFlip;
                break;

            case FAULT_BITFLIP_RANDOM:
                ind = utils::randInt(0, 31);
                lng ^= 1 << ind;
                break;

            case FAULT_STUCK_TO_CURRENT:
                db = info->mLastDouble;
                break;

            case FAULT_AMPLIFICATION:
                db *= info->mScalingFactor;
                break;

            case FAULT_OFFSET:
                db += info->mAdditionConstant;
                break;

            case FAULT_SET_TO:
                db = info->mLastDouble;
                break;
            }

            *number = db;
        }

        info = getNextFault(identifier);
    }
}

int FaultCheck::numActiveFaults(const QString &identifier)
{
    int num = 0;

    if (mStoredFaults.contains(identifier)) {
        QList<FaultInfo> &faults = mStoredFaults[identifier];

        for (int i = 0;i < faults.size();i++) {
            if (faults[i].mTrigger.isTriggered() && faults[i].mIsActive) {
                num++;
            }
        }
    }

    return num;
}

QStringList FaultCheck::getIdentifiers()
{
    QStringList ids;

    QHashIterator<QString, QList<FaultInfo> > i(mStoredFaults);
    while (i.hasNext()) {
        i.next();
        ids.append(i.key());
    }

    return ids;
}

// =============== Private methods =================

/**
  * Returns a pointer to the fault object and updates its trigger.
  * Will return a new FaultInfo as long as there are faults left for this
  * identifier.
  */
FaultInfo* FaultCheck::getNextFault(const QString &identifier)
{
    if (!mStoredFaults.contains(identifier)) {
        FaultInfo info;
        info.mIsActive = false;
        info.mType = FAULT_NONE;
        addFault(identifier, info);
        return 0;
    }

    QList<FaultInfo> &infoVector = mStoredFaults[identifier];

    // Check if there is any element left in the FaultInfo vector for identifier
    if (mLastIdentifierIndex >= infoVector.size()) {
        return 0;
    }

    FaultInfo *info = &infoVector[mLastIdentifierIndex++];

    // Update the trigger
    info->mTrigger.update();

    return info;
}

void FaultCheck::resetFaultGetter()
{
    mLastIdentifierIndex = 0;
}

FaultInfo *FaultCheck::getLatestAddedFault(const QString &identifier)
{
    FaultInfo *retVal = 0;

    if (mStoredFaults.contains(identifier)) {
        QList<FaultInfo> &infoVector = mStoredFaults[identifier];
        if (infoVector.size() > 0) {
            retVal = &infoVector[infoVector.size() - 1];
        }
    }

    return retVal;
}

void FaultCheck::addFault(const QString &identifier, FaultInfo info)
{
    if (mStoredFaults.contains(identifier)) {
        mStoredFaults[identifier].append(info);
    } else {
        QList<FaultInfo> newVec;
        newVec.append(info);
        mStoredFaults.insert(identifier, newVec);
    }
}
