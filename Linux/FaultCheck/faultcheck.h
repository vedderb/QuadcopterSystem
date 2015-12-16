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

#ifndef FAULTCHECK_H
#define FAULTCHECK_H

#include "FaultCheck_global.h"
#include <QHash>
#include "faultinfo.h"

class FAULTCHECKSHARED_EXPORT FaultCheck {

public:
    FaultCheck();
    ~FaultCheck();
    void addFaultBitFlip(const QString &identifier, int bitToFlip);
    void addFaultBitFlipRandom(const QString &identifier);
    void addFaultAmplification(const QString &identifier, double factor);
    void addFaultOffset(const QString &identifier, double constant);
    void addFaultSetTo(const QString &identifier, double value);
    Trigger* trigger(const QString &identifier);
    void activateFault(const QString &identifier, bool isActive);
    void removeAllFaultsIdentifier(const QString &identifier);
    void removeAllFaults();
    void injectFaultInt(const QString &identifier, int *integer);
    void injectFaultDouble(const QString &identifier, double *number);
    int numActiveFaults(const QString &identifier);
    QStringList getIdentifiers();

private:
    FaultInfo* getNextFault(const QString &identifier);
    void resetFaultGetter();
    FaultInfo* getLatestAddedFault(const QString &identifier);
    void addFault(const QString &identifier, FaultInfo info);
    int randInt(int low, int high);
    double randZeroToOne();
    bool randBool(double probTrue);

    QHash<QString, QList<FaultInfo> > mStoredFaults;
    int mLastIdentifierIndex;
};

#endif // FAULTCHECK_H
