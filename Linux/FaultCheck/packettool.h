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

#ifndef PACKETTOOL_H
#define PACKETTOOL_H

#include "faultcheck_types.h"
#include "packetfaultinfo.h"
#include "packetinfo.h"
#include <QHash>
#include <QVector>

class PacketTool
{
public:
    PacketTool();
    void addPacket(const QString &identifier, const char *data, int len);
    bool getPacket(const QString &identifier, char *data, int *len);
    void addFaultCorruptionBitFlip(const QString &identifier, int byteIndex, int bitToFlip);
    void addFaultDrop(const QString &identifier, int numPackets);
    void addFaultRepeat(const QString &identifier, int numPackets);
    Trigger* trigger(const QString &identifier);
    void removeAllFaultsIdentifier(const QString &identifier);
    void removeAllFaults();

private:
    QHash<QString, PacketInfo> mStoredPackets;
    QString faultCheckTmpId;

    void addFault(const QString &identifier, PacketFaultInfo info);

};

#endif // PACKETTOOL_H
