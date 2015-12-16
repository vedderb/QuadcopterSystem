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

#include "packettool.h"
#include <QTime>

PacketTool::PacketTool()
{
    faultCheckTmpId = "A";
    QTime now = QTime::currentTime();
    qsrand(now.msec());
}

void PacketTool::addPacket(const QString &identifier, const char *data, int len)
{
    QByteArray dataArray;
    dataArray.append(data, len);

    if (mStoredPackets.contains(identifier)) {
        mStoredPackets[identifier].mPackets.append(dataArray);
    } else {
        PacketInfo packet;
        packet.mPackets.append(dataArray);
        mStoredPackets.insert(identifier, packet);
    }
}

bool PacketTool::getPacket(const QString &identifier, char *data, int *len)
{
    QByteArray toSend, toSendNoFault;
    int injTmp;
    bool hasSendData = false;

    if (!mStoredPackets.contains(identifier)) {
        return false;
    } else {
        PacketInfo *packet = &mStoredPackets[identifier];

        packet->resetFaultGetter();
        PacketFaultInfo *info = packet->getNextFault();

        // Read the first packet and remove it from the buffer
        if (packet->mPackets.size()) {
            toSend = packet->mPackets.first();
            toSendNoFault = toSend;
            packet->mPackets.remove(0);
            hasSendData = true;
        }

        // Apply all faults for this identifier
        while (info) {
            if (info->mTrigger.isTriggered()) {
                switch(info->mType) {
                case PACKET_FAULT_NONE:
                    // Do nothing
                    break;

                case PACKET_FAULT_DROP:
                    while ((info->mPacketsToDrop > 0) && hasSendData) {
                        info->mPacketsToDrop--;

                        if (packet->mPackets.size()) {
                            toSend = packet->mPackets.first();
                            packet->mPackets.remove(0);
                            hasSendData = true;
                        } else {
                            hasSendData = false;
                        }
                    }
                    break;

                case PACKET_FAULT_REPETITION:
                    // If there are packets, put the current one back into the
                    // send buffer mPacketsToRepeat of times
                    while ((info->mPacketsToRepeat > 0) && hasSendData) {
                        info->mPacketsToRepeat--;
                        packet->mPackets.insert(0, toSendNoFault);
                    }
                    break;

                case PACKET_FAULT_CURRUPTION:
                    injTmp = toSend[info->mCorruptionByte];
                    info->mFaultCheck.injectFaultInt(faultCheckTmpId, &injTmp);
                    toSend[info->mCorruptionByte] = injTmp;
                    break;

                }
            }

            info = packet->getNextFault();
        }
    }

    if (hasSendData) {
        memcpy(data, toSend.data(), toSend.size());
        if (len) {
            *len = toSend.size();
        }
        return true;
    }

    return false;
}

void PacketTool::addFaultCorruptionBitFlip(const QString &identifier, int byteIndex, int bitToFlip)
{
    PacketFaultInfo info;
    info.mType = PACKET_FAULT_CURRUPTION;
    info.mCorruptionByte = byteIndex;
    info.mFaultCheck.addFaultBitFlip(faultCheckTmpId, bitToFlip);
    addFault(identifier, info);
}

void PacketTool::addFaultDrop(const QString &identifier, int numPackets)
{
    PacketFaultInfo info;
    info.mType = PACKET_FAULT_DROP;
    info.mPacketsToDrop = numPackets;
    addFault(identifier, info);
}

void PacketTool::addFaultRepeat(const QString &identifier, int numPackets)
{
    PacketFaultInfo info;
    info.mType = PACKET_FAULT_REPETITION;
    info.mPacketsToRepeat = numPackets;
    addFault(identifier, info);
}

Trigger *PacketTool::trigger(const QString &identifier)
{
    if (mStoredPackets.contains(identifier)) {
        PacketFaultInfo *info = mStoredPackets[identifier].getLatestAddedFault();

        if (info) {
            return &(info->mTrigger);
        }
    }

    return NULL;
}

void PacketTool::removeAllFaultsIdentifier(const QString &identifier)
{
    if (mStoredPackets.contains(identifier)) {
        mStoredPackets[identifier].mFaults.clear();
    }
}

void PacketTool::removeAllFaults()
{
    QHashIterator<QString, PacketInfo> i(mStoredPackets);
    while (i.hasNext()) {
        i.next();
        mStoredPackets[i.key()].mFaults.clear();
    }
}

// =============== Private methods =================

void PacketTool::addFault(const QString &identifier, PacketFaultInfo info)
{
    if (mStoredPackets.contains(identifier)) {
        mStoredPackets[identifier].mFaults.append(info);
    } else {
        PacketInfo newInfo;
        newInfo.mFaults.append(info);
        mStoredPackets.insert(identifier, newInfo);
    }
}
