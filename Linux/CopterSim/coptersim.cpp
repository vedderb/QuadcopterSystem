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

/*
 * Probing faults that can be injected:
 * RangeId%dAnch%d
 * PxId%d
 * PyId%d
 * PitchId%d
 * RollId%d
 * YawId%d
 *
 * Communication faults that can be injected:
 * ITS_DATA%d
 */

#include "coptersim.h"
#include "utility.h"
#include <QUdpSocket>
#include <faultcheck_packet_wrapper.h>
#include <faultcheck_wrapper.h>
#include <cmath>

CopterSim::CopterSim()
{
    mUdpSocket = new QUdpSocket();

    mSelectedCopter = 0;
    mHasRemovedCopters = true;
    mCollisionQuadId = 0;

    resetCollision();
}

CopterSim::~CopterSim()
{
    delete mUdpSocket;
}

void CopterSim::addCopter(double px, double py, int id)
{
    mItsUpdateCnt = 0;
    mAnchUpdateCnt = 0;
    mCopters.insert(id, CopterModel(px, py, id));

    // Add all line segments to this copter
    QMapIterator<int, SEGMENT_t> j(mLineSegments);
    while (j.hasNext()) {
        j.next();
        mCopters[id].mItsStation.addLineSegment(j.key(), j.value());
    }
}

void CopterSim::removeCopter(int id)
{
    mCopters.remove(id);
    mHasRemovedCopters = true;
    mItsUpdateCnt = 0;
    mAnchUpdateCnt = 0;
}

void CopterSim::removeAllCopters()
{
    mCopters.clear();
    mHasRemovedCopters = true;
    mItsUpdateCnt = 0;
    mAnchUpdateCnt = 0;
}

void CopterSim::runIterations(unsigned long num_iterations, unsigned long iteration_len,
                              int its_update_div, int anch_update_div)
{
    if (mCollisionDetected) {
        return;
    }

    const double copter_radius = 0.4;
    uint8_t its_buffer[sizeof(ITS_MSG_t)];
    uint8_t its_buffer2[sizeof(ITS_MSG_t)];

    for (unsigned int i = 0;i < num_iterations;i++) {
        mItsUpdateCnt++;
        if (mItsUpdateCnt >= its_update_div) {
            mItsUpdateCnt = 0;
        }

        mAnchUpdateCnt++;
        if (mAnchUpdateCnt >= anch_update_div) {
            mAnchUpdateCnt = 0;

            mAnchCnt++;
            if (mAnchCnt >= mAnchors.size()) {
                mAnchCnt = 0;
            }
        }

        bool clockFetched = false;
        uint64_t gTime = 0;

        QMapIterator<int, CopterModel> j(mCopters);
        while (j.hasNext()) {
            j.next();
            CopterModel &copter1 = mCopters[j.key()];

            if (!clockFetched) {
                gTime = copter1.getGlobalTime();
                clockFetched = true;
            } else {
                copter1.syncGlobaltime(gTime);
            }

            copter1.runIteration(iteration_len);

            // Line segment collision detection
            POINT_t cp1_p;
            POS_STATE_t cp1_ap = copter1.getActualPos();
            cp1_p.x = cp1_ap.px;
            cp1_p.y = cp1_ap.py;

            QMapIterator<int, SEGMENT_t> si(mLineSegments);
            while (si.hasNext()) {
                si.next();

                POINT_t cp = utility::closest_point_on_segment(si.value(), cp1_p);

                if (utility::dist(cp, cp1_p) < copter_radius) {
                    mCollisionDetected = true;
                    mCollisionQuadId = j.key();
                    mCollPoint = cp;
                }
            }

            // Ranging emulation
            if (mAnchUpdateCnt == 0 && mAnchors.size() > 0) {
                copter1.correct_altitude(0.5);
                copter1.correct_pos_anchor(mAnchors[mAnchCnt]);
            }

            // Copter collision detection and ITS message broadcasting
            copter1.mItsStation.getMsg(its_buffer);
            QMapIterator<int, CopterModel> k(mCopters);
            while (k.hasNext()) {
                k.next();
                CopterModel &copter2 = mCopters[k.key()];

                if (j.key() != k.key()) {
                    if (copter1.getActualDistToCopter(copter2) < (copter_radius * 2)) {
                        mCollisionDetected = true;
                        mCollisionQuadId = j.key();

                        const POS_STATE_t &p1 = copter1.getActualPos();
                        const POS_STATE_t &p2 = copter2.getActualPos();
                        mCollPoint.x = (p1.px + p2.px) / 2.0;
                        mCollPoint.y = (p1.py + p2.py) / 2.0;
                        return;
                    }

                    if (mItsUpdateCnt == 0) {
                        const QString packet_str = QString().sprintf("ITS_DATA%d", k.key());
                        faultcheck_packet_addPacket(packet_str.toLocal8Bit().data(), (char*)its_buffer, sizeof(its_buffer));

                        while (faultcheck_packet_getPacket(packet_str.toLocal8Bit().data(), (char*)its_buffer2, 0)) {
                            copter2.mItsStation.processMsg(its_buffer2);
                        }
                    }
                }
            }
        }
    }
}

bool CopterSim::setControlInput(int copterId, CONTROL_INPUT_t input)
{
    bool retval = false;
    if (mCopters.contains(copterId)) {
        mCopters[copterId].setControlInput(input);
        retval = true;
    }
    return retval;
}

void CopterSim::selectCopter(int id)
{
    mSelectedCopter = id;
}

void CopterSim::sendStateToMap()
{
    uint8_t buffer[256];
    int32_t index;
    const QHostAddress address(QHostAddress::LocalHost);
    const int port = 3001;

    // Possibly clear the already drawn copters
    if (mHasRemovedCopters || mDoResetMap) {
        mHasRemovedCopters = false;
        QByteArray sendData;
        sendData.append((char)CMD_SERVER_QUAD_PACKET);
        sendData.append((char)0);
        sendData.append((char)PACKET_CMD_MAP);
        sendData.append((char)MAP_CMD_REMOVE_ALL_QUADS);
        sendData.append((char)0);
        mUdpSocket->writeDatagram(sendData, address, port);
    }

    // Do a map reset
    if (mDoResetMap) {
        QByteArray sendData;
        sendData.append((char)CMD_SERVER_QUAD_PACKET);
        sendData.append((char)0);
        sendData.append((char)PACKET_CMD_MAP);
        sendData.append((char)MAP_CMD_REMOVE_ALL_COLLISIONS);
        sendData.append((char)0);
        mUdpSocket->writeDatagram(sendData, address, port);

        sendData.clear();
        sendData.append((char)CMD_SERVER_QUAD_PACKET);
        sendData.append((char)0);
        sendData.append((char)PACKET_CMD_MAP);
        sendData.append((char)MAP_CMD_REMOVE_ALL_LINE_SEGMENTS);
        sendData.append((char)0);
        mUdpSocket->writeDatagram(sendData, address, port);

        sendData.clear();
        sendData.append((char)CMD_SERVER_QUAD_PACKET);
        sendData.append((char)0);
        sendData.append((char)PACKET_CMD_MAP);
        sendData.append((char)MAP_CMD_REMOVE_ALL_ANCHORS);
        sendData.append((char)0);
        mUdpSocket->writeDatagram(sendData, address, port);

        // Send all map line segments
        QMapIterator<int, SEGMENT_t> j(mLineSegments);
        while (j.hasNext()) {
            j.next();
            const SEGMENT_t &seg = j.value();

            sendData.clear();
            sendData.append((char)CMD_SERVER_QUAD_PACKET);
            sendData.append((char)0);
            sendData.append((char)PACKET_CMD_LINE_SEGMENT);

            index = 0;

            utility::buffer_append_int16(buffer, j.key(), &index);
            utility::buffer_append_int32(buffer, (int32_t)(seg.p1.x * 10000.0), &index);
            utility::buffer_append_int32(buffer, (int32_t)(seg.p1.y * 10000.0), &index);
            utility::buffer_append_int32(buffer, (int32_t)(seg.p2.x * 10000.0), &index);
            utility::buffer_append_int32(buffer, (int32_t)(seg.p2.y * 10000.0), &index);
            sendData.append((char*)buffer, index);

            mUdpSocket->writeDatagram(sendData, address, port);
        }
        
        // Send all anchors
        sendData.clear();
        sendData.append((char)CMD_SERVER_QUAD_PACKET);
        sendData.append((char)0);
        sendData.append((char)PACKET_CMD_GET_ANCHORS);

        // Map-limits are zero
        index = 0;
        utility::buffer_append_int32(buffer, 0, &index);
        utility::buffer_append_int32(buffer, 0, &index);
        utility::buffer_append_int32(buffer, 0, &index);
        utility::buffer_append_int32(buffer, 0, &index);
        sendData.append((char*)buffer, index);

        for (int i = 0;i < mAnchors.size();i++) {
            sendData.append((char)mAnchors.at(i).id);
            index = 0;
            utility::buffer_append_int32(buffer, (int32_t)(mAnchors.at(i).px * 10000.0), &index);
            utility::buffer_append_int32(buffer, (int32_t)(mAnchors.at(i).py * 10000.0), &index);
            utility::buffer_append_int32(buffer, (int32_t)(mAnchors.at(i).pz * 10000.0), &index);
            sendData.append((char*)buffer, index);
        }

        mUdpSocket->writeDatagram(sendData, address, port);
    }

    // If the selected copter has changed, remove all risk contours
    static int prevSelectedCopter = 0;
    if (prevSelectedCopter != mSelectedCopter || mDoResetMap) {
        QByteArray sendData;
        sendData.append((char)CMD_SERVER_QUAD_PACKET);
        sendData.append((char)0);
        sendData.append((char)PACKET_CMD_MAP);
        sendData.append((char)MAP_CMD_REMOVE_ALL_RISKS);
        sendData.append((char)0);
        mUdpSocket->writeDatagram(sendData, address, port);
    }
    prevSelectedCopter = mSelectedCopter;

    // Clear the reset state for the next iteration
    mDoResetMap = false;

    // Plot the current collision (if any)
    if (mCollisionDetected) {
        QByteArray sendData;
        sendData.append((char)CMD_SERVER_QUAD_PACKET);
        sendData.append((char)0);
        sendData.append((char)PACKET_CMD_GET_POS);

        index = 0;

        utility::buffer_append_int16(buffer, POS_TYPE_COLLISION, &index);
        utility::buffer_append_int32(buffer, (int32_t)(mCollPoint.x * 10000.0), &index);
        utility::buffer_append_int32(buffer, (int32_t)(mCollPoint.y * 10000.0), &index);
        sendData.append((char*)buffer, index);

        mUdpSocket->writeDatagram(sendData, address, port);
    }

    // Plot the copters with their actual position
    QMapIterator<int, CopterModel> j(mCopters);
    while (j.hasNext()) {
        j.next();
        CopterModel &copter = mCopters[j.key()];
        const POS_STATE_t &pos = copter.getActualPos();
        int id = copter.mItsStation.getId();

        QByteArray sendData;
        sendData.append((char)CMD_SERVER_QUAD_PACKET);
        sendData.append((char)id);
        sendData.append((char)PACKET_CMD_GET_POS);

        index = 0;

        int type = POS_TYPE_NORMAL;
        if (id == mSelectedCopter) {
            type |= POS_MASK_FOLLOWING;
        }

        if (copter.mItsStation.hasCollision()) {
            type |= POS_MASK_HAS_COLLISION;
        }

        utility::buffer_append_int16(buffer, type, &index);

        utility::buffer_append_int32(buffer, (int32_t)(pos.px * 10000.0), &index);
        utility::buffer_append_int32(buffer, (int32_t)(pos.py * 10000.0), &index);
        utility::buffer_append_int32(buffer, (int32_t)(pos.vx * 10000.0), &index);
        utility::buffer_append_int32(buffer, (int32_t)(pos.vx * 10000.0), &index);
        utility::buffer_append_int32(buffer, (int32_t)(pos.roll * 100000.0), &index);
        utility::buffer_append_int32(buffer, (int32_t)(pos.pitch * 100000.0), &index);
        utility::buffer_append_int32(buffer, (int32_t)(pos.yaw * 100000.0), &index);

        sendData.append((char*)buffer, index);

        mUdpSocket->writeDatagram(sendData, address, port);
    }

    // Plot all LDM elements for the currently selected copter
    static int prevLdmSize = 0; // To determine if we should send an reset cmd
    if (mCopters.contains(mSelectedCopter)) {
        const QMap<int, LDM_ELEMENT_t> &ldm = mCopters[mSelectedCopter].mItsStation.getLdm();

        if (ldm.size() != prevLdmSize) {
            mDoResetMap = true;
        }

        prevLdmSize = ldm.size();

        QMapIterator<int, LDM_ELEMENT_t> j(ldm);
        while (j.hasNext()) {
            j.next();
            const LDM_ELEMENT_t &element = j.value();
            const LDM_RISK_ELEMENT_t &risk = j.value().risk;

            QByteArray sendData;
            sendData.append((char)CMD_SERVER_QUAD_PACKET);
            sendData.append((char)0);
            sendData.append((char)PACKET_CMD_LDM_ELEMENT);

            index = 0;
            utility::buffer_append_int16(buffer, (int16_t)element.id, &index);
            utility::buffer_append_int16(buffer, (int16_t)element.type, &index);
            buffer[index++] = element.risk.has_collision;
            utility::buffer_append_int32(buffer, (int32_t)(risk.px * 10000.0), &index);
            utility::buffer_append_int32(buffer, (int32_t)(risk.py * 10000.0), &index);
            utility::buffer_append_int32(buffer, (int32_t)(risk.rotation * 100000.0), &index);
            utility::buffer_append_int32(buffer, (int32_t)(risk.width * 10000.0), &index);
            utility::buffer_append_int32(buffer, (int32_t)(risk.height * 10000.0), &index);

            sendData.append((char*)buffer, index);
            mUdpSocket->writeDatagram(sendData, address, port);
        }

        if (!mCollisionDetected) {
            // Send position and velocity error
            QByteArray sendData;
            sendData.append((char)CMD_SERVER_QUAD_PACKET);
            sendData.append((char)0);
            sendData.append((char)PACKET_CMD_SIM);
            sendData.append((char)SIM_CMD_POS_ERROR);

            index = 0;

            CopterModel &cm = mCopters[mSelectedCopter];
            double pxa = cm.getActualPos().px;
            double pya = cm.getActualPos().py;
            double pza = cm.getActualPos().pz;
            double pxp = cm.getPerceivedPos().px;
            double pyp = cm.getPerceivedPos().py;
            double pzp = cm.getPerceivedPos().pz;
            double posError = sqrt(pow(pxa - pxp, 2) + pow(pya - pyp, 2) + pow(pza - pzp, 2));

            double vxa = cm.getActualPos().vx;
            double vya = cm.getActualPos().vy;
            double vxp = cm.getPerceivedPos().vx;
            double vyp = cm.getPerceivedPos().vy;
            double velError = sqrt(pow(vxa - vxp, 2) + pow(vya - vyp, 2));

            utility::buffer_append_int32(buffer, (int32_t)(cm.getGlobalTime() / 1000), &index);
            utility::buffer_append_int32(buffer, (int32_t)(posError * 100000.0), &index);
            utility::buffer_append_int32(buffer, (int32_t)(velError * 100000.0), &index);
            sendData.append((char*)buffer, index);

            mUdpSocket->writeDatagram(sendData, address, port);

            // Send active faults
            sendData.clear();
            sendData.append((char)CMD_SERVER_QUAD_PACKET);
            sendData.append((char)0);
            sendData.append((char)PACKET_CMD_SIM);
            sendData.append((char)SIM_CMD_FAULTS);

            index = 0;

            utility::buffer_append_int32(buffer, (int32_t)(cm.getGlobalTime() / 1000), &index);
            QString str;
            str.sprintf("PxId%d", cm.mItsStation.getId());
            buffer[index++] = faultcheck_numActiveFaults(str.toLocal8Bit().data());
            str.sprintf("PyId%d", cm.mItsStation.getId());
            buffer[index++] = faultcheck_numActiveFaults(str.toLocal8Bit().data());
            str.sprintf("RollId%d", cm.mItsStation.getId());
            buffer[index++] = faultcheck_numActiveFaults(str.toLocal8Bit().data());
            str.sprintf("PitchId%d", cm.mItsStation.getId());
            buffer[index++] = faultcheck_numActiveFaults(str.toLocal8Bit().data());
            str.sprintf("YawId%d", cm.mItsStation.getId());
            buffer[index++] = faultcheck_numActiveFaults(str.toLocal8Bit().data());
            str.sprintf("RangeId%dAnch0", cm.mItsStation.getId());
            buffer[index++] = faultcheck_numActiveFaults(str.toLocal8Bit().data());
            buffer[index++] = 0;
            sendData.append((char*)buffer, index);

            mUdpSocket->writeDatagram(sendData, address, port);
        }
    }
}

CopterModel *CopterSim::getCopter(int id)
{
    if (mCopters.contains(id)) {
        return &mCopters[id];
    } else {
        return 0;
    }
}

bool CopterSim::collisionOccured()
{
    return mCollisionDetected;
}

POINT_t CopterSim::collisionPoint()
{
    return mCollPoint;
}

void CopterSim::resetCollision()
{
    mItsUpdateCnt = 0;
    mAnchUpdateCnt = 0;
    mCollisionDetected = false;
    mDoResetMap = true;
    mAnchCnt = 0;
}

void CopterSim::stopSimulation()
{
    mCollisionDetected = true;
}

void CopterSim::addLineSegment(int id, double p1x, double p1y, double p2x, double p2y)
{
    SEGMENT_t seg;
    seg.p1.x = p1x;
    seg.p1.y = p1y;
    seg.p2.x = p2x;
    seg.p2.y = p2y;
    mLineSegments.insert(id, seg);

    QMapIterator<int, CopterModel> cItr(mCopters);
    while (cItr.hasNext()) {
        cItr.next();
        CopterModel &copter = mCopters[cItr.key()];

        copter.mItsStation.removeAllLineSegments();

        // Add all line segments to this copter
        QMapIterator<int, SEGMENT_t> lItr(mLineSegments);
        while (lItr.hasNext()) {
            lItr.next();
            copter.mItsStation.addLineSegment(lItr.key(), lItr.value());
        }
    }

    mDoResetMap = true;
}

void CopterSim::removeAllLineSegments()
{
    mLineSegments.clear();

    QMapIterator<int, CopterModel> cItr(mCopters);
    while (cItr.hasNext()) {
        cItr.next();
        mCopters[cItr.key()].mItsStation.removeAllLineSegments();
    }

    mDoResetMap = true;
}

void CopterSim::addAnchor(int id, double px, double py, double pz)
{
    ANCHOR_t anch;
    anch.id = id;
    anch.px = px;
    anch.py = py;
    anch.pz = pz;
    mAnchors.append(anch);
    mDoResetMap = true;
}

void CopterSim::removeAllAnchors()
{
    mAnchors.clear();
    mDoResetMap = true;
}
