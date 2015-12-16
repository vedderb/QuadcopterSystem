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

#include "itsstation.h"
#include "utility.h"
#include <cmath>
#include <QDebug>

namespace {
bool compute_collision(const LDM_ELEMENT_t &element, const LDM_ELEMENT_t &comf_zone, COL_OPP_VECTOR_t &col) {
    const LDM_RISK_ELEMENT_t &risk = element.risk;
    const LDM_RISK_ELEMENT_t &comf_risk = comf_zone.risk;

    const double cosa = cos(risk.rotation);
    const double sina = sin(risk.rotation);
    const double dd = (risk.width + comf_risk.width) * (risk.width + comf_risk.width);
    const double DD = (risk.height + comf_risk.height) * (risk.height + comf_risk.height);

    const double a = pow(cosa * (comf_risk.px - risk.px) + sina * (comf_risk.py - risk.py), 2);
    const double b = pow(sina * (comf_risk.px - risk.px) - cosa * (comf_risk.py - risk.py), 2);
    const double ellipse = (a / dd) + (b / DD);

    if (ellipse < 1.0) {
        // This means that a risk contour collision has occured
        if (element.type == LDM_ELEMENT_TYPE_QUADCOPTER ||
                element.type == LDM_ELEMENT_TYPE_POINT) {
            col.dir.x = (element.pos.px - comf_risk.px);
            col.dir.y = (element.pos.py - comf_risk.py);

            // Compute gain
            col.mag = 2.0 / ellipse; // 2.0 works
        } else if (element.type == LDM_ELEMENT_TYPE_LINE) {
            POINT_t pos_p;
            pos_p.x = comf_risk.px;
            pos_p.y = comf_risk.py;
            POINT_t pc = utility::closest_point_on_segment(element.segment, pos_p);

            col.dir.x = (pc.x - pos_p.x);
            col.dir.y = (pc.y - pos_p.y);
            col.mag = 3.5;
        }

        // Normalize the direction vector
        const double len = sqrt(col.dir.x * col.dir.x + col.dir.y * col.dir.y);
        col.dir.x /= len;
        col.dir.y /= len;

        return true;
    }

    return false;
}
}

ItsStation::ItsStation()
{
    mId = 0;
    mDoOverride = 0;
    mHasCollision = false;
    mGlobalTime = 0;
}

/**
 * @brief ItsStation::runIteration
 * Run an iteration for this ITS station. This will update all risk contours
 * and possibly attempt to take over control of the copter.
 *
 * @param dt_l
 * The time step for this iteration in microseconds.
 *
 * @param pos
 * The position of this copter.
 */
void ItsStation::runIteration(unsigned long dt_l, const POS_STATE_t &pos)
{
    mPos = pos;
    mGlobalTime += dt_l;
    double dt = (double)dt_l / 1000000.0;
    const bool use_simple_pos_advance = true;

    // Update my own comfort zone
    // NOTE: width and height have to be the same
    {
        LDM_ELEMENT_t &comf_zone = mLdm[mId];
        comf_zone.type = LDM_ELEMENT_TYPE_COMFORT_ZONE;
        comf_zone.id = mId;
        comf_zone.pos = mPos;
        comf_zone.risk.px = mPos.px;
        comf_zone.risk.py = mPos.py;
        comf_zone.risk.rotation = mPos.yaw * M_PI / 180.0;
        comf_zone.risk.width = 0.45;
        comf_zone.risk.height = 0.45;
        comf_zone.risk.has_collision = 1;
    }

    const LDM_ELEMENT_t comf_zone_copy = mLdm[mId];

    QVector<COL_OPP_VECTOR_t> collisions;

    QMapIterator<int, LDM_ELEMENT_t> j(mLdm);
    while (j.hasNext()) {
        j.next();
        LDM_ELEMENT_t &element = mLdm[j.key()];
        LDM_RISK_ELEMENT_t &risk = element.risk;

        if (element.type == LDM_ELEMENT_TYPE_QUADCOPTER) {
            // Remove too old elements
            if (fabs((double)element.timestamp - (double)mGlobalTime) > 1000000.0) {
                mLdm.remove(j.key());
            } else {
                if (use_simple_pos_advance) {
                    // Simple velocity-based position advance
                    element.pos.px += element.pos.vx * dt;
                    element.pos.py += element.pos.vy * dt;
                } else {
                    // More accurate but slower position advance
                    const double roll = element.pos.roll * M_PI / 180.0;
                    const double pitch = element.pos.pitch * M_PI / 180.0;
                    const double yaw = element.pos.yaw * M_PI / 180.0;

                    const double acc_v = 9.82;
                    const double cos_y = cos(-yaw);
                    const double sin_y = sin(-yaw);

                    const double dvx = acc_v * tan(roll) * dt;
                    const double dvy = -acc_v * tan(pitch) * dt;

                    element.pos.vx += cos_y * dvx + sin_y * dvy;
                    element.pos.vy += -sin_y * dvx + cos_y * dvy;
                    element.pos.px += element.pos.vx * dt;
                    element.pos.py += element.pos.vy * dt;
                }

                const double dSpeedX = element.pos.vx - mPos.vx;
                const double dSpeedY = element.pos.vy - mPos.vy;
                const double dSpeedTot = sqrt(dSpeedX * dSpeedX + dSpeedY * dSpeedY);
                const double dSpeedAng = atan2(dSpeedY, dSpeedX);
                double baseRad = 1.0;
                const double speedFact = 0.15 * dSpeedTot;

                // Make the risk larger when the time since the last update increases
                //baseRad += 1.0 * (double)(mGlobalTime - element.timestamp) / 1000000.0;

                risk.width = baseRad + dSpeedTot * speedFact;
                risk.height = baseRad + dSpeedTot * speedFact * 0.2;
                risk.rotation = dSpeedAng;

                risk.px = element.pos.px + dSpeedX * speedFact;
                risk.py = element.pos.py + dSpeedY * speedFact;

                COL_OPP_VECTOR_t col;
                if (compute_collision(element, comf_zone_copy, col)) {
                    collisions.append(col);
                    risk.has_collision = 1;
                } else {
                    risk.has_collision = 0;
                }
            }
        } else if (element.type == LDM_ELEMENT_TYPE_LINE) {
            const SEGMENT_t &seg = element.segment;
            POINT_t c;
            c.x = (seg.p1.x + seg.p2.x) / 2;
            c.y = (seg.p1.y + seg.p2.y) / 2;
            const double angle = atan2(seg.p2.y - seg.p1.y, seg.p2.x - seg.p1.x);
            const double len = utility::dist(seg.p1, seg.p2);
            const double speedTot = sqrt(mPos.vx * mPos.vx + mPos.vy * mPos.vy);
            const double baseRad = 1.0;
            const double speedFact = 0.15 * speedTot;

            risk.width = baseRad + len / 1.5 + speedTot * speedFact;
            risk.height = baseRad + speedTot * speedFact;
            risk.rotation = angle;

            risk.px = c.x + (-mPos.vx * speedFact);
            risk.py = c.y + (-mPos.vy * speedFact);

            COL_OPP_VECTOR_t col;
            if (compute_collision(element, comf_zone_copy, col)) {
                collisions.append(col);
                risk.has_collision = 1;
            } else {
                risk.has_collision = 0;
            }
        }
    }

    mHasCollision = collisions.size() > 0;
    mDoOverride = mHasCollision;
    mLdm[mId].risk.has_collision = mHasCollision ? 1 : 0;

    if (mHasCollision) {
        double dx = 0;
        double dy = 0;

        for(QVector<COL_OPP_VECTOR_t>::Iterator it_col = collisions.begin();it_col < collisions.end();it_col++) {
            COL_OPP_VECTOR_t *colInfo = it_col;

            dx += colInfo->dir.x * colInfo->mag;
            dy += colInfo->dir.y * colInfo->mag;
        }

        dx *= 0.2;
        dy *= 0.2;

        const double mag = sqrt(dx * dx + dy * dy);
        if (mag > 1.0) {
            dx /= mag;
            dy /= mag;
        }

        const double siny = sin(mPos.yaw * M_PI / 180.0);
        const double cosy = cos(mPos.yaw * M_PI / 180.0);

        mOverrideControl.rollCmd = -cosy * dx + -siny * dy;
        mOverrideControl.pitchCmd = -siny * dx + cosy * dy;
    }
}

int ItsStation::getMsg(uint8_t *buffer)
{
    ITS_MSG_t msg;
    msg.id = mId;
    msg.state = mPos;
    msg.timestamp = mGlobalTime;

    int len = sizeof(ITS_MSG_t);
    memcpy(buffer, &msg, len);

    return len;
}

/**
 * @brief ItsStation::processMsg
 * This function is called externally with new ITS messages. Update the affected
 * entries in the LDM here.
 *
 * @param msg
 * The ITS message.
 */
void ItsStation::processMsg(uint8_t *buffer)
{
    ITS_MSG_t *msg = (ITS_MSG_t*)buffer;

    if (mLdm.contains(msg->id)) {
        LDM_ELEMENT_t &element = mLdm[msg->id];
        element.pos = msg->state;
        element.timestamp = msg->timestamp;
    } else {
        LDM_ELEMENT_t new_element;
        new_element.pos = msg->state;
        new_element.timestamp = msg->timestamp;
        new_element.id = msg->id;
        new_element.type = LDM_ELEMENT_TYPE_QUADCOPTER;
        mLdm.insert(msg->id, new_element);
    }
}

/**
 * @brief ItsStation::getOverride
 * Get the overridden control value in case we are inside a risk contour
 *
 * @param input
 * The input to the controller that should be updated and used. If override
 * should not be used, this data will not be updated.
 *
 * @return
 * true if override should be used, false otherwise.
 */
bool ItsStation::getOverride(CONTROL_INPUT_t *input)
{
    if (mDoOverride && input != 0) {
        *input = mOverrideControl;
    }

    return mDoOverride;
}


/**
 * @brief ItsStation::syncGlobaltime
 * This function is called externally when ITS messages should be distributed
 * to the other ITS stations.
 *
 * @param time
 * The ITS message to distribute to the other ITS stations.
 */
void ItsStation::syncGlobaltime(uint64_t time)
{
    mGlobalTime = time;
}

void ItsStation::setId(int id)
{
    mId = id;
}

int ItsStation::getId()
{
    return mId;
}

const QMap<int, LDM_ELEMENT_t> &ItsStation::getLdm()
{
    return mLdm;
}

bool ItsStation::hasCollision()
{
    return mHasCollision;
}

void ItsStation::addLineSegment(int id, const SEGMENT_t &segment)
{
    LDM_ELEMENT_t new_element;
    new_element.id = id;
    new_element.type = LDM_ELEMENT_TYPE_LINE;
    new_element.segment = segment;
    mLdm.insert(new_element.id, new_element);
}

void ItsStation::removeAllLineSegments()
{
    QMapIterator<int, LDM_ELEMENT_t> j(mLdm);
    while (j.hasNext()) {
        j.next();
        LDM_ELEMENT_t &element = mLdm[j.key()];

        if (element.type == LDM_ELEMENT_TYPE_LINE) {
            mLdm.remove(j.key());
        }
    }
}
