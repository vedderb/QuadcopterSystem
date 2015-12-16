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

#ifndef COPTERSIM_H
#define COPTERSIM_H

#include <QMap>
#include <QVector>
#include "coptersim_global.h"
#include "coptermodel.h"
#include "itsstation.h"

class COPTERSIMSHARED_EXPORT CopterSim
{

public:
    CopterSim();
    ~CopterSim();
    void addCopter(double px, double py, int id);
    void removeCopter(int id);
    void removeAllCopters();
    void runIterations(unsigned long num_iterations, unsigned long iteration_len,
                                  int its_update_div, int anch_update_div);
    bool setControlInput(int copterId, CONTROL_INPUT_t input);
    void selectCopter(int id);
    void sendStateToMap();
    CopterModel* getCopter(int id);
    bool collisionOccured();
    POINT_t collisionPoint();
    void resetCollision();
    void stopSimulation();
    void addLineSegment(int id, double p1x, double p1y, double p2x, double p2y);
    void removeAllLineSegments();
    void addAnchor(int id, double px, double py, double pz);
    void removeAllAnchors();

private:
    class QUdpSocket *mUdpSocket;
    QMap<int, CopterModel> mCopters;
    QMap<int, SEGMENT_t> mLineSegments;
    int mItsUpdateCnt;
    int mAnchUpdateCnt;
    bool mCollisionDetected;
    POINT_t mCollPoint;
    int mSelectedCopter;
    bool mHasRemovedCopters;
    int mCollisionQuadId;
    bool mDoResetMap;
    int mAnchCnt;
    QList<ANCHOR_t> mAnchors;

};

#endif // COPTERSIM_H
