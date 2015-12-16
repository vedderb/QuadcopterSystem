/*
    Copyright 2013 - 2015 Benjamin Vedder	benjamin@vedder.se

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef MAPWIDGET_H
#define MAPWIDGET_H

#include <QWidget>
#include <QPainter>
#include <QPaintEvent>
#include <QBrush>
#include <QFont>
#include <QPen>
#include <QPalette>
#include <QVector>
#include <QInputDialog>

#include "locpoint.h"
#include "objectinfo.h"
#include "perspectivepixmap.h"
#include "datatypes.h"

class MapWidget : public QWidget
{
    Q_OBJECT
public:
    explicit MapWidget(QWidget *parent = 0);
    ObjectInfo* getQuadInfo(int id);
    ObjectInfo* getAnchorInfo(int anchor);
    LDM_ELEMENT_t* getLdmElement(int id);
    void setFollowQuad(int quad);
    void addQuad(int id, ObjectInfo quad);
    void removeQuad(int id);
    void removeAllQuads();
    void addCollision(int id, ObjectInfo collision);
    void removeCollision(int id);
    void removeAllCollisions();
    void addLdmElement(LDM_ELEMENT_t element);
    void removeLdmElement(int id);
    void removeAllLddmElements();
    void addAnchor(ObjectInfo anchor);
    void removeAnchor(int id);
    void removeAllAnchors();
    void addLineSegment(int id, SEGMENT_t lineSegment);
    void removeAllLineSegments();
    void setScaleFactor(double scale);
    void setRotation(double rotation);
    void setXOffset(double offset);
    void setYOffset(double offset);
    void clearTrace();
    void addPerspectivePixmap(PerspectivePixmap map);
    void clearPerspectivePixmaps();
    QPoint getMousePosRelative();
    void setDrawTrace(bool drawTrace);
    bool getDrawTrace();

Q_SIGNALS:
    void scaleChanged(double newScale);
    void offsetChanged(double newXOffset, double newYOffset);
    void posSet(LocPoint pos);

public Q_SLOTS:

protected:
    void paintEvent(QPaintEvent *event);
    void mouseMoveEvent (QMouseEvent * e);
    void mousePressEvent(QMouseEvent * e);
    void mouseReleaseEvent(QMouseEvent * e);
    void wheelEvent(QWheelEvent * e);

private:
    QHash<int, ObjectInfo> mQuadInfo;
    QHash<int, ObjectInfo> mCollisionInfo;
    QHash<int, LDM_ELEMENT_t> mLdmElements;
    QHash<int, SEGMENT_t> mLineSegments;
    QVector<ObjectInfo> mAnchorInfo;
    QVector<QPointF> mQuadTrace;
    QVector<PerspectivePixmap> mPerspectivePixmaps;
    double mScaleFactor;
    double mRotation;
    double mXOffset;
    double mYOffset;
    int mMouseLastX;
    int mMouseLastY;
    int mFollowQuad;
    double mXRealPos;
    double mYRealPos;
    bool mDrawTrace;
};

#endif // MAPWIDGET_H
