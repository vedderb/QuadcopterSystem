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

#include <QDebug>
#include <math.h>
#include <qmath.h>

#include "mapwidget.h"

MapWidget::MapWidget(QWidget *parent) :
    QWidget(parent)
{
    mScaleFactor = 0.3;
    mRotation = 0;
    mXOffset = 0;
    mYOffset = 0;
    mMouseLastX = 1000000;
    mMouseLastY = 1000000;
    mFollowQuad = -1;
    mXRealPos = 0;
    mYRealPos = 0;
    mQuadInfo.clear();
    mAnchorInfo.clear();
    mQuadTrace.clear();
    mCollisionInfo.clear();
    mDrawTrace = false;
}

ObjectInfo *MapWidget::getQuadInfo(int id)
{
    if (mQuadInfo.contains(id)) {
        return &mQuadInfo[id];
    } else {
        return 0;
    }
}

ObjectInfo *MapWidget::getAnchorInfo(int anchor)
{
    if (anchor < mAnchorInfo.size()) {
        return &mAnchorInfo[anchor];
    } else {
        return 0;
    }
}

LDM_ELEMENT_t *MapWidget::getLdmElement(int id)
{
    if (mLdmElements.contains(id)) {
        return &mLdmElements[id];
    } else {
        return 0;
    }
}

void MapWidget::addQuad(int id, ObjectInfo quad)
{
    mQuadInfo.insert(id, quad);
}

void MapWidget::removeQuad(int id)
{
    mQuadInfo.remove(id);
}

void MapWidget::removeAllQuads()
{
    mQuadInfo.clear();
}

void MapWidget::addCollision(int id, ObjectInfo collision)
{
    mCollisionInfo.insert(id, collision);
}

void MapWidget::removeCollision(int id)
{
    mCollisionInfo.remove(id);
}

void MapWidget::removeAllCollisions()
{
    mCollisionInfo.clear();
}

void MapWidget::addLdmElement(LDM_ELEMENT_t element)
{
    mLdmElements.insert(element.id, element);
}

void MapWidget::removeLdmElement(int id)
{
    mLdmElements.remove(id);
}

void MapWidget::removeAllLddmElements()
{
    mLdmElements.clear();
}

void MapWidget::addAnchor(ObjectInfo anchor)
{
    mAnchorInfo.append(anchor);
}

void MapWidget::removeAnchor(int id)
{
    mAnchorInfo.remove(id);
}

void MapWidget::removeAllAnchors()
{
    mAnchorInfo.clear();
}

void MapWidget::addLineSegment(int id, SEGMENT_t lineSegment)
{
    mLineSegments.insert(id, lineSegment);
}

void MapWidget::removeAllLineSegments()
{
    mLineSegments.clear();
}

void MapWidget::setScaleFactor(double scale)
{
    double scaleDiff = scale / mScaleFactor;
    mScaleFactor = scale;
    mXOffset *= scaleDiff;
    mYOffset *= scaleDiff;
    repaint();
}

void MapWidget::setRotation(double rotation)
{
    mRotation = rotation;
    repaint();
}

void MapWidget::setXOffset(double offset)
{
    mXOffset = offset;
    repaint();
}

void MapWidget::setYOffset(double offset)
{
    mYOffset = offset;
    repaint();
}

void MapWidget::clearTrace()
{
    mQuadTrace.clear();
    repaint();
}

void MapWidget::addPerspectivePixmap(PerspectivePixmap map)
{
    mPerspectivePixmaps.append(map);
}

void MapWidget::clearPerspectivePixmaps()
{
    mPerspectivePixmaps.clear();
    repaint();
}

QPoint MapWidget::getMousePosRelative()
{
    QPoint p = this->mapFromGlobal(QCursor::pos());
    p.setX((p.x() - mXOffset - width() / 2) / mScaleFactor);
    p.setY((-p.y() - mYOffset + height() / 2) / mScaleFactor);
    return p;
}

void MapWidget::setDrawTrace(bool drawTrace)
{
    mDrawTrace = drawTrace;
}

bool MapWidget::getDrawTrace()
{
    return mDrawTrace;
}

void MapWidget::setFollowQuad(int quad)
{
    mFollowQuad = quad;
    repaint();
}

void MapWidget::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    const double scaleMax = 5;
    const double scaleMin = 0.0001;
    const double offsetViewMaxFact = 8;

    // Make sure scale and offsetappend is reasonable
    if (mScaleFactor < scaleMin)
    {
        double scaleDiff = scaleMin / mScaleFactor;
        mScaleFactor = scaleMin;
        mXOffset *= scaleDiff;
        mYOffset *= scaleDiff;
    } else if (mScaleFactor > scaleMax)
    {
        double scaleDiff = scaleMax / mScaleFactor;
        mScaleFactor = scaleMax;
        mXOffset *= scaleDiff;
        mYOffset *= scaleDiff;
    }

    if (mXOffset < -(width() * offsetViewMaxFact))
    {
        mXOffset = -(width() * offsetViewMaxFact);
    } else if (mXOffset > (width() * offsetViewMaxFact))
    {
        mXOffset = (width() * offsetViewMaxFact);
    }

    if (mYOffset < -(height() * offsetViewMaxFact))
    {
        mYOffset = -(height() * offsetViewMaxFact);
    } else if (mYOffset > (height() * offsetViewMaxFact))
    {
        mYOffset = (height() * offsetViewMaxFact);
    }

    if (mFollowQuad >= 0)
    {
        if (mQuadInfo.contains(mFollowQuad)) {
            LocPoint followLoc = mQuadInfo[mFollowQuad].getLocation();
            mXOffset = -followLoc.getX() * mScaleFactor;
            mYOffset = -followLoc.getY() * mScaleFactor;
        }
    }

    // Paint begins here
    painter.fillRect(event->rect(), QBrush(Qt::transparent));

    double angle, x, y;
    QString txt;
    QPointF pt_txt;
    QRectF rect_txt;
    QPen pen;
    QFont font = this->font();

    // Map coordinate transforms
    QTransform drawTrans;
    drawTrans.translate(event->rect().width() / 2 + mXOffset, event->rect().height() / 2 - mYOffset);
    drawTrans.scale( mScaleFactor, -mScaleFactor );
    drawTrans.rotate(mRotation);

    // Text coordinates
    QTransform txtTrans;
    txtTrans.translate(0, 0);
    txtTrans.scale( 1, 1 );
    txtTrans.rotate(0);

    // Set font
    font.setPointSize(10);
    painter.setFont(font);

    // Axis parameters
    const double scaleMult = ceil((1 / ((mScaleFactor * 100) / 50)));
    const double step = 100 * scaleMult;
    const double zeroAxisWidth = 3;
    const QColor zeroAxisColor = Qt::red;
    const QColor firstAxisColor = Qt::gray;
    const QColor secondAxisColor = Qt::blue;
    const QColor textColor = QPalette::Foreground;
    const double xStepFact = ceil(width() * offsetViewMaxFact / step / mScaleFactor);
    const double yStepFact = ceil(height() * offsetViewMaxFact / step / mScaleFactor);

    // Draw perspective pixmaps first
    painter.setTransform(drawTrans);
    for(int i = 0;i < mPerspectivePixmaps.size();i++) {
        mPerspectivePixmaps[i].drawUsingPainter(painter);
    }

    // Draw Y-axis segments
    for (double i = -xStepFact * step;i < xStepFact * step;i += step)
    {
        if ((int)(i / step) % 2) {
            painter.setPen(firstAxisColor);
        } else {
            txt.sprintf("%.0f", i);
            pt_txt.setX(i);
            pt_txt.setY(0);
            painter.setTransform(txtTrans);
            pt_txt = drawTrans.map(pt_txt);
            pt_txt.setX(pt_txt.x() + 5);
            pt_txt.setY(height() - 10);
            painter.setPen(QPen(textColor));
            painter.drawText(pt_txt, txt);

            if (i == 0)
            {
                pen.setWidthF(zeroAxisWidth / mScaleFactor);
                pen.setColor(zeroAxisColor);
            } else
            {
                pen.setWidth(0);
                pen.setColor(secondAxisColor);
            }
            painter.setPen(pen);
        }

        painter.setTransform(drawTrans);
        painter.drawLine(i, -yStepFact * step, i, yStepFact * step);
    }

    // Draw X-axis segments
    for (double i = -yStepFact * step;i < yStepFact * step;i += step)
    {
        if ((int)(i / step) % 2) {
            painter.setPen(firstAxisColor);
        } else {
            txt.sprintf("%.0f", i);
            pt_txt.setY(i);
            painter.setTransform(txtTrans);
            pt_txt = drawTrans.map(pt_txt);
            pt_txt.setX(10);
            pt_txt.setY(pt_txt.y() - 5);
            painter.setPen(QPen(textColor));
            painter.drawText(pt_txt, txt);

            if (i == 0)
            {
                pen.setWidthF(zeroAxisWidth / mScaleFactor);
                pen.setColor(zeroAxisColor);
            } else
            {
                pen.setWidth(0);
                pen.setColor(secondAxisColor);
            }
            painter.setPen(pen);
        }
        painter.setTransform(drawTrans);
        painter.drawLine(-xStepFact * step, i, xStepFact * step, i);
    }

    // Handle and draw trace for all quads
    if (mDrawTrace) {
        QHashIterator<int, ObjectInfo> qItr(mQuadInfo);
        while (qItr.hasNext()) {
            qItr.next();
            mQuadTrace.append(mQuadInfo[qItr.key()].getLocation().getPointF());
        }

        pen.setWidth(30);
        pen.setColor(Qt::red);
        painter.setTransform(drawTrans);
        for (int i = 0;i < mQuadTrace.size();i++) {
            painter.setPen(pen);
            painter.drawEllipse(mQuadTrace.at(i), 5, 5);
        }
    }

    // Draw LDM elements
    painter.setPen(QPen(textColor));
    QHashIterator<int, LDM_ELEMENT_t> rItr(mLdmElements);
    while (rItr.hasNext()) {
        rItr.next();
        LDM_ELEMENT_t *ldmInfo = &mLdmElements[rItr.key()];
        LDM_RISK_ELEMENT_t *riskInfo = &ldmInfo->risk;
        x = riskInfo->px;
        y = riskInfo->py;
        angle = riskInfo->rotation * 180.0 / M_PI;
        painter.setTransform(drawTrans);

        painter.translate(x, y);
        painter.rotate(angle);

        QColor col = Qt::yellow;

        switch(ldmInfo->type) {
        case LDM_ELEMENT_TYPE_LINE:
            break;

        case LDM_ELEMENT_TYPE_COMFORT_ZONE:
            col = Qt::green;
            break;

        case LDM_ELEMENT_TYPE_QUADCOPTER:
            break;

        case LDM_ELEMENT_TYPE_POINT:
            break;
        }

        if (riskInfo->has_collision) {
            col = Qt::red;
        }

        switch(ldmInfo->type) {
        case LDM_ELEMENT_TYPE_LINE:
        case LDM_ELEMENT_TYPE_COMFORT_ZONE:
        case LDM_ELEMENT_TYPE_QUADCOPTER:
        case LDM_ELEMENT_TYPE_POINT:
            col.setAlphaF(0.3);
            painter.setBrush(QBrush(col));
            painter.drawEllipse(QPointF(0.0, 0.0), riskInfo->width, riskInfo->height);
            break;
        }
    }

    // Draw quads
    painter.setPen(QPen(textColor));
    QHashIterator<int, ObjectInfo> qItr(mQuadInfo);
    while (qItr.hasNext()) {
        qItr.next();
        ObjectInfo *quadInfo = &mQuadInfo[qItr.key()];
        LocPoint pos = quadInfo->getLocation();
        x = pos.getX();
        y = pos.getY();
        angle = pos.getAngle() * 180.0 / M_PI;
        painter.setTransform(drawTrans);

        painter.translate(x, y);
        painter.rotate(angle);

        // Draw the Quadcopter
        painter.setBrush(QBrush(Qt::red));
        painter.drawRoundedRect(-20, 0, 40, 300, 10, 10);
        painter.setBrush(quadInfo->getType() & POS_MASK_FOLLOWING ?
                             Qt::yellow : QBrush(quadInfo->getColor()));
        painter.drawRoundedRect(-300, -20, 600, 40, 10, 10);
        painter.drawRoundedRect(-20, -300, 40, 300, 10, 10);

        QColor col = Qt::red;
        col.setAlphaF(0.3);
        painter.setBrush(QBrush(col));
        painter.drawEllipse(QPointF(0, 275), 130, 130);
        col = Qt::green;
        col.setAlphaF(0.3);
        painter.setBrush(QBrush(col));
        painter.drawEllipse(QPointF(0, -275), 130, 130);
        painter.drawEllipse(QPointF(275, 0), 130, 130);
        painter.drawEllipse(QPointF(-275, 0), 130, 130);

        // Draw the acceleration vector
        painter.setBrush(QBrush(Qt::green));
        painter.setPen(QPen(Qt::green, 30));
        painter.drawLine(QPointF(0.0, 0.0), QPointF(pos.getRoll() * 800.0, -pos.getPitch() * 800.0));
        painter.setPen(QPen(textColor));

        // Print data
        txt.sprintf("%s\n(%.0f, %.0f, %.0f)", quadInfo->getName().toLocal8Bit().data(),
                    x, y, angle);
        pt_txt.setX(x + 450);
        pt_txt.setY(y);
        painter.setTransform(txtTrans);
        pt_txt = drawTrans.map(pt_txt);
        rect_txt.setCoords(pt_txt.x(), pt_txt.y() - 20,
                           pt_txt.x() + 150, pt_txt.y() + 25);
        painter.drawText(rect_txt, txt);
    }

    // Draw collisions
    painter.setPen(QPen(Qt::red, 30));
    QHashIterator<int, ObjectInfo> cItr(mCollisionInfo);
    while (cItr.hasNext()) {
        cItr.next();
        ObjectInfo *collInfo = &mCollisionInfo[cItr.key()];
        LocPoint pos = collInfo->getLocation();
        x = pos.getX();
        y = pos.getY();
        painter.setTransform(drawTrans);

        painter.translate(x, y);

        painter.setBrush(QBrush(Qt::red));
        painter.drawLine(QPointF(-100, -100), QPointF(100, 100));
        painter.drawLine(QPointF(100, -100), QPointF(-100, 100));
    }

    // Draw anchors
    painter.setPen(QPen(textColor));
    for(QVector<ObjectInfo>::Iterator it_anchor = mAnchorInfo.begin();it_anchor < mAnchorInfo.end();it_anchor++) {
        ObjectInfo *anchorInfo = it_anchor;
        LocPoint pos = anchorInfo->getLocation();
        x = pos.getX();
        y = pos.getY();
        angle = pos.getAngle() * 180.0 / M_PI;
        painter.setTransform(drawTrans);

        // Draw anchor
        painter.setBrush(QBrush(Qt::red));
        painter.translate(x, y);
        painter.rotate(angle);

        painter.drawRoundedRect(-40, -40, 80, 80, 10, 10);

        // Print data
        txt.sprintf("%s\n(%.0f, %.0f, %.0f)", anchorInfo->getName().toLocal8Bit().data(),
                    x, y, angle);
        pt_txt.setX(x + 80);
        pt_txt.setY(y);
        painter.setTransform(txtTrans);
        pt_txt = drawTrans.map(pt_txt);
        rect_txt.setCoords(pt_txt.x(), pt_txt.y() - 20,
                           pt_txt.x() + 150, pt_txt.y() + 25);
        painter.drawText(rect_txt, txt);
    }

    // Draw Line segments
    QHashIterator<int, SEGMENT_t> segItr(mLineSegments);
    painter.setPen(QPen(Qt::red, 5.0 / mScaleFactor));
    painter.setTransform(drawTrans);
    while (segItr.hasNext()) {
        segItr.next();
        const SEGMENT_t &seg = segItr.value();
        painter.drawLine(QPointF(seg.p1.x * 1000.0, seg.p1.y * 1000.0),
                         QPointF(seg.p2.x * 1000.0, seg.p2.y * 1000.0));
    }

    // Draw units (mm)
    painter.setPen(QPen(textColor));
    painter.setTransform(txtTrans);
    font.setPointSize(16);
    painter.setFont(font);
    txt = "(mm)";
    painter.drawText(width() - 50, 35, txt);
}

void MapWidget::mouseMoveEvent(QMouseEvent *e)
{
    if (e->buttons() & Qt::LeftButton && !(e->modifiers() & Qt::ControlModifier))
    {
        int x = e->pos().x();
        int y = e->pos().y();

        if (mMouseLastX < 100000)
        {
            int diffx = x - mMouseLastX;
            mXOffset += diffx;

            Q_EMIT(offsetChanged(mXOffset, mYOffset));
            repaint();
        }

        if (mMouseLastY < 100000)
        {
            int diffy = y - mMouseLastY;
            mYOffset -= diffy;

            Q_EMIT(offsetChanged(mXOffset, mYOffset));
            repaint();
        }

        mMouseLastX = x;
        mMouseLastY = y;
    }
}

void MapWidget::mousePressEvent(QMouseEvent *e)
{
    if ((e->buttons() & Qt::LeftButton) && (e->modifiers() & Qt::ControlModifier)) {
        LocPoint pos = mQuadInfo[0].getLocation();
        QPoint p = getMousePosRelative();
        pos.setXY(p.x(), p.y());
        if (mQuadInfo.contains(0)) {
            mQuadInfo[0].setLocation(pos);
        }
        emit posSet(pos);
        repaint();
    }
}

void MapWidget::mouseReleaseEvent(QMouseEvent *e)
{
    if (!(e->buttons() & Qt::LeftButton))
    {
        mMouseLastX = 1000000;
        mMouseLastY = 1000000;
    }
}

void MapWidget::wheelEvent(QWheelEvent *e)
{
    if (e->modifiers() & Qt::ControlModifier) {
        LocPoint pos = mQuadInfo[0].getLocation();
        pos.setAngle(pos.getAngle() + (double)e->delta() * 0.0005);
        mQuadInfo[0].setLocation(pos);
        emit posSet(pos);
        repaint();
    } else {
        int x = e->pos().x();
        int y = e->pos().y();
        double scaleDiff = ((double)e->delta() / 600.0);
        if (scaleDiff > 0.8)
        {
            scaleDiff = 0.8;
        }

        if (scaleDiff < -0.8)
        {
            scaleDiff = -0.8;
        }

        x -= width() / 2;
        y -= height() / 2;

        mScaleFactor += mScaleFactor * scaleDiff;
        mXOffset += mXOffset * scaleDiff;
        mYOffset += mYOffset * scaleDiff;

        Q_EMIT(scaleChanged(mScaleFactor));
        Q_EMIT(offsetChanged(mXOffset, mYOffset));
        repaint();
    }
}
