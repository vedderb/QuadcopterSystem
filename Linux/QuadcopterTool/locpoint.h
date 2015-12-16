/*
    Copyright 2012 Benjamin Vedder	benjamin@vedder.se

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

#ifndef LOCPOINT_H
#define LOCPOINT_H

#include <QPointF>

class LocPoint
{
public:
    LocPoint(double x = 0, double y = 0, double angle = 0, double width = 0, double height = 0, double roll = 0, double pitch = 0);

    double getX();
    double getY();
    double getAngle();
    double getWidth();
    double getHeight();
    double getRoll();
    double getPitch();
    QPointF getPointF();

    void setX(double x);
    void setY(double y);
    void setXY(double x, double y);
    void setAngle(double angle);
    void setWidth(double w);
    void setHeight(double h);
    void setWidthHeight(double w, double h);
    void setRoll(double roll);
    void setPitch(double pitch);
    void setRollPitch(double roll, double pitch);
    double getDistanceTo(const LocPoint &point);
    double getDistanceTo(const QPointF &point);

    // Operators
    LocPoint& operator=(const LocPoint& point);
    bool operator==(const LocPoint& point);
    bool operator!=(const LocPoint& point);

private:
    double mX;
    double mY;
    double mAngle;
    double mWidth;
    double mHeight;
    double mRoll;
    double mPitch;

};

#endif // LOCPOINT_H
