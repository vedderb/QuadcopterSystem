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

#include "locpoint.h"
#include <cmath>

LocPoint::LocPoint(double x, double y, double angle, double width, double height, double roll, double pitch) :
    mX(x), mY(y), mAngle(angle), mWidth(width), mHeight(height), mRoll(roll), mPitch(pitch)
{

}

double LocPoint::getX()
{
    return mX;
}

double LocPoint::getY()
{
    return mY;
}

double LocPoint::getAngle()
{
    return mAngle;
}

double LocPoint::getWidth()
{
    return mWidth;
}

double LocPoint::getHeight()
{
    return mHeight;
}

double LocPoint::getRoll()
{
    return mRoll;
}

double LocPoint::getPitch()
{
    return mPitch;
}

QPointF LocPoint::getPointF()
{
    return QPointF(mX, mY);
}

void LocPoint::setX(double x)
{
    mX = x;
}

void LocPoint::setY(double y)
{
    mY = y;
}

void LocPoint::setXY(double x, double y)
{
    mX = x;
    mY = y;
}

LocPoint &LocPoint::operator =(const LocPoint &point)
{
    mX = point.mX;
    mY = point.mY;
    mAngle = point.mAngle;
    mWidth = point.mWidth;
    mHeight = point.mHeight;
    mRoll = point.mRoll;
    mPitch = point.mPitch;
    return *this;
}

bool LocPoint::operator ==(const LocPoint &point)
{
    if (    mX == point.mX &&
            mY == point.mY &&
            mAngle == point.mAngle &&
            mWidth == point.mWidth &&
            mHeight == point.mHeight &&
            mRoll == point.mRoll &&
            mPitch == point.mPitch) {
        return true;
    } else {
        return false;
    }
}

bool LocPoint::operator !=(const LocPoint &point)
{
    return !(operator==(point));
}

void LocPoint::setAngle(double angle)
{
    angle = fmod(angle, 2.0 * M_PI);

    if (angle < 0.0) {
        angle += 2.0 * M_PI;
    }

    mAngle = angle;
}

void LocPoint::setWidth(double w)
{
    mWidth = w;
}

void LocPoint::setHeight(double h)
{
    mHeight = h;
}

void LocPoint::setWidthHeight(double w, double h)
{
    mWidth = w;
    mHeight = h;
}

void LocPoint::setRoll(double roll)
{
    mRoll = roll;
}

void LocPoint::setPitch(double pitch)
{
    mPitch = pitch;
}

void LocPoint::setRollPitch(double roll, double pitch)
{
    mRoll = roll;
    mPitch = pitch;
}

double LocPoint::getDistanceTo(const LocPoint &point)
{
    return sqrt((point.mX - mX) * (point.mX - mX) + (point.mY - mY) * (point.mY - mY));
}

double LocPoint::getDistanceTo(const QPointF &point)
{
    return sqrt((point.x() - mX) * (point.x() - mX) + (point.y() - mY) * (point.y() - mY));
}
