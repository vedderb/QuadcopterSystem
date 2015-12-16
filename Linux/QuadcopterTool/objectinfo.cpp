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

#include "objectinfo.h"

ObjectInfo::ObjectInfo(int type, QString name, Qt::GlobalColor color)
{
    mType = type;
    mName = name;
    mColor = color;
}

int ObjectInfo::getType()
{
    return mType;
}

void ObjectInfo::setType(int type)
{
    mType = type;
}

QString ObjectInfo::getName()
{
    return mName;
}

void ObjectInfo::setName(QString name)
{
    mName = name;
}

void ObjectInfo::setLocation(LocPoint &point)
{
    mLocation = point;
}

Qt::GlobalColor ObjectInfo::getColor()
{
    return mColor;
}

void ObjectInfo::setColor(Qt::GlobalColor color)
{
    mColor = color;
}
double ObjectInfo::getVBat()
{
    return mVBat;
}

void ObjectInfo::setVBat(float value)
{
    mVBat = value;
}

quint16 ObjectInfo::getAdcVal(int ch)
{
    if (ch < 0 || ch >= mAdcVal.size()) {
        return 0;
    }

    return mAdcVal[ch];
}

void ObjectInfo::setAdcVals(quint16 *values, int val_num)
{
    mAdcVal.clear();
    for (int i = 0;i < val_num;i++) {
        mAdcVal.append(values[i]);
    }
}

int ObjectInfo::getAdcValNum()
{
    return mAdcVal.size();
}

double ObjectInfo::getAdcVoltage(int ch)
{
    return (getAdcVal(ch) / 4095.0) * 3.3;
}


LocPoint ObjectInfo::getLocation()
{
    return mLocation;
}

