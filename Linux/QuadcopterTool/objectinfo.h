/*
    Copyright 2012 - 2015 Benjamin Vedder	benjamin@vedder.se

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

#ifndef CARINFO_H
#define CARINFO_H

#define QT_NO_KEYWORDS
#include <QVector>
#include <QString>
#include "locpoint.h"

class ObjectInfo
{
public:
    ObjectInfo(int type = 0, QString name = "NewObject", Qt::GlobalColor color = Qt::blue);
    int getType();
    void setType(int type);
    QString getName();
    void setName(QString name);
    void setLocation(LocPoint &point);
    LocPoint getLocation();
    Qt::GlobalColor getColor();
    void setColor(Qt::GlobalColor color);
    double getVBat();
    void setVBat(float value);
    quint16 getAdcVal(int ch);
    void setAdcVals(quint16 *values, int val_num);
    int getAdcValNum();
    double getAdcVoltage(int ch);

private:
    QString mName;
    LocPoint mLocation;
    Qt::GlobalColor mColor;
    int mType;
    double mVBat;
    QVector<quint16> mAdcVal;

};

#endif // CARINFO_H
