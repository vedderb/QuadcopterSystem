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

#ifndef SERIALIZATION_H
#define SERIALIZATION_H

#include <QObject>
#include <QVector>
#include "datatypes.h"

class Serialization : public QObject
{
    Q_OBJECT
public:
    explicit Serialization(QObject *parent = 0);
    bool writeAncorConf(const QVector<ANCHOR_SETTINGS_t> &anch_set, const MAP_LIMITS_t &map_lim, QWidget *parent);
    bool readAncorConf(QVector<ANCHOR_SETTINGS_t> &anch_set, MAP_LIMITS_t &map_lim, QWidget *parent);

signals:

public slots:

};

#endif // SERIALIZATION_H
