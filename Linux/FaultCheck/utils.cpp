/*
    Copyright 2014 Benjamin Vedder	benjamin.vedder@sp.se

    This file is part of FaultCheck.

    FaultCheck is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    FaultCheck is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with FaultCheck.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "utils.h"

namespace utils {

int randInt(int low, int high)
{
    return qrand() % ((high + 1) - low) + low;
}

double randZeroToOne()
{
    return qrand() / static_cast<double>(RAND_MAX);
}

bool randBool(double probTrue)
{
    return (randZeroToOne() < probTrue);
}

QString parseStringFromByteArray(QByteArray &array)
{
    QString res(array.data()); // Cast to char* so the first 0 byte terminates
    array.remove(0, res.size() + 1);
    return res;
}

}
