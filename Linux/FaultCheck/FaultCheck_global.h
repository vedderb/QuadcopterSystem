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

#ifndef FAULTCHECK_GLOBAL_H
#define FAULTCHECK_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(FAULTLIB_LIBRARY)
#  define FAULTCHECKSHARED_EXPORT Q_DECL_EXPORT
#else
#  define FAULTCHECKSHARED_EXPORT Q_DECL_IMPORT
#endif

#endif // FAULTCHECK_GLOBAL_H
