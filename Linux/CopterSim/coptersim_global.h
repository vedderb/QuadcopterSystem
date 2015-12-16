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

#ifndef COPTERSIM_GLOBAL_H
#define COPTERSIM_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(COPTERSIM_LIBRARY)
#  define COPTERSIMSHARED_EXPORT Q_DECL_EXPORT
#else
#  define COPTERSIMSHARED_EXPORT Q_DECL_IMPORT
#endif

#endif // COPTERSIM_GLOBAL_H
