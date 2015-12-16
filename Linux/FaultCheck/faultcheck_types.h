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

#ifndef FAULTCHECK_TYPES_H
#define FAULTCHECK_TYPES_H

typedef enum {
    FAULT_NONE = 0,
    FAULT_BITFLIP,
    FAULT_BITFLIP_RANDOM,
    FAULT_STUCK_TO_CURRENT,
    FAULT_AMPLIFICATION,
    FAULT_OFFSET,
    FAULT_SET_TO
    // Stuck at bitwise
    // oscillation (sine with amplitude and frequency)
} FAULT_TYPE;

typedef enum {
    TRG_PERMANENT = 0,
    TRG_AFTER_ITR, // Intermittent combined with a duration
    TRG_RANDOM,
    TRG_RANDOM_ONCE // Intermittent at a random location
    // Temporal (the triggers above are temporal)
    // Spatial (based on some condition?)
} TRIGGER_TYPE;

typedef enum {
    PACKET_FAULT_NONE = 0,
    PACKET_FAULT_DROP,
    PACKET_FAULT_REPETITION,
    PACKET_FAULT_CURRUPTION
} PACKET_FAULT_TYPE;

#endif // FAULT_TYPES_H
