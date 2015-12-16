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

#ifndef FAULTCHECK_WRAPPER_H
#define FAULTCHECK_WRAPPER_H

#include "faultcheck_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void faultcheck_addFaultBitFlip(const char* identifier, int bitToFlip);
void faultcheck_addFaultBitFlipRandom(const char* identifier);
void faultcheck_addFaultAmplification(const char* identifier, double factor);
void faultcheck_addFaultOffset(const char* identifier, double constant);
void faultcheck_addFaultSetTo(const char* identifier, double value);
void faultcheck_setTriggerOnceAfterIterations(const char* identifier, unsigned long iterations);
void faultcheck_setTriggerAfterIterations(const char* identifier, unsigned long iterations);
void faultcheck_setDurationAfterTrigger(const char* identifier, int iterations);
void faultcheck_removeAllFaultsIdentifier(const char* identifier);
void faultcheck_removeAllFaults();
void faultcheck_injectFaultInt(const char* identifier, int *integer);
void faultcheck_injectFaultDouble(const char* identifier, double *value);
int faultcheck_numActiveFaults(const char* identifier);
void *faultcheck_instance(void);

#ifdef __cplusplus
}
#endif

#endif // FAULTCHECK_WRAPPER_H
