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

#include "faultcheck_wrapper.h"
#include "faultcheck.h"

static FaultCheck faultcheck;

void faultcheck_addFaultBitFlip(const char* identifier, int bitToFlip) {
    faultcheck.addFaultBitFlip(identifier, bitToFlip);
}

void faultcheck_addFaultBitFlipRandom(const char* identifier) {
    faultcheck.addFaultBitFlipRandom(identifier);
}

void faultcheck_addFaultAmplification(const char* identifier, double factor) {
    faultcheck.addFaultAmplification(identifier, factor);
}

void faultcheck_addFaultOffset(const char* identifier, double constant) {
    faultcheck.addFaultOffset(identifier, constant);
}

void faultcheck_addFaultSetTo(const char* identifier, double value) {
    faultcheck.addFaultSetTo(identifier, value);
}

void faultcheck_setTriggerOnceAfterIterations(const char* identifier, unsigned long iterations) {
    Trigger *trg = faultcheck.trigger(identifier);

    if (trg) {
        trg->setOnceAfterItr(iterations);
    }
}

void faultcheck_setTriggerAfterIterations(const char* identifier, unsigned long iterations) {
    Trigger *trg = faultcheck.trigger(identifier);

    if (trg) {
        trg->setAfterItr(iterations);
    }
}

void faultcheck_setDurationAfterTrigger(const char* identifier, int iterations) {
    Trigger *trg = faultcheck.trigger(identifier);

    if (trg) {
        trg->setDuration(iterations);
    }
}

void faultcheck_removeAllFaultsIdentifier(const char *identifier)
{
    faultcheck.removeAllFaultsIdentifier(identifier);
}

void faultcheck_removeAllFaults() {
    faultcheck.removeAllFaults();
}

void faultcheck_injectFaultInt(const char* identifier, int *integer) {
    faultcheck.injectFaultInt(identifier, integer);
}

void faultcheck_injectFaultDouble(const char *identifier, double *value)
{
    faultcheck.injectFaultDouble(identifier, value);
}


void *faultcheck_instance()
{
    return &faultcheck;
}


int faultcheck_numActiveFaults(const char *identifier)
{
    return faultcheck.numActiveFaults(identifier);
}
