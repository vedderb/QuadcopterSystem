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

#ifndef COPTERSIM_WRAPPER_H
#define COPTERSIM_WRAPPER_H

#include "datatypes.h"

#ifdef __cplusplus
extern "C" {
#endif

void coptersim_addCopter(double px, double py, int id);
void coptersim_removeCopter(int id);
void coptersim_removeAllCopters(void);
void coptersim_runIterations(unsigned long num_iterations, unsigned long iteration_len,
                             int its_update_div, int anch_update_div);
int coptersim_setControlInput(int copterId, CONTROL_INPUT_t input);
void coptersim_selectCopter(int id);
void coptersim_sendStateToMap(void);
int coptersim_collisionOccured(void);
POINT_t coptersim_collisionPoint(void);
void coptersim_resetCollision(void);
void coptersim_stopSimulation(void);
void coptersim_addLineSegment(int id, double p1x, double p1y, double p2x, double p2y);
void coptersim_removeAllLineSegments(void);
void coptersim_addAnchor(int id, double px, double py, double pz);
void coptersim_removeAllAnchors(void);

#ifdef __cplusplus
}
#endif

#endif // COPTERSIM_WRAPPER_H
