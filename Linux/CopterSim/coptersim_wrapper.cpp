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

#include "coptersim_wrapper.h"
#include "coptersim.h"

static CopterSim copterSim;


void coptersim_addCopter(double px, double py, int id)
{
    copterSim.addCopter(px, py, id);
}


void coptersim_removeCopter(int id)
{
    copterSim.removeCopter(id);
}


void coptersim_removeAllCopters(void)
{
    copterSim.removeAllCopters();
}


void coptersim_runIterations(unsigned long num_iterations, unsigned long iteration_len,
                             int its_update_div, int anch_update_div)
{
    copterSim.runIterations(num_iterations, iteration_len, its_update_div, anch_update_div);
}


int coptersim_setControlInput(int copterId, CONTROL_INPUT_t input)
{
    return copterSim.setControlInput(copterId, input);
}


void coptersim_selectCopter(int id)
{
    copterSim.selectCopter(id);
}


void coptersim_sendStateToMap(void)
{
    copterSim.sendStateToMap();
}


int coptersim_collisionOccured(void)
{
    return copterSim.collisionOccured();
}


POINT_t coptersim_collisionPoint(void)
{
    return copterSim.collisionPoint();
}


void coptersim_resetCollision(void)
{
    copterSim.resetCollision();
}


void coptersim_stopSimulation(void)
{
    copterSim.stopSimulation();
}


void coptersim_addLineSegment(int id, double p1x, double p1y, double p2x, double p2y)
{
    copterSim.addLineSegment(id, p1x, p1y, p2x, p2y);
}


void coptersim_removeAllLineSegments(void)
{
    copterSim.removeAllLineSegments();
}

void coptersim_addAnchor(int id, double px, double py, double pz) {
    copterSim.addAnchor(id, px, py, pz);
}

void coptersim_removeAllAnchors(void) {
    copterSim.removeAllAnchors();
}
