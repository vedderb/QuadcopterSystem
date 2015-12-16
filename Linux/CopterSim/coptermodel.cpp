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

#include "coptermodel.h"
#include "utility.h"
#include <cmath>
#include <QString>
#include <QDebug>
#include <faultcheck_wrapper.h>

CopterModel::CopterModel(double px, double py, int id)
{
    memset(&mActualPos, 0, sizeof(mActualPos));
    memset(&mControlInput, 0, sizeof(mControlInput));
    mActualPos.px = px;
    mActualPos.py = py;
    mPerceivedPos = mActualPos;
    //mPerceivedPos.px -= 0.1;
    //mPerceivedPos.py += 0.1;
    mGlobalTime = 0;
    mItsStation.setId(id);

    mCorrLastTime = 0;

    mCorrParam.anchor_pos_gain_p = 0.8;
    mCorrParam.anchor_pos_gain_i = 0.0;
    mCorrParam.anchor_pos_gain_d = 0.0;

    mCorrParam.anchor_vel_gain_p = 2.0;
    mCorrParam.anchor_vel_gain_i = 0.0;
    mCorrParam.anchor_vel_gain_d = 0.0;

    mCorrParam.anchor_acc_gain_p = 0.0;
    mCorrParam.anchor_acc_gain_i = 0.0;
    mCorrParam.anchor_acc_gain_d = 0.0;

    mCorrParam.anchor_max_corr = 0.5;
    mCorrParam.anchor_max_tilt = 6.0;
}

void CopterModel::setPos(POS_STATE_t state)
{
    mActualPos = state;
    mPerceivedPos = state;
}

const POS_STATE_t &CopterModel::getActualPos()
{
    return mActualPos;
}

const POS_STATE_t &CopterModel::getPerceivedPos()
{
    return mPerceivedPos;
}

void CopterModel::runIteration(unsigned long dt_l)
{
    updatePositionRK(mActualPos, dt_l);
    updatePositionRK(mPerceivedPos, dt_l);
    //mPerceivedPos = mActualPos;

    mGlobalTime += dt_l;

    mPerceivedPos.pitch = mActualPos.pitch;
    mPerceivedPos.roll = mActualPos.roll;
    mPerceivedPos.yaw = mActualPos.yaw;
    const QString pStr = QString().sprintf("PitchId%d", mItsStation.getId());
    const QString rStr = QString().sprintf("RollId%d", mItsStation.getId());
    const QString yawStr = QString().sprintf("YawId%d", mItsStation.getId());
    faultcheck_injectFaultDouble(pStr.toLocal8Bit().data(), &mPerceivedPos.pitch);
    faultcheck_injectFaultDouble(rStr.toLocal8Bit().data(), &mPerceivedPos.roll);
    faultcheck_injectFaultDouble(yawStr.toLocal8Bit().data(), &mPerceivedPos.yaw);

    //mPerceivedPos.pitch = mActualPos.pitch + 0.2;
    //mPerceivedPos.roll = mActualPos.roll + 0.2;

    const QString xStr = QString().sprintf("PxId%d", mItsStation.getId());
    const QString yStr = QString().sprintf("PyId%d", mItsStation.getId());
    faultcheck_injectFaultDouble(xStr.toLocal8Bit().data(), &mPerceivedPos.px);
    faultcheck_injectFaultDouble(yStr.toLocal8Bit().data(), &mPerceivedPos.py);

    mItsStation.runIteration(dt_l, mPerceivedPos);

    CONTROL_INPUT_t ctrl_input = mControlInput;
    mItsStation.getOverride(&ctrl_input);
    updateOrientation(mActualPos, ctrl_input, dt_l);
    updateOrientation(mPerceivedPos, ctrl_input, dt_l);
}

void CopterModel::setControlInput(CONTROL_INPUT_t &input)
{
    mControlInput = input;
}

double CopterModel::getActualDistToCopter(CopterModel &copter)
{
    const POS_STATE_t &pos = copter.getActualPos();
    const double px = pos.px;
    const double py = pos.py;
    const double px1 = mActualPos.px;
    const double py1 = mActualPos.py;

    return sqrt((px - px1) * (px - px1) + (py - py1) * (py - py1));
}

/**
 * @brief CopterModel::updateOrientation
 * This is the algorithm on the quadcopter that translates stick input to
 * the goal values of the control loop. In this simulator, we assume that the
 * control loop will keep the error at 0 and thus we update the actual position
 * directly.
 *
 * @param state
 * The state to update the orientation in.
 *
 * @param input
 * The control input to update the orientation with.
 */
void CopterModel::updateOrientation(POS_STATE_t &state, CONTROL_INPUT_t input, unsigned long dt_l)
{
    const double dt = (double)dt_l / 1000000.0;

    // Truncate the control input
    utility::truncate_number(&input.rollCmd, -1.0, 1.0);
    utility::truncate_number(&input.pitchCmd, -1.0, 1.0);
    utility::truncate_number(&input.yawCmd, -1.0, 1.0);
    utility::truncate_number(&input.throttleCmd, 0.0, 1.0);

    const double roll_iteration_gain = 1.0;
    const double pitch_iteration_gain = 1.0;
    const double yaw_iteration_gain = 0.6;

    double roll = state.roll + input.rollCmd * roll_iteration_gain * dt * 250.0;
    double pitch = state.pitch + input.pitchCmd * pitch_iteration_gain * dt * 250.0;
    double yaw = state.yaw + input.yawCmd * yaw_iteration_gain * dt * 250.0;

    double fac = pow(0.995, dt * 1000.0);
    roll = utility::weight_angle(roll, -state.roll, fac);
    pitch = utility::weight_angle(pitch, -state.pitch, fac);

    utility::norm_angle(&roll);
    utility::norm_angle(&pitch);
    utility::norm_angle(&yaw);

    state.roll = roll;
    state.pitch = pitch;
    state.yaw = yaw;
}

/**
 * @brief CopterModel::updatePosition
 * Advance the position of the copter based on the current roll, pitch and yaw. One
 * assumption is that the throttle is controlled so that the same height is maintained,
 * which means that the vertical force component is 9.82g to oppose the gravity vector.
 * This can be considered true as long as the roll and pitch are low enough so that the
 * motors can keep up with that acceleration.
 *
 * Updating this algorithm to use an arbitrary acceleration force and also update the
 * z (height) position should be easy, but the ITS station and visualization will become
 * complictaed this way.
 *
 * @param state
 * The current position and orientation of the copter.
 *
 * @param dt_l
 * The time step in microseconds.
 *
 */
void CopterModel::updatePosition(POS_STATE_t &state, unsigned long dt_l)
{
    const double dt = (double)dt_l / 1000000.0;

    const double vel_decay_e = 0.9; // was 0.7
    const double vel_decay_l = 0.00; // was 0.08

    const double roll = (state.roll + state.acc_roll_err) * M_PI / 180.0;
    const double pitch = (state.pitch + state.acc_pitch_err) * M_PI / 180.0;
    const double yaw = state.yaw * M_PI / 180.0;

    const double acc_v = 9.82;
    const double cos_y = cos(-yaw);
    const double sin_y = sin(-yaw);

    const double dvx = acc_v * tan(roll) * dt;
    const double dvy = -acc_v * tan(pitch) * dt;

    state.vx += cos_y * dvx + sin_y * dvy;
    state.vy += -sin_y * dvx + cos_y * dvy;
    state.px += state.vx * dt;
    state.py += state.vy * dt;

    // Exponential decay
    const double decay_factor = pow(vel_decay_e, dt);
    state.vx *= decay_factor;
    state.vy *= decay_factor;

    // Linear decay
    utility::step_towards(&state.vx, 0.0, vel_decay_l * dt);
    utility::step_towards(&state.vy, 0.0, vel_decay_l * dt);
}

/**
 * Note: This is the same function as above, but it uses the RK4 mothod to approximate
 * some differential equations instead of the euler method.
 *
 * @brief CopterModel::updatePositionRK
 * Advance the position of the copter based on the current roll, pitch and yaw. One
 * assumption is that the throttle is controlled so that the same height is maintained,
 * which means that the vertical force component is 9.82g to oppose the gravity vector.
 * This can be considered true as long as the roll and pitch are low enough so that the
 * motors can keep up with that acceleration.
 *
 * Updating this algorithm to use an arbitrary acceleration force and also update the
 * z (height) position should be easy, but the ITS station and visualization will become
 * complictaed this way.
 *
 * @param state
 * The current position and orientation of the copter.
 *
 * @param dt_l
 * The time step in microseconds.
 *
 */
void CopterModel::updatePositionRK(POS_STATE_t &state, unsigned long dt_l)
{
    const double dt = (double)dt_l / 1000000.0;

    const double vel_decay_e = 0.9; // was 0.7
    const double vel_decay_l = 0.00; // was 0.08

    const double roll = (state.roll + state.acc_roll_err) * M_PI / 180.0;
    const double pitch = (state.pitch + state.acc_pitch_err) * M_PI / 180.0;
    const double yaw = state.yaw * M_PI / 180.0;

    const double acc_v = 9.82;
    const double cos_y = cos(-yaw);
    const double sin_y = sin(-yaw);

    const double dvx = acc_v * tan(roll);
    const double dvy = -acc_v * tan(pitch);

    state.vx += cos_y * dvx * dt + sin_y * dvy * dt;
    state.vy += -sin_y * dvx * dt + cos_y * dvy * dt;

    const double kx1 = state.vx;
    const double kx2 = kx1 + cos_y * dvx * dt / 2.0 + sin_y * dvy * dt / 2.0;
    const double kx3 = kx2 + cos_y * dvx * dt / 2.0 + sin_y * dvy * dt / 2.0;
    const double kx4 = kx3 + cos_y * dvx * dt + sin_y * dvy * dt;

    const double ky1 = state.vy;
    const double ky2 = ky1 + -sin_y * dvx * dt / 2.0 + cos_y * dvy * dt / 2.0;
    const double ky3 = ky2 + -sin_y * dvx * dt / 2.0 + cos_y * dvy * dt / 2.0;
    const double ky4 = ky3 + -sin_y * dvx * dt + cos_y * dvy * dt;

    state.px += (dt / 6.0) * (kx1 + 2.0 * kx2 + 2.0 * kx3 + kx4);
    state.py += (dt / 6.0) * (ky1 + 2.0 * ky2 + 2.0 * ky3 + ky4);

    // Exponential decay
    const double decay_factor = pow(vel_decay_e, dt);
    state.vx *= decay_factor;
    state.vy *= decay_factor;

    // Linear decay
    utility::step_towards(&state.vx, 0.0, vel_decay_l * dt);
    utility::step_towards(&state.vy, 0.0, vel_decay_l * dt);
}

void CopterModel::syncGlobaltime(uint64_t time)
{
    mGlobalTime = time;
    mItsStation.syncGlobaltime(time);
}

uint64_t CopterModel::getGlobalTime()
{
    return mGlobalTime;
}

void CopterModel::correct_pos_anchor(ANCHOR_t &anchor)
{
    double dt = (double)(mGlobalTime - mCorrLastTime) / 1000000.0;
    mCorrLastTime = mGlobalTime;

    if (dt < 0.0001) {
        return;
    }

    // Store anchors locally to keep track of their state
    if (!mAnchors.contains(anchor.id)) {
        mAnchors.insert(anchor.id, anchor);
    }
    ANCHOR_t &anch_int = mAnchors[anchor.id];
    anch_int.px = anchor.px;
    anch_int.py = anchor.py;
    anch_int.pz = anchor.pz;

    // Compute the distance to the anchor based on the actual position
    const double pdiffx = mActualPos.px - anch_int.px;
    const double pdiffy = mActualPos.py - anch_int.py;
    const double pdiffz = mActualPos.pz - anch_int.pz;
    double anchor_distance = sqrt(pdiffx*pdiffx + pdiffy*pdiffy + pdiffz*pdiffz);

    // Inject ranging fault
    const QString fcStr = QString().sprintf("RangeId%dAnch%d",
                                            mItsStation.getId(), anch_int.id);
    faultcheck_injectFaultDouble(fcStr.toLocal8Bit().data(), &anchor_distance);

    const double da_x = mPerceivedPos.px - anch_int.px;
    const double da_y = mPerceivedPos.py - anch_int.py;
    const double da_z = mPerceivedPos.pz - anch_int.pz;
    const double da = sqrtf(da_x*da_x + da_y*da_y + da_z*da_z);

    double error = da - anchor_distance;

    if (fabsf(error) > 0.2) {
        anch_int.wrong_iterations++;
        if (anch_int.wrong_iterations < 3 && !anch_int.out_of_pos) {
            return;
        } else {
            anch_int.out_of_pos = 1;
        }
    } else {
        anch_int.wrong_iterations = 0;
        anch_int.out_of_pos = 0;
    }

    utility::truncate_number(&error, -mCorrParam.anchor_max_corr, mCorrParam.anchor_max_corr);

    anch_int.update_time = mGlobalTime;
    anch_int.last_dist = anchor_distance;

    const double comp_factor = error / da;

    const double pcx = da_x * comp_factor;
    const double pcy = da_y * comp_factor;

    const double pcx_p = pcx * mCorrParam.anchor_pos_gain_p;
    const double pcy_p = pcy * mCorrParam.anchor_pos_gain_p;

    anch_int.corr_pcxI += pcx * mCorrParam.anchor_pos_gain_i * dt;
    anch_int.corr_pcyI += pcy * mCorrParam.anchor_pos_gain_i * dt;
    utility::truncate_number(&anch_int.corr_pcxI, -0.2, 0.2);
    utility::truncate_number(&anch_int.corr_pcyI, -0.2, 0.2);

    const double pcx_d = (pcx - anch_int.corr_pcxLast) * mCorrParam.anchor_pos_gain_d / dt;
    const double pcy_d = (pcy - anch_int.corr_pcyLast) * mCorrParam.anchor_pos_gain_d / dt;
    anch_int.corr_vcxLast = pcx;
    anch_int.corr_vcyLast = pcy;

    const double pcx_out = pcx_p + anch_int.corr_pcxI + pcx_d;
    const double pcy_out = pcy_p + anch_int.corr_pcyI + pcy_d;

    mPerceivedPos.px -= pcx_out;
    mPerceivedPos.py -= pcy_out;

    const double cosy = cos(-mPerceivedPos.yaw * M_PI / 180.0);
    const double siny = sin(-mPerceivedPos.yaw * M_PI / 180.0);

    const double vcx = da_x * comp_factor;
    const double vcy = da_y * comp_factor;

    const double vcx_p = vcx * mCorrParam.anchor_vel_gain_p;
    const double vcy_p = vcy * mCorrParam.anchor_vel_gain_p;

    anch_int.corr_vcxI += vcx * mCorrParam.anchor_vel_gain_i * dt;
    anch_int.corr_vcyI += vcy * mCorrParam.anchor_vel_gain_i * dt;
    utility::truncate_number(&anch_int.corr_vcxI, -1.0, 1.0);
    utility::truncate_number(&anch_int.corr_vcyI, -1.0, 1.0);

    const double vcx_d = (vcx - anch_int.corr_vcxLast) * mCorrParam.anchor_vel_gain_d / dt;
    const double vcy_d = (vcy - anch_int.corr_vcyLast) * mCorrParam.anchor_vel_gain_d / dt;
    anch_int.corr_vcxLast = vcx;
    anch_int.corr_vcyLast = vcy;

    const double vcx_out = vcx_p + anch_int.corr_vcxI + vcx_d;
    const double vcy_out = vcy_p + anch_int.corr_vcyI + vcy_d;

    mPerceivedPos.vx -= vcx_out;
    mPerceivedPos.vy -= vcy_out;

    const double acx = da_x * comp_factor;
    const double acy = da_y * comp_factor;

    const double acx_p = acx * mCorrParam.anchor_acc_gain_p;
    const double acy_p = acy * mCorrParam.anchor_acc_gain_p;

    anch_int.corr_acxI += acx * mCorrParam.anchor_acc_gain_i * dt;
    anch_int.corr_acyI += acy * mCorrParam.anchor_acc_gain_i * dt;
    utility::truncate_number(&anch_int.corr_acxI, -1.0, 1.0);
    utility::truncate_number(&anch_int.corr_acyI, -1.0, 1.0);

    const double acx_d = (acx - anch_int.corr_acxLast) * mCorrParam.anchor_acc_gain_d / dt;
    const double acy_d = (acy - anch_int.corr_acyLast) * mCorrParam.anchor_acc_gain_d / dt;
    anch_int.corr_acxLast = acx;
    anch_int.corr_acyLast = acy;

    const double acx_out = acx_p + anch_int.corr_acxI + acx_d;
    const double acy_out = acy_p + anch_int.corr_acyI + acy_d;

    mPerceivedPos.acc_roll_err -= acx_out * cosy - acy_out * siny;
    mPerceivedPos.acc_pitch_err += acy_out * cosy + acx_out * siny;

    utility::truncate_number(&mPerceivedPos.acc_roll_err, -mCorrParam.anchor_max_tilt, mCorrParam.anchor_max_tilt);
    utility::truncate_number(&mPerceivedPos.acc_pitch_err, -mCorrParam.anchor_max_tilt, mCorrParam.anchor_max_tilt);
}

void CopterModel::correct_altitude(double altitude)
{
    mActualPos.pz = altitude;
    mPerceivedPos.pz = altitude;
}
