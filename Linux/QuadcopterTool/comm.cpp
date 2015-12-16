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

#include "comm.h"
#include "utility.h"
#include <QDebug>

Comm::Comm(QObject *parent) :
    QObject(parent)
{
    mSendBuffer = new quint8[255];
}

Comm::~Comm()
{
    delete mSendBuffer;
}

void Comm::setPosition(quint8 quad, POS_STATE_t pos)
{
    qint32 send_index = 0;

    send_index = 0;
    mSendBuffer[send_index++] = quad;
    mSendBuffer[send_index++] = PACKET_CMD_SET_POS;

    utility::buffer_append_int32(mSendBuffer, (int32_t)(pos.px * 10000.0), &send_index);
    utility::buffer_append_int32(mSendBuffer, (int32_t)(pos.py * 10000.0), &send_index);
    utility::buffer_append_int32(mSendBuffer, (int32_t)(pos.vx * 10000.0), &send_index);
    utility::buffer_append_int32(mSendBuffer, (int32_t)(pos.vy * 10000.0), &send_index);
    utility::buffer_append_int32(mSendBuffer, (int32_t)(pos.yaw * 100000.0), &send_index);

    emit dataToSend(QByteArray((const char*)mSendBuffer, send_index));
}

void Comm::setControlParameters(quint8 quad, CONTROL_PARAMETERS_t ctrl_param)
{
    qint32 send_index = 0;

    mSendBuffer[send_index++] = quad;
    mSendBuffer[send_index++] = PACKET_CMD_SET_CONTROL_PARAMETERS;
    utility::buffer_append_int32(mSendBuffer, (int32_t)(ctrl_param.roll_p_gain * 100000.0), &send_index);
    utility::buffer_append_int32(mSendBuffer, (int32_t)(ctrl_param.roll_i_gain * 100000.0), &send_index);
    utility::buffer_append_int32(mSendBuffer, (int32_t)(ctrl_param.roll_d_gain_process * 100000.0), &send_index);
    utility::buffer_append_int32(mSendBuffer, (int32_t)(ctrl_param.roll_d_gain_error * 100000.0), &send_index);

    utility::buffer_append_int32(mSendBuffer, (int32_t)(ctrl_param.pitch_p_gain * 100000.0), &send_index);
    utility::buffer_append_int32(mSendBuffer, (int32_t)(ctrl_param.pitch_i_gain * 100000.0), &send_index);
    utility::buffer_append_int32(mSendBuffer, (int32_t)(ctrl_param.pitch_d_gain_process * 100000.0), &send_index);
    utility::buffer_append_int32(mSendBuffer, (int32_t)(ctrl_param.pitch_d_gain_error * 100000.0), &send_index);

    utility::buffer_append_int32(mSendBuffer, (int32_t)(ctrl_param.yaw_p_gain * 100000.0), &send_index);
    utility::buffer_append_int32(mSendBuffer, (int32_t)(ctrl_param.yaw_i_gain * 100000.0), &send_index);
    utility::buffer_append_int32(mSendBuffer, (int32_t)(ctrl_param.yaw_d_gain_process * 100000.0), &send_index);
    utility::buffer_append_int32(mSendBuffer, (int32_t)(ctrl_param.yaw_d_gain_error * 100000.0), &send_index);

    emit dataToSend(QByteArray((const char*)mSendBuffer, send_index));
}

void Comm::setSticks(quint8 quad, double roll, double pitch, double yaw, double throttle, double pot1, double pot2)
{
    // TODO: Change this function to use CONTROL_INPUT_t
    QByteArray packet;

    packet.append(quad);
    packet.append(PACKET_CMD_SET_STICK_INPUT);

    packet.append((char)0);
    packet.append((char)(roll * 255));
    packet.append((char)1);
    packet.append((char)(pitch * 255));
    packet.append((char)2);
    packet.append((char)(yaw * 255));
    packet.append((char)3);
    packet.append((char)(throttle * 255));
    packet.append((char)4);
    packet.append((char)(pot1 * 255));
    packet.append((char)5);
    packet.append((char)(pot2 * 255));

    emit dataToSend(packet);
}

void Comm::setMotorOverride(quint8 quad, QVector<double> pwm)
{
    qint32 send_index = 0;

    mSendBuffer[send_index++] = quad;
    mSendBuffer[send_index++] = PACKET_CMD_OVERRIDE_POWER;

    for (int i = 0;i < pwm.size();i++) {
        utility::buffer_append_int32(mSendBuffer, pwm[i] * 100000, &send_index);
    }

    emit dataToSend(QByteArray((const char*)mSendBuffer, send_index));
}

void Comm::getOrientation(quint8 quad)
{
    QByteArray packet;
    packet.append(quad);
    packet.append(PACKET_CMD_GET_ORIENTATION);
    emit dataToSend(packet);
}

void Comm::getRawIMU(quint8 quad)
{
    QByteArray packet;
    packet.append(quad);
    packet.append(PACKET_CMD_GET_RAW_IMU);
    emit dataToSend(packet);
}

void Comm::getPosition(quint8 quad)
{
    QByteArray packet;
    packet.append(quad);
    packet.append(PACKET_CMD_GET_POS);
    emit dataToSend(packet);
}

void Comm::getControlParameters(quint8 quad)
{
    QByteArray packet;
    packet.append(quad);
    packet.append(PACKET_CMD_GET_CONTROL_PARAMETERS);
    emit dataToSend(packet);
}

void Comm::zeroGyroSticks(quint8 quad)
{
    QByteArray packet;
    packet.append(quad);
    packet.append(PACKET_CMD_RESET_GYRO_STICKS);
    emit dataToSend(packet);
}

void Comm::zeroOrientation(quint8 quad)
{
    QByteArray packet;
    packet.append(quad);
    packet.append(PACKET_CMD_RESET_ORIENTATION);
    emit dataToSend(packet);
}

void Comm::getAltitude(quint8 quad)
{
    QByteArray packet;
    packet.append(quad);
    packet.append(PACKET_CMD_GET_ALTITUDE);
    emit dataToSend(packet);
}

void Comm::getUltraDist(quint8 quad, int anchor_id)
{
    QByteArray packet;
    packet.append(quad);
    packet.append(PACKET_CMD_GET_ULTRA_DIST);
    packet.append((char)anchor_id);
    emit dataToSend(packet);
}

void Comm::setAnchors(quint8 quad, const QVector<ANCHOR_SETTINGS_t> &anchors, const MAP_LIMITS_t &map_lim)
{
    qint32 send_index = 0;

    mSendBuffer[send_index++] = quad;
    mSendBuffer[send_index++] = PACKET_CMD_SET_ANCHORS;

    utility::buffer_append_int32(mSendBuffer, map_lim.min_x * 10000, &send_index);
    utility::buffer_append_int32(mSendBuffer, map_lim.max_x * 10000, &send_index);
    utility::buffer_append_int32(mSendBuffer, map_lim.min_y * 10000, &send_index);
    utility::buffer_append_int32(mSendBuffer, map_lim.max_y * 10000, &send_index);

    for (int i = 0;i < anchors.size();i++) {
        mSendBuffer[send_index++] = anchors[i].id;
        utility::buffer_append_int32(mSendBuffer, anchors[i].px * 10000, &send_index);
        utility::buffer_append_int32(mSendBuffer, anchors[i].py * 10000, &send_index);
        utility::buffer_append_int32(mSendBuffer, anchors[i].pz * 10000, &send_index);
    }

    emit dataToSend(QByteArray((const char*)mSendBuffer, send_index));
}

void Comm::getAnchors(quint8 quad)
{
    QByteArray packet;
    packet.append(quad);
    packet.append(PACKET_CMD_GET_ANCHORS);
    emit dataToSend(packet);
}

void Comm::emergencyStop(quint8 quad)
{
    QByteArray packet;
    packet.append(quad);
    packet.append(PACKET_CMD_EMERGENCY_STOP);
    emit dataToSend(packet);
}

void Comm::setYaw(quint8 quad, double yaw)
{
    qint32 send_index = 0;

    send_index = 0;
    mSendBuffer[send_index++] = quad;
    mSendBuffer[send_index++] = PACKET_CMD_SET_YAW;

    utility::buffer_append_int32(mSendBuffer, (int32_t)(yaw * 100000.0), &send_index);

    emit dataToSend(QByteArray((const char*)mSendBuffer, send_index));
}

void Comm::setLedExtIndicationLanded(quint8 quad, LED_EXT_INDICATION_LANDED ind_type)
{
    qint32 send_index = 0;

    send_index = 0;
    mSendBuffer[send_index++] = quad;
    mSendBuffer[send_index++] = PACKET_CMD_SET_LED_EXT_IND_LANDED;
    mSendBuffer[send_index++] = ind_type;

    emit dataToSend(QByteArray((const char*)mSendBuffer, send_index));
}

void Comm::packetReceived(QByteArray data)
{
    quint8 id = data[0];
    data.remove(0, 1);

    PACKET_CMD_t cmd = (PACKET_CMD_t)(quint8)data[0];
    data.remove(0, 1);

    quint8 *data_ptr = (quint8*)data.data();
    int data_ind = 0;
    qint32 tmp_i32;
    float tmp_float;
    CONTROL_PARAMETERS_t ctrl_param;
    POS_STATE_t pos_state;
    int pos_id;
    int pos_type;
    RAW_IMU_t imu_values;
    PPM_RADIO_t ppm_radio;
    MAP_CMD_t map_cmd;
    LDM_ELEMENT_t ldm_element;
    SEGMENT_t seg;
    qint32 anch_id;
    SAFETY_MSG_t safety_msg;
    STATUS_MSG_t status_msg;
    QVector<ANCHOR_SETTINGS_t> anchors;
    MAP_LIMITS_t map_lim;
    SIM_CMD_t sim_cmd;

    switch (cmd) {
    case PACKET_CMD_SET_STICK_INPUT:
        for (data_ind = 0; data_ind < data.length() / 2; data_ind++) {
            tmp_float = (float) data_ptr[2*data_ind + 1] / 255;
            switch(data_ptr[2*data_ind]) {
            case 0:
                ppm_radio.roll = tmp_float;
                break;
            case 1:
                ppm_radio.pitch = tmp_float;
                break;
            case 2:
                ppm_radio.yaw = tmp_float;
                break;
            case 3:
                ppm_radio.throttle = tmp_float;
                break;
            case 4:
                ppm_radio.pot1 = tmp_float;
                break;
            case 5:
                ppm_radio.pot2 = tmp_float;
                break;
            default:
                break;
            }
        }

        emit ppmReceived(ppm_radio);
        break;

    case PACKET_CMD_GET_CONTROL_PARAMETERS:
        data_ind = 0;
        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        ctrl_param.roll_p_gain = (float)tmp_i32 / 100000;
        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        ctrl_param.roll_i_gain = (float)tmp_i32 / 100000;
        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        ctrl_param.roll_d_gain_process = (float)tmp_i32 / 100000;
        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        ctrl_param.roll_d_gain_error = (float)tmp_i32 / 100000;

        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        ctrl_param.pitch_p_gain = (float)tmp_i32 / 100000;
        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        ctrl_param.pitch_i_gain = (float)tmp_i32 / 100000;
        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        ctrl_param.pitch_d_gain_process = (float)tmp_i32 / 100000;
        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        ctrl_param.pitch_d_gain_error = (float)tmp_i32 / 100000;

        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        ctrl_param.yaw_p_gain = (float)tmp_i32 / 100000;
        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        ctrl_param.yaw_i_gain = (float)tmp_i32 / 100000;
        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        ctrl_param.yaw_d_gain_process = (float)tmp_i32 / 100000;
        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        ctrl_param.yaw_d_gain_error = (float)tmp_i32 / 100000;

        emit controlParametersReceived(ctrl_param);
        break;

    case PACKET_CMD_GET_ORIENTATION:
        data_ind = 0;

        // Roll
        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        pos_state.roll = (float)tmp_i32 / 100000;

        // Pitch
        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        pos_state.pitch = (float)tmp_i32 / 100000;

        // Yaw
        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        pos_state.yaw = (float)tmp_i32 / 100000;

        emit orientationReceived(pos_state);
        break;

    case PACKET_CMD_GET_RAW_IMU:
        data_ind = 0;

        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        imu_values.accX = (double)tmp_i32 / 100000;
        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        imu_values.accY = (double)tmp_i32 / 100000;
        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        imu_values.accZ = (double)tmp_i32 / 100000;

        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        imu_values.gyroX = (double)tmp_i32 / 100000;
        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        imu_values.gyroY = (double)tmp_i32 / 100000;
        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        imu_values.gyroZ = (double)tmp_i32 / 100000;

        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        imu_values.magX = (double)tmp_i32 / 100000;
        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        imu_values.magY = (double)tmp_i32 / 100000;
        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        imu_values.magZ = (double)tmp_i32 / 100000;

        emit rawIMUReceived(imu_values);
        break;

    case PACKET_CMD_PRINT:
        emit printReceived(data);
        break;

    case PACKET_CMD_GET_POS:
        data_ind = 0;

        // Id and type
        pos_id = id;
        pos_type = utility::buffer_get_int16(data_ptr, &data_ind);

        // px
        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        tmp_float = (float)tmp_i32 / 10000;
        pos_state.px = tmp_float;

        // py
        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        tmp_float = (float)tmp_i32 / 10000;
        pos_state.py = tmp_float;

        // Sending the velocity is optional
        if (data_ind < data.size()) {
            // vx
            tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
            tmp_float = (float)tmp_i32 / 10000;
            pos_state.vx = tmp_float;

            // vy
            tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
            tmp_float = (float)tmp_i32 / 10000;
            pos_state.vy = tmp_float;
        }

        // Sending the orientation is also optional
        if (data_ind < data.size()) {
            // Roll
            tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
            tmp_float = (float)tmp_i32 / 100000;
            pos_state.roll = tmp_float;

            // Pitch
            tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
            tmp_float = (float)tmp_i32 / 100000;
            pos_state.pitch = tmp_float;

            // Yaw
            tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
            tmp_float = (float)tmp_i32 / 100000;
            pos_state.yaw = tmp_float;

            if (pos_state.roll > 180.0) {
                pos_state.roll -= 360.0;
            }
            if (pos_state.pitch > 180.0) {
                pos_state.pitch -= 360.0;
            }
            if (pos_state.yaw > 180.0) {
                pos_state.yaw -= 360.0;
            }
        }

        emit positionReceived(pos_id, pos_type, pos_state);
        break;

    case PACKET_CMD_GET_ALTITUDE:
        data_ind = 0;
        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        tmp_float = (float)tmp_i32 / 100000;
        emit altitudeReceived(tmp_float);
        break;

    case PACKET_CMD_MAP:
        map_cmd = (MAP_CMD_t)data_ptr[data_ind++];
        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        emit mapCmdReceived(map_cmd, tmp_i32);
        break;

    case PACKET_CMD_LDM_ELEMENT:
        data_ind = 0;

        // NOTE: The LocPoint has mm coordinates (NOT m)

        // Id, type and collision
        ldm_element.id = utility::buffer_get_int16(data_ptr, &data_ind);
        ldm_element.type = (LDM_ELEMENT_TYPE_t)utility::buffer_get_int16(data_ptr, &data_ind);
        ldm_element.risk.has_collision = data_ptr[data_ind++];

        // px
        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        tmp_float = (float)tmp_i32 / 10;
        ldm_element.risk.px = tmp_float;

        // py
        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        tmp_float = (float)tmp_i32 / 10;
        ldm_element.risk.py = tmp_float;

        // angle
        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        tmp_float = (float)tmp_i32 / 100000;
        ldm_element.risk.rotation = tmp_float;

        // width
        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        tmp_float = (float)tmp_i32 / 10;
        ldm_element.risk.width = tmp_float;

        // height
        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        tmp_float = (float)tmp_i32 / 10;
        ldm_element.risk.height = tmp_float;

        emit LdmElementReceived(ldm_element);
        break;

    case PACKET_CMD_LINE_SEGMENT:
        data_ind = 0;

        pos_id = utility::buffer_get_int16(data_ptr, &data_ind);

        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        tmp_float = (float)tmp_i32 / 10000;
        seg.p1.x = tmp_float;

        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        tmp_float = (float)tmp_i32 / 10000;
        seg.p1.y = tmp_float;

        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        tmp_float = (float)tmp_i32 / 10000;
        seg.p2.x = tmp_float;

        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        tmp_float = (float)tmp_i32 / 10000;
        seg.p2.y = tmp_float;

        emit lineSegmentReceived(pos_id, seg);
        break;

    case PACKET_CMD_GET_ULTRA_DIST:
        data_ind = 0;
        anch_id = data_ptr[data_ind++];
        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        tmp_float = (float)tmp_i32 / 100000;
        emit ultraDistReceived(anch_id, tmp_float);
        break;

    case PACKET_CMD_RADIO_ALIVE:
        emit aliveReceived(id);
        break;

    case PACKET_CMD_SAFETY_MSG:
        data_ind = 0;

        safety_msg.id = data[data_ind++];
        safety_msg.los_local = (LEVEL_OF_SERVICE_t)data_ptr[data_ind++];
        safety_msg.los_global = (LEVEL_OF_SERVICE_t)data_ptr[data_ind++];

        emit safetyMsgReceived(id, safety_msg);
        break;

    case PACKET_CMD_STATUS_MSG:
        data_ind = 0;

        status_msg.id = data[data_ind++];
        status_msg.fw_version_major = data[data_ind++];
        status_msg.fw_version_minor = data[data_ind++];
        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        status_msg.px = (float)tmp_i32 / 10000.0;
        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        status_msg.py = (float)tmp_i32 / 10000.0;
        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        status_msg.pz = (float)tmp_i32 / 10000.0;
        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        status_msg.roll = (float)tmp_i32 / 10000.0;
        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        status_msg.pitch = (float)tmp_i32 / 10000.0;
        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        status_msg.yaw = (float)tmp_i32 / 10000.0;
        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        status_msg.vx = (float)tmp_i32 / 10000.0;
        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        status_msg.vy = (float)tmp_i32 / 10000.0;
        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        status_msg.vbat = (float)tmp_i32 / 10000.0;
        for (int i = 0;i < 4;i++) {
            status_msg.adc_in[i] = utility::buffer_get_uint16(data_ptr, &data_ind);
        }

        emit statusMsgReceived(id, status_msg);
        break;

    case PACKET_CMD_GET_ANCHORS:
        data_ind = 0;

        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        map_lim.min_x = (float)tmp_i32 / 10000.0;
        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        map_lim.max_x = (float)tmp_i32 / 10000.0;
        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        map_lim.min_y = (float)tmp_i32 / 10000.0;
        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        map_lim.max_y = (float)tmp_i32 / 10000.0;

        anchors.resize((data.length() - data_ind) / 13);
        for (int i = 0;data_ind < data.length();i++) {
            anchors[i].id = data_ptr[data_ind++];
            tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
            anchors[i].px = (float)tmp_i32 / 10000.0;
            tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
            anchors[i].py = (float)tmp_i32 / 10000.0;
            tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
            anchors[i].pz = (float)tmp_i32 / 10000.0;
        }

        emit anchorsReceived(id, anchors, map_lim);
        break;

    case PACKET_CMD_SIM:
        sim_cmd = (SIM_CMD_t)(quint8)data[0];
        data.remove(0, 1);
        emit simCmdReceived(sim_cmd, data);
        break;

    default:
        break;
    }
}
