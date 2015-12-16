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

#ifndef SERIALCOMM_H
#define SERIALCOMM_H

#include <QObject>
#include <QLabel>
#include "datatypes.h"
#include "locpoint.h"

class Comm : public QObject
{
    Q_OBJECT
public:
    explicit Comm(QObject *parent = 0);
    ~Comm();

    void setPosition(quint8 quad, POS_STATE_t pos);
    void setControlParameters(quint8 quad, CONTROL_PARAMETERS_t ctrl_param);
    void setSticks(quint8 quad, double roll, double pitch, double yaw, double throttle, double pot1, double pot2);
    void setMotorOverride(quint8 quad, QVector<double> pwm);
    void getOrientation(quint8 quad);
    void getRawIMU(quint8 quad);
    void getPosition(quint8 quad);
    void getControlParameters(quint8 quad);
    void zeroGyroSticks(quint8 quad);
    void zeroOrientation(quint8 quad);
    void getAltitude(quint8 quad);
    void getUltraDist(quint8 quad, int anchor_id);
    void setAnchors(quint8 quad, const QVector<ANCHOR_SETTINGS_t> &anchors, const MAP_LIMITS_t &map_lim);
    void getAnchors(quint8 quad);
    void emergencyStop(quint8 quad);
    void setYaw(quint8 quad, double yaw);
    void setLedExtIndicationLanded(quint8 quad, LED_EXT_INDICATION_LANDED ind_type);

signals:
    void dataToSend(QByteArray data);
    void controlParametersReceived(CONTROL_PARAMETERS_t ctrl_param);
    void orientationReceived(POS_STATE_t pos_state);
    void rawIMUReceived(RAW_IMU_t imu_values);
    void printReceived(QString str);
    void positionReceived(int id, int type, POS_STATE_t pos_state);
    void altitudeReceived(float altitude);
    void ppmReceived(PPM_RADIO_t ppm_state);
    void mapCmdReceived(MAP_CMD_t cmd, int param);
    void LdmElementReceived(LDM_ELEMENT_t);
    void lineSegmentReceived(int id, SEGMENT_t segment);
    void ultraDistReceived(qint32 id, float dist);
    void aliveReceived(int id);
    void safetyMsgReceived(int id, SAFETY_MSG_t msg);
    void statusMsgReceived(int id, STATUS_MSG_t msg);
    void anchorsReceived(int id, QVector<ANCHOR_SETTINGS_t> anchors, MAP_LIMITS_t map_lim);
    void simCmdReceived(SIM_CMD_t cmd, QByteArray data);

public slots:
    void packetReceived(QByteArray data);

private:
    quint8 *mSendBuffer;

};

#endif // SERIALCOMM_H
