/*
    Copyright 2013 - 2015 Benjamin Vedder	benjamin@vedder.se
    Copyright 2013 - 2014 Daniel Skarin     daniel.skarin@sp.se

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

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QUdpSocket>
#include "serialport.h"
#include "packetinterface.h"
#include "joystick.h"
#include "comm.h"
#include "serialization.h"
#include "datatypes.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    
private slots:
    void readPendingDatagrams();
    void timerSlot();
    void mapPosSet(LocPoint pos);
    void serialDataAvailable();
    void commDataToSend(QByteArray data);
    void packetDataToSend(QByteArray &data);
    void controlParametersReceived(CONTROL_PARAMETERS_t ctrl_param);
    void orientationReceived(POS_STATE_t pos);
    void rawIMUReceived(RAW_IMU_t imu_values);
    void printReceived(QString str);
    void positionReceived(int id, int type, POS_STATE_t pos_state);
    void altitudeReceived(float altitude);
    void ppmReceived(PPM_RADIO_t ppm_state);
    void mapCmdReceived(MAP_CMD_t cmd, int param);
    void LdmElementReceived(LDM_ELEMENT_t ldmElement);
    void lineSegmentReceived(int id, SEGMENT_t segment);
    void ultraDistReceived(qint32 id, float dist);
    void aliveReceived(int id);
    void safetyMsgReceived(int id, SAFETY_MSG_t msg);
    void statusMsgReceived(int id, STATUS_MSG_t msg);
    void anchorsReceived(int id, QVector<ANCHOR_SETTINGS_t> anchors, MAP_LIMITS_t map_lim);

    void on_measureShortButton_clicked();
    void on_measureAllButton_clicked();
    void on_clearTraceButton_clicked();
    void on_pidSetParametersButton_clicked();
    void on_pidGetParametersButton_clicked();
    void on_serialConnectButton_clicked();
    void on_serialDisconnectButton_clicked();
    void on_zeroGyroSticksButton_clicked();
    void on_joystickConnectButton_clicked();
    void on_joystickDisconnectButton_clicked();
    void on_zeroOrientationButton_clicked();
    void on_clearSampleButton_clicked();
    void on_saveMagButton_clicked();
    void on_zeroPosButton_clicked();
    void on_clearMapButton_clicked();
    void on_getAnchorsButton_clicked();
    void on_setAnchorsButton_clicked();
    void on_allSetAnchorsButton_clicked();
    void on_allDemoButton_clicked();
    void on_allAutolandButton_clicked();
    void on_allZeroPosButton_clicked();
    void on_allZeroGyroSticksButton_clicked();
    void on_fiLosLowButton_clicked();
    void on_fiLosMedButton_clicked();
    void on_fiLosHighButton_clicked();
    void on_anchSaveXmlButton_clicked();
    void on_anchLoadXmlButton_clicked();
    void on_ledLandedLosButton_clicked();
    void on_ledLandedDirButton_clicked();
    void on_ledLandedBothButton_clicked();

private:
    static const int mQuadLabelNum = 6;

    Ui::MainWindow *ui;
    QUdpSocket *mUdpSocket;
    PacketInterface *mPacketInterface;
    SerialPort *mSerialPort;
    QVector<double> recData;
    QVector<double> longCorrData;
    QVector<double> longRecData;
    Comm *mSerialComm;
    QTimer *mTimer;
    Joystick *mJoystick;
    Serialization *mSerialization;
    QVector<double> accelXData;
    QVector<double> accelYData;
    QVector<double> accelZData;
    QVector<double> gyroXData;
    QVector<double> gyroYData;
    QVector<double> gyroZData;
    QVector<double> magXData;
    QVector<double> magYData;
    QVector<double> magZData;
    QVector<double> accelGyroMagXAxis;
    int maxSampleSize;
    QVector<QVector<double> > magSamples;
    QVector<double> altitudeData;
    QVector<double> ultraDistData;
    bool repaintMap;
    int mQuadUpdateTime[mQuadLabelNum];
    STATUS_MSG_t mQuadStatus[mQuadLabelNum];
    QLabel *mQuadLabels[mQuadLabelNum];

    void clearPlotsAndData();
    void setAnchors(int quadId);
    void plotAnchorsOnMap(const QVector<ANCHOR_SETTINGS_t> &anchors, const MAP_LIMITS_t &map_lim);
    void guiReadAnchorsLimits(QVector<ANCHOR_SETTINGS_t> &anchors, MAP_LIMITS_t &map_lim);
    void guiSetAnchorsLimits(const QVector<ANCHOR_SETTINGS_t> &anchors, const MAP_LIMITS_t &map_lim);
};

#endif // MAINWINDOW_H
