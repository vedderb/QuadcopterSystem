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

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "locpoint.h"
#include "objectinfo.h"
#include "utility.h"
#include <QtEndian>
#include <QFileDialog>
#ifdef WIN32
#include <winsock2.h>
#else
#include <netinet/in.h>
#endif
#include <cmath>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    setWindowTitle("QuadcopterTool");

    mUdpSocket = new QUdpSocket(this);
    connect(mUdpSocket, SIGNAL(readyRead()),
            this, SLOT(readPendingDatagrams()));
    mUdpSocket->bind(QHostAddress::AnyIPv6, 3001);

#ifdef WIN32
    QByteArray sendData;
    QHostAddress address("aaaa::200:0:0:7");
    mUdpSocket->writeDatagram(sendData, address, 1028);
    address.setAddress("aaaa::200:0:0:8");
    mUdpSocket->writeDatagram(sendData, address, 1028);
    address.setAddress("aaaa::200:0:0:2");
    mUdpSocket->writeDatagram(sendData, address, 1028);
    address.setAddress("aaaa::200:0:0:3");
    mUdpSocket->writeDatagram(sendData, address, 1028);
#endif

    // Plots
    ui->recPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui->corrPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui->longRecPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui->longCorrPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    ui->rawAccelPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui->rawGyroPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui->rawMagPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    ui->altitudePlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui->ultraDistPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    // Map
    ui->mapWidget->addQuad(0, ObjectInfo(0, "Quad 0", Qt::blue));

    LocPoint quadLoc(1000, 500);
    ui->mapWidget->getQuadInfo(0)->setLocation(quadLoc);
    repaintMap = true;

    connect(ui->mapWidget, SIGNAL(posSet(LocPoint)), this, SLOT(mapPosSet(LocPoint)));

    // Serial communication
    mSerialPort = new SerialPort(this);
    mPacketInterface = new PacketInterface(this);
    mSerialComm = new Comm(this);

    connect(mSerialPort, SIGNAL(serial_data_available()),
            this, SLOT(serialDataAvailable()));
    connect(mPacketInterface, SIGNAL(dataToSend(QByteArray&)),
            this, SLOT(packetDataToSend(QByteArray&)));
    connect(mPacketInterface, SIGNAL(packetReceived(QByteArray)),
            mSerialComm, SLOT(packetReceived(QByteArray)));
    connect(mSerialComm, SIGNAL(dataToSend(QByteArray)),
            this, SLOT(commDataToSend(QByteArray)));

    connect(mSerialComm, SIGNAL(controlParametersReceived(CONTROL_PARAMETERS_t)),
            this, SLOT(controlParametersReceived(CONTROL_PARAMETERS_t)));
    connect(mSerialComm, SIGNAL(orientationReceived(POS_STATE_t)),
            this, SLOT(orientationReceived(POS_STATE_t)));
    connect(mSerialComm, SIGNAL(positionReceived(int,int,POS_STATE_t)), this,
            SLOT(positionReceived(int,int,POS_STATE_t)));
    connect(mSerialComm, SIGNAL(printReceived(QString)),
            this, SLOT(printReceived(QString)));
    connect(mSerialComm, SIGNAL(rawIMUReceived(RAW_IMU_t)),
            this, SLOT(rawIMUReceived(RAW_IMU_t)));
    connect(mSerialComm, SIGNAL(altitudeReceived(float)),
            this, SLOT(altitudeReceived(float)));
    connect(mSerialComm, SIGNAL(ppmReceived(PPM_RADIO_t)),
            this, SLOT(ppmReceived(PPM_RADIO_t)));
    connect(mSerialComm, SIGNAL(mapCmdReceived(MAP_CMD_t,int)),
            this, SLOT(mapCmdReceived(MAP_CMD_t,int)));
    connect(mSerialComm, SIGNAL(LdmElementReceived(LDM_ELEMENT_t)),
            this, SLOT(LdmElementReceived(LDM_ELEMENT_t)));
    connect(mSerialComm, SIGNAL(lineSegmentReceived(int, SEGMENT_t)),
            this, SLOT(lineSegmentReceived(int, SEGMENT_t)));
    connect(mSerialComm, SIGNAL(ultraDistReceived(qint32,float)),
            this, SLOT(ultraDistReceived(qint32,float)));
    connect(mSerialComm, SIGNAL(aliveReceived(int)),
            this, SLOT(aliveReceived(int)));
    connect(mSerialComm, SIGNAL(safetyMsgReceived(int,SAFETY_MSG_t)),
            this, SLOT(safetyMsgReceived(int,SAFETY_MSG_t)));
    connect(mSerialComm, SIGNAL(statusMsgReceived(int,STATUS_MSG_t)),
            this, SLOT(statusMsgReceived(int,STATUS_MSG_t)));
    connect(mSerialComm, SIGNAL(anchorsReceived(int,QVector<ANCHOR_SETTINGS_t>,MAP_LIMITS_t)),
            this, SLOT(anchorsReceived(int,QVector<ANCHOR_SETTINGS_t>,MAP_LIMITS_t)));

    connect(mSerialComm, SIGNAL(simCmdReceived(SIM_CMD_t,QByteArray)),
            ui->simViewer, SLOT(simCmdReceived(SIM_CMD_t,QByteArray)));

    // Timer
    mTimer = new QTimer(this);
    mTimer->setInterval(20);
    mTimer->start();
    connect(mTimer, SIGNAL(timeout()), this, SLOT(timerSlot()));

    mJoystick = new Joystick(this);
    mSerialization = new Serialization(this);

    // The raw IMU plots
    maxSampleSize = 1000;
    accelXData.resize(maxSampleSize);
    accelYData.resize(maxSampleSize);
    accelZData.resize(maxSampleSize);
    gyroXData.resize(maxSampleSize);
    gyroYData.resize(maxSampleSize);
    gyroZData.resize(maxSampleSize);
    magXData.resize(maxSampleSize);
    magYData.resize(maxSampleSize);
    magZData.resize(maxSampleSize);
    accelGyroMagXAxis.resize(maxSampleSize);
    for(int i = 0;i < accelGyroMagXAxis.size();i++) {
        accelGyroMagXAxis[i] = (20.0 / 1000.0 * i);
    }
    altitudeData.resize(maxSampleSize);
    ultraDistData.resize(maxSampleSize);

    for(unsigned int i = 0;i < sizeof(mQuadUpdateTime) / sizeof(int);i++) {
        mQuadUpdateTime[i] = 0;
    }

    for (int i = 0;i < mQuadLabelNum;i++) {
        mQuadLabels[i] = new QLabel(this);
        mQuadLabels[i]->setMinimumWidth(120);
        mQuadLabels[i]->setAlignment(Qt::AlignCenter);
        ui->statusBar->addPermanentWidget(mQuadLabels[i]);
    }

    // Red emergency stop button
    ui->emergencyStopButton->setStyleSheet("QPushButton {background-color: red; color: black;}");

    on_joystickConnectButton_clicked();
    on_serialConnectButton_clicked();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::readPendingDatagrams()
{
    while (mUdpSocket->hasPendingDatagrams()) {
        QByteArray data;
        data.resize(mUdpSocket->pendingDatagramSize());
        QHostAddress sender;
        quint16 senderPort;

        const double f_samp = 200000.0;
        const double trans_delay = 50.0;

        mUdpSocket->readDatagram(data.data(), data.size(),
                                 &sender, &senderPort);
        
        if (data.size() > 0) {
            int cmd = data[0];
            data.remove(0, 1);
            
            if (cmd == CMD_SERVER_TEXT) {
                // Print command
                ui->terminalBrowser->append(data);
            } else if (cmd == CMD_SERVER_LOGGED_PULSE) {
                qDebug() << "Received pulse data: " << data.size() << "bytes";
                // Add data
                recData.clear();
                QVector<double> xAxis;

                for(int i = 0;i < data.size();i++) {
                    double number = (double)((quint8)data[i]);
                    number -= 127.0;
                    number *= 3.3 / 4096.0;
                    recData.append(number);
                    xAxis.append(1.0 / f_samp * i);

                    //qDebug() << (quint8)data[i];
                }

                ui->recPlot->clearGraphs();
                ui->recPlot->addGraph();
                ui->recPlot->graph()->setPen(QPen(Qt::blue));
                ui->recPlot->graph()->setData(xAxis, recData);
                ui->recPlot->graph()->rescaleAxes();
                ui->recPlot->xAxis->setLabel("Seconds");
                ui->recPlot->yAxis->setLabel("Volts");
                ui->recPlot->replot();
            } else if (cmd == CMD_SERVER_LOGGED_CORRELATION) {
                qDebug() << "Received correlation data: " << data.size() << "bytes";
                recData.clear();
                QVector<double> xAxis;

                for(int i = 0;i < data.size();i += 2) {
                    double number = (double)(quint16)((quint16)data[i] << 8);
                    number += (double)(quint8)data[i + 1];
                    recData.append(number);
                    xAxis.append(1.0 / f_samp * i);
                }
                ui->corrPlot->clearGraphs();
                ui->corrPlot->addGraph();
                ui->corrPlot->graph()->setPen(QPen(Qt::black));
                ui->corrPlot->graph()->setData(xAxis, recData);
                ui->corrPlot->graph()->rescaleAxes();
                ui->corrPlot->xAxis->setLabel("Seconds");
                ui->corrPlot->yAxis->setLabel("Correlation");
                ui->corrPlot->replot();
            } else if (cmd == CMD_SERVER_LOGGED_CORRELATION_PART) {
                data.remove(0, 1); // Remove dummy byte
                quint16 *data_u16 = (quint16*)data.data();
                quint16 start = ntohs(*data_u16++);
                int len = (data.size() - 2) / 2;
                longCorrData.resize(start + len);
                qDebug() << "Start:" << start << "Len:" << len;
                QVector<double> xAxis;

                for(int i = 0;i < len;i++) {
                    quint16 samp = ntohs(data_u16[i]);
                    longCorrData[start + i] = samp;
                }

                for (int i = 0;i < longCorrData.size();i++) {
                    xAxis.append(1.0 / f_samp * 340.0 * (i - trans_delay));
                }

                ui->longCorrPlot->clearGraphs();
                ui->longCorrPlot->addGraph();
                ui->longCorrPlot->graph()->setPen(QPen(Qt::black));
                ui->longCorrPlot->graph()->setData(xAxis, longCorrData);
                ui->longCorrPlot->graph()->rescaleAxes();
                ui->longCorrPlot->xAxis->setLabel("Meters");
                ui->longCorrPlot->yAxis->setLabel("Correlation");
                ui->longCorrPlot->replot();
            } else if (cmd == CMD_SERVER_LOGGED_PULSE_PART) {
                data.remove(0, 1); // Remove dummy byte
                quint16 *data_u16 = (quint16*)data.data();
                quint16 start = ntohs(*data_u16++);
                int len = (data.size() - 2) / 2;
                longRecData.resize(start + len);
                qDebug() << "Start:" << start << "Len:" << len;
                QVector<double> xAxis;

                for(int i = 0;i < len;i++) {
                    quint16 samp = ntohs(data_u16[i]);
                    longRecData[start + i] = samp;
                }

                for (int i = 0;i < longRecData.size();i++) {
                    xAxis.append(1.0 / f_samp * 340.0 * (i - trans_delay));
                }

                ui->longRecPlot->clearGraphs();
                ui->longRecPlot->addGraph();
                ui->longRecPlot->graph()->setPen(QPen(Qt::black));
                ui->longRecPlot->graph()->setData(xAxis, longRecData);
                ui->longRecPlot->graph()->rescaleAxes();
                ui->longRecPlot->xAxis->setLabel("Meters");
                ui->longRecPlot->yAxis->setLabel("Amplitude");
                ui->longRecPlot->replot();
            } else if (cmd == CMD_SERVER_DISTANCES) {
                int num_pos = data.size() / 4;
                quint32 rec_pos[num_pos];

                for (int i = 0;i < num_pos;i++) {
                    rec_pos[i] = (quint8)data[4 * i] << 24;
                    rec_pos[i] |= (quint8)data[4 * i + 1] << 16;
                    rec_pos[i] |= (quint8)data[4 * i + 2] << 8;
                    rec_pos[i] |= (quint8)data[4 * i + 3];
                }

                const double d0 = (double)rec_pos[0];
                const double d1 = (double)rec_pos[1];
                const double b = 2100.0;

                float x = (d0 * d0 - d1 * d1 + b * b) / (2.0 * b);
                float y = sqrt(fabs(d0 * d0 - x * x));

                LocPoint loc(x, y);
                LocPoint a0Loc(0, 0);
                LocPoint a1Loc(b, 0);
                ui->mapWidget->getQuadInfo(0)->setLocation(loc);
                ui->mapWidget->getAnchorInfo(0)->setLocation(a0Loc);
                ui->mapWidget->getAnchorInfo(1)->setLocation(a1Loc);
                ui->mapWidget->repaint();
                qDebug() << "D0:" << d0 << "D1:" << d1;
                qDebug() << "X:" << x << "Y:" << y;
            } else if (cmd == CMD_SERVER_QUAD_PACKET) {
                mSerialComm->packetReceived(data);
            }
        }
    }
}

void MainWindow::timerSlot()
{
    if (repaintMap) {
        repaintMap = false;
        ui->mapWidget->repaint();
    }

    static bool quad_present[mQuadLabelNum] = {true};
    for(unsigned int i = 0;i < mQuadLabelNum;i++) {
        if (mQuadUpdateTime[i] < 50) {
            mQuadUpdateTime[i]++;

            quad_present[i] = true;

            double val = mQuadStatus[i].vbat;
            const double batt_high = 11.8;
            const double batt_low = 10.5;
            QString str;

            str.sprintf("Q%i v%i.%i, %.2fV",
                        i, mQuadStatus[i].fw_version_major, mQuadStatus[i].fw_version_minor, val);
            if (val > batt_high) {
                mQuadLabels[i]->setStyleSheet("QLabel { background-color : lightgreen; color : black; }");
            } else if (val > batt_low) {
                mQuadLabels[i]->setStyleSheet("QLabel { background-color : yellow; color : black; }");
            } else {
                mQuadLabels[i]->setStyleSheet("QLabel { background-color : red; color : black; }");
            }
            mQuadLabels[i]->setText(str);
        } else {
            if (quad_present[i]) {
                quad_present[i] = false;
                mQuadLabels[i]->setStyleSheet(qApp->styleSheet());
                mQuadLabels[i]->setText("N/A");
                ui->mapWidget->removeQuad(i);
                ui->mapWidget->removeLdmElement(i);
                repaintMap = true;
            }
        }
    }

    ui->mapWidget->setDrawTrace(ui->drawTraceBoxBox->isChecked());

    static bool serialWasOpen = false;
    if (serialWasOpen != mSerialPort->isOpen()) {
        if (mSerialPort->isOpen()) {
            ui->serialConnectedLabel->setText(tr("Connected"));
        } else {
            ui->serialConnectedLabel->setText(tr("Not connected"));
        }
    }
    serialWasOpen = mSerialPort->isOpen();

    static int lastMagSamples = 0;
    if (magSamples.size() != lastMagSamples) {
        ui->magCalSampleLabel->setText(QString::number(magSamples.size()) + " Samples");
    }
    lastMagSamples = magSamples.size();

    if (ui->emergencyStopButton->isChecked()) {
        mSerialComm->emergencyStop(255);
        on_allAutolandButton_clicked();
    }

    if (ui->demo0Button->isChecked()) {
        mSerialComm->setSticks(0, 0.5, 0.5, 0.5, 1.0, 0.0, 0.0);
    }

    if (ui->demo1Button->isChecked()) {
        mSerialComm->setSticks(1, 0.5, 0.5, 0.5, 1.0, 0.0, 0.0);
    }

    if (ui->demo2Button->isChecked()) {
        mSerialComm->setSticks(2, 0.5, 0.5, 0.5, 1.0, 0.0, 0.0);
    }

    if (ui->demo3Button->isChecked()) {
        mSerialComm->setSticks(3, 0.5, 0.5, 0.5, 1.0, 0.0, 0.0);
    }

    if (ui->demo4Button->isChecked()) {
        mSerialComm->setSticks(4, 0.5, 0.5, 0.5, 1.0, 0.0, 0.0);
    }

    if (ui->demo5Button->isChecked()) {
        mSerialComm->setSticks(5, 0.5, 0.5, 0.5, 1.0, 0.0, 0.0);
    }

    if (mJoystick->isConnected()) {
        ui->joystickConnectedLabel->setText(tr("Connected"));
        ui->joystickA1Bar->setValue(mJoystick->getAxis(0));
        ui->joystickA2Bar->setValue(mJoystick->getAxis(1));
        ui->joystickA3Bar->setValue(mJoystick->getAxis(2));
        ui->joystickA4Bar->setValue(mJoystick->getAxis(4));

        if (ui->joystickSendBox->isChecked()) {
            double throttle = (double)mJoystick->getAxis(2) / 65536.0 + 0.5;
            double roll = -(double)mJoystick->getAxis(0) / 65536.0 + 0.5;
            double pitch = -(double)mJoystick->getAxis(1) / 65536.0 + 0.5;
            double yaw = (double)mJoystick->getAxis(4) / 65536.0 + 0.5;
            double pot1 = 1.0;
            double pot2 = 0.0;

            if (ui->modeSimpleButton->isChecked()) {
                pot1 = 0.0;
            }

            mSerialComm->setSticks(ui->selectedQuadBox->value(), roll, pitch, yaw, throttle, pot1, pot2);
        }
    } else {
        ui->joystickConnectedLabel->setText(tr("Not connected"));
    }

    if (ui->overridePowerBox->isChecked()) {
        QVector<double> pwm;

        pwm.append(ui->motor1Button->isChecked() ? ((double)ui->motorPowerBox->value() / 100.0) : (int32_t)0.0);
        pwm.append(ui->motor2Button->isChecked() ? ((double)ui->motorPowerBox->value() / 100.0) : (int32_t)0.0);
        pwm.append(ui->motor3Button->isChecked() ? ((double)ui->motorPowerBox->value() / 100.0) : (int32_t)0.0);
        pwm.append(ui->motor4Button->isChecked() ? ((double)ui->motorPowerBox->value() / 100.0) : (int32_t)0.0);
        pwm.append(ui->motor5Button->isChecked() ? ((double)ui->motorPowerBox->value() / 100.0) : (int32_t)0.0);
        pwm.append(ui->motor6Button->isChecked() ? ((double)ui->motorPowerBox->value() / 100.0) : (int32_t)0.0);

        mSerialComm->setMotorOverride(ui->selectedQuadBox->value(), pwm);
    }

    if (ui->updateOrientationBox->isChecked()) {
        mSerialComm->getOrientation(ui->selectedQuadBox->value());
    }

    if (ui->updateRawImuBox->isChecked()) {
        mSerialComm->getRawIMU(ui->selectedQuadBox->value());
    }

    if (ui->updatePositionBox->isChecked()) {
        mSerialComm->getPosition(ui->selectedQuadBox->value());
    }

    if (ui->updateAltitudeBox->isChecked()) {
        mSerialComm->getAltitude(ui->selectedQuadBox->value());
    }

    if (ui->updateUltraDistBox->isChecked()) {
        mSerialComm->getUltraDist(ui->selectedQuadBox->value(), ui->ultraDistIdBox->value());
    }
}

void MainWindow::mapPosSet(LocPoint pos)
{
    POS_STATE_t pos_state;
    pos_state.px = pos.getX() / 1000.0;
    pos_state.py = pos.getY() / 1000.0;
    pos_state.vx = 0.0;
    pos_state.vy = 0.0;
    pos_state.yaw = pos.getAngle() * 180.0 / M_PI;

    mSerialComm->setPosition(ui->selectedQuadBox->value(), pos_state);
}

void MainWindow::serialDataAvailable()
{
    while (mSerialPort->bytesAvailable() > 0) {
        QByteArray data = mSerialPort->readAll();
        mPacketInterface->processData(data);
    }
}

void MainWindow::commDataToSend(QByteArray data)
{
    mPacketInterface->sendPacket(data);
}

void MainWindow::packetDataToSend(QByteArray &data)
{
    if (mSerialPort->isOpen()) {
        mSerialPort->writeData(data.data(), data.size());
    }
}

void MainWindow::controlParametersReceived(CONTROL_PARAMETERS_t ctrl_param)
{
    ui->rollPBox->setValue(ctrl_param.roll_p_gain);
    ui->rollIBox->setValue(ctrl_param.roll_i_gain);
    ui->rollDPBox->setValue(ctrl_param.roll_d_gain_process);
    ui->rollDEBox->setValue(ctrl_param.roll_d_gain_error);

    ui->pitchPBox->setValue(ctrl_param.pitch_p_gain);
    ui->pitchIBox->setValue(ctrl_param.pitch_i_gain);
    ui->pitchDPBox->setValue(ctrl_param.pitch_d_gain_process);
    ui->pitchDEBox->setValue(ctrl_param.pitch_d_gain_error);

    ui->yawPBox->setValue(ctrl_param.yaw_p_gain);
    ui->yawIBox->setValue(ctrl_param.yaw_i_gain);
    ui->yawDPBox->setValue(ctrl_param.yaw_d_gain_process);
    ui->yawDEBox->setValue(ctrl_param.yaw_d_gain_error);
}

void MainWindow::orientationReceived(POS_STATE_t pos_state)
{
    ui->orientationWidget->setRollPitchYaw(pos_state.roll,
                                           pos_state.pitch,
                                           pos_state.yaw);
    //qDebug() << "Roll: " << pos_state.roll << "Pitch" << pos_state.pitch << "Yaw" << pos_state.yaw;
}

void MainWindow::rawIMUReceived(RAW_IMU_t imu_values)
{
    QVector<double> magXYZ;

    ui->accXBar->setValue((int)(imu_values.accX * 1000.0));
    accelXData.append(imu_values.accX);
    accelXData.remove(0, 1);

    ui->accYBar->setValue((int)(imu_values.accY * 1000.0));
    accelYData.append(imu_values.accY);
    accelYData.remove(0, 1);

    ui->accZBar->setValue((int)(imu_values.accZ * 1000.0));
    accelZData.append(imu_values.accZ);
    accelZData.remove(0, 1);

    ui->rawAccelPlot->clearGraphs();
    ui->rawAccelPlot->addGraph();
    ui->rawAccelPlot->graph()->setPen(QPen(Qt::black));
    ui->rawAccelPlot->graph()->setData(accelGyroMagXAxis, accelXData);
    ui->rawAccelPlot->graph()->setName(tr("X"));
    ui->rawAccelPlot->addGraph();
    ui->rawAccelPlot->graph()->setPen(QPen(Qt::green));
    ui->rawAccelPlot->graph()->setData(accelGyroMagXAxis, accelYData);
    ui->rawAccelPlot->graph()->setName(tr("Y"));
    ui->rawAccelPlot->addGraph();
    ui->rawAccelPlot->graph()->setPen(QPen(Qt::blue));
    ui->rawAccelPlot->graph()->setData(accelGyroMagXAxis, accelZData);
    ui->rawAccelPlot->graph()->setName(tr("Z"));
    ui->rawAccelPlot->rescaleAxes();
    ui->rawAccelPlot->xAxis->setLabel("Seconds");
    ui->rawAccelPlot->yAxis->setLabel("G");
    ui->rawAccelPlot->legend->setVisible(true);
    ui->rawAccelPlot->replot();

    ui->gyroXBar->setValue((int)(imu_values.gyroX));
    gyroXData.append(imu_values.gyroX);
    gyroXData.remove(0, 1);

    ui->gyroYBar->setValue((int)(imu_values.gyroY));
    gyroYData.append(imu_values.gyroY);
    gyroYData.remove(0, 1);

    ui->gyroZBar->setValue((int)(imu_values.gyroZ));
    gyroZData.append(imu_values.gyroZ);
    gyroZData.remove(0, 1);

    ui->rawGyroPlot->clearGraphs();
    ui->rawGyroPlot->addGraph();
    ui->rawGyroPlot->graph()->setPen(QPen(Qt::black));
    ui->rawGyroPlot->graph()->setData(accelGyroMagXAxis, gyroXData);
    ui->rawGyroPlot->graph()->setName(tr("X"));
    ui->rawGyroPlot->addGraph();
    ui->rawGyroPlot->graph()->setPen(QPen(Qt::green));
    ui->rawGyroPlot->graph()->setData(accelGyroMagXAxis, gyroYData);
    ui->rawGyroPlot->graph()->setName(tr("Y"));
    ui->rawGyroPlot->addGraph();
    ui->rawGyroPlot->graph()->setPen(QPen(Qt::blue));
    ui->rawGyroPlot->graph()->setData(accelGyroMagXAxis, gyroZData);
    ui->rawGyroPlot->graph()->setName(tr("Z"));
    ui->rawGyroPlot->rescaleAxes();
    ui->rawGyroPlot->xAxis->setLabel("Seconds");
    ui->rawGyroPlot->yAxis->setLabel("deg/s");
    ui->rawGyroPlot->legend->setVisible(true);
    ui->rawGyroPlot->replot();

    magXYZ.clear();

    ui->magXBar->setValue((int)(imu_values.magX));
    magXData.append(imu_values.magX);
    magXData.remove(0, 1);
    magXYZ.append(imu_values.magX);

    ui->magYBar->setValue((int)(imu_values.magY));
    magYData.append(imu_values.magY);
    magYData.remove(0, 1);
    magXYZ.append(imu_values.magY);

    ui->magZBar->setValue((int)(imu_values.magZ));
    magZData.append(imu_values.magZ);
    magZData.remove(0, 1);
    magXYZ.append(imu_values.magZ);

    if (ui->storeMagBox->isChecked()) {
        magSamples.append(magXYZ);
    }

    ui->rawMagPlot->clearGraphs();
    ui->rawMagPlot->addGraph();
    ui->rawMagPlot->graph()->setPen(QPen(Qt::black));
    ui->rawMagPlot->graph()->setData(accelGyroMagXAxis, magXData);
    ui->rawMagPlot->graph()->setName(tr("X"));
    ui->rawMagPlot->addGraph();
    ui->rawMagPlot->graph()->setPen(QPen(Qt::green));
    ui->rawMagPlot->graph()->setData(accelGyroMagXAxis, magYData);
    ui->rawMagPlot->graph()->setName(tr("Y"));
    ui->rawMagPlot->addGraph();
    ui->rawMagPlot->graph()->setPen(QPen(Qt::blue));
    ui->rawMagPlot->graph()->setData(accelGyroMagXAxis, magZData);
    ui->rawMagPlot->graph()->setName(tr("Z"));
    ui->rawMagPlot->rescaleAxes();
    ui->rawMagPlot->xAxis->setLabel("Seconds");
    ui->rawMagPlot->yAxis->setLabel("uT");
    ui->rawMagPlot->legend->setVisible(true);
    ui->rawMagPlot->replot();
}

void MainWindow::printReceived(QString str)
{
    ui->quadTerminalBrowser->append(str);
}

void MainWindow::positionReceived(int id, int type, POS_STATE_t pos_state)
{
    LocPoint loc;
    loc.setXY(pos_state.px * 1000.0, pos_state.py * 1000.0);
    loc.setAngle(pos_state.yaw * M_PI / 180.0);
    loc.setRollPitch(pos_state.roll * M_PI / 180.0, pos_state.pitch * M_PI / 180.0);

    if ((type & POS_MASK_TYPE) == POS_TYPE_NORMAL) {
        ObjectInfo *quad = ui->mapWidget->getQuadInfo(id);

        if (quad) {
            quad->setLocation(loc);
        } else {
            ObjectInfo new_quad = ObjectInfo(0, QString().sprintf("Quad %d", id), Qt::blue);
            new_quad.setLocation(loc);
            ui->mapWidget->addQuad(id, new_quad);
            quad = ui->mapWidget->getQuadInfo(id);
        }

        quad->setType(type);

        if (type & POS_MASK_FOLLOWING) {
            ui->orientationWidget->setRollPitchYaw(pos_state.roll,
                                                   pos_state.pitch,
                                                   pos_state.yaw);
            quad->setColor(Qt::yellow);
        } else {
            quad->setColor(Qt::blue);
        }
    } else if ((type & POS_MASK_TYPE) == POS_TYPE_COLLISION) {
        ObjectInfo coll = ObjectInfo(0, QString().sprintf("Collision %d", id), Qt::blue);
        coll.setLocation(loc);
        ui->mapWidget->addCollision(id, coll);
    }

    repaintMap = true;
}

void MainWindow::altitudeReceived(float altitude)
{
    ui->altitudeBar->setValue((int)(altitude * 100.0));
    altitudeData.append(altitude);
    altitudeData.remove(0, 1);

    ui->altitudePlot->clearGraphs();
    ui->altitudePlot->addGraph();
    ui->altitudePlot->graph()->setPen(QPen(Qt::black));
    ui->altitudePlot->graph()->setData(accelGyroMagXAxis, altitudeData);
    ui->altitudePlot->graph()->setName(tr("Altitude"));
    ui->altitudePlot->rescaleAxes();
    ui->altitudePlot->xAxis->setLabel("Seconds");
    ui->altitudePlot->yAxis->setLabel("Meters");
    ui->altitudePlot->legend->setVisible(true);
    ui->altitudePlot->replot();
}

void MainWindow::ppmReceived(PPM_RADIO_t ppm_state) {
    ui->joystickA1Bar->setValue((ppm_state.roll -0.5) * 65536);
    ui->joystickA2Bar->setValue((ppm_state.pitch -0.5) * 65536);
    ui->joystickA3Bar->setValue((ppm_state.throttle -0.5) * 65536);
    ui->joystickA4Bar->setValue((ppm_state.yaw -0.5) * 65536);
    ui->joystickA5Bar->setValue((ppm_state.pot1 -0.5) * 65536);
    ui->joystickA6Bar->setValue((ppm_state.pot2 -0.5) * 65536);
}

void MainWindow::mapCmdReceived(MAP_CMD_t cmd, int param)
{
    switch (cmd) {
    case MAP_CMD_REMOVE_QUAD:
        ui->mapWidget->removeQuad(param);
        break;

    case MAP_CMD_REMOVE_ALL_QUADS:
        ui->mapWidget->removeAllQuads();
        break;

    case MAP_CMD_REMOVE_COLLISION:
        ui->mapWidget->removeCollision(param);
        break;

    case MAP_CMD_REMOVE_ALL_COLLISIONS:
        ui->mapWidget->removeAllCollisions();
        break;

    case MAP_CMD_REMOVE_ALL_RISKS:
        ui->mapWidget->removeAllLddmElements();
        break;

    case MAP_CMD_REMOVE_ALL_LINE_SEGMENTS:
        ui->mapWidget->removeAllLineSegments();
        break;

    case MAP_CMD_REMOVE_ANCHOR:
        ui->mapWidget->removeAnchor(param);
        break;

    case MAP_CMD_REMOVE_ALL_ANCHORS:
        ui->mapWidget->removeAllAnchors();
        break;

    default:
        break;
    }

    repaintMap = true;
}

void MainWindow::LdmElementReceived(LDM_ELEMENT_t ldmElement)
{
    LDM_ELEMENT_t *element = ui->mapWidget->getLdmElement(ldmElement.id);

    if (element) {
        *element = ldmElement;
    } else {
        ui->mapWidget->addLdmElement(ldmElement);
    }

    repaintMap = true;
}

void MainWindow::lineSegmentReceived(int id, SEGMENT_t segment)
{
    ui->mapWidget->addLineSegment(id, segment);
    repaintMap = true;
}

void MainWindow::ultraDistReceived(qint32 id, float dist)
{
    (void)id;
    ultraDistData.append(dist);
    ultraDistData.remove(0, 1);

    ui->ultraDistPlot->clearGraphs();
    ui->ultraDistPlot->addGraph();
    ui->ultraDistPlot->graph()->setPen(QPen(Qt::black));
    ui->ultraDistPlot->graph()->setData(accelGyroMagXAxis, ultraDistData);
    ui->ultraDistPlot->graph()->setName(tr("Distance"));
    ui->ultraDistPlot->rescaleAxes();
    ui->ultraDistPlot->xAxis->setLabel("Seconds");
    ui->ultraDistPlot->yAxis->setLabel("Meters");
    ui->ultraDistPlot->legend->setVisible(true);
    ui->ultraDistPlot->replot();
}

void MainWindow::aliveReceived(int id)
{
    (void)id;
}

void MainWindow::safetyMsgReceived(int id, SAFETY_MSG_t msg)
{
    (void)id;

    LDM_ELEMENT_t *element = ui->mapWidget->getLdmElement(msg.id);

    if (!element) {
        LDM_ELEMENT_t new_element;
        new_element.id = msg.id;
        new_element.type = LDM_ELEMENT_TYPE_COMFORT_ZONE;
        new_element.risk.px = 0.0;
        new_element.risk.py = 0.0;
        new_element.risk.rotation = 0.0;
        ui->mapWidget->addLdmElement(new_element);
        element = ui->mapWidget->getLdmElement(msg.id);
    }

    element->id = msg.id;
    element->timestamp = 0;
    element->risk.has_collision = msg.los_global == LEVEL_OF_SERVICE_HIGH ? 0 : 1;

    switch (msg.los_local) {
    case LEVEL_OF_SERVICE_HIGH:
        element->risk.height = 450.0;
        element->risk.width = 450.0;
        break;

    case LEVEL_OF_SERVICE_MEDIUM:
        element->risk.height = 800.0;
        element->risk.width = 800.0;
        break;

    case LEVEL_OF_SERVICE_LOW:
        element->risk.height = 1500.0;
        element->risk.width = 1500.0;
        break;

    default:
        break;
    }

    repaintMap = true;
}

void MainWindow::statusMsgReceived(int id, STATUS_MSG_t msg)
{
    (void)id;

    LDM_ELEMENT_t *element = ui->mapWidget->getLdmElement(msg.id);

    if (!element) {
        LDM_ELEMENT_t new_element;
        new_element.id = msg.id;
        new_element.type = LDM_ELEMENT_TYPE_COMFORT_ZONE;
        new_element.risk.px = 0.0;
        new_element.risk.py = 0.0;
        new_element.risk.rotation = 0.0;
        ui->mapWidget->addLdmElement(new_element);
        element = ui->mapWidget->getLdmElement(msg.id);
    }

    element->id = msg.id;
    element->timestamp = 0;
    element->risk.px = msg.px * 1000.0;
    element->risk.py = msg.py * 1000.0;

    ObjectInfo *quad = ui->mapWidget->getQuadInfo(msg.id);
    if (!quad) {
        ObjectInfo new_quad = ObjectInfo(0, QString().sprintf("Quad %d", msg.id), Qt::blue);
        ui->mapWidget->addQuad(msg.id, new_quad);
        quad = ui->mapWidget->getQuadInfo(msg.id);
    }

    LocPoint loc;
    loc.setXY(msg.px * 1000.0, msg.py * 1000.0);
    loc.setAngle(msg.yaw * M_PI / 180.0);
    loc.setRollPitch(msg.roll * M_PI / 180.0, msg.pitch * M_PI / 180.0);
    quad->setLocation(loc);

    quad->setVBat(msg.vbat);
    quad->setAdcVals(msg.adc_in, 4);

    if (msg.id >= 0 && msg.id < mQuadLabelNum) {
        mQuadStatus[msg.id] = msg;
        mQuadUpdateTime[msg.id] = 0;
    }

    repaintMap = true;
}

void MainWindow::anchorsReceived(int id, QVector<ANCHOR_SETTINGS_t> anchors, MAP_LIMITS_t map_lim)
{
    (void)id;

    guiSetAnchorsLimits(anchors, map_lim);
    plotAnchorsOnMap(anchors, map_lim);
}

void MainWindow::on_measureShortButton_clicked()
{
    clearPlotsAndData();

    QByteArray sendData;
    sendData.append((char) CMD_CLIENT_SEND_PULSE_SAMPLE);
    sendData.append((char)ui->receiverUsBox->value());
    QHostAddress address(ui->addressReceiver->text());
    mUdpSocket->writeDatagram(sendData, address, 3000);
}

void MainWindow::on_measureAllButton_clicked()
{
    clearPlotsAndData();

    QByteArray sendData;
    sendData.append((char) CMD_CLIENT_SEND_PULSE_SAMPLE_ALLDATA);
    sendData.append((char)ui->receiverUsBox->value());
    QHostAddress address(ui->addressReceiver->text());
    mUdpSocket->writeDatagram(sendData, address, 3000);
}

void MainWindow::clearPlotsAndData()
{
    ui->recPlot->clearGraphs();
    ui->recPlot->replot();
    ui->corrPlot->clearGraphs();
    ui->corrPlot->replot();
    ui->longRecPlot->clearGraphs();
    ui->longRecPlot->replot();
    ui->longCorrPlot->clearGraphs();
    ui->longCorrPlot->replot();
    longRecData.clear();
    longCorrData.clear();
}

void MainWindow::setAnchors(int quadId)
{
    QVector<ANCHOR_SETTINGS_t> anchors;
    MAP_LIMITS_t map_lim;

    guiReadAnchorsLimits(anchors, map_lim);

    mSerialComm->setAnchors(quadId, anchors, map_lim);
    plotAnchorsOnMap(anchors, map_lim);
}

void MainWindow::plotAnchorsOnMap(const QVector<ANCHOR_SETTINGS_t> &anchors, const MAP_LIMITS_t &map_lim)
{
    ui->mapWidget->removeAllAnchors();
    for (int i = 0;i < anchors.size();i++) {
        QString name;
        LocPoint aLoc(anchors[i].px * 1000.0, anchors[i].py * 1000.0);
        name.sprintf("Anchor %i", anchors[i].id);
        ui->mapWidget->addAnchor(ObjectInfo(0, name, Qt::red));
        ui->mapWidget->getAnchorInfo(i)->setLocation(aLoc);
    }

    if (fabs(map_lim.max_x - map_lim.min_x) > 0.001 && fabs(map_lim.max_y - map_lim.min_y) > 0.001) {
        ui->mapWidget->removeAllLineSegments();
        SEGMENT_t seg;

        seg.p1.x = map_lim.min_x;
        seg.p1.y = map_lim.min_y;
        seg.p2.x = map_lim.max_x;
        seg.p2.y = map_lim.min_y;
        ui->mapWidget->addLineSegment(110, seg);

        seg.p1.x = map_lim.max_x;
        seg.p1.y = map_lim.min_y;
        seg.p2.x = map_lim.max_x;
        seg.p2.y = map_lim.max_y;
        ui->mapWidget->addLineSegment(111, seg);

        seg.p1.x = map_lim.max_x;
        seg.p1.y = map_lim.max_y;
        seg.p2.x = map_lim.min_x;
        seg.p2.y = map_lim.max_y;
        ui->mapWidget->addLineSegment(112, seg);

        seg.p1.x = map_lim.min_x;
        seg.p1.y = map_lim.max_y;
        seg.p2.x = map_lim.min_x;
        seg.p2.y = map_lim.min_y;
        ui->mapWidget->addLineSegment(113, seg);
    }
}

void MainWindow::guiReadAnchorsLimits(QVector<ANCHOR_SETTINGS_t> &anchors, MAP_LIMITS_t &map_lim)
{
    anchors.resize(4);

    map_lim.min_x = ui->mapLimMinXBox->value();
    map_lim.max_x = ui->mapLimMaxXBox->value();
    map_lim.min_y = ui->mapLimMinYBox->value();
    map_lim.max_y = ui->mapLimMaxYBox->value();

    anchors[0].id = ui->anch0IdBox->value();
    anchors[0].px = ui->anch0PxBox->value();
    anchors[0].py = ui->anch0PyBox->value();
    anchors[0].pz = ui->anch0PzBox->value();

    anchors[1].id = ui->anch1IdBox->value();
    anchors[1].px = ui->anch1PxBox->value();
    anchors[1].py = ui->anch1PyBox->value();
    anchors[1].pz = ui->anch1PzBox->value();

    anchors[2].id = ui->anch2IdBox->value();
    anchors[2].px = ui->anch2PxBox->value();
    anchors[2].py = ui->anch2PyBox->value();
    anchors[2].pz = ui->anch2PzBox->value();

    anchors[3].id = ui->anch3IdBox->value();
    anchors[3].px = ui->anch3PxBox->value();
    anchors[3].py = ui->anch3PyBox->value();
    anchors[3].pz = ui->anch3PzBox->value();
}

void MainWindow::guiSetAnchorsLimits(const QVector<ANCHOR_SETTINGS_t> &anchors, const MAP_LIMITS_t &map_lim)
{
    ui->mapLimMinXBox->setValue(map_lim.min_x);
    ui->mapLimMaxXBox->setValue(map_lim.max_x);
    ui->mapLimMinYBox->setValue(map_lim.min_y);
    ui->mapLimMaxYBox->setValue(map_lim.max_y);

    if (anchors.size() > 0) {
        ui->anch0IdBox->setValue(anchors[0].id);
        ui->anch0PxBox->setValue(anchors[0].px);
        ui->anch0PyBox->setValue(anchors[0].py);
        ui->anch0PzBox->setValue(anchors[0].pz);
    }

    if (anchors.size() > 1) {
        ui->anch1IdBox->setValue(anchors[1].id);
        ui->anch1PxBox->setValue(anchors[1].px);
        ui->anch1PyBox->setValue(anchors[1].py);
        ui->anch1PzBox->setValue(anchors[1].pz);
    }

    if (anchors.size() > 2) {
        ui->anch2IdBox->setValue(anchors[2].id);
        ui->anch2PxBox->setValue(anchors[2].px);
        ui->anch2PyBox->setValue(anchors[2].py);
        ui->anch2PzBox->setValue(anchors[2].pz);
    }

    if (anchors.size() > 3) {
        ui->anch3IdBox->setValue(anchors[3].id);
        ui->anch3PxBox->setValue(anchors[3].px);
        ui->anch3PyBox->setValue(anchors[3].py);
        ui->anch3PzBox->setValue(anchors[3].pz);
    }
}

void MainWindow::on_clearTraceButton_clicked()
{
    ui->mapWidget->clearTrace();
}

void MainWindow::on_pidSetParametersButton_clicked()
{
    CONTROL_PARAMETERS_t ctrl_param;

    ctrl_param.roll_p_gain = ui->rollPBox->value();
    ctrl_param.roll_i_gain = ui->rollIBox->value();
    ctrl_param.roll_d_gain_process = ui->rollDPBox->value();
    ctrl_param.roll_d_gain_error = ui->rollDEBox->value();

    ctrl_param.pitch_p_gain = ui->pitchPBox->value();
    ctrl_param.pitch_i_gain = ui->pitchIBox->value();
    ctrl_param.pitch_d_gain_process = ui->pitchDPBox->value();
    ctrl_param.pitch_d_gain_error = ui->pitchDEBox->value();

    ctrl_param.yaw_p_gain = ui->yawPBox->value();
    ctrl_param.yaw_i_gain = ui->yawIBox->value();
    ctrl_param.yaw_d_gain_process = ui->yawDPBox->value();
    ctrl_param.yaw_d_gain_error = ui->yawDEBox->value();

    mSerialComm->setControlParameters(ui->selectedQuadBox->value(), ctrl_param);
}

void MainWindow::on_pidGetParametersButton_clicked()
{
    mSerialComm->getControlParameters(ui->selectedQuadBox->value());
}

void MainWindow::on_serialConnectButton_clicked()
{
    mSerialPort->openPort(ui->serialPortEdit->text());
}

void MainWindow::on_serialDisconnectButton_clicked()
{
    mSerialPort->closePort();
}

void MainWindow::on_zeroGyroSticksButton_clicked()
{
    mSerialComm->zeroGyroSticks(ui->selectedQuadBox->value());
}

void MainWindow::on_joystickConnectButton_clicked()
{
    QString port = ui->joystickPortEdit->text();
    if (mJoystick->init(port) == 0) {
        qDebug() << "Axes:" << mJoystick->numAxes();
        qDebug() << "Buttons:" << mJoystick->numButtons();
        qDebug() << "Name:" << mJoystick->getName();
    } else {
        qWarning() << "Opening joystick failed.";
    }
}

void MainWindow::on_joystickDisconnectButton_clicked()
{
    mJoystick->stop();
}

void MainWindow::on_zeroOrientationButton_clicked()
{
    mSerialComm->zeroOrientation(ui->selectedQuadBox->value());
}

void MainWindow::on_clearSampleButton_clicked()
{
    magSamples.clear();
}

void MainWindow::on_saveMagButton_clicked()
{
    QString path;
    path = QFileDialog::getSaveFileName(this, tr("Choose where to save the magnetometer samples"));
    if (path.isNull()) {
        return;
    }

    QFile file(path);
    file.open(QIODevice::WriteOnly | QIODevice::Text);
    QTextStream out(&file);

    QVectorIterator<QVector<double> > i(magSamples);
    while (i.hasNext()) {
        QVector<double> element = i.next();
        out << element[0] << "\t" << element[1] << "\t" << element[2] << "\n";
    }

    file.close();
}

void MainWindow::on_zeroPosButton_clicked()
{
    mSerialComm->setYaw(ui->selectedQuadBox->value(), 0.0);
}

void MainWindow::on_clearMapButton_clicked()
{
    ui->mapWidget->clearTrace();
    ui->mapWidget->removeAllCollisions();
    ui->mapWidget->removeAllLddmElements();
    ui->mapWidget->removeAllLineSegments();
    ui->mapWidget->removeAllQuads();
    repaintMap = true;
}

void MainWindow::on_getAnchorsButton_clicked()
{
    mSerialComm->getAnchors(ui->selectedQuadBox->value());
}

void MainWindow::on_setAnchorsButton_clicked()
{
    setAnchors(ui->selectedQuadBox->value());
}

void MainWindow::on_allSetAnchorsButton_clicked()
{
    // Note: Three attempts are made
    for (int i = 0;i < 3;i++) {
        setAnchors(255);
    }
}

void MainWindow::on_allDemoButton_clicked()
{
    ui->demo0Button->setChecked(true);
    ui->demo1Button->setChecked(true);
    ui->demo2Button->setChecked(true);
    ui->demo3Button->setChecked(true);
    ui->demo4Button->setChecked(true);
    ui->demo5Button->setChecked(true);
}

void MainWindow::on_allAutolandButton_clicked()
{
    ui->demo0Button->setChecked(false);
    ui->demo1Button->setChecked(false);
    ui->demo2Button->setChecked(false);
    ui->demo3Button->setChecked(false);
    ui->demo4Button->setChecked(false);
    ui->demo5Button->setChecked(false);
}

void MainWindow::on_allZeroPosButton_clicked()
{
    // Note: Three attempts are made
    for (int i = 0;i < 3;i++) {
        mSerialComm->setYaw(255, 0.0);
    }
}

void MainWindow::on_allZeroGyroSticksButton_clicked()
{
    mSerialComm->zeroGyroSticks(255);
}

void MainWindow::on_fiLosLowButton_clicked()
{
    QByteArray sendData;
    sendData.append((char) CMD_CLIENT_SET_LOS);
    sendData.append((char) LEVEL_OF_SERVICE_LOW);
    QHostAddress address(ui->addressReceiver->text());
    mUdpSocket->writeDatagram(sendData, address, 3000);
}

void MainWindow::on_fiLosMedButton_clicked()
{
    QByteArray sendData;
    sendData.append((char) CMD_CLIENT_SET_LOS);
    sendData.append((char) LEVEL_OF_SERVICE_MEDIUM);
    QHostAddress address(ui->addressReceiver->text());
    mUdpSocket->writeDatagram(sendData, address, 3000);
}

void MainWindow::on_fiLosHighButton_clicked()
{
    QByteArray sendData;
    sendData.append((char) CMD_CLIENT_SET_LOS);
    sendData.append((char) LEVEL_OF_SERVICE_HIGH);
    QHostAddress address(ui->addressReceiver->text());
    mUdpSocket->writeDatagram(sendData, address, 3000);
}

void MainWindow::on_anchSaveXmlButton_clicked()
{
    QVector<ANCHOR_SETTINGS_t> anchors;
    MAP_LIMITS_t map_lim;

    guiReadAnchorsLimits(anchors, map_lim);
    mSerialization->writeAncorConf(anchors, map_lim, this);
}

void MainWindow::on_anchLoadXmlButton_clicked()
{
    QVector<ANCHOR_SETTINGS_t> anchors;
    MAP_LIMITS_t map_lim;
    mSerialization->readAncorConf(anchors, map_lim, this);
    guiSetAnchorsLimits(anchors, map_lim);
    plotAnchorsOnMap(anchors, map_lim);
}

void MainWindow::on_ledLandedLosButton_clicked()
{
    // Note: Three attempts are made
    for (int i = 0;i < 3;i++) {
        mSerialComm->setLedExtIndicationLanded(255, LED_EXT_INDICATION_LANDED_LOS);
    }
}

void MainWindow::on_ledLandedDirButton_clicked()
{
    // Note: Three attempts are made
    for (int i = 0;i < 3;i++) {
        mSerialComm->setLedExtIndicationLanded(255, LED_EXT_INDICATION_LANDED_DIR);
    }
}

void MainWindow::on_ledLandedBothButton_clicked()
{
    // Note: Three attempts are made
    for (int i = 0;i < 3;i++) {
        mSerialComm->setLedExtIndicationLanded(255, LED_EXT_INDICATION_LANDED_LOS_DIR);
    }
}
