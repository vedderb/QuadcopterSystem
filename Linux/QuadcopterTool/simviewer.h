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

#ifndef SIMVIEWER_H
#define SIMVIEWER_H

#include <QWidget>
#include <QTimer>
#include "datatypes.h"

namespace Ui {
class SimViewer;
}

class SimViewer : public QWidget
{
    Q_OBJECT

public:
    explicit SimViewer(QWidget *parent = 0);
    ~SimViewer();

public slots:
    void simCmdReceived(SIM_CMD_t cmd, QByteArray data);

private slots:
    void timerSlot();

private:
    Ui::SimViewer *ui;
    QTimer *mTimer;
    QVector<double> mPosErrorTime;
    QVector<double> mPosError;
    QVector<double> mVelError;
    QVector<double> mFaultTime;
    QVector<double> mFaultPx;
    QVector<double> mFaultPy;
    QVector<double> mFaultRoll;
    QVector<double> mFaultPitch;
    QVector<double> mFaultYaw;
    QVector<double> mFaultRange;
    QVector<double> mFaultComm;
    bool mRepaintGraphs;

};

#endif // SIMVIEWER_H
