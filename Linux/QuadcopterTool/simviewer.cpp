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

#include "simviewer.h"
#include "ui_simviewer.h"
#include "utility.h"

SimViewer::SimViewer(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::SimViewer)
{
    ui->setupUi(this);

    mRepaintGraphs = false;

    // Plots
    ui->posErrorPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui->faultPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    int maxSampleSize = 1000;
    mPosErrorTime.resize(maxSampleSize);
    mPosError.resize(maxSampleSize);
    mVelError.resize(maxSampleSize);
    mFaultTime.resize(maxSampleSize);
    mFaultPx.resize(maxSampleSize);
    mFaultPy.resize(maxSampleSize);
    mFaultRoll.resize(maxSampleSize);
    mFaultPitch.resize(maxSampleSize);
    mFaultYaw.resize(maxSampleSize);
    mFaultRange.resize(maxSampleSize);
    mFaultComm.resize(maxSampleSize);

    mTimer = new QTimer(this);
    mTimer->setInterval(20);
    mTimer->start();
    connect(mTimer, SIGNAL(timeout()), this, SLOT(timerSlot()));
}

SimViewer::~SimViewer()
{
    delete ui;
}

void SimViewer::simCmdReceived(SIM_CMD_t cmd, QByteArray data)
{
    quint8 *data_ptr = (quint8*)data.data();
    qint32 data_ind = 0;
    qint32 tmp_i32 = 0;

    switch (cmd) {
    case SIM_CMD_POS_ERROR:
        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        if (tmp_i32 < mPosErrorTime.last()) {
            for (int i = 0;i < mPosErrorTime.size();i++) {
                mPosErrorTime[i] = 0;
            }
        }
        mPosErrorTime.append((double)tmp_i32 / 1000.0);
        mPosErrorTime.remove(0);

        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        mPosError.append((double)tmp_i32 / 100000.0);
        mPosError.remove(0);

        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        mVelError.append((double)tmp_i32 / 100000.0);
        mVelError.remove(0);

        mRepaintGraphs = true;
        break;

    case SIM_CMD_FAULTS:
        tmp_i32 = utility::buffer_get_int32(data_ptr, &data_ind);
        if (tmp_i32 < mFaultTime.last()) {
            for (int i = 0;i < mFaultTime.size();i++) {
                mFaultTime[i] = 0;
            }
        }
        mFaultTime.append((double)tmp_i32 / 1000.0);
        mFaultTime.remove(0);

        mFaultPx.append(data_ptr[data_ind++]);
        mFaultPx.remove(0);
        mFaultPy.append(data_ptr[data_ind++]);
        mFaultPy.remove(0);
        mFaultRoll.append(data_ptr[data_ind++]);
        mFaultRoll.remove(0);
        mFaultPitch.append(data_ptr[data_ind++]);
        mFaultPitch.remove(0);
        mFaultYaw.append(data_ptr[data_ind++]);
        mFaultYaw.remove(0);
        mFaultRange.append(data_ptr[data_ind++]);
        mFaultRange.remove(0);
        mFaultComm.append(data_ptr[data_ind++]);
        mFaultComm.remove(0);
        mRepaintGraphs = true;
        break;

    default:
        break;
    }
}

void SimViewer::timerSlot()
{
    if (mRepaintGraphs) {
        ui->posErrorPlot->clearGraphs();
        ui->posErrorPlot->addGraph();
        ui->posErrorPlot->graph()->setPen(QPen(Qt::black));
        ui->posErrorPlot->graph()->setData(mPosErrorTime, mPosError);
        ui->posErrorPlot->graph()->rescaleAxes();
        ui->posErrorPlot->graph()->setName("Position Error");

        ui->posErrorPlot->addGraph(ui->posErrorPlot->xAxis, ui->posErrorPlot->yAxis2);
        ui->posErrorPlot->graph()->setPen(QPen(Qt::red));
        ui->posErrorPlot->graph()->setData(mPosErrorTime, mVelError);
        ui->posErrorPlot->graph()->rescaleAxes();
        ui->posErrorPlot->graph()->setName("Velocity Error");

        ui->posErrorPlot->xAxis->setLabel("Time (s)");
        ui->posErrorPlot->yAxis->setLabel("Position Error (m)");
        ui->posErrorPlot->yAxis2->setLabel("Velocity Error (m/s)");
        ui->posErrorPlot->yAxis2->setVisible(true);
        ui->posErrorPlot->legend->setVisible(true);
        ui->posErrorPlot->replot();

        ui->faultPlot->clearGraphs();
        ui->faultPlot->addGraph();
        ui->faultPlot->graph()->setPen(QPen(Qt::black));
        ui->faultPlot->graph()->setData(mFaultTime, mFaultPx);
        ui->faultPlot->graph()->rescaleAxes();
        ui->faultPlot->graph()->setName("Px");

        ui->faultPlot->addGraph();
        ui->faultPlot->graph()->setPen(QPen(Qt::black));
        ui->faultPlot->graph()->setData(mFaultTime, mFaultPy);
        ui->faultPlot->graph()->rescaleAxes();
        ui->faultPlot->graph()->setName("Py");

        ui->faultPlot->addGraph();
        ui->faultPlot->graph()->setPen(QPen(Qt::green));
        ui->faultPlot->graph()->setData(mFaultTime, mFaultRoll);
        ui->faultPlot->graph()->rescaleAxes();
        ui->faultPlot->graph()->setName("Roll");

        ui->faultPlot->addGraph();
        ui->faultPlot->graph()->setPen(QPen(Qt::green));
        ui->faultPlot->graph()->setData(mFaultTime, mFaultPitch);
        ui->faultPlot->graph()->rescaleAxes();
        ui->faultPlot->graph()->setName("Pitch");

        ui->faultPlot->addGraph();
        ui->faultPlot->graph()->setPen(QPen(Qt::green));
        ui->faultPlot->graph()->setData(mFaultTime, mFaultYaw);
        ui->faultPlot->graph()->rescaleAxes();
        ui->faultPlot->graph()->setName("Yaw");

        ui->faultPlot->addGraph();
        ui->faultPlot->graph()->setPen(QPen(Qt::red));
        ui->faultPlot->graph()->setData(mFaultTime, mFaultRange);
        ui->faultPlot->graph()->rescaleAxes();
        ui->faultPlot->graph()->setName("Range");

        ui->faultPlot->addGraph();
        ui->faultPlot->graph()->setPen(QPen(Qt::blue));
        ui->faultPlot->graph()->setData(mFaultTime, mFaultComm);
        ui->faultPlot->graph()->rescaleAxes();
        ui->faultPlot->graph()->setName("Comm");

        ui->faultPlot->xAxis->setLabel("Time (s)");
        ui->faultPlot->yAxis->setLabel("Is Active");
        ui->faultPlot->legend->setVisible(true);
        ui->faultPlot->replot();

        mRepaintGraphs = false;
    }
}
