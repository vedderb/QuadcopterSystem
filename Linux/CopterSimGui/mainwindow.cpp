/*
    Copyright 2014 Benjamin Vedder	benjamin.vedder@sp.se

    This file is part of CopterSimGui.

    CopterSimGui is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    CopterSimGui is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with CopterSimGui.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "faultcheck_wrapper.h"
#include "faultcheck.h"
#include <QDebug>
#include <QKeyEvent>
#include <QFileDialog>
#include <cmath>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    mJoystick = new Joystick(this);

    mTimer = new QTimer(this);
    mTimer->setInterval(30);
    mTimer->start();
    connect(mTimer, SIGNAL(timeout()), this, SLOT(timerSlot()));

    mCopterItemModel = new QStandardItemModel(0, 3, this);
    mCopterItemModel->setHorizontalHeaderItem(0, new QStandardItem("ID"));
    mCopterItemModel->setHorizontalHeaderItem(1, new QStandardItem("Actual pos"));
    mCopterItemModel->setHorizontalHeaderItem(2, new QStandardItem("Perceived pos"));

    ui->copterTableView->setModel(mCopterItemModel);
    on_connectJoystickButton_clicked();

    mRollKey = 0.0;
    mPitchKey = 0.0;
    mYawKey = 0.0;

    // Add line segments to the copter simulator map
    int segcnt = 1000;
    mCopterSim.addLineSegment(segcnt++, -5, -5, 5, -5);
    mCopterSim.addLineSegment(segcnt++, 5, -5, 8, 0);
    mCopterSim.addLineSegment(segcnt++, 8, 0, 5, 5);
    mCopterSim.addLineSegment(segcnt++, -5, 5, 5, 5);
    mCopterSim.addLineSegment(segcnt++, -5, -5, -8, 0);
    mCopterSim.addLineSegment(segcnt++, -8, 0, -5, 5);

    // Add some anchors
    mCopterSim.addAnchor(0, -8.0, -5.0, 1.0);
    mCopterSim.addAnchor(1, -8.0, 5.0, 1.0);
    mCopterSim.addAnchor(2, 8.0, 5.0, 1.0);
    mCopterSim.addAnchor(3, 8.0, -5.0, 1.0);

    qApp->installEventFilter(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

bool MainWindow::eventFilter(QObject *object, QEvent *e)
{
    Q_UNUSED(object);

    if (e->type() == QEvent::KeyPress || e->type() == QEvent::KeyRelease)
    {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(e);
        bool isPress = e->type() == QEvent::KeyPress;

        switch(keyEvent->key()) {
        case Qt::Key_Up:
        case Qt::Key_Down:
        case Qt::Key_Left:
        case Qt::Key_Right:
        case Qt::Key_Z:
        case Qt::Key_X:
            if (mCopterSim.collisionOccured()) {
                return false;
            }
            break;

        default:
            return false;
        }

        if(keyEvent->isAutoRepeat()) {
            return true;
        }

        switch(keyEvent->key()) {
        case Qt::Key_Up:
            if (isPress) {
                mPitchKey = -1.0;
            } else {
                mPitchKey = 0.0;
            }
            break;

        case Qt::Key_Down:
            if (isPress) {
                mPitchKey = 1.0;
            } else {
                mPitchKey = 0.0;
            }
            break;

        case Qt::Key_Left:
            if (isPress) {
                mRollKey = -1.0;
            } else {
                mRollKey = 0.0;
            }
            break;

        case Qt::Key_Right:
            if (isPress) {
                mRollKey = 1.0;
            } else {
                mRollKey = 0.0;
            }
            break;

        case Qt::Key_Z:
            if (isPress) {
                mYawKey = 1.0;
            } else {
                mYawKey = 0.0;
            }
            break;

        case Qt::Key_X:
            if (isPress) {
                mYawKey = -1.0;
            } else {
                mYawKey = 0.0;
            }
            break;

        default:
            break;
        }

        return true;
    }

    return false;
}

void MainWindow::timerSlot()
{
    mCopterSim.sendStateToMap();
    mCopterSim.runIterations(3, 10000, 20, 5);

    int selId = 0;
    int selRow = ui->copterTableView->currentIndex().row();

    if (selRow >= 0) {
        QModelIndex index = mCopterItemModel->index(selRow, 0);
        selId = mCopterItemModel->data(index).toInt();
    }

    mCopterSim.selectCopter(selId);

    // Update the logged speed label
    static int speedSamplesBefore = 0;
    if (speedSamplesBefore != mCopterSpeeds.size()) {
        ui->speedSampleLabel->setText(QString().sprintf("%d samples", mCopterSpeeds.size()));
        speedSamplesBefore = mCopterSpeeds.size();
    }

    // Update the quadcopter coordinate display and optionally log speeds
    for (int row = 0;row < mCopterItemModel->rowCount();row++) {
        QModelIndex index = mCopterItemModel->index(row, 0);
        CopterModel *copter = mCopterSim.getCopter(mCopterItemModel->data(index).toInt());

        if (copter) {
            const POS_STATE_t &ap = copter->getActualPos();
            const POS_STATE_t &pp = copter->getPerceivedPos();

            // Actual pos
            QModelIndex index = mCopterItemModel->index(row, 1);
            mCopterItemModel->setData(index, QString().sprintf("(%.2f, %.2f, %.2f)", ap.px, ap.py, ap.yaw));

            // Perceived pos
            index = mCopterItemModel->index(row, 2);
            mCopterItemModel->setData(index, QString().sprintf("(%.2f, %.2f, %.2f)", pp.px, pp.py, pp.yaw));

            if (ui->saveSpeedBox->isChecked()) {
                double vx = copter->getActualPos().vx;
                double vy = copter->getActualPos().vy;
                double speed = sqrt(vx * vx + vy * vy);
                mCopterSpeeds.append(speed);
            }
        }
    }

    // Joystick input
    if (mJoystick->isConnected()) {
        ui->joystickConnectedLabel->setText(tr("Connected"));
        ui->joystickA1Bar->setValue(mJoystick->getAxis(0));
        ui->joystickA2Bar->setValue(mJoystick->getAxis(1));
        ui->joystickA3Bar->setValue(mJoystick->getAxis(2));
        ui->joystickA4Bar->setValue(mJoystick->getAxis(4));

        if (ui->joystickSendBox->isChecked()) {
            if (selRow >= 0) {
                double throttle = (double)mJoystick->getAxis(2) / 32768.0;
                double roll = -(double)mJoystick->getAxis(0) / 32768.0;
                double pitch = -(double)mJoystick->getAxis(1) / 32768.0;
                double yaw = (double)mJoystick->getAxis(4) / 32768.0;

                CONTROL_INPUT_t input;
                input.rollCmd = roll;
                input.pitchCmd = pitch;
                input.yawCmd = yaw;
                input.throttleCmd = throttle;

                mCopterSim.setControlInput(selId, input);
            }
        }
    } else {
        ui->joystickConnectedLabel->setText(tr("Not connected"));
        if (ui->joystickSendBox->isChecked()) {
            if (selRow >= 0) {
                CONTROL_INPUT_t input;
                input.rollCmd = mRollKey;
                input.pitchCmd = mPitchKey;
                input.yawCmd = mYawKey;
                input.throttleCmd = 0.0;

                mCopterSim.setControlInput(selId, input);
            }
        }
    }

    // Update the collision label
    static bool coll_before = true;
    if (coll_before != mCopterSim.collisionOccured()) {
        if (mCopterSim.collisionOccured()) {
            POINT_t col = mCopterSim.collisionPoint();
            ui->collisionLabel->setText(QString().sprintf("Collision at (%.3f, %.3f)", col.x, col.y));
            QPalette palette = ui->collisionLabel->palette();
            palette.setColor(ui->collisionLabel->backgroundRole(), Qt::red);
            ui->collisionLabel->setAutoFillBackground(true);
            ui->collisionLabel->setPalette(palette);
        } else {
            ui->collisionLabel->setText("No collision");
            QPalette palette = ui->collisionLabel->palette();
            palette.setColor(ui->collisionLabel->backgroundRole(), Qt::green);
            ui->collisionLabel->setAutoFillBackground(true);
            ui->collisionLabel->setPalette(palette);
        }
        coll_before = mCopterSim.collisionOccured();
    }
}

void MainWindow::on_addCopterButton_clicked()
{
    int rowNum = mCopterItemModel->rowCount();
    int copterId = 0;

    if (rowNum > 0) {
        QModelIndex index = mCopterItemModel->index(rowNum - 1, 0);
        CopterModel *lastCopter = mCopterSim.getCopter(mCopterItemModel->data(index).toInt());
        if (lastCopter) {
            copterId = lastCopter->mItsStation.getId() + 1;
        }
    }

    mCopterSim.addCopter(ui->addXBox->value(), ui->addYBox->value(), copterId);

    mCopterItemModel->insertRows(rowNum, 1);
    QModelIndex index = mCopterItemModel->index(rowNum, 0);
    mCopterItemModel->setData(index, copterId);
}

void MainWindow::on_deleteCopterButton_clicked()
{
    int selRow = ui->copterTableView->currentIndex().row();

    if (selRow >= 0) {
        QModelIndex index = mCopterItemModel->index(selRow, 0);
        int id = mCopterItemModel->data(index).toInt();
        mCopterSim.removeCopter(id);
        mCopterItemModel->removeRow(selRow);
    }
}

void MainWindow::on_resetSimulationButton_clicked()
{
    mCopterSim.resetCollision();
}

void MainWindow::on_connectJoystickButton_clicked()
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

void MainWindow::on_disconnectJoystickButton_clicked()
{
    mJoystick->stop();
}

void MainWindow::on_crashButton_clicked()
{
    mCopterSim.stopSimulation();
}

void MainWindow::on_saveSpeedFileButton_clicked()
{
    QString path;

    path = QFileDialog::getSaveFileName(this, tr("Choose where to save the magnetometer samples"));
    if (path.isNull()) {
        return;
    }

    QFile file(path);

    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        qDebug() << "Could not open" << path;
        return;
    }

    QTextStream out(&file);

    bool isFirst = true;
    QVectorIterator<double> i(mCopterSpeeds);
    while (i.hasNext()) {
        double element = i.next();

        if (isFirst) {
            isFirst = false;
            out << QString().sprintf("%.4f", element);
        } else {
            out << QString().sprintf(", %.4f", element);
        }
    }

    file.close();
}

void MainWindow::on_clearSpeedButton_clicked()
{
    mCopterSpeeds.clear();
}

void MainWindow::on_scenario1Button_clicked()
{
    mCopterSim.removeAllCopters();
    mCopterItemModel->removeRows(0, mCopterItemModel->rowCount());

    int id = 0;

    for (int i = -1;i < 2;i++) {
        for (int j = -5;j < 6;j++) {
            mCopterItemModel->insertRows(id, 1);
            QModelIndex index = mCopterItemModel->index(id, 0);
            mCopterItemModel->setData(index, id);

            mCopterSim.addCopter(j, i, id++);
        }
    }
}

void MainWindow::on_scenario2Button_clicked()
{
    mCopterSim.removeAllCopters();
    mCopterItemModel->removeRows(0, mCopterItemModel->rowCount());

    int id = 0;

    for (int i = 50;i < 70;i += 2) {
        mCopterItemModel->insertRows(id, 1);
        QModelIndex index = mCopterItemModel->index(id, 0);
        mCopterItemModel->setData(index, id);

        CONTROL_INPUT_t ctrl;
        ctrl.pitchCmd = 1.0;
        ctrl.rollCmd = 0.0;
        ctrl.yawCmd = 0.0;
        ctrl.throttleCmd = 0.0;

        mCopterSim.addCopter(0, i, id);
        mCopterSim.getCopter(id)->setControlInput(ctrl);
        id++;
    }
}

void MainWindow::on_scenario3Button_clicked()
{

}

void MainWindow::on_deletAllCoptersButton_clicked()
{
    mCopterSim.removeAllCopters();
    mCopterItemModel->removeRows(0, mCopterItemModel->rowCount());
}

void MainWindow::on_fcResetButton_clicked()
{
    faultcheck_removeAllFaults();
}

void MainWindow::on_fcOffsetButton_clicked()
{
    faultcheck_addFaultOffset(ui->fcIdBox->currentText().toLocal8Bit(), ui->fcValueBox->value());
    faultcheck_setTriggerAfterIterations(ui->fcIdBox->currentText().toLocal8Bit(), ui->fcDelayBox->value());
    faultcheck_setDurationAfterTrigger(ui->fcIdBox->currentText().toLocal8Bit(), ui->fcDurationBox->value());
}

void MainWindow::on_fcAmpButton_clicked()
{
    faultcheck_addFaultAmplification(ui->fcIdBox->currentText().toLocal8Bit(), ui->fcValueBox->value());
    faultcheck_setTriggerAfterIterations(ui->fcIdBox->currentText().toLocal8Bit(), ui->fcDelayBox->value());
    faultcheck_setDurationAfterTrigger(ui->fcIdBox->currentText().toLocal8Bit(), ui->fcDurationBox->value());
}

void MainWindow::on_fcSetToButton_clicked()
{
    faultcheck_addFaultSetTo(ui->fcIdBox->currentText().toLocal8Bit(), ui->fcValueBox->value());
    faultcheck_setTriggerAfterIterations(ui->fcIdBox->currentText().toLocal8Bit(), ui->fcDelayBox->value());
    faultcheck_setDurationAfterTrigger(ui->fcIdBox->currentText().toLocal8Bit(), ui->fcDurationBox->value());
}

void MainWindow::on_fcBitFlipButton_clicked()
{
    faultcheck_addFaultBitFlip(ui->fcIdBox->currentText().toLocal8Bit(), ui->fcValueBox->value());
    faultcheck_setTriggerAfterIterations(ui->fcIdBox->currentText().toLocal8Bit(), ui->fcDelayBox->value());
    faultcheck_setDurationAfterTrigger(ui->fcIdBox->currentText().toLocal8Bit(), ui->fcDurationBox->value());
}

void MainWindow::on_fcUpdateIdButton_clicked()
{
    FaultCheck *fc = (FaultCheck*)faultcheck_instance();
    ui->fcIdBox->clear();
    QStringList ids = fc->getIdentifiers();
    ids.sort();
    ui->fcIdBox->addItems(ids);
}

void MainWindow::on_paramSetButton_clicked()
{
    for (int row = 0;row < mCopterItemModel->rowCount();row++) {
        QModelIndex index = mCopterItemModel->index(row, 0);
        CopterModel *copter = mCopterSim.getCopter(mCopterItemModel->data(index).toInt());

        if (copter) {
            POS_CORR_PARAM_t &param = copter->mCorrParam;

            param.anchor_pos_gain_p = ui->paramPgpBox->value();
            param.anchor_pos_gain_i = ui->paramPgiBox->value();
            param.anchor_pos_gain_d = ui->paramPgdBox->value();

            param.anchor_vel_gain_p = ui->paramVgpBox->value();
            param.anchor_vel_gain_i = ui->paramVgiBox->value();
            param.anchor_vel_gain_d = ui->paramVgdBox->value();

            param.anchor_acc_gain_p = ui->paramAgpBox->value();
            param.anchor_acc_gain_i = ui->paramAgiBox->value();
            param.anchor_acc_gain_d = ui->paramAgdBox->value();

            param.anchor_max_corr = ui->paramMaxCorrBox->value();
            param.anchor_max_tilt = ui->paramMaxTiltBox->value();
        }
    }
}

void MainWindow::on_paramGetButton_clicked()
{
    CopterModel *copter = mCopterSim.getCopter(0);

    if (copter) {
        POS_CORR_PARAM_t &param = copter->mCorrParam;

        ui->paramPgpBox->setValue(param.anchor_pos_gain_p);
        ui->paramPgiBox->setValue(param.anchor_pos_gain_i);
        ui->paramPgdBox->setValue(param.anchor_pos_gain_d);

        ui->paramVgpBox->setValue(param.anchor_vel_gain_p);
        ui->paramVgiBox->setValue(param.anchor_vel_gain_i);
        ui->paramVgdBox->setValue(param.anchor_vel_gain_d);

        ui->paramAgpBox->setValue(param.anchor_acc_gain_p);
        ui->paramAgiBox->setValue(param.anchor_acc_gain_i);
        ui->paramAgdBox->setValue(param.anchor_acc_gain_d);

        ui->paramMaxCorrBox->setValue(param.anchor_max_corr);
        ui->paramMaxTiltBox->setValue(param.anchor_max_tilt);
    }
}
