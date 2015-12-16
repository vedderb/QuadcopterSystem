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

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QStandardItemModel>
#include <QVector>
#include "coptersim.h"
#include "joystick.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    bool eventFilter(QObject *object, QEvent *e);

private slots:
    void timerSlot();

    void on_addCopterButton_clicked();
    void on_deleteCopterButton_clicked();
    void on_resetSimulationButton_clicked();
    void on_connectJoystickButton_clicked();
    void on_disconnectJoystickButton_clicked();
    void on_crashButton_clicked();
    void on_saveSpeedFileButton_clicked();
    void on_clearSpeedButton_clicked();
    void on_scenario1Button_clicked();
    void on_scenario2Button_clicked();
    void on_scenario3Button_clicked();
    void on_deletAllCoptersButton_clicked();
    void on_fcResetButton_clicked();
    void on_fcOffsetButton_clicked();
    void on_fcAmpButton_clicked();
    void on_fcSetToButton_clicked();
    void on_fcBitFlipButton_clicked();
    void on_fcUpdateIdButton_clicked();
    void on_paramSetButton_clicked();
    void on_paramGetButton_clicked();

private:
    Ui::MainWindow *ui;
    CopterSim mCopterSim;
    QTimer *mTimer;
    Joystick *mJoystick;
    QStandardItemModel *mCopterItemModel;
    double mRollKey;
    double mPitchKey;
    double mYawKey;
    QVector<double> mCopterSpeeds;

};

#endif // MAINWINDOW_H
