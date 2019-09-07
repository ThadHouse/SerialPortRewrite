/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include "HALSerialPort.h"
#include "frc/SmartDashboard/SmartDashboard.h"
#include <frc/Timer.h>
#include "cameraserver/CameraServer.h"
#include <iostream>
#include "frcSerialPort.h"

namespace hal {
    namespace init {
        void InitializeSerialPort2();
    }
}

void Robot::RobotInit() {
    hal::init::InitializeSerialPort2();

    gyro = new AHRS{frc2::SerialPort::kUSB1, AHRS::SerialDataType::kProcessedData, 200};

    frc::CameraServer::GetInstance()->StartAutomaticCapture(0).SetResolution(1280, 720);

    // serialPort = new frc2::SerialPort{9600, frc2::SerialPort::kMXP};
    // serialPort->SetTimeout(5.0);
    // serialPort->EnableTermination('\n');

    // std::thread([&]{
    //     while (true) {
    //         char buf[256];
    //         int len = serialPort->Read(buf, sizeof(buf));
    //         if (len > 0) {
    //             std::cout << wpi::StringRef{buf, len} << ' '<< len << std::endl;
    //         }
    //     }
    // }).detach();

    // std::thread([&]{
    //     int32_t status = 0;

        

    //     //serialPortHandle = HAL2_InitializeSerialPort(HAL_SerialPort::HAL_SerialPort_MXP, &status);
    //     //HAL_InitializeSerialPort(HAL_SerialPort::HAL_SerialPort_MXP, &status);
    //     frc::SmartDashboard::PutNumber("SerialPortInitStatus", status);

    //     //HAL2_SetSerialTimeout(serialPortHandle, 0.5, &status);
    //     //HAL2_EnableSerialTermination(serialPortHandle, '\n', &status);

    //     //HAL_SetSerialTimeout(HAL_SerialPort::HAL_SerialPort_MXP, 0.5, &status);
    //     //HAL_EnableSerialTermination(HAL_SerialPort::HAL_SerialPort_MXP, '\n', &status);

    //     //HAL_SetSerialWriteMode(HAL_SerialPort::HAL_SerialPort_MXP, 1, &status);
    //     std::cout << status << std::endl;

    //     frc::Timer timer;
    //     timer.Start();

    //     int count = 0;

    //     while (true) {
    //         //char data[100];

    //         //std::cout << "Entering loop!" << std::endl;

    //         wpi::SmallVector<char, 64> data;
    //         wpi::raw_svector_ostream stream{data};
    //         stream << count << "\r\n"; 
    //         count++;

    //         int len = serialPort->Write(data.data(), data.size());

    //         //int len = HAL_WriteSerial(HAL_SerialPort::HAL_SerialPort_MXP, data.data(), data.size(), &status);
    //         //std::cout << status << std::endl;
    //         std::cout << len << std::endl;
    //         //std::cout << wpi::StringRef{data, len} << std::endl;
    //         frc::SmartDashboard::PutNumber("timeout", timer.Get());
    //         std::this_thread::sleep_for(std::chrono::milliseconds(500));
    //         timer.Reset();
    //     }
        
    // }).detach();
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
