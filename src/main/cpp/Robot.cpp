/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include "HALSerialPort.h"
#include "SmartDashboard/SmartDashboard.h"
#include <frc/Timer.h>
#include <iostream>

namespace hal {
    namespace init {
        void InitializeSerialPort2();
    }
}

void Robot::RobotInit() {
    hal::init::InitializeSerialPort2();
    std::thread([&]{
        int32_t status = 0;
        serialPortHandle = HAL2_InitializeSerialPort(HAL_SerialPort::HAL_SerialPort_USB1, &status);
        frc::SmartDashboard::PutNumber("SerialPortInitStatus", status);

        HAL2_SetSerialTimeout(serialPortHandle, 3, &status);
        HAL2_EnableSerialTermination(serialPortHandle, '\n', &status);

        frc::Timer timer;
        timer.Start();

        while (true) {
            char data[100];

            //std::cout << "Entering loop!" << std::endl;

            int len = HAL2_ReadSerial(serialPortHandle, data, sizeof(data), &status);
            std::cout << wpi::StringRef{data, len} << std::endl;
            frc::SmartDashboard::PutNumber("timeout", timer.Get());
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            timer.Reset();
        }
        
    }).detach();
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
