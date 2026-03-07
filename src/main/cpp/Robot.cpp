// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <cameraserver/CameraServer.h>
#include <cmath>

#include "Robot.h"
#include "LimelightHelpers.h"

Robot::Robot() {}

void Robot::RobotInit() {
    auto camera = CameraServer::StartAutomaticCapture();
    camera.SetResolution(320, 240);
    camera.SetFPS(60);
}

void Robot::RobotPeriodic() {   
    CommandScheduler::GetInstance().Run();
    m_timeAndJoystickReplay.Update();

    // SmartDashboard Basic Informations Output
    SmartDashboard::PutNumber("Match Time", DriverStation::GetMatchTime().value());
    SmartDashboard::PutNumber("Battery Voltage", RobotController::GetBatteryVoltage().value());

    // Auto Aiming Target Selection
    if (DriverStation::IsDisabled()) {
        auto const allianceColor = DriverStation::GetAlliance();
        auto isRed = *allianceColor == DriverStation::Alliance::kRed;
        if (allianceColor) {
            // Target position and direction based on alliance color
            m_container.targetTranslation = Translation2d{isRed ? 11.915394_m : 4.625594_m, 4.033663_m};
            m_container.mirroredOffset = isRed ? Rotation2d{180_deg} : Rotation2d{0_deg};
            m_container.resetPose = Pose2d{isRed ? 12.892988_m : 3.648_m, 4.033663_m, isRed ? 0_deg : 180_deg};
        }
    }

    // Limelight Measurement
    auto& drivetrain = m_container.drivetrain;
    auto& vision = m_container.m_vision;
    auto& vision2 = m_container.m_vision2;
    auto const driveState = drivetrain.GetState();
    auto const pose = driveState.Pose;

    meters_per_second_t translationSpeed = math::hypot(driveState.Speeds.vx, driveState.Speeds.vy);
    m_container.m_Field2d.SetRobotPose(pose);

    if (kUseLimelight) {
        vision.PeriodicUpdate(pose, translationSpeed, driveState.Speeds.omega);
        vision2.Update(pose, translationSpeed, driveState.Speeds.omega);
        if(auto meas = vision2.GetLatestUpdate()) {
            if (meas->suggestSeed){
                drivetrain.ResetPose(meas->pose);
                SmartDashboard::PutString("llmeasurement: ", "VisionSeeding");
            }
            else {
                drivetrain.AddVisionMeasurement(meas->pose, meas->timestamp, {meas->xyStdDev, meas->xyStdDev, meas->rotStdDev});
                SmartDashboard::PutString("llmeasurement: ", "AddVisionMeasurement");
            }
        }
        else{
            SmartDashboard::PutString("llmeasurement: ", "No Measurement");
        }
    }
}

void Robot::AutonomousInit() {
    m_autonomousCommand = m_container.GetAutonomousCommand();

    if (m_autonomousCommand) {
        frc2::CommandScheduler::GetInstance().Schedule(m_autonomousCommand);
    }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
    if (m_autonomousCommand) {
        frc2::CommandScheduler::GetInstance().Cancel(m_autonomousCommand);
    }
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

// void Robot::TestInit() {
//     frc2::CommandScheduler::GetInstance().CancelAll();
// }

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif
