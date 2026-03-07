// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <pathplanner/lib/auto/NamedCommands.h>

#include "subsystems/CommandSwerveDrivetrain.h"
#include "subsystems/Conveyer.h"
#include "subsystems/Intake.h"
#include "subsystems/Shooter.h"
#include "subsystems/Vision.h"
#include "subsystems/Vision2.h"
#include "Telemetry.h"

using namespace std;
using namespace frc;
using namespace frc2;
using namespace units;

class RobotContainer {
private:
    meters_per_second_t MaxSpeed = 1.0 * TunerConstants::kSpeedAt12Volts; // kSpeedAt12Volts desired top speed
    radians_per_second_t MaxAngularRate = 0.75_tps; // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    swerve::requests::FieldCentric FieldCentric_Manualdrive = swerve::requests::FieldCentric{}
        .WithDeadband(MaxSpeed * 0.05).WithRotationalDeadband(MaxAngularRate * 0.05) // Add a 5% deadband
        .WithDriveRequestType(swerve::DriveRequestType::Velocity); // Use open-loop control for drive motors
    swerve::requests::SwerveDriveBrake brake{};
    swerve::requests::PointWheelsAt point{};
    swerve::requests::RobotCentric forwardStraight = swerve::requests::RobotCentric{}
        .WithDriveRequestType(swerve::DriveRequestType::Velocity);
    swerve::requests::FieldCentricFacingAngle FieldCentricFacingAngle_Manualdrive = swerve::requests::FieldCentricFacingAngle{}
        .WithDeadband(MaxSpeed * 0.05).WithRotationalDeadband(MaxAngularRate * 0.05) // Add a 5% deadband
        .WithDriveRequestType(swerve::DriveRequestType::Velocity).WithHeadingPID(5, 0, 0); // Use open-loop control for drive motors

    /* Note: This must be constructed before the drivetrain, otherwise we need to
     *       define a destructor to un-register the telemetry from the drivetrain */
    Telemetry logger{MaxSpeed};

    CommandXboxController joystick{0};

public:
    subsystems::CommandSwerveDrivetrain drivetrain{TunerConstants::CreateDrivetrain()};
    VisionSubsystem m_vision;
    VisionSubsystem2 m_vision2;

private:
    /* Path follower */
    SendableChooser<Command *> autoChooser;

public:
    RobotContainer();
    Command *GetAutonomousCommand();
    Field2d m_Field2d;
    Pose2d resetPose{};
    Translation2d targetTranslation{}; 
    Rotation2d mirroredOffset{}; 


private:
    void ConfigureBindings();
    void TestBindings();

    ConveyerSubsystem conveyer{
        55, 56,
        DualMotorModule::Config{
            .motorRightInvert = false,
            .motorLeftInvert = true,
            .kS = 0.55,
            .kV = 0.11,
            .kA = 0.0,
            .kP = 0,
            .kI = 0,
            .kD = 0,
            .PeakVoltage = 12_V,
            .PeakCurrent = 40_A,
            .RampPeriod = 0.5_s
        }
    };

    IntakeSubsystem intake{
        1, 2,
        61, 62,
        DualMotorModule::Config{
            .motorRightInvert = true,
            .motorLeftInvert = false,
            .kS = 0.25,
            .kV = 0.101,
            .kA = 0.0,
            .kP = 0.0,
            .kI = 0,
            .kD = 0,
            .PeakVoltage = 12_V,
            .PeakCurrent = 40_A,
            .RampPeriod = 0.5_s
        },
        DualMotorModule::Config{
            .motorRightInvert = true,
            .motorLeftInvert = false,
            .kS = 0.195,
            .kV = 0.108,
            .kA = 0.0,
            .kP = 0.5,
            .kI = 0,
            .kD = 0,
            .PeakVoltage = 12_V,
            .PeakCurrent = 40_A,
            .RampPeriod = 0.5_s,
            .MotionMagicCruiseVelocity = 50_tps,
            .MotionMagicAcceleration   = 30_tr_per_s_sq
        }
    };

    ShooterSubsystem shooter{
        51, 52,
        54,
        DualMotorModule::Config{
            .motorRightInvert = false,
            .motorLeftInvert = true,
            .kS = 0.19,
            .kV = 0.111,
            .kA = 0.0,
            .kP = 0.015,
            .kI = 0,
            .kD = 0,
            .PeakVoltage = 12_V,
            .PeakCurrent = 40_A,
            .RampPeriod = 0.3_s
        },
        SingleMotorModule::Config{
            .motorInvert = true,
            .kS = 0.197,
            .kV = 0.11,
            .kA = 0.0,
            .kP = 0.01,
            .kI = 0,
            .kD = 0,
            .PeakVoltage = 12_V,
            .PeakCurrent = 40_A,
            .RampPeriod = 0.5_s
        }
    };
};
