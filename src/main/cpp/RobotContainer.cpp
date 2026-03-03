// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/RobotModeTriggers.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

#include "RobotContainer.h"
#include "utils/math_utils.h"

RobotContainer::RobotContainer()
{ 
    pathplanner::NamedCommands::registerCommand("Shooting", shooter.Shooting([] { return 60_tps; }).WithTimeout(6_s));
    pathplanner::NamedCommands::registerCommand("IntakeStop", intake.StopIntaking() );
    autoChooser = pathplanner::AutoBuilder::buildAutoChooser();
    SmartDashboard::PutData("Auto Mode", &autoChooser);
    Shuffleboard::GetTab("Field").Add("Field", m_Field2d).WithSize(6, 4);

    ConfigureBindings();
}

void RobotContainer::ConfigureBindings()
{
    // Subsystem Default Command
    drivetrain.SetDefaultCommand(
        drivetrain.ApplyRequest([this]() -> auto&& {
            return FieldCentric_Manualdrive.WithVelocityX(-joystick.GetLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .WithVelocityY(-joystick.GetLeftX() * MaxSpeed)                            // Drive left with negative X (left)
                .WithRotationalRate(-joystick.GetRightX() * MaxAngularRate);               // Drive counterclockwise with negative X (left)
        })
    );
    conveyer.SetDefaultCommand(
        conveyer.Conveying([] { return 20_tps; }, [this] { return shooter.isActive(); })
    );
    intake.SetDefaultCommand(
        intake.StopIntaking()
    );
    shooter.SetDefaultCommand(
        shooter.Stop()
    );

    // Disable Mode Trigger
    RobotModeTriggers::Disabled().WhileTrue(
        drivetrain.ApplyRequest([] {
            return swerve::requests::Idle{};
        }).IgnoringDisable(true)
    );

    // Test Mode Trigger
    RobotModeTriggers::Test().WhileTrue(
        intake.ManualArmControl([this] { return -joystick.GetRightY(); })
    );
    
    // Joystick Binding
    joystick.Y().WhileTrue(
        drivetrain.ApplyRequest([this]() -> auto&& {
            return FieldCentricFacingAngle_Manualdrive.WithVelocityX(-joystick.GetLeftY() * MaxSpeed * 0.6) 
                .WithVelocityY(-joystick.GetLeftX() * MaxSpeed) 
                .WithTargetDirection(drivetrain.GetState().Pose.Rotation() + 
                mirroredOffset +
                calcHeadingError(targetTranslation , drivetrain.GetState()) + 
                calcShootComp(61.32_deg, 1.27935_m, targetTranslation, drivetrain.GetState(), 0.050585_m, 1, 1, 1, 1).compAngle);
        })
    );

    joystick.X().WhileTrue(
        drivetrain.ApplyRequest([this]() -> auto&& {
            return FieldCentricFacingAngle_Manualdrive.WithVelocityX(-joystick.GetLeftY() * MaxSpeed) 
                .WithVelocityY(-joystick.GetLeftX() * MaxSpeed)  
                .WithTargetDirection(Rotation2d(180_deg));
        })
    );

    joystick.RightTrigger().WhileTrue(
        shooter.Shooting([this] { 
            return calcShootComp(61.32_deg, 1.27935_m, targetTranslation, drivetrain.GetState(), 0.050585_m, 1, 6.5, 6.5, 1).tps;
        })
    );

   joystick.RightBumper().WhileTrue(
        shooter.Shooting([] { 
            return 60_tps; 
        })
    );

    joystick.LeftTrigger().ToggleOnTrue(
        intake.Intaking([] { 
            return 30_tps; 
        })
    );

    joystick.LeftBumper().OnTrue(
        drivetrain.RunOnce([this] { 
            drivetrain.SeedFieldCentric(); 
        })
    ); // reset the field-centric heading on left bumper press

    joystick.B().OnTrue(
        intake.Lifting()
    );

    joystick.A().OnTrue(
        intake.Lowering()
    );

    joystick.POVLeft().ToggleOnTrue(
        intake.Intaking([] { 
            return -30_tps; 
        }).AlongWith(conveyer.Conveying(
            [] { return -20_tps; }, 
            [] { return true; }
        ))
    );

    joystick.Back().OnTrue(
        drivetrain.RunOnce([this] { 
            drivetrain.ResetPose(resetPose);
        })
    );

    // Register Telemetry
    drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });
}
  
void RobotContainer::TestBindings(){

    joystick.POVUp().WhileTrue(
        drivetrain.ApplyRequest([this]() -> auto&& {
            return forwardStraight.WithVelocityX(0.5_mps).WithVelocityY(0_mps);
        })
    );
    joystick.POVDown().WhileTrue(
        drivetrain.ApplyRequest([this]() -> auto&& {
            return forwardStraight.WithVelocityX(-0.5_mps).WithVelocityY(0_mps);
        })
    );

    joystick.A().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& { return brake; }));
    joystick.B().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& {
        return point.WithModuleDirection(Rotation2d{-joystick.GetLeftY(), -joystick.GetLeftX()});
    }));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    (joystick.Back() && joystick.Y()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kForward));
    (joystick.Back() && joystick.X()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kReverse));
    (joystick.Start() && joystick.Y()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kForward));
    (joystick.Start() && joystick.X()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kReverse));
}

Command *RobotContainer::GetAutonomousCommand()
{
    return autoChooser.GetSelected(); 
}

