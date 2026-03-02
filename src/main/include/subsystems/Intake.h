// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "ctre/phoenix6/TalonFX.hpp"

#include <frc/Timer.h>
#include <frc2/command/Commands.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/modules/DualMotorModule.h"
#include "subsystems/modules/SingleMotorModule.h"

using namespace std;
using namespace frc;
using namespace frc2;
using namespace units;
using namespace ctre::phoenix6;
using TPS = units::turns_per_second_t;
using Turn = units::turn_t;

class IntakeSubsystem : public SubsystemBase {
    public:
        IntakeSubsystem(
            int intakeRightID, 
            int intakeLeftID, 
            int armRightID, 
            int armLeftID,
            DualMotorModule::Config intakeConfig,
            DualMotorModule::Config armConfig
        );

        CommandPtr Intaking(function<TPS()> intakeTps);
        CommandPtr StopIntaking();
        CommandPtr ManualArmControl(function<double()> joystickValue);
        CommandPtr Lifting();
        CommandPtr Lowering();

        void ActivateIntake(TPS tps);
        void DeactivateIntake();
        void LiftByTurns(Turn turns);
        void LiftByOpenLoop(double dutyPercentage);
        bool isIntakeActive() const;
        void Periodic() override;

    private:
        DualMotorModule intakeModule;
        DualMotorModule armModule;
        
        bool intakeStatus = false;
        bool armStatus = true;
};
