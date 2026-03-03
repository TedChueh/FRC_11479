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

class ConveyerSubsystem : public SubsystemBase {
  public:
      ConveyerSubsystem(
        int conveyerRightID,
        int conveyerLeftID,
        DualMotorModule::Config conveyerConfig
      );

      CommandPtr Conveying(function<TPS()> tps, function<bool()> status);
      
      void ActivateConveyer(TPS tps);
      
      void DeactivateConveyer();

      void Periodic() override;


  private:
      DualMotorModule conveyerModule;

      bool systemStatus = false;
};
