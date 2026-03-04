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

class ShooterSubsystem : public SubsystemBase {
  public:
      ShooterSubsystem(
        int shootRightID,
        int shootLeftID,
        int suctionID,
        DualMotorModule::Config shootConfig,
        SingleMotorModule::Config suctionConfig
      );

      CommandPtr Shooting(function<TPS()> shootTps);
      CommandPtr StopShooting();

      void ActivateShooter(TPS tps);
      void ActivateSuction(TPS tps);  
      void DeactivateShooter();
      void DeactivateSuction();
      bool isActive() const;
      void Periodic() override;

  private:
      Timer m_timer; 
      DualMotorModule shootModule;
      SingleMotorModule suctionModule;

      bool systemStatus = false;
};
