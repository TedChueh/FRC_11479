// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"

ShooterSubsystem::ShooterSubsystem(
  int shootRightID,   int shootLeftID, 
  int suctionID,
  DualMotorModule::Config shootConfig,
  SingleMotorModule::Config suctionConfig
): shootModule{shootRightID, shootLeftID, shootConfig}, suctionModule{suctionID, suctionConfig} {}

CommandPtr ShooterSubsystem::Shooting(function<TPS()> shootTps) {
  return cmd::Run(
      [this, shootTps] {
          TPS currentTps = shootTps();
          SmartDashboard::PutNumber("Raw TPS", currentTps.value());
          
          if (currentTps < 0_tps || currentTps > 100_tps) {
              systemStatus = false;
              DeactivateShooter();
              DeactivateSuction();
          } 
          else {
              systemStatus = true;
              ActivateShooter(currentTps);  
              if (m_timer.HasElapsed(0.5_s)) {
                  ActivateSuction(currentTps * 0.6);
              }
          }
      },{this}
  ).BeforeStarting(
      [this] {
          m_timer.Reset();
          m_timer.Start();
      }
  );
}

CommandPtr ShooterSubsystem::StopShooting() {
  return cmd::Run(
      [this]{
        systemStatus = false;
        DeactivateShooter();
        DeactivateSuction();
      },{this}
  );
}

void ShooterSubsystem::ActivateShooter(TPS tps) {
  shootModule.motorLeft.SetControl(shootModule.velocityControl.WithVelocity(tps));
  shootModule.motorRight.SetControl(shootModule.velocityControl.WithVelocity(tps));
}

void ShooterSubsystem::ActivateSuction(TPS tps) {
  suctionModule.motor.SetControl(suctionModule.velocityControl.WithVelocity(tps));
}

void ShooterSubsystem::DeactivateShooter() {
  shootModule.motorLeft.SetControl(controls::NeutralOut{});
  shootModule.motorRight.SetControl(controls::NeutralOut{});
}

void ShooterSubsystem::DeactivateSuction() {
  suctionModule.motor.SetControl(controls::NeutralOut{});
}

bool ShooterSubsystem::isActive() const {
  return systemStatus;
}

void ShooterSubsystem::Periodic() {
   SmartDashboard::PutBoolean("Shooter Status", systemStatus);
} 
