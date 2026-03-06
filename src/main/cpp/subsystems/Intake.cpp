// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"

IntakeSubsystem::IntakeSubsystem(
    int intakeRightID, int intakeLeftID,
    int armRightID, int armLeftID,
    DualMotorModule::Config intakeConfig,
    DualMotorModule::Config armConfig
): intakeModule{intakeRightID, intakeLeftID, intakeConfig}, armModule{armRightID, armLeftID, armConfig} {}

CommandPtr IntakeSubsystem::Intaking(function<TPS()> intakeTps) {
  return cmd::Run(
      [this, intakeTps] {
        intakeStatus = true;
        ActivateIntake(intakeTps());
      },{this}
  );
}

CommandPtr IntakeSubsystem::StopIntaking() {
  return cmd::Run(
      [this] {
        intakeStatus = false;
        DeactivateIntake();
      },{this}
  );
}

CommandPtr IntakeSubsystem::ManualArmControl(function<double()> joystickValue) {
  return cmd::Run(
      [this, joystickValue] {
        LiftByOpenLoop(joystickValue());
      },{this}
  );
}

CommandPtr IntakeSubsystem::Lifting() {
  return cmd::RunOnce(
      [this] {
        if(!armStatus) {
          armStatus = true;
          LiftByTurns(100_tr);
        }
      },{this}
  );
}

CommandPtr IntakeSubsystem::Lowering() {
  return cmd::RunOnce(
      [this] {
        if(armStatus) {
          armStatus = false;
          LiftByTurns(-100_tr);
        }
      },{this}
  );
}

void IntakeSubsystem::ActivateIntake(TPS tps) {
  intakeModule.motorLeft.SetControl(intakeModule.velocityControl.WithVelocity(tps));
  intakeModule.motorRight.SetControl(intakeModule.velocityControl.WithVelocity(tps));
}

void IntakeSubsystem::DeactivateIntake() {
  intakeModule.motorLeft.SetControl(controls::NeutralOut{});
  intakeModule.motorRight.SetControl(controls::NeutralOut{});
}

void IntakeSubsystem::LiftByTurns(Turn turns) {
  auto leftPos  = armModule.motorLeft.GetPosition().GetValue();
  auto rightPos = armModule.motorRight.GetPosition().GetValue();

  Turn leftTarget  = leftPos  - turns;
  Turn rightTarget = rightPos - turns;

  auto s1 = armModule.motorLeft.SetControl(armModule.motionMagicControl.WithPosition(leftTarget));
  auto s2 = armModule.motorRight.SetControl(armModule.motionMagicControl.WithPosition(rightTarget));
  fmt::print("pressed  s1={}  s2={}\n", s1.GetName(), s2.GetName());
}

void IntakeSubsystem::LiftByOpenLoop(double dutyPercentage) {
  armModule.motorLeft.SetControl(armModule.dutyCycleControl.WithOutput(dutyPercentage));
  armModule.motorRight.SetControl(armModule.dutyCycleControl.WithOutput(dutyPercentage));
}

bool IntakeSubsystem::isIntakeActive() const {
  return intakeStatus;
}

void IntakeSubsystem::Periodic() {
    SmartDashboard::PutBoolean("Intake Status", intakeStatus);
    SmartDashboard::PutString("Arm Status", armStatus ? "Lifting↑" : "Lowering↓"  );
}