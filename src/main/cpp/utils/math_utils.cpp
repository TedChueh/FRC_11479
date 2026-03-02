#include "utils/math_utils.h"

Rotation2d calcHeadingError(Translation2d targetPosition, const impl::SwerveDrivetrainImpl::SwerveDriveState& robotState) {
    Translation2d robotPosition  = robotState.Pose.Translation();
    Rotation2d    robotDirection = robotState.Pose.Rotation();
    Translation2d toTarget = targetPosition - robotPosition;
    Translation2d botDirVec(meter_t(robotDirection.Cos()), meter_t(robotDirection.Sin()));
    
    double cross = botDirVec.X().value() * toTarget.Y().value() - botDirVec.Y().value() * toTarget.X().value();
    double dot   = botDirVec.X().value() * toTarget.X().value() + botDirVec.Y().value() * toTarget.Y().value();
    double angle = atan2(cross, dot);

    return Rotation2d(radian_t(angle));
}

ShootCompOutput calcShootComp(degree_t shootDegree,
                              meter_t deltaHeight,
                              Translation2d targetPosition,
                              const impl::SwerveDrivetrainImpl::SwerveDriveState& robotState,
                              meter_t wheelRadius_m,   
                              double kApproachGain,
                              double kStationaryGain,
                              double kRetreatGain,
                              double kAngleGain) 
{
    Translation2d robotPosition = robotState.Pose.Translation();
    ChassisSpeeds robotVelocity = robotState.Speeds;
    Translation2d targetVector = targetPosition - robotPosition;
    meter_t targetDistance = targetVector.Norm();

    double shootRad = shootDegree.to<double>() * M_PI / 180.0;

    meter_t denom = 2.0 * (targetDistance * tan(shootRad) - deltaHeight);
    if (denom <= 0_m) {
        SmartDashboard::PutString("Velocity Comp Angle Warning⚠️: ", "Denominator <= 0");
        return ShootCompOutput{Rotation2d(0_rad), 0.0_tps};
    }

    meters_per_second_t desiredVx{
        std::sqrt((g_accel * targetDistance * targetDistance / denom).value())
    };

    double ux = targetVector.X().value() / targetDistance.value();
    double uy = targetVector.Y().value() / targetDistance.value();

    double vForward = robotVelocity.vx.value() * ux + robotVelocity.vy.value() * uy;
    double v_comp = desiredVx.value() - vForward;

    double kWheelCircumferenceMeters  = 2.0 * M_PI * wheelRadius_m.value();

    double approachComponent  = (vForward >= 0.0) ? kApproachGain * vForward : kRetreatGain * vForward;
    TPS tps = 1_tps * (desiredVx.value() * kStationaryGain + approachComponent) / kWheelCircumferenceMeters;

    double RVVcrossTV = robotVelocity.vx.value() * targetVector.Y().value() -
                        targetVector.X().value() * robotVelocity.vy.value();
    double tangentialSpeed = RVVcrossTV / targetDistance.value();
    double sign = (RVVcrossTV >= 0.0) ? 1.0 : -1.0;
    double compAngleRad = sign * atan2(abs(tangentialSpeed), abs(v_comp)) * kAngleGain;
        
    SmartDashboard::PutNumber("Raw TPS: ", tps.value());
    if(tps < 0_tps) {
        return ShootCompOutput{Rotation2d(0_rad), 0.0_tps};
    }
    else if(tps > 100_tps) {
        return ShootCompOutput{Rotation2d(0_rad), 0.0_tps};
    }
    else {
        return ShootCompOutput{Rotation2d(radian_t(compAngleRad)), tps};
    }
}
