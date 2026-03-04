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

ShootCompOutput calcShootComp(degree_t shootDegree, meter_t deltaHeight, Translation2d targetPosition,
                              const impl::SwerveDrivetrainImpl::SwerveDriveState& robotState,
                              meter_t wheelRadius_m,   
                              double kApproachGain, double kStationaryGain, double kRetreatGain,
                              double kAngleGain) {
    Translation2d robotPosition = robotState.Pose.Translation();
    ChassisSpeeds robotVelocity = ChassisSpeeds::FromRobotRelativeSpeeds(robotState.Speeds, robotState.Pose.Rotation());
    Translation2d targetVector = targetPosition - robotPosition;
    meter_t targetDistance = targetVector.Norm();

    double shootRad = shootDegree.to<double>() * M_PI / 180.0;
    double kWheelCircumferenceMeters  = 2.0 * M_PI * wheelRadius_m.value();

    meter_t denom = 2.0 * (targetDistance * tan(shootRad) - deltaHeight);
    
    if (denom <= 0_m) {
        return ShootCompOutput{Rotation2d(0_rad), 0_tps};
    }
    else {
        meters_per_second_t desiredVx{sqrt((g_accel * targetDistance * targetDistance / denom).value())};

        double ux = targetVector.X().value() / targetDistance.value(); // Unit vector X components towards the target
        double uy = targetVector.Y().value() / targetDistance.value(); // Unit vector Y components towards the target

        double vForward  = robotVelocity.vx.value() * ux + robotVelocity.vy.value() * uy; // Robot velocity component in the direction of the target
        double vSideways = robotVelocity.vx.value() * uy - robotVelocity.vy.value() * ux; // Robot velocity component perpendicular to the direction of the target
        
        // Velocity error in the direction of the target
        double v_comp = desiredVx.value() - vForward; 

        // Calculate the feedforward velocity command with gains for stationary, approach, and retreat scenarios
        double velocity = desiredVx.value() * kStationaryGain - ((vForward >= 0) ? kApproachGain * vForward : kRetreatGain * vForward); //
        
        // Convert the velocity command to TPS (Turns Per Second) for the shooter motor
        TPS tps = 1_tps * velocity / kWheelCircumferenceMeters; 

        // Calculate the compensation angle based on the sideways velocity, with a gain to adjust the sensitivity
        double sign = (vSideways >= 0) ? 1.0 : -1.0;
        
        // The atan2 function is used to calculate the angle of compensation, 
        // and the absolute value of vSideways and v_comp are used to ensure the angle is calculated correctly regardless of direction. 
        // The kAngleGain is applied to adjust how aggressively the system compensates for sideways motion.
        double compAngleRad = sign * atan2(abs(vSideways), abs(v_comp)) * kAngleGain;
        return ShootCompOutput{Rotation2d(radian_t(compAngleRad)), tps};
    }
}
