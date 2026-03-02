#ifndef MATHUTILS_H
#define MATHUTILS_H

#include "units/angular_velocity.h"

#include <ctre/phoenix6/swerve/SwerveDrivetrain.hpp>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include <cmath>

using namespace std;
using namespace frc;
using namespace units;
using namespace ctre::phoenix6::swerve;
using TPS = turns_per_second_t;

constexpr acceleration::meters_per_second_squared_t g_accel = 9.80665_mps_sq;

struct ShootCompOutput {
    Rotation2d compAngle;    
    TPS tps;              
};

Rotation2d calcHeadingError(Translation2d targetPosition, const impl::SwerveDrivetrainImpl::SwerveDriveState& robotState);
ShootCompOutput calcShootComp(degree_t shootDegree,
                              meter_t deltaHeight,
                              Translation2d targetPosition,
                              const impl::SwerveDrivetrainImpl::SwerveDriveState& robotState,
                              meter_t wheelRadius_m, 
                              double kApproachGain = 1.0,  
                              double kStationaryGain = 1.0,
                              double kRetreatGain = 1.0,
                              double kAngleGain = 1.0);
#endif 