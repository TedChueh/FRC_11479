#include "subsystems/Vision.h"
#include <frc/geometry/Rotation2d.h>
#include <cmath>

void VisionSubsystem::PeriodicUpdate(const Pose2d& robotPose, const meters_per_second_t translationSpeed, const degrees_per_second_t angularVelocity) {
    // Provide gyroscope data to Limelight (a core requirement of MegaTag2)
    LimelightHelpers::SetRobotOrientation(
        "limelight",
        robotPose.Rotation().Degrees().value(),
        angularVelocity.value(),
        0, 0, 0, 0
    );

    // Retrieve MegaTag2 estimate
    auto llMeasurement = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

    // Basic check: clear data if no tag is detected
    if (!llMeasurement || llMeasurement->tagCount < 1) {
        m_measurement.reset();
        return;
    }

    // Get current timestamp
    second_t currentTimestamp = second_t(llMeasurement->timestampSeconds);

    // Prevent time from going backward or from being outdated
    if (currentTimestamp < lastVisionUpdate || currentTimestamp - lastVisionUpdate > 1_s) {
        lastVisionUpdate = currentTimestamp;
        m_measurement.reset();
        return;
    }

    // Update rate limit: at least 50ms between updates to prevent flooding the system with too much data
    if (currentTimestamp - lastVisionUpdate < 0.05_s) {
        m_measurement.reset();
        return;
    }
    lastVisionUpdate = currentTimestamp;
    
    // --- Entering filtering stage ---
    double dist = llMeasurement->avgTagDist;
    int tagCount = llMeasurement->tagCount;

    // A. Distance filtering
    if (meter_t{dist} > 3.5_m) {
        m_measurement.reset();
        return;
    }

    // B. Velocity filtering
    bool isAuto = DriverStation::IsAutonomous();
    meters_per_second_t maxSpeed = isAuto ? 1_mps : 2.0_mps; 
    if (translationSpeed > maxSpeed || math::abs(angularVelocity) > 360_deg_per_s) {
        m_measurement.reset();
        return;
    }

    // C. Error spike and reset detection
    auto poseError = llMeasurement->pose.Translation().Distance(robotPose.Translation());
    
    // [Key Logic] Determine if this data qualifies for a "forced reset (Seed)"
    // Conditions: large error, robot nearly stationary, and multiple tags detected to ensure absolute position accuracy
    bool canForceSeed = (poseError > 0.5_m && translationSpeed < 0.1_mps && tagCount >= 2);

    // If the error is greater than 0.5 meters but doesn't meet the "forced reset" conditions, this data is invalid and should be discarded
    if (poseError > 0.5_m && !canForceSeed) {
        m_measurement.reset();
        return;
    }

    // Calculate dynamic trust weight (Standard Deviations)
    double xyStdDev;
    double rotStdDev = 999999.0; // Never trust visual rotation; delegate to Pigeon 2

    if (tagCount >= 2) {
        xyStdDev = 0.12 + (dist * 0.08);
    } else {
        xyStdDev = 0.35 + (dist * 0.22);
    }
    xyStdDev = clamp(xyStdDev, 0.1, 2.5);
    
    // Package data into VisionMeasurement struct
    VisionMeasurement vm;
    vm.pose = llMeasurement->pose;
    vm.xyStdDev = xyStdDev;
    vm.rotStdDev = rotStdDev;
    vm.tagCount = tagCount;
    vm.timestamp = currentTimestamp;
    
    // --- New: Send determination results to the chassis ---
    vm.isReliableForSeeding = canForceSeed; 

    m_measurement = vm; 

    // Telemetry
    SmartDashboard::PutNumber("Vision/PoseError_m", poseError.value());
    SmartDashboard::PutNumber("Vision/XY_StdDev", xyStdDev);
    SmartDashboard::PutNumber("Vision/TagsSeen", (double)tagCount);
    SmartDashboard::PutBoolean("Vision/IsReliableForSeed", canForceSeed);
}