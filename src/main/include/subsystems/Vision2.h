#pragma once

#include <optional>
#include <string>

#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/geometry/Pose2d.h>
#include <frc2/command/SubsystemBase.h>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>

#include "LimelightHelpers.h"

class VisionSubsystem2 : public frc2::SubsystemBase {
public:
    struct VisionUpdate {
        frc::Pose2d pose;
        units::second_t timestamp;

        double xyStdDev;
        double rotStdDev;

        int tagCount;
        units::meter_t avgTagDist;
        units::meter_t poseError;

        bool accepted;
        bool suggestSeed;
    };

    struct Config {
        std::string limelightName = "limelight";

        units::meter_t maxAcceptTagDistance = 5.5_m;
        units::meter_t maxHardRejectPoseError = 1.8_m;
        units::meters_per_second_t maxRejectLinearSpeed = 4.8_mps;
        units::degrees_per_second_t maxRejectAngularSpeed = 540_deg_per_s;

        double baseXYStdDev = 0.05;
        double distanceScalar = 0.025;
        double twoTagPenalty = 0.05;
        double oneTagPenalty = 0.18;
        double farSingleTagPenalty = 0.10;
        units::meter_t farSingleTagDistance = 3.5_m;

        double linearSpeedScalar = 0.03;
        double angularSpeedScalar = 0.0008;
        double mediumErrorPenalty = 0.10;
        double largeErrorPenalty = 0.25;

        units::meter_t mediumPoseError = 0.35_m;
        units::meter_t largePoseError = 0.70_m;

        double minXYStdDev = 0.04;
        double maxXYStdDev = 1.2;
        double rotStdDev = 999999.0;

        units::meter_t seedMinPoseError = 1.0_m;
        units::meters_per_second_t seedMaxLinearSpeed = 0.05_mps;
        units::degrees_per_second_t seedMaxAngularSpeed = 20_deg_per_s;
        int seedMinTagCount = 2;
        int seedStableFramesRequired = 3;
        units::meter_t seedConsistencyTolerance = 0.15_m;
        units::second_t seedCooldown = 1.0_s;
    };

    VisionSubsystem2();
    explicit VisionSubsystem2(Config config);

    void Update(
        const frc::Pose2d& robotPose,
        units::meters_per_second_t translationSpeed,
        units::degrees_per_second_t angularVelocity
    );

    std::optional<VisionUpdate> GetLatestUpdate() const;

private:
    Config m_cfg;
    std::optional<VisionUpdate> m_latestUpdate;
    units::second_t m_lastVisionTimestamp{-1_s};

    frc::Pose2d m_lastSeedCandidatePose;
    int m_seedStableFrames = 0;
    units::second_t m_lastSeedTime{-999_s};

    bool IsNewTimestamp(units::second_t timestamp);
    bool ShouldHardReject(
        int tagCount,
        units::meter_t avgTagDist,
        units::meter_t poseError,
        units::meters_per_second_t translationSpeed,
        units::degrees_per_second_t angularVelocity
    ) const;

    double ComputeXYStdDev(
        int tagCount,
        units::meter_t avgTagDist,
        units::meter_t poseError,
        units::meters_per_second_t translationSpeed,
        units::degrees_per_second_t angularVelocity
    ) const;

    bool ComputeSeedSuggestion(
        const frc::Pose2d& visionPose,
        units::second_t timestamp,
        units::meter_t poseError,
        int tagCount,
        units::meters_per_second_t translationSpeed,
        units::degrees_per_second_t angularVelocity
    );

    void PublishAcceptedTelemetry(const VisionUpdate& update) const;
    void PublishRejectedTelemetry(
        const char* reason,
        int tagCount = 0,
        units::meter_t avgTagDist = 0_m,
        units::meter_t poseError = 0_m
    ) const;
};