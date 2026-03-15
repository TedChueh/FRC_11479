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

using namespace units::literals;

class VisionSubsystem : public frc2::SubsystemBase {
public:
    struct VisionUpdate {
        frc::Pose2d pose;
        units::second_t timestamp{0_s};

        double xyStdDev = 999999.0;
        double rotStdDev = 999999.0;

        int tagCount = 0;
        units::meter_t avgTagDist{0_m};

        units::meter_t translationError{0_m};
        units::degree_t headingError{0_deg};

        bool accepted = false;
        bool suggestSeed = false;
    };

    struct Config {
        std::string limelightName = "limelight";

        // Hard reject thresholds
        units::meter_t maxAcceptTagDistance = 5.5_m;
        units::meter_t maxHardRejectTranslationError = 1.5_m;
        units::degree_t maxHardRejectHeadingErrorSingleTag = 25_deg;
        units::meters_per_second_t maxRejectLinearSpeed = 5.0_mps;
        units::degrees_per_second_t maxRejectAngularSpeed = 360_deg_per_s;

        // Base XY trust
        double baseXYStdDev = 0.12;
        double minXYStdDev = 0.08;
        double maxXYStdDev = 2.0;

        // Distance / tag-count trust shaping
        double distanceScalar = 0.020;
        double oneTagPenalty = 0.18;
        double twoTagPenalty = 0.06;
        double farSingleTagPenalty = 0.10;
        units::meter_t farSingleTagDistance = 4.0_m;

        // Motion trust shaping
        double linearSpeedScalar = 0.025;
        double angularSpeedScalar = 0.0006;

        // Translation disagreement shaping
        units::meter_t mediumTranslationError = 0.45_m;
        units::meter_t largeTranslationError = 0.90_m;
        double mediumTranslationPenalty = 0.12;
        double largeTranslationPenalty = 0.28;

        // Heading disagreement shaping
        units::degree_t mediumHeadingError = 10_deg;
        units::degree_t largeHeadingError = 20_deg;
        double headingErrorScalar = 0.002;
        double mediumHeadingPenalty = 0.06;
        double largeHeadingPenalty = 0.16;

        // Rotation trust
        double defaultRotStdDev = 999999.0;   // almost ignore rotation by default
        double closeMultiTagRotStdDev = 999999.0; 
        units::meter_t rotTrustMaxTagDistance = 2.5_m;
        units::degrees_per_second_t rotTrustMaxAngularSpeed = 120_deg_per_s;
        units::degree_t rotTrustMaxHeadingError = 10_deg;

        // Seed suggestion
        units::meter_t seedMinTranslationError = 1.2_m;
        units::meters_per_second_t seedMaxLinearSpeed = 0.05_mps;
        units::degrees_per_second_t seedMaxAngularSpeed = 15_deg_per_s;
        int seedMinTagCount = 2;
        int seedStableFramesRequired = 4;
        units::meter_t seedConsistencyTolerance = 0.15_m;
        units::degree_t seedHeadingTolerance = 8_deg;
        units::degree_t seedMaxHeadingError = 12_deg;
        units::second_t seedCooldown = 1.0_s;
    };

    enum class RejectReason {
        None,
        NoMeasurement,
        NoTags,
        DuplicateTimestamp,
        TagTooFar,
        LinearSpeedTooHigh,
        AngularSpeedTooHigh,
        LargeHeadingErrorSingleTag,
        LargeDisagreementWhileMoving
    };

    VisionSubsystem();
    explicit VisionSubsystem(Config config);

    void Update(
        const frc::Pose2d& robotPose,
        units::meters_per_second_t translationSpeed,
        units::degrees_per_second_t angularVelocity,
        bool isAuto
    );

    std::optional<VisionUpdate> GetLatestUpdate() const;

private:
    Config m_cfg;
    std::optional<VisionUpdate> m_latestUpdate;
    units::second_t m_lastVisionTimestamp{-1_s};

    frc::Pose2d m_lastSeedCandidatePose;
    int m_seedStableFrames = 0;
    units::second_t m_lastSeedTime{-999_s};

    bool IsNewTimestamp(units::second_t timestamp) const;

    RejectReason GetRejectReason(
        int tagCount,
        units::meter_t avgTagDist,
        units::meter_t translationError,
        units::degree_t headingError,
        units::meters_per_second_t translationSpeed,
        units::degrees_per_second_t angularVelocity
    ) const;

    double ComputeXYStdDev(
        int tagCount,
        units::meter_t avgTagDist,
        units::meter_t translationError,
        units::degree_t headingError,
        units::meters_per_second_t translationSpeed,
        units::degrees_per_second_t angularVelocity
    ) const;

    double ComputeRotStdDev(
        int tagCount,
        units::meter_t avgTagDist,
        units::degree_t headingError,
        units::degrees_per_second_t angularVelocity
    ) const;

    bool ComputeSeedSuggestion(
        const frc::Pose2d& visionPose,
        units::second_t timestamp,
        units::meter_t translationError,
        units::degree_t headingError,
        int tagCount,
        units::meters_per_second_t translationSpeed,
        units::degrees_per_second_t angularVelocity
    );

    static const char* RejectReasonToString(RejectReason reason);

    void PublishAcceptedTelemetry(const VisionUpdate& update) const;
    void PublishRejectedTelemetry(
        RejectReason reason,
        units::second_t timestamp = 0_s,
        int tagCount = 0,
        units::meter_t avgTagDist = 0_m,
        units::meter_t translationError = 0_m,
        units::degree_t headingError = 0_deg
    ) const;
};