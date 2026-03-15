#include "subsystems/Vision.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include <units/math.h>

VisionSubsystem::VisionSubsystem()
    : VisionSubsystem(Config{}) {}

VisionSubsystem::VisionSubsystem(Config config)
    : m_cfg(std::move(config)) {}

std::optional<VisionSubsystem::VisionUpdate> VisionSubsystem::GetLatestUpdate() const {
    return m_latestUpdate;
}

bool VisionSubsystem::IsNewTimestamp(units::second_t timestamp) const {
    return !(m_lastVisionTimestamp >= 0_s && timestamp <= m_lastVisionTimestamp);
}

VisionSubsystem::RejectReason VisionSubsystem::GetRejectReason(
    int tagCount,
    units::meter_t avgTagDist,
    units::meter_t translationError,
    units::degree_t headingError,
    units::meters_per_second_t translationSpeed,
    units::degrees_per_second_t angularVelocity
) const {
    if (tagCount < 1) {
        return RejectReason::NoTags;
    }

    if (avgTagDist > m_cfg.maxAcceptTagDistance) {
        return RejectReason::TagTooFar;
    }

    if (translationSpeed > m_cfg.maxRejectLinearSpeed) {
        return RejectReason::LinearSpeedTooHigh;
    }

    if (units::math::abs(angularVelocity) > m_cfg.maxRejectAngularSpeed) {
        return RejectReason::AngularSpeedTooHigh;
    }

    if (tagCount == 1 && headingError > m_cfg.maxHardRejectHeadingErrorSingleTag) {
        return RejectReason::LargeHeadingErrorSingleTag;
    }

    if (translationError > m_cfg.maxHardRejectTranslationError &&
        (translationSpeed > 0.20_mps || units::math::abs(angularVelocity) > 120_deg_per_s)) {
        return RejectReason::LargeDisagreementWhileMoving;
    }

    return RejectReason::None;
}

double VisionSubsystem::ComputeXYStdDev(
    int tagCount,
    units::meter_t avgTagDist,
    units::meter_t translationError,
    units::degree_t headingError,
    units::meters_per_second_t translationSpeed,
    units::degrees_per_second_t angularVelocity
) const {
    double xyStdDev = m_cfg.baseXYStdDev;

    // Farther tags -> less trust
    xyStdDev += avgTagDist.value() * m_cfg.distanceScalar;

    // Fewer tags -> slightly less trust, but not overly harsh for MegaTag2
    if (tagCount == 1) {
        xyStdDev += m_cfg.oneTagPenalty;

        if (avgTagDist > m_cfg.farSingleTagDistance) {
            xyStdDev += m_cfg.farSingleTagPenalty;
        }
    } else if (tagCount == 2) {
        xyStdDev += m_cfg.twoTagPenalty;
    }

    // More motion -> less trust
    xyStdDev += translationSpeed.value() * m_cfg.linearSpeedScalar;
    xyStdDev += units::math::abs(angularVelocity).value() * m_cfg.angularSpeedScalar;

    // Translation disagreement with drivetrain pose
    if (translationError > m_cfg.mediumTranslationError) {
        xyStdDev += m_cfg.mediumTranslationPenalty;
    }
    if (translationError > m_cfg.largeTranslationError) {
        xyStdDev += m_cfg.largeTranslationPenalty;
    }

    // Heading disagreement with drivetrain pose
    xyStdDev += headingError.value() * m_cfg.headingErrorScalar;

    if (headingError > m_cfg.mediumHeadingError) {
        xyStdDev += m_cfg.mediumHeadingPenalty;
    }
    if (headingError > m_cfg.largeHeadingError) {
        xyStdDev += m_cfg.largeHeadingPenalty;
    }

    return std::clamp(xyStdDev, m_cfg.minXYStdDev, m_cfg.maxXYStdDev);
}

double VisionSubsystem::ComputeRotStdDev(
    int tagCount,
    units::meter_t avgTagDist,
    units::degree_t headingError,
    units::degrees_per_second_t angularVelocity
) const {
    if (tagCount >= 2 &&
        avgTagDist <= m_cfg.rotTrustMaxTagDistance &&
        units::math::abs(angularVelocity) <= m_cfg.rotTrustMaxAngularSpeed &&
        headingError <= m_cfg.rotTrustMaxHeadingError) {
        return m_cfg.closeMultiTagRotStdDev;
    }

    return m_cfg.defaultRotStdDev;
}

bool VisionSubsystem::ComputeSeedSuggestion(
    const frc::Pose2d& visionPose,
    units::second_t timestamp,
    units::meter_t translationError,
    units::degree_t headingError,
    int tagCount,
    units::meters_per_second_t translationSpeed,
    units::degrees_per_second_t angularVelocity
) {
    const bool seedCandidate =
        (translationError > m_cfg.seedMinTranslationError) &&
        (headingError <= m_cfg.seedMaxHeadingError) &&
        (translationSpeed < m_cfg.seedMaxLinearSpeed) &&
        (units::math::abs(angularVelocity) < m_cfg.seedMaxAngularSpeed) &&
        (tagCount >= m_cfg.seedMinTagCount);

    if (!seedCandidate) {
        m_seedStableFrames = 0;
        return false;
    }

    if (m_seedStableFrames == 0) {
        m_seedStableFrames = 1;
        m_lastSeedCandidatePose = visionPose;
        return false;
    }

    const units::meter_t translationDelta =
        visionPose.Translation().Distance(m_lastSeedCandidatePose.Translation());

    const units::degree_t headingDelta = units::degree_t{
        std::abs((visionPose.Rotation().Degrees() - m_lastSeedCandidatePose.Rotation().Degrees()).value())
    };

    if (translationDelta < m_cfg.seedConsistencyTolerance &&
        headingDelta < m_cfg.seedHeadingTolerance) {
        ++m_seedStableFrames;
    } else {
        m_seedStableFrames = 1;
    }

    m_lastSeedCandidatePose = visionPose;

    const bool cooldownOk = (timestamp - m_lastSeedTime) > m_cfg.seedCooldown;
    if (m_seedStableFrames >= m_cfg.seedStableFramesRequired && cooldownOk) {
        m_lastSeedTime = timestamp;
        m_seedStableFrames = 0;
        return true;
    }

    return false;
}

const char* VisionSubsystem::RejectReasonToString(RejectReason reason) {
    switch (reason) {
        case RejectReason::None:
            return "None";
        case RejectReason::NoMeasurement:
            return "NoMeasurement";
        case RejectReason::NoTags:
            return "NoTags";
        case RejectReason::DuplicateTimestamp:
            return "DuplicateTimestamp";
        case RejectReason::TagTooFar:
            return "TagTooFar";
        case RejectReason::LinearSpeedTooHigh:
            return "LinearSpeedTooHigh";
        case RejectReason::AngularSpeedTooHigh:
            return "AngularSpeedTooHigh";
        case RejectReason::LargeHeadingErrorSingleTag:
            return "LargeHeadingErrorSingleTag";
        case RejectReason::LargeDisagreementWhileMoving:
            return "LargeDisagreementWhileMoving";
        default:
            return "Unknown";
    }
}

void VisionSubsystem::PublishAcceptedTelemetry(const VisionUpdate& update) const {
    frc::SmartDashboard::PutBoolean("Vision/Accepted", update.accepted);
    frc::SmartDashboard::PutBoolean("Vision/SuggestSeed", update.suggestSeed);
    frc::SmartDashboard::PutString("Vision/RejectReason", "Accepted");

    frc::SmartDashboard::PutNumber("Vision/TagCount", static_cast<double>(update.tagCount));
    frc::SmartDashboard::PutNumber("Vision/AvgTagDistM", update.avgTagDist.value());
    frc::SmartDashboard::PutNumber("Vision/TranslationErrorM", update.translationError.value());
    frc::SmartDashboard::PutNumber("Vision/HeadingErrorDeg", update.headingError.value());
    frc::SmartDashboard::PutNumber("Vision/XYStdDev", update.xyStdDev);
    frc::SmartDashboard::PutNumber("Vision/RotStdDev", update.rotStdDev);
    frc::SmartDashboard::PutNumber("Vision/Timestamp", update.timestamp.value());

    frc::SmartDashboard::PutNumber("Vision/PoseX", update.pose.X().value());
    frc::SmartDashboard::PutNumber("Vision/PoseY", update.pose.Y().value());
    frc::SmartDashboard::PutNumber("Vision/PoseDeg", update.pose.Rotation().Degrees().value());
}

void VisionSubsystem::PublishRejectedTelemetry(
    RejectReason reason,
    units::second_t timestamp,
    int tagCount,
    units::meter_t avgTagDist,
    units::meter_t translationError,
    units::degree_t headingError
) const {
    frc::SmartDashboard::PutBoolean("Vision/Accepted", false);
    frc::SmartDashboard::PutBoolean("Vision/SuggestSeed", false);
    frc::SmartDashboard::PutString("Vision/RejectReason", RejectReasonToString(reason));

    frc::SmartDashboard::PutNumber("Vision/Timestamp", timestamp.value());
    frc::SmartDashboard::PutNumber("Vision/TagCount", static_cast<double>(tagCount));
    frc::SmartDashboard::PutNumber("Vision/AvgTagDistM", avgTagDist.value());
    frc::SmartDashboard::PutNumber("Vision/TranslationErrorM", translationError.value());
    frc::SmartDashboard::PutNumber("Vision/HeadingErrorDeg", headingError.value());
}

void VisionSubsystem::Update(
    const frc::Pose2d& robotPose,
    units::meters_per_second_t translationSpeed,
    units::degrees_per_second_t angularVelocity,
    bool isAuto
) {
    m_latestUpdate.reset();

    // MegaTag2 requires the robot heading each cycle
    LimelightHelpers::SetRobotOrientation(
        m_cfg.limelightName,
        robotPose.Rotation().Degrees().value(),
        angularVelocity.value(),
        0.0,
        0.0,
        0.0,
        0.0
    );

    auto mt2 = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2(m_cfg.limelightName);

    if (!mt2.has_value()) {
        m_seedStableFrames = 0;
        PublishRejectedTelemetry(RejectReason::NoMeasurement);
        return;
    }

    const units::second_t timestamp{mt2->timestampSeconds};
    const frc::Pose2d visionPose = mt2->pose;
    const int tagCount = mt2->tagCount;
    const units::meter_t avgTagDist{mt2->avgTagDist};

    if (tagCount < 1) {
        m_seedStableFrames = 0;
        PublishRejectedTelemetry(RejectReason::NoTags, timestamp, tagCount, avgTagDist);
        return;
    }

    if (!IsNewTimestamp(timestamp)) {
        m_seedStableFrames = 0;
        PublishRejectedTelemetry(RejectReason::DuplicateTimestamp, timestamp, tagCount, avgTagDist);
        return;
    }

    const units::meter_t translationError =
        visionPose.Translation().Distance(robotPose.Translation());

    const units::degree_t headingError = units::degree_t{
        std::abs((visionPose.Rotation().Degrees() - robotPose.Rotation().Degrees()).value())
    };

    const RejectReason rejectReason = GetRejectReason(
        tagCount,
        avgTagDist,
        translationError,
        headingError,
        translationSpeed,
        angularVelocity
    );

    if (rejectReason != RejectReason::None) {
        m_seedStableFrames = 0;
        PublishRejectedTelemetry(
            rejectReason,
            timestamp,
            tagCount,
            avgTagDist,
            translationError,
            headingError
        );
        return;
    }

    // Consume timestamp only after validation passes
    m_lastVisionTimestamp = timestamp;

    double xyStdDev = ComputeXYStdDev(
        tagCount,
        avgTagDist,
        translationError,
        headingError,
        translationSpeed,
        angularVelocity
    );

    if(isAuto){
        xyStdDev *= 1.3;
    }

    const double rotStdDev = ComputeRotStdDev(
        tagCount,
        avgTagDist,
        headingError,
        angularVelocity
    );

    const bool suggestSeed = ComputeSeedSuggestion(
        visionPose,
        timestamp,
        translationError,
        headingError,
        tagCount,
        translationSpeed,
        angularVelocity
    );

    VisionUpdate update;
    update.pose = visionPose;
    update.timestamp = timestamp;
    update.xyStdDev = xyStdDev;
    update.rotStdDev = rotStdDev;
    update.tagCount = tagCount;
    update.avgTagDist = avgTagDist;
    update.translationError = translationError;
    update.headingError = headingError;
    update.accepted = true;
    update.suggestSeed = suggestSeed;

    m_latestUpdate = update;
    PublishAcceptedTelemetry(update);
}