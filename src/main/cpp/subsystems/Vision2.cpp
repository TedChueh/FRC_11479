#include "subsystems/Vision2.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include <units/math.h>

VisionSubsystem2::VisionSubsystem2(Config config)
    : m_cfg(std::move(config)) {}

std::optional<VisionSubsystem2::VisionUpdate> VisionSubsystem2::GetLatestUpdate() const {
    return m_latestUpdate;
}

bool VisionSubsystem2::IsNewTimestamp(units::second_t timestamp) {
    if (m_lastVisionTimestamp >= 0_s && timestamp <= m_lastVisionTimestamp) {
        return false;
    }

    m_lastVisionTimestamp = timestamp;
    return true;
}

bool VisionSubsystem2::ShouldHardReject(
    int tagCount,
    units::meter_t avgTagDist,
    units::meter_t poseError,
    units::meters_per_second_t translationSpeed,
    units::degrees_per_second_t angularVelocity
) const {
    if (tagCount < 1) {
        return true;
    }

    if (avgTagDist > m_cfg.maxAcceptTagDistance) {
        return true;
    }

    if (translationSpeed > m_cfg.maxRejectLinearSpeed) {
        return true;
    }

    if (units::math::abs(angularVelocity) > m_cfg.maxRejectAngularSpeed) {
        return true;
    }

    // Very large disagreement while moving -> reject instead of sudden pull
    if (poseError > m_cfg.maxHardRejectPoseError &&
        (translationSpeed > 0.20_mps || units::math::abs(angularVelocity) > 90_deg_per_s)) {
        return true;
    }

    return false;
}

double VisionSubsystem2::ComputeXYStdDev(
    int tagCount,
    units::meter_t avgTagDist,
    units::meter_t poseError,
    units::meters_per_second_t translationSpeed,
    units::degrees_per_second_t angularVelocity
) const {
    double xyStdDev = m_cfg.baseXYStdDev;

    // Farther tags -> less trust
    xyStdDev += avgTagDist.value() * m_cfg.distanceScalar;

    // Fewer tags -> less trust
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
    xyStdDev += std::abs(angularVelocity.value()) * m_cfg.angularSpeedScalar;

    // More disagreement with drivetrain pose -> less trust
    if (poseError > m_cfg.mediumPoseError) {
        xyStdDev += m_cfg.mediumErrorPenalty;
    }
    if (poseError > m_cfg.largePoseError) {
        xyStdDev += m_cfg.largeErrorPenalty;
    }

    return std::clamp(xyStdDev, m_cfg.minXYStdDev, m_cfg.maxXYStdDev);
}

void VisionSubsystem2::PublishAcceptedTelemetry(const VisionUpdate& update) const {
    frc::SmartDashboard::PutBoolean("Vision/Accepted", update.accepted);
    frc::SmartDashboard::PutBoolean("Vision/SuggestSeed", update.suggestSeed);
    frc::SmartDashboard::PutString("Vision/RejectReason", "Accepted");

    frc::SmartDashboard::PutNumber("Vision/TagCount", static_cast<double>(update.tagCount));
    frc::SmartDashboard::PutNumber("Vision/AvgTagDistM", update.avgTagDist.value());
    frc::SmartDashboard::PutNumber("Vision/PoseErrorM", update.poseError.value());
    frc::SmartDashboard::PutNumber("Vision/XYStdDev", update.xyStdDev);
    frc::SmartDashboard::PutNumber("Vision/RotStdDev", update.rotStdDev);
    frc::SmartDashboard::PutNumber("Vision/Timestamp", update.timestamp.value());

    frc::SmartDashboard::PutNumber("Vision/PoseX", update.pose.X().value());
    frc::SmartDashboard::PutNumber("Vision/PoseY", update.pose.Y().value());
    frc::SmartDashboard::PutNumber("Vision/PoseDeg", update.pose.Rotation().Degrees().value());
}

void VisionSubsystem2::PublishRejectedTelemetry(
    const char* reason,
    int tagCount,
    units::meter_t avgTagDist,
    units::meter_t poseError
) const {
    frc::SmartDashboard::PutBoolean("Vision/Accepted", false);
    frc::SmartDashboard::PutString("Vision/RejectReason", reason);
    frc::SmartDashboard::PutNumber("Vision/TagCount", static_cast<double>(tagCount));
    frc::SmartDashboard::PutNumber("Vision/AvgTagDistM", avgTagDist.value());
    frc::SmartDashboard::PutNumber("Vision/PoseErrorM", poseError.value());
}

bool VisionSubsystem2::ComputeSeedSuggestion(
    const frc::Pose2d& visionPose,
    units::second_t timestamp,
    units::meter_t poseError,
    int tagCount,
    units::meters_per_second_t translationSpeed,
    units::degrees_per_second_t angularVelocity
) {
    const bool seedCandidate =
        (poseError > m_cfg.seedMinPoseError) &&
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

    const auto candidateDelta =
        visionPose.Translation().Distance(m_lastSeedCandidatePose.Translation());

    if (candidateDelta < m_cfg.seedConsistencyTolerance) {
        m_seedStableFrames++;
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

void VisionSubsystem2::Update(
    const frc::Pose2d& robotPose,
    units::meters_per_second_t translationSpeed,
    units::degrees_per_second_t angularVelocity
) {
    m_latestUpdate.reset();

    // Feed drivetrain heading / yaw rate to Limelight MegaTag2
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

    if (!mt2.has_value() || mt2->tagCount < 1) {
        PublishRejectedTelemetry("NoTags");
        return;
    }

    const units::second_t timestamp{mt2->timestampSeconds};
    const frc::Pose2d visionPose = mt2->pose;
    const int tagCount = mt2->tagCount;
    const units::meter_t avgTagDist{mt2->avgTagDist};
    const units::meter_t poseError =
        visionPose.Translation().Distance(robotPose.Translation());

    if (!IsNewTimestamp(timestamp)) {
        PublishRejectedTelemetry("OldOrDuplicate", tagCount, avgTagDist, poseError);
        return;
    }

    if (ShouldHardReject(tagCount, avgTagDist, poseError, translationSpeed, angularVelocity)) {
        PublishRejectedTelemetry("HardReject", tagCount, avgTagDist, poseError);
        return;
    }

    const double xyStdDev = ComputeXYStdDev(
        tagCount,
        avgTagDist,
        poseError,
        translationSpeed,
        angularVelocity
    );

    const bool suggestSeed = ComputeSeedSuggestion(
        visionPose,
        timestamp,
        poseError,
        tagCount,
        translationSpeed,
        angularVelocity
    );

    VisionUpdate update;
    update.pose = visionPose;
    update.timestamp = timestamp;
    update.xyStdDev = xyStdDev;
    update.rotStdDev = m_cfg.rotStdDev;
    update.tagCount = tagCount;
    update.avgTagDist = avgTagDist;
    update.poseError = poseError;
    update.accepted = true;
    update.suggestSeed = suggestSeed;

    m_latestUpdate = update;
    PublishAcceptedTelemetry(update);
}