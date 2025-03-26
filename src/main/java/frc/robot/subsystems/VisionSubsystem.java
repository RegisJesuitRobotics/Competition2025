package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// @Logged
public class VisionSubsystem extends SubsystemBase {
  private final CommandSwerveDrivetrain drivetrain;

  public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }
}

//   public OptionalDouble getTargetVerticalOffset() {
//     LimelightHelpers.LimelightTarget_Detector detector =
//         LimelightHelpers.getLatestResults(Constants.VisionConstants.OBJECT_LIMELIGHT)
//             .targets_Detector[0];
//     if (detector.confidence > Constants.VisionConstants.CONFIDENCE_THRESHOLD) {
//       return OptionalDouble.of(detector.ty);
//     } else {
//       return OptionalDouble.empty();
//     }
//   }

//   public OptionalDouble getTargetHorizontalOffset() {
//     LimelightHelpers.LimelightTarget_Detector detector =
//         LimelightHelpers.getLatestResults(Constants.VisionConstants.OBJECT_LIMELIGHT)
//             .targets_Detector[0];
//     if (detector.confidence > Constants.VisionConstants.CONFIDENCE_THRESHOLD) {
//       return OptionalDouble.of(detector.tx);
//     } else {
//       return OptionalDouble.empty();
//     }
//   }

//   // @Logged(name = "estimated target distance")
//   public double getEstimatedDistanceTarget() {
//     double angleToGoalRadians =
//         getTargetVerticalOffset().getAsDouble() + Constants.VisionConstants.CAMERA_MOUNT_ANGLE;

//     return (Constants.VisionConstants.CORAL_HEIGHT
//             - Constants.VisionConstants.CAMERA_MOUNT_HEIGHT_METERS)
//         / Math.tan(angleToGoalRadians);
//   }

//   // @Logged(name = "target trajectory") // horizontal offset?
//   public Pose2d getTargetTrajectory() {
//     double estimatedDistanceTarget = getEstimatedDistanceTarget();
//     Pose2d poseFromRobot = new Pose2d(
//             (estimatedDistanceTarget
//                     * Math.cos(
//                     90-getTargetHorizontalOffset().getAsDouble()
//                             )),
//             (estimatedDistanceTarget
//                     * Math.sin(
//                     90-getTargetHorizontalOffset().getAsDouble()
//                             )),
//             new Rotation2d(0.0));

//     return poseFromRobot.relativeTo(drivetrain.getPose());
//   }
// }
