package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import java.util.OptionalDouble;

public class VisionSubsystem extends SubsystemBase {

  private VisionSubsystem() {}

  public OptionalDouble getTargetVerticalOffset() {
    LimelightHelpers.LimelightTarget_Detector detector =
        LimelightHelpers.getLatestResults(Constants.VisionConstants.OBJECT_LIMELIGHT)
            .targets_Detector[0];
    if (detector.confidence > 80) {
      return OptionalDouble.of(detector.ty);
    } else {
      return OptionalDouble.empty();
    }
  }

  public OptionalDouble getTargetHorizontalOffset() {
    LimelightHelpers.LimelightTarget_Detector detector =
        LimelightHelpers.getLatestResults(Constants.VisionConstants.OBJECT_LIMELIGHT)
            .targets_Detector[0];
    if (detector.confidence > 80) {
      return OptionalDouble.of(detector.tx);
    } else {
      return OptionalDouble.empty();
    }
  }

  public double getEstimatedDistanceTarget() {
    double angleToGoalRadians =
        getTargetVerticalOffset().getAsDouble() + Constants.VisionConstants.CAMERA_MOUNT_ANGLE;

    return (Constants.VisionConstants.CORAL_HEIGHT
            - Constants.VisionConstants.CAMERA_MOUNT_HEIGHT_METERS)
        / Math.tan(angleToGoalRadians);
  }

  public Pose2d getTargetTrajectory() {
    double estimatedDistanceTarget = getEstimatedDistanceTarget();

    return new Pose2d(
        (estimatedDistanceTarget
            * Math.cos(
                getTargetVerticalOffset().getAsDouble()
                    + Constants.VisionConstants.CAMERA_MOUNT_ANGLE)),
        (estimatedDistanceTarget
            * Math.sin(
                getTargetVerticalOffset().getAsDouble()
                    + Constants.VisionConstants.CAMERA_MOUNT_ANGLE)),
        new Rotation2d(0.0));
  }
}
