package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {

  private VisionSubsystem() {}

  public double getTargetVerticalOffset() {
    return LimelightHelpers.getLatestResults(Constants.VisionConstants.OBJECT_LIMELIGHT)
        .targets_Detector[0]
        .ty;
  }

  public double getTargetHorizontalOffset() {
    return LimelightHelpers.getLatestResults(Constants.VisionConstants.OBJECT_LIMELIGHT)
        .targets_Detector[0]
        .tx;
  }

  public double getEstimatedDistanceTarget() {
    double angleToGoalRadians =
        getTargetVerticalOffset() + Constants.VisionConstants.CAMERA_MOUNT_ANGLE;

    return (Constants.VisionConstants.CORAL_HEIGHT
            - Constants.VisionConstants.CAMERA_MOUNT_HEIGHT_METERS)
        / Math.tan(angleToGoalRadians);
  }

  public Pose2d getTargetTrajectory() {
    double estimatedDistanceTarget = getEstimatedDistanceTarget();

    return new Pose2d(
        (estimatedDistanceTarget
            * Math.cos(getTargetVerticalOffset() + Constants.VisionConstants.CAMERA_MOUNT_ANGLE)),
        (estimatedDistanceTarget
            * Math.sin(getTargetVerticalOffset() + Constants.VisionConstants.CAMERA_MOUNT_ANGLE)),
        new Rotation2d(0.0));
  }
}
