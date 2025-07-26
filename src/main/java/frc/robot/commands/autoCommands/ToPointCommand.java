package frc.robot.commands.autoCommands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.telemetry.tunable.TunableTelemetryProfiledPIDController;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.types.Pose2dEntry;
import java.util.function.Supplier;

public class ToPointCommand extends Command {
  private final CommandSwerveDrivetrain drive;
  
  
  private final SwerveRequest.FieldCentric swerveRequestField = new SwerveRequest.FieldCentric();
  private final SwerveRequest.ApplyFieldSpeeds swerveRequest = new SwerveRequest.ApplyFieldSpeeds();

  private Supplier<Pose2d> desiredPoseSupplier;
  private Pose2d desiredPoseCurrent = new Pose2d();

  private final TunableTelemetryProfiledPIDController translationController =
      new TunableTelemetryProfiledPIDController(
          "/drive/auto",
          Constants.AutoConstants.pointTranslationGains,
          Constants.AutoConstants.trapPointTranslationGains);

  private final Pose2dEntry desiredPoseEntry = new Pose2dEntry("/drive/toPoint/neuralDesiredPose", true);
  private final DoubleTelemetryEntry speedXEntry = new DoubleTelemetryEntry("/drive/toPoint/desiredXSpeeds", true);
  private final DoubleTelemetryEntry speedYEntry = new DoubleTelemetryEntry("/drive/toPoint/desiredYSpeeds", true);
    private final SimpleMotorFeedforward ffController =
      Constants.AutoConstants.pointTranslationFFGains.createFeedforward();

  public ToPointCommand(CommandSwerveDrivetrain drive, Supplier<Pose2d> desiredPoseSupplier) {
    this.drive = drive;
    this.desiredPoseSupplier = desiredPoseSupplier;
    translationController.setTolerance(0.05);
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    desiredPoseEntry.append(desiredPoseCurrent);

    desiredPoseCurrent = desiredPoseSupplier.get();
    translationController.setGoal(0.0);

    translationController.reset(
        -getTranslationError().getNorm(),
        Math.hypot(
            drive.getPigeon2().getAngularVelocityXDevice().getValueAsDouble(),
            drive.getPigeon2().getAngularVelocityYDevice().getValueAsDouble()));
  }

  @Override
  public void execute() {
    Translation2d translationVeloctiy = new Translation2d();
    if (getTranslationError().getNorm() > .05) {
      double translationFeedback =
          translationController.calculate(-getTranslationError().getNorm());
      double translationFF = translationController.getSetpoint().velocity;

      translationVeloctiy =
          new Translation2d(translationFeedback + translationFF, getTranslationError().getAngle());

        Translation2d finalTranslationVeloctiy = translationVeloctiy;

      speedXEntry.append(finalTranslationVeloctiy.getX());
      speedYEntry.append(finalTranslationVeloctiy.getY());
      
      drive.applyRequest(
        ()->
              swerveRequest
                  .withSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                    finalTranslationVeloctiy.getX(),
                    finalTranslationVeloctiy.getY(),
                    1.0,
                    drive.getPose().getRotation()
                  ))).alongWith(Commands.run(()-> System.out.println("AHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH")));
    }
  }

  @Override
  public boolean isFinished() {
    return translationController.atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    drive.applyRequest(()-> swerveRequestField.withVelocityY(0).withVelocityX(0).withRotationalRate(0));
  }

  private Translation2d getTranslationError() {
    return desiredPoseCurrent.getTranslation().minus(drive.getPose().getTranslation());
  }
}