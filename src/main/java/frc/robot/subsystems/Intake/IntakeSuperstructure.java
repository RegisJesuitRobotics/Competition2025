package frc.robot.subsystems.Intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSuperstructure extends SubsystemBase {
  private final IntakeSpinningSubsystem SpinningSubsystem = new IntakeSpinningSubsystem();
  private final IntakeRotationSubsystem rotationSubsystem = new IntakeRotationSubsystem();

  public IntakeRotationSubsystem getIntakeRotationSubsystem() {
    return rotationSubsystem;
  }

  public IntakeSpinningSubsystem getIntakeSpinningSubsystem() {
    return SpinningSubsystem;
  }

  public Command setDownAndRunCommand() {
    return Commands.parallel(
            SpinningSubsystem.setVoltageCommand(IntakeConstants.SPINNING_VOLTAGE),
            rotationSubsystem.setRotationGoalCommand(Rotation2d.fromRadians(IntakeConstants.ROTATION_DOWN_ANGLE)))
        .withName("IntakeSetDownAndRun");
  }

  public Command setUpCommand() {
    return Commands.parallel(
            SpinningSubsystem.setVoltageCommand(0.0),
            rotationSubsystem.setRotationGoalCommand(Rotation2d.fromRadians(IntakeConstants.ROTATION_UP_ANGLE)))
        .withName("IntakeSetUp");
  }
}