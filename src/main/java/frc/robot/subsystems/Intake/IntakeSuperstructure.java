package frc.robot.subsystems.Intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSuperstructure extends SubsystemBase {
  private final IntakeSpinningSubsystem intakeSpinningSubsystem;
  private final IntakeRotationSubsystem intakeRotationSubsystem;

  public IntakeSuperstructure(
      IntakeSpinningSubsystem intakeSpinningSubsystem,
      IntakeRotationSubsystem intakeRotationSubsystem) {
    this.intakeSpinningSubsystem = intakeSpinningSubsystem;
    this.intakeRotationSubsystem = intakeRotationSubsystem;
  }

  public IntakeRotationSubsystem getIntakeRotationSubsystem() {
    return intakeRotationSubsystem;
  }

  public IntakeSpinningSubsystem getIntakeSpinningSubsystem() {
    return intakeSpinningSubsystem;
  }

  public Command setDownAndRunCommand() {
    return Commands.parallel(
            intakeSpinningSubsystem.setVoltageCommand(IntakeConstants.SPINNING_VOLTAGE),
            intakeRotationSubsystem.setRotationGoalCommand(
                Rotation2d.fromRadians(IntakeConstants.ROTATION_DOWN_ANGLE)))
        .withName("IntakeSetDownAndRun");
  }

  public Command setUpCommand() {
    return Commands.parallel(
            intakeSpinningSubsystem.setVoltageCommand(0.0),
            intakeRotationSubsystem.setRotationGoalCommand(
                Rotation2d.fromRadians(IntakeConstants.ROTATION_UP_ANGLE)))
        .withName("IntakeSetUp");
  }
}
