package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ElevatorWristCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.hid.CommandNintendoSwitchController;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Intake.IntakeRotationSubsystem;
import frc.robot.subsystems.Intake.IntakeSpinningSubsystem;
import frc.robot.subsystems.Intake.IntakeSuperstructure;

public class RobotContainer {

  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final WristSubsystem wristSubsystem = new WristSubsystem();
  private final IntakeRotationSubsystem intakeRotationSubsystem = new IntakeRotationSubsystem();
  private final IntakeSpinningSubsystem intakeSpinningSubsystem = new IntakeSpinningSubsystem();
  private final IntakeSuperstructure intakeSuperstructure =
      new IntakeSuperstructure(intakeSpinningSubsystem, intakeRotationSubsystem);
  private final CoralSubsystem coralSubsystem = new CoralSubsystem();

  private final AlgaeSubsystem algaeSubsystem = new AlgaeSubsystem();
  // private final NintendoSwitchController joystick = new NintendoSwitchController(0);
  private final CommandNintendoSwitchController joystick = new CommandNintendoSwitchController(0);
  private final CommandPS4Controller operator = new CommandPS4Controller(1);
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public RobotContainer() {
    configureBindings();
    configureOperatorBindings();
  }

  private void configureOperatorBindings() {
    operator
        .povDown()
        .onTrue(ElevatorWristCommands.elevatorWristL2(elevatorSubsystem, wristSubsystem));
    operator
        .povRight()
        .onTrue(ElevatorWristCommands.elevatorWristL3(elevatorSubsystem, wristSubsystem));
    operator
        .povUp()
        .onTrue(ElevatorWristCommands.elevatorWristL4(elevatorSubsystem, wristSubsystem));
    operator
        .povLeft()
        .onTrue(ElevatorWristCommands.elevatorWristL1(elevatorSubsystem, wristSubsystem));
    operator
        .cross()
        .onTrue(ElevatorWristCommands.elevatorWristProcessor(elevatorSubsystem, wristSubsystem));
    operator
        .circle()
        .onTrue(ElevatorWristCommands.elevatorWristBallLow(elevatorSubsystem, wristSubsystem));
    operator
        .square()
        .onTrue(ElevatorWristCommands.elevatorWristNet(elevatorSubsystem, wristSubsystem));
    operator
        .R2()
        .onTrue(ElevatorWristCommands.elevatorWristHuman(elevatorSubsystem, wristSubsystem));
    operator
        .L2()
        .onTrue(ElevatorWristCommands.elevatorWristReset(elevatorSubsystem, wristSubsystem));
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        -joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        -joystick.getRightX()
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    joystick.leftStick().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.leftStick().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick
        .leftStick()
        .and(joystick.y())
        .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick
        .leftStick()
        .and(joystick.x())
        .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    joystick.home().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    joystick
        .leftTrigger()
        .whileTrue(coralSubsystem.setVoltageCommand(Constants.CoralConstants.OUTPUT_VOLTAGE));

    joystick
        .rightTrigger()
        .whileTrue(
            Commands.parallel(
                    coralSubsystem.setVoltageCommand(Constants.CoralConstants.RUNNING_VOLTAGE),
                    algaeSubsystem.setVoltageCommand(Constants.AlgaeConstants.RUNNING_VOLTAGE))
                .until(() -> coralSubsystem.getSwitchState() || algaeSubsystem.getSwitchState()));
    joystick
        .leftBumper()
        .whileTrue(
            Commands.parallel(
                    ElevatorWristCommands.elevatorWristGroundIntake(
                        elevatorSubsystem, wristSubsystem),
                    intakeSuperstructure.setDownAndRunCommand(),
                    coralSubsystem.setVoltageCommand(Constants.CoralConstants.RUNNING_VOLTAGE))
                .until(coralSubsystem::getSwitchState));
    joystick
        .a()
        .whileTrue(algaeSubsystem.setVoltageCommand(Constants.AlgaeConstants.OUTPUT_VOLTAGE));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
