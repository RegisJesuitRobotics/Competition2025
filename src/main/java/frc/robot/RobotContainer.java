package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ElevatorWristCommands;
import frc.robot.commands.MiscCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.hid.CommandButtonBoard;
import frc.robot.hid.CommandNintendoSwitchController;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Intake.IntakeRotationSubsystem;
import frc.robot.subsystems.Intake.IntakeSpinningSubsystem;
import frc.robot.subsystems.Intake.IntakeSuperstructure;
import frc.robot.utils.Reef;
import java.util.concurrent.atomic.AtomicBoolean;

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
                private final SwerveRequest.RobotCentric centricdrive =
      new SwerveRequest.RobotCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private AtomicBoolean onCoral = new AtomicBoolean(true);

  private final Telemetry logger = new Telemetry(MaxSpeed);

  public AtomicBoolean scoringFlipped = new AtomicBoolean(false);

  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final WristSubsystem wristSubsystem = new WristSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final IntakeRotationSubsystem intakeRotationSubsystem = new IntakeRotationSubsystem();
  private final IntakeSpinningSubsystem intakeSpinningSubsystem = new IntakeSpinningSubsystem();
  private final IntakeSuperstructure intakeSuperstructure =
      new IntakeSuperstructure(intakeSpinningSubsystem, intakeRotationSubsystem);
  private final CoralSubsystem coralSubsystem = new CoralSubsystem();
  private final AlgaeSubsystem algaeSubsystem = new AlgaeSubsystem();
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final Autos autos = new Autos(
    intakeSpinningSubsystem,intakeSuperstructure,algaeSubsystem,climberSubsystem,
  drivetrain,coralSubsystem,elevatorSubsystem,wristSubsystem);
  
  private final CommandNintendoSwitchController joystick = new CommandNintendoSwitchController(0);
  private final CommandPS4Controller operator = new CommandPS4Controller(1);
  private final CommandButtonBoard buttonBoard =
      new CommandButtonBoard(Constants.OperatorConstants.BUTTON_BOARD_ID);

  public RobotContainer() {
    configureBindings();
    configureOperatorBindings();
    configureBoard();

     SmartDashboard.putData("Auto", autos.getAutoChooser());
  }

  private void configureOperatorBindings() {
    operator
        .povDown()
        .onTrue(
            ElevatorWristCommands.elevatorWristL2(
                elevatorSubsystem, wristSubsystem, scoringFlipped));
    operator
        .povRight()
        .onTrue(
            ElevatorWristCommands.elevatorWristL3(
                elevatorSubsystem, wristSubsystem, scoringFlipped));
    operator
        .povUp()
        .onTrue(
            ElevatorWristCommands.elevatorWristL4(
                elevatorSubsystem, wristSubsystem, scoringFlipped));
    operator
        .povLeft()
        .onTrue(
            ElevatorWristCommands.elevatorWristL1(
                elevatorSubsystem, wristSubsystem, scoringFlipped));
    operator
        .cross()
        .onTrue(
            ElevatorWristCommands.elevatorWristProcessor(
                elevatorSubsystem, wristSubsystem, scoringFlipped));
    operator
        .circle()
        .onTrue(
            ElevatorWristCommands.elevatorWristBallLow(
                elevatorSubsystem, wristSubsystem, scoringFlipped));
    operator
        .square()
        .onTrue(
            ElevatorWristCommands.elevatorWristNet(
                elevatorSubsystem, wristSubsystem, scoringFlipped));
    operator
        .R2()
        .onTrue(
            ElevatorWristCommands.elevatorWristHuman(
                elevatorSubsystem, wristSubsystem, scoringFlipped));
    operator
        .L2()
        .onTrue(ElevatorWristCommands.elevatorWristReset(elevatorSubsystem, wristSubsystem));
    operator.L1().whileTrue(MiscCommands.ClimberUpCommand(climberSubsystem));
    operator.R1().whileTrue(MiscCommands.ClimberDownCommand(climberSubsystem));
    operator.options().whileTrue(intakeRotationSubsystem.homeIntakeCommand());
    operator.share().whileTrue(elevatorSubsystem.homeElevatorCommand());
  }

  private void configureBoard() {

    buttonBoard
        .Button1()
        .whileTrue(
            drivetrain.autoDriveTrajectory(
                onCoral.get() ? Reef.MidUpCoralLeft.value : Reef.MidUpAlgae.value, scoringFlipped));
    buttonBoard
        .Button2()
        .whileTrue(
            drivetrain.autoDriveTrajectory(
                onCoral.get() ? Reef.MidUpCoralRight.value : Reef.MidUpAlgae.value,
                scoringFlipped));
    buttonBoard
        .Button3()
        .whileTrue(
            drivetrain.autoDriveTrajectory(
                onCoral.get() ? Reef.MidCoralLeft.value : Reef.MidAlgae.value, scoringFlipped));
    buttonBoard
        .Button4()
        .whileTrue(
            drivetrain.autoDriveTrajectory(
                onCoral.get() ? Reef.MidCoralRight.value : Reef.MidAlgae.value, scoringFlipped));
    buttonBoard
        .Button5()
        .whileTrue(
            drivetrain.autoDriveTrajectory(
                onCoral.get() ? Reef.MidDownCoralLeft.value : Reef.MidDownAlgae.value,
                scoringFlipped));
    buttonBoard
        .Button6()
        .whileTrue(
            drivetrain.autoDriveTrajectory(
                onCoral.get() ? Reef.MidDownCoralRight.value : Reef.MidDownAlgae.value,
                scoringFlipped));
    buttonBoard
        .Button7()
        .whileTrue(
            drivetrain.autoDriveTrajectory(
                onCoral.get() ? Reef.StationDownCoralRight.value : Reef.StationDownAlgae.value,
                scoringFlipped));
    buttonBoard
        .Button8()
        .whileTrue(
            drivetrain.autoDriveTrajectory(
                onCoral.get() ? Reef.StationDownCoralLeft.value : Reef.StationDownAlgae.value,
                scoringFlipped));
    buttonBoard
        .Button9()
        .whileTrue(
            drivetrain.autoDriveTrajectory(
                onCoral.get() ? Reef.StationCoralRight.value : Reef.StationAlgae.value,
                scoringFlipped));
    buttonBoard
        .Button10()
        .whileTrue(
            drivetrain.autoDriveTrajectory(
                onCoral.get() ? Reef.StationCoralLeft.value : Reef.StationAlgae.value,
                scoringFlipped));
    buttonBoard
        .Button11()
        .whileTrue(
            drivetrain.autoDriveTrajectory(
                onCoral.get() ? Reef.StationUpCoralRight.value : Reef.StationUpAlgae.value,
                scoringFlipped));
    buttonBoard
        .Button12()
        .whileTrue(
            drivetrain.autoDriveTrajectory(
                onCoral.get() ? Reef.StationUpCoralLeft.value : Reef.StationUpAlgae.value,
                scoringFlipped));
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
    joystick.rightBumper().whileTrue(
        drivetrain.applyRequest(
        () -> 
        centricdrive
            .withVelocityX(-joystick.getLeftY() * MaxSpeed)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate)));

    joystick.b().onTrue(Commands.runOnce(() -> onCoral.set(!onCoral.get())));
    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    /*joystick
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));
*/
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
                .until(
                    () ->
                        (coralSubsystem.getRightSwitchState()) || algaeSubsystem.getSwitchState()));
    joystick
        .leftBumper()
        .whileTrue(
            Commands.sequence(
                intakeSuperstructure
                    .setDownAndRunCommand()
                    .until(intakeSpinningSubsystem::getSwitchValue),
                intakeSuperstructure.setUpCommand(),
                Commands.parallel(
                    coralSubsystem.runVelolocityCenterCommand(
                        Constants.CoralConstants.RUNNING_VOLTAGE),
                    ElevatorWristCommands.elevatorWristGroundIntake(
                        elevatorSubsystem, wristSubsystem))));

    joystick
        .a()
        .whileTrue(algaeSubsystem.setVoltageCommand(Constants.AlgaeConstants.OUTPUT_VOLTAGE));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return autos.getAutoChooser().getSelected();
  }
}
