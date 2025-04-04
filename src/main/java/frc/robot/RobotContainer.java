package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.MiscCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.hid.CommandButtonBoard;
import frc.robot.hid.CommandNintendoSwitchController;
import frc.robot.hid.CommandXboxPlaystationController;
import frc.robot.subsystems.*;
import frc.robot.utils.*;
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
          .withDeadband(MaxSpeed * 0.03)
          .withRotationalDeadband(MaxAngularRate * 0.03) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.RobotCentric centricdrive =
      new SwerveRequest.RobotCentric()
          .withDeadband(MaxSpeed * 0.03)
          .withRotationalDeadband(MaxAngularRate * 0.03)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.FieldCentricFacingAngle robotCentricFacingAngle =
      new SwerveRequest.FieldCentricFacingAngle();
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final VectorRateLimiter vectorRateLimiter =
      new VectorRateLimiter(Constants.MiscConstants.TRANSLATION_RATE_LIMIT);

  private AtomicBoolean onCoral = new AtomicBoolean(true);

  private final Telemetry logger = new Telemetry(MaxSpeed);

  public AtomicBoolean scoringFlipped = new AtomicBoolean(false);

  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final WristSubsystem wristSubsystem = new WristSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final CoralSubsystem coralSubsystem = new CoralSubsystem();
  private final AlgaeSubsystem algaeSubsystem = new AlgaeSubsystem();
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem(drivetrain);
  private final Autos autos =
      new Autos(
          algaeSubsystem,
          climberSubsystem,
          drivetrain,
          coralSubsystem,
          elevatorSubsystem,
          wristSubsystem,
          scoringFlipped,
          visionSubsystem);

  public final CommandNintendoSwitchController joystick = new CommandNintendoSwitchController(0);
  public final CommandXboxPlaystationController operator = new CommandXboxPlaystationController(1);
  private final CommandButtonBoard buttonBoard = new CommandButtonBoard(Constants.OperatorConstants.BUTTON_BOARD_ID);

  public RobotContainer() {
    configureBindings();
    configureOperatorBindings();
    configureBoard();
    elevatorSubsystem.setDefaultCommand(elevatorSubsystem.setVoltageCommand(0.0));
    coralSubsystem.setDefaultCommand(coralSubsystem.setVoltageCommand(0.0));
    SmartDashboard.putData("Auto", autos.getAutoChooser());
    SmartDashboard.putData("Alerts", Alert.getDefaultGroup());
    SmartDashboard.putNumber("MatchTime", DriverStation.getMatchTime());
    SmartDashboard.putData("commands", CommandScheduler.getInstance());
   

  }


  private void configureOperatorBindings() {
      operator.povUp().onTrue(elevatorSubsystem.setPosition(Constants.ElevatorConstants.L4_REEF));
      operator.povRight().onTrue(elevatorSubsystem.setPosition(Constants.ElevatorConstants.L3_REEF));
      operator.povLeft().onTrue(elevatorSubsystem.setPosition(Constants.ElevatorConstants.L1_REEF));
      operator.povDown().onTrue(elevatorSubsystem.setPosition(Constants.ElevatorConstants.L2_REEF));
      operator.leftTrigger().onTrue(elevatorSubsystem.setPosition(0.0));
      operator.circle().onTrue(elevatorSubsystem.setPosition(Constants.ElevatorConstants.LOW_BALL_PICKUP));
      operator.triangle().onTrue(elevatorSubsystem.setPosition(Constants.ElevatorConstants.HIGH_BALL_PICKUP));
      operator.rightBumper().whileTrue(MiscCommands.ClimberDownCommand(climberSubsystem));
      operator.leftBumper().whileTrue(MiscCommands.ClimberUpCommand(climberSubsystem));
      operator.options().whileTrue(elevatorSubsystem.homeElevatorCommand());
      operator.triangle().onTrue(elevatorSubsystem.setPosition(Constants.ElevatorConstants.HIGH_BALL_PICKUP));
      operator.circle().onTrue(Commands.parallel(elevatorSubsystem.setPosition(Constants.ElevatorConstants.LOW_BALL_PICKUP), wristSubsystem.setPositionCommand(Constants.WristConstants.BALL_PICKUP), coralSubsystem.setVoltageCommand(2.0), algaeSubsystem.setVoltageCommand(2.0)));
      operator.x().onTrue(elevatorSubsystem.setPosition(Constants.ElevatorConstants.NET));

      elevatorSubsystem.setDefaultCommand(elevatorSubsystem.setVoltageCommand(operator.getLeftY() * 6));
      
  }

  private void configureBoard() {

    buttonBoard
        .Button1()
        .whileTrue(
            drivetrain.autoDriveTrajectory(
                onCoral.get() ? "Blue-1A" : "Blue-12L1A-Algae", scoringFlipped));
    buttonBoard
        .Button2()
        .whileTrue(
            drivetrain.autoDriveTrajectory(
                onCoral.get() ? "Blue-2B" : "Blue-2B3C-Algae", scoringFlipped));
    buttonBoard
        .Button3()
        .whileTrue(
            drivetrain.autoDriveTrajectory(
                onCoral.get() ? "Blue-3C" : "Blue-2B3C-Algae", scoringFlipped));
    buttonBoard
        .Button4()
        .whileTrue(
            drivetrain.autoDriveTrajectory(
                onCoral.get() ? "Blue-4D" : "Blue-4D5E-Algae", scoringFlipped));
    buttonBoard
        .Button5()
        .whileTrue(
            drivetrain.autoDriveTrajectory(
                onCoral.get() ? "Blue-5E" : "Blue-4D5E-Algae", scoringFlipped));
    buttonBoard
        .Button6()
        .whileTrue(
            drivetrain.autoDriveTrajectory(
                onCoral.get() ? "Blue-6F" : "Blue-6F7G-Algae", scoringFlipped));
    buttonBoard
        .Button7()
        .whileTrue(
            drivetrain.autoDriveTrajectory(
                onCoral.get() ? "Blue-7G" : "Blue-6F7G-Algae", scoringFlipped));
    buttonBoard
        .Button8()
        .whileTrue(
            drivetrain.autoDriveTrajectory(
                onCoral.get() ? "Blue-8H" : "Blue-8H9I-Algae", scoringFlipped));
    buttonBoard
        .Button9()
        .whileTrue(
            drivetrain.autoDriveTrajectory(
                onCoral.get() ? "Blue-9I" : "Blue-8H9I-Algae", scoringFlipped));
    buttonBoard
        .Button10()
        .whileTrue(
            drivetrain.autoDriveTrajectory(
                onCoral.get() ? "Blue-10J" : "Blue-10J11K-Algae", scoringFlipped));
    buttonBoard
        .Button11()
        .whileTrue(
            drivetrain.autoDriveTrajectory(
                onCoral.get() ? "Blue-11K" : "Blue-10J11K-Algae", scoringFlipped));
    buttonBoard
        .Button12()
        .whileTrue(
            drivetrain.autoDriveTrajectory(
                onCoral.get() ? "Blue-12L" : "Blue-12L1A-Algae", scoringFlipped));
  }

  private void configureBindings() {
    elevatorSubsystem.setDefaultCommand(elevatorSubsystem.setVoltageCommand(0.0));
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () -> {
              Translation2d translation =
                  vectorRateLimiter.calculate(
                      new Translation2d(
                          RaiderMathUtils.deadZoneAndCubeJoystick(-joystick.getLeftY()) * MaxSpeed,
                          RaiderMathUtils.deadZoneAndCubeJoystick(-joystick.getLeftX())
                              * MaxSpeed));
              return drive
                  .withVelocityX(translation.getX()) // Drive forward with negative Y (forward)
                  .withVelocityY(translation.getY()) // Drive left with negative X (left)
                  .withRotationalRate(-joystick.getRightX() * MaxAngularRate);
            } // Drive counterclockwise with negative X (left)
            ));
    joystick
        .rightBumper()
        .whileTrue(
            Commands.run(
                () ->
                    drivetrain.setControl(
                        centricdrive
                            .withVelocityX(-joystick.getLeftY() * MaxSpeed)
                            .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                            .withRotationalRate(-joystick.getRightX() * MaxAngularRate)),
                drivetrain));

    /*
     * joystick
     * .b()
     * .whileTrue(
     * drivetrain.applyRequest(
     * () ->
     * point.withModuleDirection(
     * new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));
     */
    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    joystick
        .leftStick()
        .and(joystick.y())
        .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick
        .leftStick()
        .and(joystick.x())
        .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    joystick
        .leftTrigger()
        .whileTrue(coralSubsystem.setVoltageCommand(Constants.CoralConstants.OUTPUT_VOLTAGE));

    joystick
        .rightTrigger()
        .toggleOnTrue(Commands.parallel(algaeSubsystem.setVoltageCommand(2), coralSubsystem.setVoltageCommand(2.0), wristSubsystem.setPositionCommand(Constants.WristConstants.GROUND_INTAKE).withInterruptBehavior(InterruptionBehavior.kCancelSelf)));

    joystick
        .leftBumper()
        .whileTrue(
            Commands.parallel(Commands.run(
                () -> {
                  Translation2d translation =
                      vectorRateLimiter.calculate(
                          new Translation2d(
                              RaiderMathUtils.deadZoneAndCubeJoystick(-joystick.getLeftY())
                                  * MaxSpeed,
                              RaiderMathUtils.deadZoneAndCubeJoystick(-joystick.getLeftX())
                                  * MaxSpeed));
                  drivetrain.setControl(
                      robotCentricFacingAngle
                          .withVelocityX(
                              translation.getX()) // Drive forward with negative Y (forward)
                          .withVelocityY(translation.getY())
                          .withTargetDirection(
                              Rotation2d.fromDegrees(
                                  RaiderUtils.shouldFlip() ? drivetrain.getPose().getMeasureY().magnitude() < 4 ? 30 + 180 : -30 + 180 : drivetrain.getPose().getMeasureY().magnitude() < 4 ? 30 + 90 : -30 + 90))

                          .withHeadingPID(5, 0, 0) // Drive left with negative X
                      // (left)
                      );
                }), elevatorSubsystem.setPosition(0.0)));
    joystick
        .a()
        .whileTrue(algaeSubsystem.setVoltageCommand(Constants.AlgaeConstants.OUTPUT_VOLTAGE));
        joystick.home().onTrue(Commands.parallel(elevatorSubsystem.setPosition(0.0), wristSubsystem.setPositionCommand(0.0)));

        
    joystick.b().onTrue(coralSubsystem.intakeUntilDetected().andThen(coralSubsystem.setVoltageCommand(Constants.CoralConstants.INTAKE_VOLTAGE).withTimeout(.01)));
    joystick.leftTrigger().whileTrue(coralSubsystem.setVoltageCommand(Constants.CoralConstants.RUNNING_VOLTAGE-2).until(() -> !coralSubsystem.getLeftSwitchState()).andThen(Commands.sequence(Commands.waitSeconds(.1), elevatorSubsystem.setPosition(0.0))));
joystick.x().onTrue(Commands.parallel(wristSubsystem.setPositionCommand(Constants.WristConstants.PROCESSOR), algaeSubsystem.setVoltageCommand(2), coralSubsystem.setVoltageCommand(2.0)));
// joystick.y().onTrue(Commands.parallel(wristSubsystem.setPositionCommand(62), algaeSubsystem.setVoltageCommand(2), coralSubsystem.setVoltageCommand(2.0)));
joystick.y().whileTrue(coralSubsystem.setVoltageCommand(2.0));    
drivetrain.registerTelemetry(logger::telemeterize);
    
  }

  public Command getAutonomousCommand() {
    return autos.getAutoChooser().getSelected();
  }
}
