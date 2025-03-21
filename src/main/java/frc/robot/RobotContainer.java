package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentricFacingAngle;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ElevatorWristCommands;
import frc.robot.commands.MiscCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.hid.CommandButtonBoard;
import frc.robot.hid.CommandNintendoSwitchController;
import frc.robot.hid.CommandXboxPlaystationController;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Intake.IntakeRotationSubsystem;
import frc.robot.subsystems.Intake.IntakeSpinningSubsystem;
import frc.robot.subsystems.Intake.IntakeSuperstructure;
import frc.robot.utils.*;

import java.util.concurrent.atomic.AtomicBoolean;

// @Logged
public class RobotContainer {

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75)
            .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(
                    DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric centricdrive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.RobotCentricFacingAngle robotCentricFacingAngle = new SwerveRequest.RobotCentricFacingAngle();
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final VectorRateLimiter vectorRateLimiter = new VectorRateLimiter(
            Constants.MiscConstants.TRANSLATION_RATE_LIMIT);

    private AtomicBoolean onCoral = new AtomicBoolean(true);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public AtomicBoolean scoringFlipped = new AtomicBoolean(false);

    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final WristSubsystem wristSubsystem = new WristSubsystem();
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    private final IntakeRotationSubsystem intakeRotationSubsystem = new IntakeRotationSubsystem();
    private final IntakeSpinningSubsystem intakeSpinningSubsystem = new IntakeSpinningSubsystem();
    private final IntakeSuperstructure intakeSuperstructure = new IntakeSuperstructure(intakeSpinningSubsystem,
            intakeRotationSubsystem);
    private final CoralSubsystem coralSubsystem = new CoralSubsystem();
    private final AlgaeSubsystem algaeSubsystem = new AlgaeSubsystem();
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final VisionSubsystem visionSubsystem = new VisionSubsystem(drivetrain);
    private final Autos autos = new Autos(
            intakeSpinningSubsystem,
            intakeSuperstructure,
            algaeSubsystem,
            climberSubsystem,
            drivetrain,
            coralSubsystem,
            elevatorSubsystem,
            wristSubsystem,
            scoringFlipped,
            visionSubsystem);

    private final CommandNintendoSwitchController joystick = new CommandNintendoSwitchController(0);
    private final CommandXboxPlaystationController operator = new CommandXboxPlaystationController(1);
    private final CommandButtonBoard buttonBoard = new CommandButtonBoard(Constants.OperatorConstants.BUTTON_BOARD_ID);
    final Orchestra orchestra = new Orchestra();
    SendableChooser<Command> musicChooser;
    SendableChooser<Command> playMusic;

    public RobotContainer() {
        configureBindings();
        configureOperatorBindings();
        configureBoard();
        // intakeRotationSubsystem.addInstrumentCommand(orchestra);
        // coralSubsystem.addInstrumentCommand(orchestra);
        // elevatorSubsystem.addInstrumentCommand(orchestra);
        // wristSubsystem.addInstrumentCommand(orchestra);
        // for (int i =0; i<4; i++){
        // TalonFX driveMotor = drivetrain.getModule(i).getDriveMotor();
        // TalonFX steerMotor = drivetrain.getModule(i).getSteerMotor();
        // orchestra.addInstrument(steerMotor);
        // orchestra.addInstrument(driveMotor);
        // }

        // musicChooser.addOption("NationalAnthem", Commands.run(()->
        // orchestra.loadMusic("NationAnthem.chrp")));
        // musicChooser.addOption("RockafellerSkank", Commands.run(()->
        // orchestra.loadMusic("RockafellerSkank.chrp")));
        // musicChooser.addOption("JigsawsFallingIntoPlace", Commands.run(()->
        // orchestra.loadMusic("JigsawsFallingIntoPlace.chrp")));
        // musicChooser.addOption("Sandstorm", Commands.run(()->
        // orchestra.loadMusic("Sandstorm.chrp")));
        // playMusic.addOption("on", Commands.run(()-> orchestra.play()));
        // playMusic.addOption("off", Commands.run(()-> orchestra.stop()));

        SmartDashboard.putData("Auto", autos.getAutoChooser());
        SmartDashboard.putData("Alerts", Alert.getDefaultGroup());
        SmartDashboard.putNumber("MatchTime", DriverStation.getMatchTime());
        // SmartDashboard.putData("MusicChooser", getMusiChooser());
        // SmartDashboard.putData("MusicOn/Off", getMusicOn());
    }

    public SendableChooser<Command> getMusiChooser() {
        return musicChooser;
    }

    public SendableChooser<Command> getMusicOn() {
        return playMusic;
    }

    private void configureOperatorBindings() {
        operator.triangle().onTrue(
                ElevatorWristCommands.elevatorWristBallHigh(elevatorSubsystem, wristSubsystem, scoringFlipped));
        operator
                .povDown()
                .onTrue(Commands.parallel(
                        ElevatorWristCommands.elevatorWristL2(
                                elevatorSubsystem, wristSubsystem, scoringFlipped)
                , intakeSuperstructure.setDownAndRunCommand().withInterruptBehavior(InterruptionBehavior.kCancelSelf)));
        operator
                .povRight()
                .onTrue(
                        ElevatorWristCommands.elevatorWristL3(
                                elevatorSubsystem, wristSubsystem, scoringFlipped)
                               );
        operator
                .povUp()
                .onTrue(
                        ElevatorWristCommands.elevatorWristL4(
                                elevatorSubsystem, wristSubsystem, scoringFlipped)
                                );
        operator
                .povLeft()
                .onTrue(
                        ElevatorWristCommands.elevatorWristL1(
                                elevatorSubsystem, wristSubsystem, scoringFlipped)
                                
                                        );
        operator
                .x()
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
                .rightTrigger()
                .onTrue(
                        ElevatorWristCommands.elevatorWristHuman(
                                elevatorSubsystem, wristSubsystem, scoringFlipped));
        operator
                .leftTrigger()
                .onTrue(ElevatorWristCommands.elevatorWristReset(elevatorSubsystem, wristSubsystem));
        operator.leftBumper().whileTrue(MiscCommands.ClimberUpCommand(climberSubsystem));
        operator.rightBumper().whileTrue(MiscCommands.ClimberDownCommand(climberSubsystem));
        operator.options().whileTrue(intakeRotationSubsystem.homeIntakeCommand());
        operator.share().whileTrue(elevatorSubsystem.homeElevatorCommand());
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
                                onCoral.get() ? "Blue-2B" : "Blue-2B3C-Algae",
                                scoringFlipped));
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
                                onCoral.get() ? "Blue-5E" : "Blue-4D5E-Algae",
                                scoringFlipped));
        buttonBoard
                .Button6()
                .whileTrue(
                        drivetrain.autoDriveTrajectory(
                                onCoral.get() ? "Blue-6F" : "Blue-6F7G-Algae",
                                scoringFlipped));
        buttonBoard
                .Button7()
                .whileTrue(
                        drivetrain.autoDriveTrajectory(
                                onCoral.get() ? "Blue-7G" : "Blue-6F7G-Algae",
                                scoringFlipped));
        buttonBoard
                .Button8()
                .whileTrue(
                        drivetrain.autoDriveTrajectory(
                                onCoral.get() ? "Blue-8H" : "Blue-8H9I-Algae",
                                scoringFlipped));
        buttonBoard
                .Button9()
                .whileTrue(
                        drivetrain.autoDriveTrajectory(
                                onCoral.get() ? "Blue-9I" : "Blue-8H9I-Algae",
                                scoringFlipped));
        buttonBoard
                .Button10()
                .whileTrue(
                        drivetrain.autoDriveTrajectory(
                                onCoral.get() ? "Blue-10J" : "Blue-10J11K-Algae",
                                scoringFlipped));
        buttonBoard
                .Button11()
                .whileTrue(
                        drivetrain.autoDriveTrajectory(
                                onCoral.get() ? "Blue-11K" : "Blue-10J11K-Algae",
                                scoringFlipped));
        buttonBoard
                .Button12()
                .whileTrue(
                        drivetrain.autoDriveTrajectory(
                                onCoral.get() ? "Blue-12L" : "Blue-12L1A-Algae",
                                scoringFlipped));
    }

    private void configureBindings() {
        // elevatorSubsystem.setDefaultCommand(elevatorSubsystem.setPosition(Units.inchesToMeters(3.15)));
        intakeRotationSubsystem.setDefaultCommand(intakeRotationSubsystem.setVoltageCommand(0));
        elevatorSubsystem.setDefaultCommand(elevatorSubsystem.setVoltageCommand(0.0));
        drivetrain.setDefaultCommand(

                drivetrain.applyRequest(
                        () -> {
                            Translation2d translation = vectorRateLimiter.calculate(new Translation2d(
                                    RaiderMathUtils.deadZoneAndCubeJoystick(-joystick.getLeftY()) * MaxSpeed,
                                    RaiderMathUtils.deadZoneAndCubeJoystick(-joystick.getLeftX()) * MaxSpeed));
                            return drive
                                    .withVelocityX(
                                            translation.getX()) // Drive forward with negative Y (forward)
                                    .withVelocityY(
                                            translation.getY()) // Drive left with negative X (left)
                                    .withRotationalRate(
                                            -joystick.getRightX()
                                                    * MaxAngularRate);
                        } // Drive counterclockwise with negative X (left)
                ));
        joystick
                .rightBumper()
                .whileTrue(
                        Commands.run(() -> drivetrain.setControl(
                                centricdrive
                                        .withVelocityX(-joystick.getLeftY() * MaxSpeed)
                                        .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate)),
                                drivetrain));

        joystick.b().onTrue(Commands.runOnce(() -> onCoral.set(!onCoral.get())));
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
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
                .toggleOnTrue(
                        algaeSubsystem.setVoltageCommand(Constants.AlgaeConstants.RUNNING_VOLTAGE));

        // joystick
        // .leftBumper()
        // .whileTrue(
        // Commands.parallel(
        // elevatorSubsystem.setPosition(Units.inchesToMeters(3.15)),
        // wristSubsystem.setPositionCommand(0))
        // .until(() -> elevatorSubsystem.atGoal() && wristSubsystem.atGoal()).andThen(
        // intakeSuperstructure
        // .setDownAndRunCommand())
        // .until(intakeSpinningSubsystem::getSwitchValue).andThen(
        // intakeSuperstructure.setUpCommand())
        // .until(intakeRotationSubsystem::atLimit).andThen(
        // Commands.sequence(
        // ElevatorWristCommands.elevatorWristGroundIntake(
        // elevatorSubsystem, wristSubsystem)
        // .until(() -> elevatorSubsystem.atGoal()
        // && wristSubsystem.atGoal()),
        // intakeSpinningSubsystem
        // .setVoltageCommand(-Constants.IntakeConstants.SPINNING_VOLTAGE))
        // .until(coralSubsystem::getLeftSwitchState)));
        joystick.leftBumper().whileTrue(
                Commands.sequence(
                        Commands.race(
                                ElevatorWristCommands.preGroundIntake(elevatorSubsystem, wristSubsystem),
                                Commands.sequence(
                                        Commands.waitUntil(() -> elevatorSubsystem.atGoal() && wristSubsystem.atGoal()),
                                        intakeSuperstructure.setDownAndRunCommand()
                                                .until(intakeSpinningSubsystem::getSwitchValue),
                                        intakeSuperstructure.setUpCommand().until(intakeRotationSubsystem::atLimit))),

                        Commands.race(
                                ElevatorWristCommands.elevatorWristGroundIntake(elevatorSubsystem, wristSubsystem),
                                Commands.sequence(Commands
                                        .waitUntil(() -> elevatorSubsystem.getElevatorPosition() < Units
                                                .inchesToMeters(.1)),
                                        Commands.parallel(intakeSpinningSubsystem
                                                .setVoltageCommand(-Constants.IntakeConstants.SPINNING_VOLTAGE_OUTAKE))
                                                ))))                .onFalse(intakeSuperstructure.setUpCommand());
        joystick.y().whileTrue(Commands.run(() -> {
            Translation2d translation = vectorRateLimiter.calculate(new Translation2d(
                    RaiderMathUtils.deadZoneAndCubeJoystick(-joystick.getLeftY()) * MaxSpeed,
                    RaiderMathUtils.deadZoneAndCubeJoystick(-joystick.getLeftX()) * MaxSpeed));
            drivetrain.setControl(robotCentricFacingAngle
                    .withVelocityX(
                            translation.getX()) // Drive forward with negative Y (forward)
                    .withVelocityY(
                            translation.getY())
                    .withTargetDirection(Rotation2d.fromDegrees(drivetrain.getPose().getRotation().getDegrees()
                            + visionSubsystem.getTargetHorizontalOffset().getAsDouble())) // Drive left with negative X
                                                                                          // (left)
            );
        }));
        joystick
                .a()
                .whileTrue(algaeSubsystem.setVoltageCommand(Constants.AlgaeConstants.OUTPUT_VOLTAGE));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autos.getAutoChooser().getSelected();
    }
}
