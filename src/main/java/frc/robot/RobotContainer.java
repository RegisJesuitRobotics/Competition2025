package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentricFacingAngle;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.hid.CommandButtonBoard;
import frc.robot.hid.CommandNintendoSwitchController;
import frc.robot.hid.CommandXboxPlaystationController;
import frc.robot.subsystems.*;
import frc.robot.utils.*;

import java.util.concurrent.atomic.AtomicBoolean;

// @Logged
public class RobotContainer {

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75)
            .in(RadiansPerSecond) ; // 3/4 of a rotation per second max angular velocit

    
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.03)
            .withRotationalDeadband(MaxAngularRate * 0.03) // Add a 10% deadband
            .withDriveRequestType(
                    DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric centricdrive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.03)
            .withRotationalDeadband(MaxAngularRate * 0.03)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.RobotCentricFacingAngle robotCentricFacingAngle = new SwerveRequest.RobotCentricFacingAngle()
    .withDeadband(MaxSpeed * 0.2)
    .withRotationalDeadband(MaxAngularRate * 0.03)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final VectorRateLimiter vectorRateLimiter = new VectorRateLimiter(
            Constants.MiscConstants.TRANSLATION_RATE_LIMIT);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final CoralSubsystem coralSubsystem = new CoralSubsystem();
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final VisionSubsystem visionSubsystem = new VisionSubsystem(drivetrain);
    private final Autos autos = new Autos(
            drivetrain,
            coralSubsystem,
            elevatorSubsystem,
            visionSubsystem);

    private final CommandNintendoSwitchController joystick = new CommandNintendoSwitchController(0);
    private final CommandXboxPlaystationController operator = new CommandXboxPlaystationController(1);

    public RobotContainer() {
        configureBindings();
        configureOperatorBindings();

        SmartDashboard.putData("Auto", autos.getAutoChooser());
        SmartDashboard.putData("Alerts", Alert.getDefaultGroup());
        SmartDashboard.putNumber("MatchTime", DriverStation.getMatchTime());

    }


    private void configureOperatorBindings() {
                 
      //  operator.povUp().onTrue(elevatorSubsystem.setPosition(ElevatorConstants.L4_REEF));
        operator.povUp().onTrue(elevatorSubsystem.setPosition(ElevatorConstants.L3_REEF));
        operator.povDown().onTrue(elevatorSubsystem.setPosition(ElevatorConstants.L1_REEF));
        operator.povRight().onTrue(elevatorSubsystem.setPosition(ElevatorConstants.L2_REEF + 0.001));
        operator.leftTrigger().onTrue(elevatorSubsystem.setPosition(ElevatorConstants.INTAKE_POSITION));
        operator.rightTrigger().whileTrue(coralSubsystem.intakeUntilDetected());
        operator.rightBumper().whileTrue(coralSubsystem.setVoltageCommand(3));
        operator.leftBumper().whileTrue(coralSubsystem.setVoltageCommand(-3));

        operator.options().whileTrue(elevatorSubsystem.setVoltageCommand(-2));
        operator.share().whileTrue(elevatorSubsystem.setVoltageCommand(2));
    }

    private void configureBindings() {
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

        joystick.leftBumper().whileTrue(
                Commands.run(() -> drivetrain.setControl(
                        robotCentricFacingAngle
                                .withVelocityX(-joystick.getLeftY() * MaxSpeed)
                                .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                                .withTargetDirection(new Rotation2d(Units.degreesToRadians(180)))),
                        drivetrain));
  
   
           
        
        joystick.home().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        joystick.plus().and(joystick.leftBumper()).whileTrue(drivetrain.ToPointCommand(drivetrain.returnAutoAlignPose(0)));
        joystick.plus().and(joystick.rightBumper()).whileTrue(drivetrain.ToPointCommand(drivetrain.returnAutoAlignPose(1)));

        joystick.rightTrigger().whileTrue(coralSubsystem.intakeUntilDetected());

        joystick
                .leftTrigger()
                .whileTrue(coralSubsystem.setVoltageCommand(Constants.CoralConstants.OUTPUT_VOLTAGE));

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
        
        //outake backwards in case stuck
        joystick.minus().whileTrue(coralSubsystem.setVoltageCommand(-5));


        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autos.getAutoChooser().getSelected();
    }
}