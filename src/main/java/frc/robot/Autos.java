// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.MiscConstants;
import frc.robot.commands.autoCommands.ToPointCommand;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Intake.IntakeSpinningSubsystem;
import frc.robot.subsystems.Intake.IntakeSuperstructure;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.utils.RaiderUtils;

public class Autos {
  /** Example static factory for an autonomous command. */
  private final SendableChooser<Command> autoChooser;

  public Autos(
      IntakeSpinningSubsystem intakeSpinningSubsystem,
      IntakeSuperstructure intakeSuperstructure,
      AlgaeSubsystem algaeSubsystem,
      ClimberSubsystem climberSubsystem,
      CommandSwerveDrivetrain drivetrain,
      CoralSubsystem coralSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      WristSubsystem wristSubsystem) {
    
    RobotConfig config = Robot.isSimulation()? 
      ChassisConstants.PP_CONFIG:
      null;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    AutoBuilder.configure(
        drivetrain::getPose,
        drivetrain::resetPose,
        drivetrain::getSpeeds,
        (ChassisSpeeds, FF) ->
            drivetrain.setControl(
                new SwerveRequest.RobotCentric()
                    .withVelocityX(ChassisSpeeds.vxMetersPerSecond)
                    .withVelocityY(ChassisSpeeds.vyMetersPerSecond)),
        new PPHolonomicDriveController(
            Constants.AutoConstants.pointTranslationGains.createPIDConstants(),
            Constants.AutoConstants.ROTATION_PID_GAINS),
        config,
        RaiderUtils::shouldFlip,
        drivetrain);

    //    NamedCommands.registerCommand("scoreL4");

    autoChooser = AutoBuilder.buildAutoChooser("JustProbe");
    if (MiscConstants.TUNING_MODE) {
      // autoChooser.addOption("elevator qf", elevatorSubsystem.sysIdQuasistatic(Direction.kForward));
      // autoChooser.addOption("elevator qr", elevatorSubsystem.sysIdQuasistatic(Direction.kReverse));
      // autoChooser.addOption("elevator df", elevatorSubsystem.sysIdDynamic(Direction.kForward));
      // autoChooser.addOption("elevator dr", elevatorSubsystem.sysIdDynamic(Direction.kReverse));

      // autoChooser.addOption("wrist qf", wristSubsystem.sysIdQuasistatic(Direction.kForward));
      // autoChooser.addOption("wrist qr", wristSubsystem.sysIdQuasistatic(Direction.kReverse));
      // autoChooser.addOption("wrist df", wristSubsystem.sysIdDynamic(Direction.kForward));
      // autoChooser.addOption("wrist dr", wristSubsystem.sysIdDynamic(Direction.kReverse));

      autoChooser.addOption(
          "intake rotation qf",
          intakeSuperstructure.getIntakeRotationSubsystem().sysIdQuasistatic(Direction.kForward));
      autoChooser.addOption(
          "intake rotation qr",
          intakeSuperstructure.getIntakeRotationSubsystem().sysIdQuasistatic(Direction.kReverse));
      autoChooser.addOption(
          "intake rotation df",
          intakeSuperstructure.getIntakeRotationSubsystem().sysIdDynamic(Direction.kForward));
      autoChooser.addOption(
          "intake rotation dr",
          intakeSuperstructure.getIntakeRotationSubsystem().sysIdDynamic(Direction.kReverse));

      // autoChooser.addOption(
      //     "intakeSpinning qf", intakeSpinningSubsystem.sysIDQuasistatic(Direction.kForward));
      // autoChooser.addOption(
      //     "intakeSpinning qr", intakeSpinningSubsystem.sysIDQuasistatic(Direction.kReverse));
      // autoChooser.addOption(
      //     "intakeSpinning df", intakeSpinningSubsystem.sysIDDynamic(Direction.kForward));
      // autoChooser.addOption(
      //     "intakeSpinning dr", intakeSpinningSubsystem.sysIDDynamic(Direction.kReverse));

      // autoChooser.addOption("algae qf", algaeSubsystem.sysIdQuasistatic(Direction.kForward));
      // autoChooser.addOption("algae qr", algaeSubsystem.sysIdQuasistatic(Direction.kReverse));
      // autoChooser.addOption("algae df", algaeSubsystem.sysIdDynamic(Direction.kForward));
      // autoChooser.addOption("algae dr", algaeSubsystem.sysIdDynamic(Direction.kReverse));

      // autoChooser.addOption("climber qf", climberSubsystem.sysIDQuasistatic(Direction.kForward));
      // autoChooser.addOption("climber qr", climberSubsystem.sysIDQuasistatic(Direction.kReverse));
      // autoChooser.addOption("climber df", climberSubsystem.sysIDDynamic(Direction.kForward));
      // autoChooser.addOption("climber dr", climberSubsystem.sysIDDynamic(Direction.kReverse));

      // autoChooser.addOption("coral qf", coralSubsystem.sysIDQuasistatic(Direction.kForward));
      // autoChooser.addOption("coral qr", coralSubsystem.sysIDQuasistatic(Direction.kReverse));
      // autoChooser.addOption("coral df", coralSubsystem.sysIDDynamic(Direction.kForward));
      // autoChooser.addOption("coral dr", coralSubsystem.sysIDDynamic(Direction.kReverse));

      // autoChooser.addOption("drive qf", drivetrain.sysIdQuasistatic(Direction.kForward));
      // autoChooser.addOption("drive qr", drivetrain.sysIdQuasistatic(Direction.kReverse));
      // autoChooser.addOption("drive df", drivetrain.sysIdDynamic(Direction.kForward));
      // autoChooser.addOption("drive dr", drivetrain.sysIdDynamic(Direction.kReverse));
      autoChooser.addOption("wrist", wristSubsystem.setPositionCommand(Units.degreesToRadians(240)));
      autoChooser.addOption("elevator", elevatorSubsystem.setPosition(Units.inchesToMeters(10)));
      autoChooser.addOption("slapdown", intakeSuperstructure.getIntakeRotationSubsystem().setRotationGoalCommand(new Rotation2d(Units.degreesToRadians(Constants.IntakeConstants.ROTATION_DOWN_ANGLE))));
      autoChooser.addOption("coral 10v", coralSubsystem.setVoltageCommand(10));
      autoChooser.addOption("intakeRun", intakeSpinningSubsystem.setVoltageCommand(-Constants.IntakeConstants.SPINNING_VOLTAGE));
    }
  }

  public SendableChooser<Command> getAutoChooser() {
    return autoChooser;
  }

  public static Command detectAndMoveTarget(VisionSubsystem vision, CommandSwerveDrivetrain drive) {
    return new ToPointCommand(drive, () -> vision.getTargetTrajectory());
  }

  public Command autoStart(
      ElevatorSubsystem elevatorSubsystem, IntakeSuperstructure intakeSuperstructure) {
    if (Robot.isSimulation()) {
      return Commands.print("Probed!");
    }
    return Commands.parallel(
            elevatorSubsystem.homeElevatorCommand(),
            intakeSuperstructure.getIntakeRotationSubsystem().homeIntakeCommand())
        .withName("AutoStart");
  }
}
