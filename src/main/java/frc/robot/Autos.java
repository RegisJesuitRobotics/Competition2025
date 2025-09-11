// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.MiscConstants;
import frc.robot.commands.ToPointCommand;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.fasterxml.jackson.databind.Module;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.RaiderUtils;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.RaiderUtils;
import frc.robot.generated.TunerConstants;

import java.util.concurrent.atomic.AtomicBoolean;

public class Autos {
  /** Example static factory for an autonomous command. */
  private final SendableChooser<Command> autoChooser;

  public Autos(
          CommandSwerveDrivetrain drivetrain,
          CoralSubsystem coralSubsystem,
          ElevatorSubsystem elevatorSubsystem,
          VisionSubsystem visionSubsystem) {
    
        autoChooser = AutoBuilder.buildAutoChooser("JustProbe");
    if (MiscConstants.TUNING_MODE) {
      autoChooser.addOption("elevator qf", elevatorSubsystem.sysIdQuasistatic(Direction.kForward));
      autoChooser.addOption("elevator qr", elevatorSubsystem.sysIdQuasistatic(Direction.kReverse));
      autoChooser.addOption("elevator df", elevatorSubsystem.sysIdDynamic(Direction.kForward));
      autoChooser.addOption("elevator dr", elevatorSubsystem.sysIdDynamic(Direction.kReverse));

      // autoChooser.addOption("drive qf", drivetrain.sysIdQuasistatic(Direction.kForward));
      // autoChooser.addOption("drive qr", drivetrain.sysIdQuasistatic(Direction.kReverse));
      // autoChooser.addOption("drive df", drivetrain.sysIdDynamic(Direction.kForward));
      // autoChooser.addOption("drive dr", drivetrain.sysIdDynamic(Direction.kReverse));

      autoChooser.addOption("elevator10", elevatorSubsystem.setPosition(Units.inchesToMeters(10)));
      autoChooser.addOption("elevator 40", elevatorSubsystem.setPosition(Units.inchesToMeters(40)));
      autoChooser.addOption("elevator0", elevatorSubsystem.setPosition(0));
     autoChooser.addOption("coral 10v", coralSubsystem.setVoltageCommand(10));
    }
  }

  public SendableChooser<Command> getAutoChooser() {
    return autoChooser;
  }

  public static Command detectAndMoveTarget(VisionSubsystem vision, CommandSwerveDrivetrain drive) {
    return new ToPointCommand(drive, () -> vision.getTargetTrajectory());
  }

  public Command autoStart(
      ElevatorSubsystem elevatorSubsystem) {
    if (Robot.isSimulation()) {
      return Commands.print("Probed!");
    }
    return Commands.parallel(
            elevatorSubsystem.homeElevatorCommand())
        .withName("AutoStart");
  }
}
