package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;
import java.util.concurrent.atomic.AtomicBoolean;

public class ElevatorWristCommands {

  public static Command elevatorWristL2(
      ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem, AtomicBoolean flipped) {
    return Commands.parallel(
        elevatorSubsystem.setPosition(Constants.ElevatorConstants.L2_REEF),
        wristSubsystem.setPositionCommand(
            flipped.get() ? Constants.WristConstants.L2_REEF : -Constants.WristConstants.L2_REEF));
  }

  public static Command elevatorWristL3(
      ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem, AtomicBoolean flipped) {
    return Commands.parallel(
        elevatorSubsystem.setPosition(Constants.ElevatorConstants.L3_REEF),
        wristSubsystem.setPositionCommand(
            flipped.get() ? Constants.WristConstants.L3_REEF : -Constants.WristConstants.L3_REEF));
  }

  public static Command elevatorWristL4(
      ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem, AtomicBoolean flipped) {
    return Commands.parallel(
        elevatorSubsystem.setPosition(Constants.ElevatorConstants.L4_REEF),
        wristSubsystem.setPositionCommand(
            flipped.get() ? Constants.WristConstants.L4_REEF : -Constants.WristConstants.L4_REEF));
  }

  public static Command elevatorWristHuman(
      ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem, AtomicBoolean flipped) {
    return Commands.parallel(
        elevatorSubsystem.setPosition(Constants.ElevatorConstants.HUMAN),
        wristSubsystem.setPositionCommand(
            flipped.get() ? Constants.WristConstants.HUMAN : -Constants.WristConstants.HUMAN));
  }

  public static Command elevatorWristBallLow(
      ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem, AtomicBoolean flipped) {
    return Commands.parallel(
        elevatorSubsystem.setPosition(Constants.ElevatorConstants.LOW_BALL_PICKUP),
        wristSubsystem.setPositionCommand(
            flipped.get()
                ? Constants.WristConstants.BALL_PICKUP
                : -Constants.WristConstants.BALL_PICKUP));
  }

  public static Command elevatorWristL1(
      ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem, AtomicBoolean flipped) {
    return Commands.parallel(
        elevatorSubsystem.setPosition(Constants.ElevatorConstants.L1_REEF),
        wristSubsystem.setPositionCommand(
            flipped.get() ? Constants.WristConstants.L1_REEF : -Constants.WristConstants.L1_REEF));
  }

  public static Command elevatorWristNet(
      ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem, AtomicBoolean flipped) {
    return Commands.parallel(
        elevatorSubsystem.setPosition(Constants.ElevatorConstants.NET),
        wristSubsystem.setPositionCommand(
            flipped.get() ? Constants.WristConstants.NET : -Constants.WristConstants.NET));
  }

  public static Command elevatorWristProcessor(
      ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem, AtomicBoolean flipped) {
    return Commands.parallel(
        elevatorSubsystem.setPosition(Constants.ElevatorConstants.PROCESSOR),
        wristSubsystem.setPositionCommand(
            flipped.get()
                ? Constants.WristConstants.PROCESSOR
                : -Constants.WristConstants.PROCESSOR));
  }

  public static Command elevatorWristReset(
      ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
    return Commands.parallel(
        elevatorSubsystem.setPosition(0.0), wristSubsystem.setPositionCommand(0.0));
  }

  public static Command elevatorWristGroundIntake(
      ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
    return Commands.parallel(
        elevatorSubsystem.setPosition(Constants.ElevatorConstants.GROUND_INTAKE),
        wristSubsystem.setPositionCommand(Constants.WristConstants.GROUND_INTAKE));
  }
}
