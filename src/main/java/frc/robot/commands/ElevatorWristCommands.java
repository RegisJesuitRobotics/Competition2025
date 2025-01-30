package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ElevatorWristCommands {

    public static Command elevatorWristLowReef(ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem){
        return Commands.parallel(elevatorSubsystem.setPosition(Constants.ElevatorConstants.LOW_REEF), wristSubsystem.setPositionCommand(Constants.WristConstants.LOW_REEF));
    }
    public static Command elevatorWristMidReef(ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem){
        return Commands.parallel(elevatorSubsystem.setPosition(Constants.ElevatorConstants.MID_REEF), wristSubsystem.setPositionCommand(Constants.WristConstants.MID_REEF));
    }
    public static Command elevatorWristHighReef(ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem){
        return Commands.parallel(elevatorSubsystem.setPosition(Constants.ElevatorConstants.HIGH_REEF), wristSubsystem.setPositionCommand(Constants.WristConstants.HIGH_REEF));
    }
}
