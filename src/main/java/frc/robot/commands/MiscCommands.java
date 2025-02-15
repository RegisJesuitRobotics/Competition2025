// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

public class MiscCommands {
    public static Command ClimberUpCommand(ClimberSubsystem climberSubsystem){
        return climberSubsystem.runVelocityCommand(Constants.ClimberConstants.CLIMBER_UP_VOLTAGE);
    }
    public static Command ClimberDownCommand(ClimberSubsystem climberSubsystem){
        return climberSubsystem.runVelocityCommand(Constants.ClimberConstants.CLIMBER_DOWN_VOLTAGE);
    }
}
