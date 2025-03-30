// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

public class MiscCommands {
  public static Command ClimberUpCommand(ClimberSubsystem climberSubsystem) {
    return climberSubsystem.setVoltageCommand(Constants.ClimberConstants.CLIMBER_UP_VOLTAGE);
  }

  public static Command ClimberDownCommand(ClimberSubsystem climberSubsystem) {
    return climberSubsystem.setVoltageCommand(Constants.ClimberConstants.CLIMBER_DOWN_VOLTAGE);
  }
  
  public static Command rumbleHIDCommand(GenericHID hid) {
    return Commands.runEnd(
            () -> hid.setRumble(RumbleType.kBothRumble, 1.0),
            () -> hid.setRumble(RumbleType.kBothRumble, 0.0))
        .withName("RumbleHID");
  }
}
