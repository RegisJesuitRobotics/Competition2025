// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.MiscConstants;
import frc.robot.commands.MiscCommands;

import org.littletonrobotics.urcl.URCL;

// @Logged
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    DataLogManager.logNetworkTables(MiscConstants.TUNING_MODE);
    DataLogManager.start();
    DataLogManager.log("*****START*****");

    DataLog dataLog = DataLogManager.getLog();
    if (MiscConstants.TUNING_MODE) {
      URCL.start();
      NetworkTableInstance.getDefault().startEntryDataLog(dataLog, "/URCL/", "URCL/");
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

        if (30 > DriverStation.getMatchTime() && DriverStation.getMatchTime()>20){
          MiscCommands.rumbleHIDCommand(m_robotContainer.operator.getHID());
          MiscCommands.rumbleHIDCommand(m_robotContainer.joystick.getHID());
        }


  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
