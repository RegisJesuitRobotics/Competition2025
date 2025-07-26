// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.MiscConstants;
import frc.robot.telemetry.tunable.TunableTelemetryPIDController;
import frc.robot.telemetry.types.BooleanTelemetryEntry;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.types.EventTelemetryEntry;
import frc.robot.telemetry.wrappers.TelemetryTalonFX;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;
import frc.robot.utils.ConfigEquality;
import frc.robot.utils.ConfigurationUtils;

// @Logged
public class CoralSubsystem extends SubsystemBase {

  private final TalonFX coralMotor =
      new TalonFX(CoralConstants.CORAL_MOTOR_ID);

  private final Alert coralMotorAlert = new Alert("Coral motor had a fault", AlertType.ERROR);
  private final EventTelemetryEntry coralEvent = new EventTelemetryEntry("/coral/events");

  private final SlewRateLimiter slewRateLimiter = new SlewRateLimiter(12.0 / .25);

    private final DigitalInput intakeBeam = new DigitalInput(0);
  private BooleanTelemetryEntry beamEntry = new BooleanTelemetryEntry("/coral/beam", true);

  private final DoubleTelemetryEntry positionEntry = new DoubleTelemetryEntry("/coral/position", true);
  private final DoubleTelemetryEntry velocityEntry = new DoubleTelemetryEntry("/coral/velocity", true);
  private final DoubleTelemetryEntry voltageEntry = new DoubleTelemetryEntry("/coral/voltage", true);
  

  public CoralSubsystem() {
    configMotor();
    setDefaultCommand(setVoltageCommand(0.0).ignoringDisable(true).withName("CoralDefault"));
  }


  private void configMotor() {
    TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
    motorConfiguration.CurrentLimits.SupplyCurrentLimit =
        Constants.CoralConstants.SUPPLY_CURRENT_LIMIT;
    motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfiguration.MotorOutput.Inverted = Constants.CoralConstants.INVERTED;
    motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfiguration.Audio.AllowMusicDurDisable = true;
    ConfigurationUtils.StringFaultRecorder faultRecorder =
        new ConfigurationUtils.StringFaultRecorder();
    ConfigurationUtils.applyCheckRecordCTRE(
        () -> coralMotor.getConfigurator().apply(motorConfiguration),
        () -> {
          TalonFXConfiguration appliedConfig = new TalonFXConfiguration();
          coralMotor.getConfigurator().refresh(appliedConfig);
          return ConfigEquality.isTalonConfigurationEqual(motorConfiguration, appliedConfig);
        },
        faultRecorder.run("Motor configuration"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordCTRE(
        coralMotor::optimizeBusUtilization,
        () -> true,
        faultRecorder.run("Optimize bus utilization"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);

    ConfigurationUtils.postDeviceConfig(
        faultRecorder.hasFault(),
        coralEvent::append,
        "coral motor fault",
        faultRecorder.getFaultString());
    coralMotorAlert.set(faultRecorder.hasFault());

    // Clear reset as this is on startup
    coralMotor.hasResetOccurred();
  }

  
  public void setVoltage(double voltage) {
    coralMotor.setVoltage(voltage);
  }

  public double getVelocity() {
    return coralMotor.getVelocity().getValueAsDouble() * CoralConstants.GEAR_RATIO;
  }
  public double getPosition(){
    return coralMotor.getPosition().getValueAsDouble() * CoralConstants.GEAR_RATIO;
  }

  public boolean getLeftSwitchState() {
    return !intakeBeam.get();
  }


  public Command setVoltageCommand(double voltage) {
    return this.run(() -> coralMotor.setVoltage(slewRateLimiter.calculate(voltage)))
        .finallyDo(() -> coralMotor.setVoltage(0.0))
        .beforeStarting(() -> slewRateLimiter.reset(0));
  }

  public Command intakeUntilDetected() {
    return setVoltageCommand(Constants.CoralConstants.INTAKE_VOLTAGE)
        .until(this::getLeftSwitchState)
        .andThen(setVoltageCommand(0));
  }


  @Override
  public void periodic() {
    beamEntry.append(getLeftSwitchState());
    positionEntry.append(getPosition());
    velocityEntry.append(getVelocity());
    voltageEntry.append(coralMotor.getMotorVoltage().getValueAsDouble());
  }
}
