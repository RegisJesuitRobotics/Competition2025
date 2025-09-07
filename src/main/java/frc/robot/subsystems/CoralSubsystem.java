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
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.MiscConstants;
import frc.robot.telemetry.types.BooleanTelemetryEntry;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.types.EventTelemetryEntry;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;
import frc.robot.utils.ConfigEquality;
import frc.robot.utils.ConfigurationUtils;

// @Logged
public class CoralSubsystem extends SubsystemBase {

  private final TalonFX coralMotor = new TalonFX(CoralConstants.CORAL_MOTOR_ID, Constants.MiscConstants.CANIVORE_NAME);

  private final DigitalInput intakeLeftBeam = new DigitalInput(Constants.CoralConstants.SWITCH_ID_LEFT);
  private final DigitalInput intakeRightBeam = new DigitalInput(Constants.CoralConstants.SWITCH_ID_RIGHT);
  private final SlewRateLimiter slewRateLimiter = new SlewRateLimiter(12.0/.25);
  
  private final Alert coralMotorAlert = new Alert("Coral motor had a fault", AlertType.ERROR);
  private final EventTelemetryEntry coralEvent = new EventTelemetryEntry("/coral/events");

  private BooleanTelemetryEntry rightBeam = new BooleanTelemetryEntry("/coral/right", true);
  private BooleanTelemetryEntry leftBeam = new BooleanTelemetryEntry("/coral/left", true);
  private DoubleTelemetryEntry voltage = new DoubleTelemetryEntry("/coral/voltage", true);
  private DoubleTelemetryEntry supplyVoltage = new DoubleTelemetryEntry("/coral/supplyVoltage", true);
  private DoubleTelemetryEntry velocity = new DoubleTelemetryEntry("/coral/velocity", true);



  public CoralSubsystem() {
    setDefaultCommand(setVoltageCommand(0.0).ignoringDisable(true).withName("CoralDefault"));
    configMotor();
  }

  private void configMotor() {
    TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
    motorConfiguration.CurrentLimits.SupplyCurrentLimit = Constants.CoralConstants.SUPPLY_CURRENT_LIMIT;
    motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfiguration.MotorOutput.Inverted = Constants.CoralConstants.INVERTED;
    motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfiguration.Audio.AllowMusicDurDisable = true;
    ConfigurationUtils.StringFaultRecorder faultRecorder = new ConfigurationUtils.StringFaultRecorder();
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

  public Command setVoltageCommand(double voltage){
    return this.run(() -> coralMotor.setVoltage(slewRateLimiter.calculate(voltage)));
  }
  // return this.run(() -> coralMotor.setVoltage(slewRateLimiter.calculate(voltage)))
  //       .finallyDo(() -> coralMotor.setVoltage(0.0)).beforeStarting(() -> slewRateLimiter.reset(0));

  public Command intakeUntilDetected() {
    return setVoltageCommand(Constants.CoralConstants.INTAKE_VOLTAGE)
        .until(this::getLeftSwitchState)
        .andThen(setVoltageCommand(0));
  }
  
  public boolean getLeftSwitchState() {
    return intakeLeftBeam.get();
  }

  public boolean getRightSwitchState() {
    return intakeRightBeam.get();
  }

  public double getVelocity(){
    return coralMotor.getMotorVoltage().getValueAsDouble() * Constants.CoralConstants.GEAR_RATIO;
  }

  @Override
  public void periodic() {

    rightBeam.append(intakeRightBeam.get());
    leftBeam.append(intakeLeftBeam.get());
    voltage.append(coralMotor.getMotorVoltage().getValueAsDouble());
    velocity.append(getVelocity());
    supplyVoltage.append(coralMotor.getSupplyVoltage().getValueAsDouble());
  }
}
