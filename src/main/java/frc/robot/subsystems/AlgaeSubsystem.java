// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.Units;
import frc.robot.Constants.AlgaeConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.telemetry.wrappers.TelemetryCANSparkFlex;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MiscConstants;
import frc.robot.telemetry.tunable.TunableTelemetryPIDController;
import frc.robot.telemetry.types.EventTelemetryEntry;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;
import frc.robot.utils.ConfigurationUtils;
import frc.robot.utils.ConfigurationUtils.StringFaultRecorder;

@Logged
public class AlgaeSubsystem extends SubsystemBase {

  
  private final TelemetryCANSparkFlex algaeMotor = new TelemetryCANSparkFlex(
    Constants.AlgaeConstants.ALGAE_MOTOR_ID,
    SparkLowLevel.MotorType.kBrushless,  //no more CANSparkLowLevel?
    "algae/motor", 
     Constants.MiscConstants.TUNING_MODE);

  public Alert algaeMotorAlert = new Alert("Algae motor not doing so well", AlertType.ERROR);
  SlewRateLimiter limiter = new SlewRateLimiter(0); //deal with later
  private RelativeEncoder algaeEncoder;
  private EventTelemetryEntry algaeEvent = new EventTelemetryEntry("AlgaeMotor/Event");

  private final SysIdRoutine wristSysId = new SysIdRoutine(
 new SysIdRoutine.Config(Volts.per(Second).of(.5), Volts.of(2), null, null),
   new SysIdRoutine.Mechanism(
              (voltage) -> setVoltage(voltage.in(Volts)),
              null,
              this
));


public AlgaeSubsystem() {
  configMotor();
  setDefaultCommand(setVoltageCommand(0.0).withName("Default Algae"));
}

public void configMotor() {
  algaeEncoder = algaeMotor.getEncoder();
  double conversionFactor = Math.PI * 2 / Constants.AlgaeConstants.SHOOTER_GEAR_RATIO;

  StringFaultRecorder faultRecorder = new StringFaultRecorder();
  ConfigurationUtils.applyCheckRecordRev(
      () -> algaeMotor.setCANTimeout(250),
      () -> true,
      faultRecorder.run("CAN timeout"),
      Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
  ConfigurationUtils.applyCheckRecordRev(
      algaeMotor::restoreFactoryDefaults,
      () -> true,
      faultRecorder.run("Factory defaults"),
      Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
  ConfigurationUtils.applyCheckRecordRev(
      () -> algaeMotor.setSmartCurrentLimit(Constants.AlgaeConstants.STALL_MOTOR_CURRENT, 
      Constants.AlgaeConstants.FREE_MOTOR_CURRENT),
      () -> true,
      faultRecorder.run("Current limits"),
      Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
  ConfigurationUtils.applyCheckRecord(
    () -> SparkBaseConfig.inverted(Constants.AlgaeConstants.INVERTED()),
    () -> SparkBaseConfig.getInverted() == Constants.AlgaeConstants.INVERTED,
     // () -> algaeMotor.setInverted(Constants.AlgaeConstants.INVERTED), //actually what is this. WHy would they depricate it this is so ctupid.
    // () -> algaeMotor.getInverted() == Constants.AlgaeConstants.INVERTED,
      faultRecorder.run("Inverted"),
      Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
  ConfigurationUtils.applyCheckRecordRev(
      () -> algaeMotor.setIdleMode(CANSparkMax.IdleMode.kCoast),
      () -> algaeMotor.getIdleMode() == CANSparkMax.IdleMode.kCoast,
      faultRecorder.run("Idle mode"),
      Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
  ConfigurationUtils.applyCheckRecordRev(
      () -> algaeEncoder.setPositionConversionFactor(conversionFactor),
      () ->
          ConfigurationUtils.fpEqual(
              algaeEncoder.getPositionConversionFactor(), conversionFactor),
      faultRecorder.run("Position conversion factor"),
      Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
  ConfigurationUtils.applyCheckRecordRev(
      () -> algaeEncoder.setVelocityConversionFactor(conversionFactor / 60),
      () ->
          ConfigurationUtils.fpEqual(
              algaeEncoder.getVelocityConversionFactor(), conversionFactor / 60),
      faultRecorder.run("Velocity conversion factor"),
      Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
  ConfigurationUtils.applyCheckRecordRev(
      algaeMotor::burnFlashWithDelay,
      () -> true,
      faultRecorder.run("Burn flash"),
      Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
  ConfigurationUtils.postDeviceConfig(
      faultRecorder.hasFault(),
      algaeEvent::append,
      "Shooter motor",
      faultRecorder.getFaultString());
  algaeMotorAlert.set(faultRecorder.hasFault());
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

