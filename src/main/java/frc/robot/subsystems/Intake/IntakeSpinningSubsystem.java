// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MiscConstants;
import frc.robot.telemetry.tunable.TunableTelemetryPIDController;
import frc.robot.telemetry.types.EventTelemetryEntry;
import frc.robot.telemetry.wrappers.TelemetryTalonFX;
import frc.robot.utils.Alert;
import frc.robot.utils.ConfigEquality;
import frc.robot.utils.ConfigurationUtils;

@Logged
public class IntakeSpinningSubsystem extends SubsystemBase {

  public final TelemetryTalonFX intakeSpinningMotor =
      new TelemetryTalonFX(
          IntakeConstants.SPINNING_MOTOR_ID, "intake/spinning", MiscConstants.TUNING_MODE);
  SlewRateLimiter rateLimiter = new SlewRateLimiter(IntakeConstants.RATE_LIMIT);
  public final TunableTelemetryPIDController spinningPid =
      new TunableTelemetryPIDController("pid/intake/spin", IntakeConstants.SPINNING_PID_GAINS);
  private SimpleMotorFeedforward spinningff = IntakeConstants.SPINNING_FF_GAINS.createFeedforward();
  private static final Alert spinningMotorAlert =
      new Alert("intake spinning motor had a fault initializing", Alert.AlertType.ERROR);
  private RelativeEncoder intakeSpinningEncoder;
  private EventTelemetryEntry intakeSpinningEvent =
      new EventTelemetryEntry("intakeSpinningMotor/Event");

  private final SysIdRoutine intakeSpinningSysId =
      new SysIdRoutine(
          new SysIdRoutine.Config(Volts.per(Second).of(.5), Volts.of(2), null, null),
          new SysIdRoutine.Mechanism((voltage) -> setVoltage(voltage.in(Volts)), null, this));

  public IntakeSpinningSubsystem() {
    configMotor();
    setDefaultCommand(
        setVoltageCommand(0.0).ignoringDisable(true).withName("Default Spinning Intake"));
  }

  private void configMotor() {
    TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
    motorConfiguration.CurrentLimits.SupplyCurrentLimit =
        Constants.IntakeConstants.SUPPLY_CURRENT_LIMIT_SPINNING;
    motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfiguration.MotorOutput.Inverted = Constants.IntakeConstants.INVERTED_SPINNING;
    motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfiguration.Audio.AllowMusicDurDisable = true;
    ConfigurationUtils.StringFaultRecorder faultRecorder =
        new ConfigurationUtils.StringFaultRecorder();
    ConfigurationUtils.applyCheckRecordCTRE(
        () -> intakeSpinningMotor.getConfigurator().apply(motorConfiguration),
        () -> {
          TalonFXConfiguration appliedConfig = new TalonFXConfiguration();
          intakeSpinningMotor.getConfigurator().refresh(appliedConfig);
          return ConfigEquality.isTalonConfigurationEqual(motorConfiguration, appliedConfig);
        },
        faultRecorder.run("Motor configuration"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordCTRE(
        intakeSpinningMotor::optimizeBusUtilization,
        () -> true,
        faultRecorder.run("Optimize bus utilization"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);

    ConfigurationUtils.postDeviceConfig(
        faultRecorder.hasFault(),
        intakeSpinningEvent::append,
        "intake spinning motor fault",
        faultRecorder.getFaultString());
    spinningMotorAlert.set(faultRecorder.hasFault());

    intakeSpinningMotor.setLoggingPositionConversionFactor(
        Constants.IntakeConstants.GEAR_RATIO_SPINNING);
    intakeSpinningMotor.setLoggingVelocityConversionFactor(
        Constants.IntakeConstants.GEAR_RATIO_SPINNING);

    // Clear reset as this is on startup
    intakeSpinningMotor.hasResetOccurred();
  }

  public void setVoltage(Double voltage) {
    intakeSpinningMotor.setVoltage(voltage);
  }

  public Command setVoltageCommand(double voltage) {
    return this.run(() -> setVoltage(voltage)).withName("intake/spinning/voltage");
  }

  public double getVelocity() {
    return intakeSpinningEncoder.getVelocity();
  }

  public Command runVelocityCommand(double setpointRadiansSecond) {
    return this.run(
            () -> {
              double rateLimited = rateLimiter.calculate(setpointRadiansSecond);
              setVoltage(
                  spinningPid.calculate(intakeSpinningEncoder.getVelocity(), rateLimited)
                      + spinningff.calculate(rateLimited));
            })
        .beforeStarting(() -> rateLimiter.reset(intakeSpinningEncoder.getVelocity()))
        .withName("IntakeRunVelocity");
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return intakeSpinningSysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return intakeSpinningSysId.dynamic(direction);
  }

  @Override
  public void periodic() {
    if (Constants.IntakeConstants.SPINNING_FF_GAINS.hasChanged()) {
      spinningff = Constants.IntakeConstants.SPINNING_FF_GAINS.createFeedforward();
    }
  }
}
