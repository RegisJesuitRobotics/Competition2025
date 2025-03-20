// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MiscConstants;
import frc.robot.telemetry.tunable.TunableTelemetryPIDController;
import frc.robot.telemetry.types.BooleanTelemetryEntry;
import frc.robot.telemetry.types.EventTelemetryEntry;
import frc.robot.telemetry.wrappers.TelemetryCANSparkFlex;
import frc.robot.telemetry.wrappers.TelemetryTalonFX;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;
import frc.robot.utils.ConfigEquality;
import frc.robot.utils.ConfigurationUtils;
import frc.robot.utils.ConfigurationUtils.StringFaultRecorder;

// @Logged
public class IntakeSpinningSubsystem extends SubsystemBase {

  private final TelemetryTalonFX intakeSpinningMotor =
      new TelemetryTalonFX(
          IntakeConstants.SPINNING_MOTOR_ID,
          "/intake/spinning/motor",
          MiscConstants.TUNING_MODE);

  private final Alert intakeSpinningMotorAlert =
      new Alert("intake spinning motor had a fault", AlertType.ERROR);
  private final SlewRateLimiter rateLimiter = new SlewRateLimiter(IntakeConstants.RATE_LIMIT);

  private final EventTelemetryEntry intakeSpinningEvent =
      new EventTelemetryEntry("/spinning/intake/events");

  private final DigitalInput intakeSlapdownSwitchLeft =
      new DigitalInput(IntakeConstants.LEFT_SWITCH);

  private final TunableTelemetryPIDController intakeSpinningPID =
      new TunableTelemetryPIDController(
          "intake/spinning/pid", Constants.IntakeConstants.SPINNING_PID_GAINS);
  private SimpleMotorFeedforward intakeSpinningFF =
      IntakeConstants.SPINNING_FF_GAINS.createFeedforward();
  private final BooleanTelemetryEntry limit = new BooleanTelemetryEntry("/intake", true);

  private final SysIdRoutine intakeSpinningSysId =
      new SysIdRoutine(
          new SysIdRoutine.Config(Volts.per(Second).of(.5), 
          Volts.of(2), null, null),
          new SysIdRoutine.Mechanism((voltage) -> setVoltage(voltage.in(Volts)), null, this));

  public IntakeSpinningSubsystem() {
    configMotor();
    setDefaultCommand(
        setVoltageCommand(0.0).ignoringDisable(true).withName("IntakeSpinningDefault"));
  }

  private void configMotor() {
    TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
    motorConfiguration.CurrentLimits.SupplyCurrentLimit =
        Constants.ClimberConstants.SUPPLY_CURRENT_LIMIT;
    motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfiguration.MotorOutput.Inverted = Constants.IntakeConstants.INVERTED_SPINNING;

    motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfiguration.Audio.AllowMusicDurDisable = true;
    ConfigurationUtils.StringFaultRecorder faultRecorder =
        new ConfigurationUtils.StringFaultRecorder();

    TalonFXConfiguration leftMotorConfiguration = new TalonFXConfiguration();
    leftMotorConfiguration.CurrentLimits.SupplyCurrentLimit =
        Constants.IntakeConstants.SUPPLY_CURRENT_LIMIT_SPINNING;
    leftMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    leftMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    ConfigurationUtils.StringFaultRecorder leftFaultRecorder =
        new ConfigurationUtils.StringFaultRecorder();
    ConfigurationUtils.applyCheckRecordCTRE(
        () -> intakeSpinningMotor.getConfigurator().apply(motorConfiguration),
        () -> {
          TalonFXConfiguration appliedConfig = new TalonFXConfiguration();
          intakeSpinningMotor.getConfigurator().refresh(appliedConfig);
          return ConfigEquality.isTalonConfigurationEqual(motorConfiguration, appliedConfig);
        },
        leftFaultRecorder.run("Motor configuration"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordCTRE(
        intakeSpinningMotor::optimizeBusUtilization,
        () -> true,
        leftFaultRecorder.run("Optimize bus utilization"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);

    ConfigurationUtils.postDeviceConfig(
        leftFaultRecorder.hasFault(),
        intakeSpinningEvent::append,
        "climb motor 1 fault",
        leftFaultRecorder.getFaultString());
    intakeSpinningMotorAlert.set(faultRecorder.hasFault());

    // Clear reset as this is on startup
    intakeSpinningMotor.hasResetOccurred();
  }

  private final VoltageOut voltageOut = new VoltageOut(0.0);

  public void setVoltage(double voltage) {
    voltageOut.Output = voltage;
    intakeSpinningMotor.setControl(voltageOut);
  }

  public boolean getSwitchValue() {
    return !intakeSlapdownSwitchLeft.get();
  }

  public double getVelocity() {
    return intakeSpinningMotor.getVelocity().getValueAsDouble();
  }

  public Command setVoltageCommand(double voltage) {
    return this.run(() -> setVoltage(voltage));
  }

  public Command runVelocityCommand(double setpointRadiansSecond) {
    return this.run(
            () -> {
              double rateLimited = rateLimiter.calculate(setpointRadiansSecond);
              setVoltage(
                  intakeSpinningPID.calculate(intakeSpinningMotor.getVelocity().getValueAsDouble(), rateLimited)
                      + intakeSpinningFF.calculate(rateLimited));
            })
        .beforeStarting(() -> rateLimiter.reset(intakeSpinningMotor.getVelocity().getValueAsDouble()))
        .withName("IntakeSpinningRunVelocity");
  }

  public Command sysIDQuasistatic(SysIdRoutine.Direction direction) {
    return intakeSpinningSysId.quasistatic(direction);
  }

  public Command sysIDDynamic(SysIdRoutine.Direction direction) {
    return intakeSpinningSysId.dynamic(direction);
  }

  @Override
  public void periodic() {
    limit.append(getSwitchValue());

    //   intakeSpinningMotor.logValues();
    // This method will be called once per scheduler run
  }
}
