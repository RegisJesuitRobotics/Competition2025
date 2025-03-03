// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.MiscConstants;
import frc.robot.telemetry.tunable.TunableTelemetryPIDController;
import frc.robot.telemetry.types.EventTelemetryEntry;
import frc.robot.telemetry.wrappers.TelemetryTalonFX;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;
import frc.robot.utils.ConfigEquality;
import frc.robot.utils.ConfigurationUtils;

@Logged
public class ClimberSubsystem extends SubsystemBase {
  // motor 1
  public final TelemetryTalonFX climbMotor1 =
      new TelemetryTalonFX(
          ClimberConstants.CLIMB_MOTOR_1_ID, "climber/motor/1", MiscConstants.TUNING_MODE);
  private final Alert climbMotor1Alert = new Alert("climb motor(1) had a fault", AlertType.ERROR);
  private final EventTelemetryEntry climbMotor1Entry =
      new EventTelemetryEntry("/climber/motor1/events");
  private final SlewRateLimiter rateLimiter = new SlewRateLimiter(ClimberConstants.LIMITER);
  // motor 2
  public final TelemetryTalonFX climbMotor2 =
      new TelemetryTalonFX(
          ClimberConstants.CLIMB_MOTOR_2_ID, "climber/motor/2", MiscConstants.TUNING_MODE);
  private final Alert climbMotor2Alert = new Alert("climb motor(2) had a fault", AlertType.ERROR);
  private final EventTelemetryEntry climbMotor2Entry =
      new EventTelemetryEntry("/climber/motor2/events");

  private final TunableTelemetryPIDController climbPID =
      new TunableTelemetryPIDController("pid/climb", ClimberConstants.CLIMB_PID_GAINS);
  private SimpleMotorFeedforward climberff = ClimberConstants.CLIMBER_FF_GAINS.createFeedforward();

  private final SysIdRoutine climberSysId =
      new SysIdRoutine(
          new SysIdRoutine.Config(Volts.per(Second).of(.5), Volts.of(2), null, null),
          new SysIdRoutine.Mechanism((voltage) -> setVoltage(voltage.in(Volts)), null, this));

  public ClimberSubsystem() {
    configMotors();
    setDefaultCommand(setVoltageCommand(0));
  }

  private void configMotors() {
    TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
    motorConfiguration.CurrentLimits.SupplyCurrentLimit =
        Constants.ClimberConstants.SUPPLY_CURRENT_LIMIT;
    motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfiguration.MotorOutput.Inverted = Constants.ClimberConstants.INVERTED_2;

    motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfiguration.Audio.AllowMusicDurDisable = true;
    ConfigurationUtils.StringFaultRecorder faultRecorder =
        new ConfigurationUtils.StringFaultRecorder();
    ConfigurationUtils.applyCheckRecordCTRE(
        () -> climbMotor2.getConfigurator().apply(motorConfiguration),
        () -> {
          TalonFXConfiguration appliedConfig = new TalonFXConfiguration();
          climbMotor2.getConfigurator().refresh(appliedConfig);
          return ConfigEquality.isTalonConfigurationEqual(motorConfiguration, appliedConfig);
        },
        faultRecorder.run("Motor configuration"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordCTRE(
        climbMotor2::optimizeBusUtilization,
        () -> true,
        faultRecorder.run("Optimize bus utilization"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);

    ConfigurationUtils.postDeviceConfig(
        faultRecorder.hasFault(),
        climbMotor2Entry::append,
        "climb motor 2 fault",
        faultRecorder.getFaultString());
    climbMotor2Alert.set(faultRecorder.hasFault());

    climbMotor2.setLoggingPositionConversionFactor(Constants.ClimberConstants.GEAR_RATIO);
    climbMotor2.setLoggingVelocityConversionFactor(Constants.ClimberConstants.GEAR_RATIO);

    // Clear reset as this is on startup
    climbMotor2.hasResetOccurred();

    TalonFXConfiguration leftMotorConfiguration = new TalonFXConfiguration();
    leftMotorConfiguration.CurrentLimits.SupplyCurrentLimit =
        Constants.ClimberConstants.SUPPLY_CURRENT_LIMIT;
    leftMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    leftMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    ConfigurationUtils.StringFaultRecorder leftFaultRecorder =
        new ConfigurationUtils.StringFaultRecorder();
    ConfigurationUtils.applyCheckRecordCTRE(
        () -> climbMotor1.getConfigurator().apply(motorConfiguration),
        () -> {
          TalonFXConfiguration appliedConfig = new TalonFXConfiguration();
          climbMotor1.getConfigurator().refresh(appliedConfig);
          return ConfigEquality.isTalonConfigurationEqual(motorConfiguration, appliedConfig);
        },
        leftFaultRecorder.run("Motor configuration"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordCTRE(
        climbMotor1::optimizeBusUtilization,
        () -> true,
        leftFaultRecorder.run("Optimize bus utilization"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);

    ConfigurationUtils.postDeviceConfig(
        leftFaultRecorder.hasFault(),
        climbMotor1Entry::append,
        "climb motor 1 fault",
        leftFaultRecorder.getFaultString());
    climbMotor1Alert.set(faultRecorder.hasFault());

    climbMotor1.setControl(
        new Follower(
            Constants.ClimberConstants.CLIMB_MOTOR_2_ID, false));
    // Clear reset as this is on startup
    climbMotor1.hasResetOccurred();
  }

  public void setVoltage(double voltage) {
    climbMotor1.setVoltage(voltage);
  }

  public double getVelocity() {
    return climbMotor1.getVelocity().getValueAsDouble();
  }

  public Command setVoltageCommand(double voltage) {
    return this.run(() -> climbMotor1.setVoltage(voltage));
  }

  public Command runVelocityCommand(double setpointRadiansSecond) {
    return this.run(
            () -> {
              double rateLimited = rateLimiter.calculate(setpointRadiansSecond);
              setVoltage(
                  climbPID.calculate(getVelocity(), rateLimited)
                      + climberff.calculate(rateLimited));
            })
        .beforeStarting(() -> rateLimiter.reset(getVelocity()))
        .withName("ClimberRunVelocity");
  }

  public Command sysIDQuasistatic(SysIdRoutine.Direction direction) {
    return climberSysId.quasistatic(direction);
  }

  public Command sysIDDynamic(SysIdRoutine.Direction direction) {
    return climberSysId.dynamic(direction);
  }

  @Override
  public void periodic() {
    if (Constants.ClimberConstants.CLIMBER_FF_GAINS.hasChanged()) {
      climberff = Constants.ClimberConstants.CLIMBER_FF_GAINS.createFeedforward();
    }
    // This method will be called once per scheduler run
  }
}
