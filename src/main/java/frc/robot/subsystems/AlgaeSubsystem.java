// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
public class AlgaeSubsystem extends SubsystemBase {

 private final TelemetryTalonFX algaeMotor = new TelemetryTalonFX(Constants.AlgaeConstants.ALGAE_MOTOR_ID, "/algae/motor", Constants.MiscConstants.TUNING_MODE);

  private final TunableTelemetryPIDController algaePID =
      new TunableTelemetryPIDController("algae/pid", Constants.AlgaeConstants.PID_GAINS);
  private SimpleMotorFeedforward algaeFF = Constants.AlgaeConstants.FF_GAINS.createFeedforward();
  private final DigitalInput intakeSwitchRight =
      new DigitalInput(Constants.AlgaeConstants.SWITCH_ID_RIGHT);
  public Alert algaeMotorAlert = new Alert("Algae motor had a fault", AlertType.ERROR);
  SlewRateLimiter limiter =
      new SlewRateLimiter(Constants.AlgaeConstants.RATE_LIMIT); // deal with later
  private RelativeEncoder algaeEncoder;
  private EventTelemetryEntry algaeEvent = new EventTelemetryEntry("AlgaeMotor/Event");
  private BooleanTelemetryEntry limit = new BooleanTelemetryEntry("/algae/limit", true);
  private final BooleanTelemetryEntry otherSwitch = new BooleanTelemetryEntry("/algae/other", true);

  private final SysIdRoutine algaeSysId =
      new SysIdRoutine(
          new SysIdRoutine.Config(Volts.per(Second).of(.5), Volts.of(2), null, null),
          new SysIdRoutine.Mechanism((voltage) -> setVoltage(voltage.in(Volts)), null, this));

  public AlgaeSubsystem() {
    configMotor();
    setDefaultCommand(setVoltageCommand(0.0).ignoringDisable(true).withName("Default Algae"));
  }

  public void configMotor() {
    TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
    motorConfiguration.CurrentLimits.SupplyCurrentLimit =
        Constants.ElevatorConstants.SUPPLY_CURRENT_LIMIT;
    motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfiguration.MotorOutput.Inverted = Constants.ElevatorConstants.INVERTED_RIGHT;
    motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfiguration.Audio.AllowMusicDurDisable = true;

    ConfigurationUtils.StringFaultRecorder faultRecorder =
        new ConfigurationUtils.StringFaultRecorder();
    ConfigurationUtils.applyCheckRecordCTRE(
        () -> algaeMotor.getConfigurator().apply(motorConfiguration),
        () -> {
          TalonFXConfiguration appliedConfig = new TalonFXConfiguration();
          algaeMotor.getConfigurator().refresh(appliedConfig);
          return ConfigEquality.isTalonConfigurationEqual(motorConfiguration, appliedConfig);
        },
        faultRecorder.run("Motor configuration 1"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordCTRE(
        algaeMotor::optimizeBusUtilization,
        () -> true,
        faultRecorder.run("Optimize bus utilization"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);

    ConfigurationUtils.postDeviceConfig(
        faultRecorder.hasFault(),
        algaeEvent::append,
        "algae motor fault",
        faultRecorder.getFaultString());
    algaeMotorAlert.set(faultRecorder.hasFault());

   

  }

  public void setVoltage(Double voltage) {
    algaeMotor.setVoltage(voltage);
  }

  public Command setVoltageCommand(double voltage) {
    return this.run(() -> setVoltage(voltage)).withName("Algae/Voltage");
  }


  public double getVelocity() {
    return algaeEncoder.getVelocity();
  }

  public boolean getSwitchState() {
    return !intakeSwitchRight.get();
  }

  public Command runVelocityCommand(double setpointRadiansSecond) {
    return this.run(
            () -> {
              double rateLimited = limiter.calculate(setpointRadiansSecond);
              setVoltage(
                  algaePID.calculate(algaeEncoder.getVelocity(), rateLimited)
                      + algaeFF.calculate(rateLimited));
            })
        .beforeStarting(() -> limiter.reset(algaeEncoder.getVelocity()))
        .withName("ShooterRunVelocity");
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return algaeSysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return algaeSysId.dynamic(direction);
  }

  @Override
  public void periodic() {
    algaeMotor.logValues();
    limit.append(getSwitchState());
  }
}
