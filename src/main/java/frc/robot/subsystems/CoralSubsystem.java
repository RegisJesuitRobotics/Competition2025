// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.MiscConstants;
import frc.robot.telemetry.tunable.TunableTelemetryPIDController;
import frc.robot.telemetry.types.EventTelemetryEntry;
import frc.robot.telemetry.wrappers.TelemetryCANSparkFlex;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;
import frc.robot.utils.ConfigurationUtils;
import frc.robot.utils.ConfigurationUtils.StringFaultRecorder;

@Logged
public class CoralSubsystem extends SubsystemBase {

  private final TelemetryCANSparkFlex coralMotor =
      new TelemetryCANSparkFlex(
          CoralConstants.CORAL_MOTOR_ID,
          SparkLowLevel.MotorType.kBrushless,
          "/coral/motor",
          MiscConstants.TUNING_MODE);

  private final Alert coralMotorAlert = new Alert("Coral motor had a fault", AlertType.ERROR);
  private final SlewRateLimiter rateLimiter = new SlewRateLimiter(CoralConstants.SLEW_RATE_LIMIT);
  private RelativeEncoder coralEncoder = coralMotor.getEncoder();
  private final EventTelemetryEntry coralEvent = new EventTelemetryEntry("/coral/events");

  private final TunableTelemetryPIDController coralpid =
      new TunableTelemetryPIDController("/coral/pid", Constants.CoralConstants.PID_GAINS);
  private SimpleMotorFeedforward coralFF = CoralConstants.FF_GAINS.createFeedforward();

  private final SysIdRoutine coralSysId =
      new SysIdRoutine(
          new SysIdRoutine.Config(Volts.per(Second).of(.5), Volts.of(2), null, null),
          new SysIdRoutine.Mechanism((voltage) -> setVoltage(voltage.in(Volts)), null, this));

  public CoralSubsystem() {
    configMotor();
    setDefaultCommand(setVoltageCommand(0.0).ignoringDisable(true).withName("CoralDefault"));
  }

  private void configMotor() {
    coralEncoder = coralMotor.getEncoder();
    double conversionFactor = Math.PI * 2 / Constants.CoralConstants.GEAR_RATIO;

    StringFaultRecorder faultRecorder = new StringFaultRecorder();
    SparkFlexConfig config = new SparkFlexConfig();

    ConfigurationUtils.applyCheckRecordRev(
        () -> coralMotor.setCANTimeout(250),
        () -> true,
        faultRecorder.run("CAN timeout"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        () ->
            coralMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters),
        () -> true,
        faultRecorder.run("Factory defaults"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecord(
        () ->
            config.smartCurrentLimit(
                Constants.CoralConstants.STALL_MOTOR_CURRENT,
                Constants.CoralConstants.FREE_MOTOR_CURRENT),
        () -> true,
        faultRecorder.run("Current limits"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecord(
        () -> config.inverted(Constants.CoralConstants.INVERTED),
        () -> coralMotor.configAccessor.getInverted() == Constants.CoralConstants.INVERTED,
        faultRecorder.run("Inverted"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecord(
        () -> config.idleMode(IdleMode.kCoast),
        () -> coralMotor.configAccessor.getIdleMode() == SparkFlexConfig.IdleMode.kCoast,
        faultRecorder.run("Idle mode"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecord(
        () -> config.encoder.positionConversionFactor(conversionFactor / 60),
        () ->
            ConfigurationUtils.fpEqual(
                coralMotor.configAccessor.encoder.getVelocityConversionFactor(),
                conversionFactor / 60),
        faultRecorder.run("Velocity conversion factor"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        coralMotor::burnFlashWithDelay,
        () -> true,
        faultRecorder.run("Burn flash"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.postDeviceConfig(
        faultRecorder.hasFault(),
        coralEvent::append,
        "Shooter motor",
        faultRecorder.getFaultString());
    coralMotorAlert.set(faultRecorder.hasFault());
  }

  public void setVoltage(double voltage) {
    coralMotor.setVoltage(voltage);
  }

  public double getVelocity() {
    return coralEncoder.getVelocity();
  }

  public Command setVoltageCommand(double voltage) {
    return this.run(() -> coralMotor.setVoltage(voltage));
  }

  public Command runVelocityCommand(double setpointRadiansSecond) {
    return this.run(
            () -> {
              double rateLimited = rateLimiter.calculate(setpointRadiansSecond);
              setVoltage(
                  coralpid.calculate(coralEncoder.getVelocity(), rateLimited)
                      + coralFF.calculate(rateLimited));
            })
        .beforeStarting(() -> rateLimiter.reset(coralEncoder.getVelocity()))
        .withName("CoralRunVelocity");
  }

  public Command sysIDQuasistatic(SysIdRoutine.Direction direction) {
    return coralSysId.quasistatic(direction);
  }

  public Command sysIDDynamic(SysIdRoutine.Direction direction) {
    return coralSysId.dynamic(direction);
  }

  @Override
  public void periodic() {
    if (Constants.CoralConstants.FF_GAINS.hasChanged()) {
      coralFF = Constants.CoralConstants.FF_GAINS.createFeedforward();
    }
    coralMotor.logValues();
    // This method will be called once per scheduler run
  }
}
