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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.telemetry.tunable.TunableTelemetryPIDController;
import frc.robot.telemetry.types.EventTelemetryEntry;
import frc.robot.telemetry.wrappers.TelemetryCANSparkFlex;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;
import frc.robot.utils.ConfigurationUtils;
import frc.robot.utils.ConfigurationUtils.StringFaultRecorder;

@Logged
public class AlgaeSubsystem extends SubsystemBase {

  private final TelemetryCANSparkFlex algaeMotor =
      new TelemetryCANSparkFlex(
          Constants.AlgaeConstants.ALGAE_MOTOR_ID,
          SparkLowLevel.MotorType.kBrushless, // no more CANSparkLowLevel?
          "algae/motor",
          Constants.MiscConstants.TUNING_MODE);

  private final TunableTelemetryPIDController algaePID =
      new TunableTelemetryPIDController("algae/pid", Constants.AlgaeConstants.PID_GAINS);
  private SimpleMotorFeedforward algaeFF = Constants.AlgaeConstants.FF_GAINS.createFeedforward();
  private final DigitalInput intakeSwitchRight =
      new DigitalInput(Constants.AlgaeConstants.SWITCH_ID_RIGHT);
  public Alert algaeMotorAlert = new Alert("Algae motor not doing so well", AlertType.ERROR);
  SlewRateLimiter limiter =
      new SlewRateLimiter(Constants.AlgaeConstants.RATE_LIMIT); // deal with later
  private RelativeEncoder algaeEncoder;
  private EventTelemetryEntry algaeEvent = new EventTelemetryEntry("AlgaeMotor/Event");

  private final SysIdRoutine algaeSysId =
      new SysIdRoutine(
          new SysIdRoutine.Config(Volts.per(Second).of(.5), Volts.of(2), null, null),
          new SysIdRoutine.Mechanism((voltage) -> setVoltage(voltage.in(Volts)), null, this));

  public AlgaeSubsystem() {
    configMotor();
    setDefaultCommand(setVoltageCommand(0.0).ignoringDisable(true).withName("Default Algae"));
  }

  public void configMotor() {
    algaeEncoder = algaeMotor.getEncoder();
    double conversionFactor = Math.PI * 2 / Constants.AlgaeConstants.GEAR_RATIO;

    StringFaultRecorder faultRecorder = new StringFaultRecorder();
    SparkFlexConfig config = new SparkFlexConfig();

    ConfigurationUtils.applyCheckRecordRev(
        () -> algaeMotor.setCANTimeout(250),
        () -> true,
        faultRecorder.run("CAN timeout"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        () ->
            algaeMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters),
        () -> true,
        faultRecorder.run("Factory defaults"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecord(
        () ->
            config.smartCurrentLimit(
                Constants.AlgaeConstants.STALL_MOTOR_CURRENT,
                Constants.AlgaeConstants.FREE_MOTOR_CURRENT),
        () -> true,
        faultRecorder.run("Current limits"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecord(
        () -> config.inverted(Constants.AlgaeConstants.INVERTED),
        () -> algaeMotor.configAccessor.getInverted() == Constants.AlgaeConstants.INVERTED,
        faultRecorder.run("Inverted"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecord(
        () -> config.idleMode(IdleMode.kCoast),
        () -> algaeMotor.configAccessor.getIdleMode() == SparkFlexConfig.IdleMode.kCoast,
        faultRecorder.run("Idle mode"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecord(
        () -> config.encoder.positionConversionFactor(conversionFactor / 60),
        () ->
            ConfigurationUtils.fpEqual(
                algaeMotor.configAccessor.encoder.getVelocityConversionFactor(),
                conversionFactor / 60),
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
    return intakeSwitchRight.get();
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
    // This method will be called once per scheduler run
  }
}
