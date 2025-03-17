// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.*;

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
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;
import frc.robot.utils.ConfigurationUtils;
import frc.robot.utils.ConfigurationUtils.StringFaultRecorder;

// @Logged
public class IntakeSpinningSubsystem extends SubsystemBase {

  private final TelemetryCANSparkFlex intakeSpinningMotor =
      new TelemetryCANSparkFlex(
          IntakeConstants.SPINNING_MOTOR_ID,
          SparkLowLevel.MotorType.kBrushless,
          "/intake/spinning/motor",
          MiscConstants.TUNING_MODE);

  private final Alert intakeSpinningMotorAlert =
      new Alert("intake spinning motor had a fault", AlertType.ERROR);
  private final SlewRateLimiter rateLimiter = new SlewRateLimiter(IntakeConstants.RATE_LIMIT);
  private RelativeEncoder intakeSpinningEncoder = intakeSpinningMotor.getEncoder();
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
    intakeSpinningEncoder = intakeSpinningMotor.getEncoder();
    double conversionFactor = Math.PI * 2 / Constants.IntakeConstants.GEAR_RATIO_SPINNING;

    StringFaultRecorder faultRecorder = new StringFaultRecorder();
    SparkFlexConfig config = new SparkFlexConfig();

    ConfigurationUtils.applyCheckRecordRev(
        () -> intakeSpinningMotor.setCANTimeout(250),
        () -> true,
        faultRecorder.run("CAN timeout"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        () ->
            intakeSpinningMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters),
        () -> true,
        faultRecorder.run("Factory defaults"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecord(
        () ->
            config.smartCurrentLimit(
                Constants.IntakeConstants.STALL_MOTOR_CURRENT,
                Constants.IntakeConstants.FREE_MOTOR_CURRENT),
        () -> true,
        faultRecorder.run("Current limits"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecord(
        () -> config.inverted(Constants.IntakeConstants.INVERTED_SPINNING),
        () ->
            intakeSpinningMotor.configAccessor.getInverted()
                == Constants.IntakeConstants.INVERTED_SPINNING,
        faultRecorder.run("Inverted"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecord(
        () -> config.idleMode(IdleMode.kCoast),
        () -> intakeSpinningMotor.configAccessor.getIdleMode() == SparkFlexConfig.IdleMode.kCoast,
        faultRecorder.run("Idle mode"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecord(
        () -> config.encoder.positionConversionFactor(conversionFactor / 60),
        () ->
            ConfigurationUtils.fpEqual(
                intakeSpinningMotor.configAccessor.encoder.getVelocityConversionFactor(),
                conversionFactor / 60),
        faultRecorder.run("Velocity conversion factor"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        intakeSpinningMotor::burnFlashWithDelay,
        () -> true,
        faultRecorder.run("Burn flash"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.postDeviceConfig(
        faultRecorder.hasFault(),
        intakeSpinningEvent::append,
        "intake spinning motor",
        faultRecorder.getFaultString());
    intakeSpinningMotorAlert.set(faultRecorder.hasFault());
  }

  public void setVoltage(double voltage) {
    intakeSpinningMotor.setVoltage(voltage);
  }

  public boolean getSwitchValue() {
    return !intakeSlapdownSwitchLeft.get();
  }

  public double getVelocity() {
    return intakeSpinningEncoder.getVelocity();
  }

  public Command setVoltageCommand(double voltage) {
    return this.run(() -> intakeSpinningMotor.setVoltage(voltage));
  }

  public Command runVelocityCommand(double setpointRadiansSecond) {
    return this.run(
            () -> {
              double rateLimited = rateLimiter.calculate(setpointRadiansSecond);
              setVoltage(
                  intakeSpinningPID.calculate(intakeSpinningEncoder.getVelocity(), rateLimited)
                      + intakeSpinningFF.calculate(rateLimited));
            })
        .beforeStarting(() -> rateLimiter.reset(intakeSpinningEncoder.getVelocity()))
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
