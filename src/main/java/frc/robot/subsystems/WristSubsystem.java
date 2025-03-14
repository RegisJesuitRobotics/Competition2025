// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;
import frc.robot.telemetry.tunable.TunableTelemetryProfiledPIDController;
import frc.robot.telemetry.types.EventTelemetryEntry;
import frc.robot.telemetry.wrappers.TelemetryTalonFX;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;
import frc.robot.utils.ConfigEquality;
import frc.robot.utils.ConfigurationUtils;
import java.util.function.DoubleSupplier;

@Logged
public class WristSubsystem extends SubsystemBase {

  private final TelemetryTalonFX wristMotor =
      new TelemetryTalonFX(
          Constants.WristConstants.WRIST_ID,
          "wrist/wristMotor",
          Constants.MiscConstants.TUNING_MODE);

  private final SysIdRoutine wristSysId =
      new SysIdRoutine(
          new SysIdRoutine.Config(Volts.per(Second).of(.5), Volts.of(2), null, null),
          new SysIdRoutine.Mechanism((voltage) -> setVoltage(voltage.in(Volts)), null, this));

  private final TunableTelemetryProfiledPIDController wristpid =
      new TunableTelemetryProfiledPIDController(
          "wrist/profiledpid", WristConstants.WRIST_PID_GAINS, WristConstants.WRIST_TRAP_GAINS);
  private final ArmFeedforward wristff =
      Constants.WristConstants.WRIST_FF_GAINS.createArmFeedforward();
  private final DutyCycleEncoder wristEncoder =
      new DutyCycleEncoder(WristConstants.WRIST_ENCODER_PORT);
  private final Alert wristAlert = new Alert("wrist died", AlertType.ERROR);
  private final EventTelemetryEntry wristEventEntry = new EventTelemetryEntry("wrist/motor/events");

  public WristSubsystem() {
    configMotor();
    wristpid.setTolerance(Units.degreesToRadians(WristConstants.PID_TOLERANCE));
    setDefaultCommand(setVoltageCommand(0));
  }

  private void configMotor() {
    TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
    motorConfiguration.CurrentLimits.SupplyCurrentLimit =
        Constants.WristConstants.SUPPLY_CURRENT_LIMIT;
    motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfiguration.MotorOutput.Inverted = Constants.WristConstants.INVERTED;
    motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfiguration.Audio.AllowMusicDurDisable = true;
    ConfigurationUtils.StringFaultRecorder faultRecorder =
        new ConfigurationUtils.StringFaultRecorder();
    ConfigurationUtils.applyCheckRecordCTRE(
        () -> wristMotor.getConfigurator().apply(motorConfiguration),
        () -> {
          TalonFXConfiguration appliedConfig = new TalonFXConfiguration();
          wristMotor.getConfigurator().refresh(appliedConfig);
          return ConfigEquality.isTalonConfigurationEqual(motorConfiguration, appliedConfig);
        },
        faultRecorder.run("Motor configuration"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordCTRE(
        wristMotor::optimizeBusUtilization,
        () -> true,
        faultRecorder.run("Optimize bus utilization"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);

    ConfigurationUtils.postDeviceConfig(
        faultRecorder.hasFault(),
        wristEventEntry::append,
        "wrist motor fault",
        faultRecorder.getFaultString());
    wristAlert.set(faultRecorder.hasFault());

    wristMotor.setLoggingPositionConversionFactor(Constants.WristConstants.GEAR_RATIO);
    wristMotor.setLoggingVelocityConversionFactor(Constants.WristConstants.GEAR_RATIO);

    // Clear reset as this is on startup
    wristMotor.hasResetOccurred();
  }

  public void setVoltage(double voltage) {
    wristMotor.setVoltage(voltage);
  }

  public boolean atGoal() {
    return wristpid.atGoal();
  }

  public Command setVoltageCommand(double voltage) {
    return this.run(() -> setVoltage(voltage)).withName("wristVoltage :P");
  }

  public double getPosition() {
    return MathUtil.angleModulus(
        Units.rotationsToRadians(wristEncoder.get()) + WristConstants.WRIST_OFFSET);
    // should be just .get() this year instead of .getAbsolutePosition()
  }

  public Command setPositionCommand(double desiredPositionRadians) {
    return setPositionCommand(() -> desiredPositionRadians);
  }

  public Command setPositionCommand(DoubleSupplier desiredPositionRadians) {
    return this.run(
            () -> {
              wristpid.setGoal(desiredPositionRadians.getAsDouble());
              double feedbackOutput = wristpid.calculate(getPosition());
              TrapezoidProfile.State currentSetpoint = wristpid.getSetpoint();

              setVoltage(
                  feedbackOutput
                      + wristff.calculate(currentSetpoint.position, currentSetpoint.velocity));
            })
        .beforeStarting(
            () -> wristpid.reset(getPosition(), wristMotor.getVelocity().getValueAsDouble()))
        .withName("SetWristPosition");
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return wristSysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return wristSysId.dynamic(direction);
  }

  @Override
  public void periodic() {
    // wristMotor.logValues(); maybe if we need it
    // This method will be called once per scheduler run
  }
}
