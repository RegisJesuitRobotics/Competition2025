// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.*;

import java.lang.ModuleLayer.Controller;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MiscConstants;
import frc.robot.telemetry.tunable.TunableTelemetryProfiledPIDController;
import frc.robot.telemetry.types.BooleanTelemetryEntry;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.types.EventTelemetryEntry;
import frc.robot.telemetry.wrappers.TelemetryTalonFX;
import frc.robot.utils.Alert;
import frc.robot.utils.ConfigEquality;
import frc.robot.utils.ConfigurationUtils;

@Logged
public class IntakeRotationSubsystem extends SubsystemBase {

  public static final TelemetryTalonFX intakeRotationMotor =
      new TelemetryTalonFX(
          IntakeConstants.ROTATION_MOTOR_ID,
          "motor/intake/rotation",
          Constants.MiscConstants.CANIVORE_NAME,
          MiscConstants.TUNING_MODE);

  private final SysIdRoutine intakeRotationSysId =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.per(Second).of(0.5),
              Volts.of(3),
              Seconds.of(5),
              (state) -> SignalLogger.writeString("slapdown", state.toString())),
          new SysIdRoutine.Mechanism(
              (voltage) -> setRotationVoltage(voltage.in(Volts)),
              null, // No log consumer, since data is recorded by URCL
              this));

  private static final Alert rotationIntakeMotorAlert =
      new Alert("Intake rotation motor had a fault initializing", Alert.AlertType.ERROR);
  private final TunableTelemetryProfiledPIDController rotationPid =
      new TunableTelemetryProfiledPIDController(
          "profiled/pid/intake",
          Constants.IntakeConstants.ROTATION_PID_GAINS,
          Constants.IntakeConstants.ROTATION_TRAP_GAINS);
  private EventTelemetryEntry intakeRotationEntry =
      new EventTelemetryEntry("intake/rotation/entry");
  private final ArmFeedforward rotationFF =
      IntakeConstants.ROTATION_FF_GAINS.createArmFeedforward();
  private boolean isHomed = false;
  private boolean isHoming = false;
  private DoubleTelemetryEntry position = new DoubleTelemetryEntry("/intake/position", true);
  private final Debouncer debouncer = new Debouncer(0.5);
  private final DigitalInput rotationLimitSwitch =
      new DigitalInput(IntakeConstants.ROTATION_LIMIT_SWITCH_ID);
  private final BooleanTelemetryEntry rotationSwitchEntry = new BooleanTelemetryEntry("/intake/switch", true);
  private final DoubleTelemetryEntry rotationGoal = new DoubleTelemetryEntry("/intake/goal", true);
  private final DoubleTelemetryEntry voltageGoal = new DoubleTelemetryEntry("/intake/goalVolt", true);
  private final BooleanTelemetryEntry atGoal = new BooleanTelemetryEntry("intake/atGoal", true);
  private final  BooleanTelemetryEntry homed = new BooleanTelemetryEntry("/intake/Homed", true);

  public IntakeRotationSubsystem() {
    configMotor();
    setDefaultCommand(setVoltageCommand(0.0).withName("IntakeRotationDefault"));
    rotationPid.setTolerance(Units.degreesToRadians(10));
    intakeRotationMotor.setPosition(0);
  }

  private void configMotor() {
    TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
    motorConfiguration.CurrentLimits.SupplyCurrentLimit =
        Constants.IntakeConstants.SUPPLY_CURRENT_LIMIT_ROTATION;
    motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfiguration.MotorOutput.Inverted = Constants.IntakeConstants.INVERTED_ROTATION;
    motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfiguration.Audio.AllowMusicDurDisable = true;
    ConfigurationUtils.StringFaultRecorder faultRecorder =
        new ConfigurationUtils.StringFaultRecorder();
    ConfigurationUtils.applyCheckRecordCTRE(
        () -> intakeRotationMotor.getConfigurator().apply(motorConfiguration),
        () -> {
          TalonFXConfiguration appliedConfig = new TalonFXConfiguration();
          intakeRotationMotor.getConfigurator().refresh(appliedConfig);
          return ConfigEquality.isTalonConfigurationEqual(motorConfiguration, appliedConfig);
        },
        faultRecorder.run("Motor configuration"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordCTRE(
        intakeRotationMotor::optimizeBusUtilization,
        () -> true,
        faultRecorder.run("Optimize bus utilization"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);

    ConfigurationUtils.postDeviceConfig(
        faultRecorder.hasFault(),
        intakeRotationEntry::append,
        "intake rotation motor fault",
        faultRecorder.getFaultString());
    rotationIntakeMotorAlert.set(faultRecorder.hasFault());


    intakeRotationMotor.setLoggingPositionConversionFactor(
        Constants.IntakeConstants.GEAR_RATIO_ROTATION);
    intakeRotationMotor.setLoggingVelocityConversionFactor(
        Constants.IntakeConstants.GEAR_RATIO_ROTATION);

    // Clear reset as this is on startup
    intakeRotationMotor.hasResetOccurred();
  }

  public void setRotationVoltage(double voltage) {
    intakeRotationMotor.setVoltage(voltage);
  }

  private double getPosition() {
    
        return Units.rotationsToRadians(intakeRotationMotor.getPosition().getValueAsDouble() / 24.0);
  }

  private boolean atLimit() {
    return !rotationLimitSwitch.get();
  }

  public boolean isHomed() {
    return isHomed;
  }

  public boolean atGoal(){
    return rotationPid.atGoal();


  }

  public Command setRotationGoalCommand(Rotation2d goal) {
    return this.run(
            () -> {
              double feedbackOutput = rotationPid.calculate(getPosition());
              TrapezoidProfile.State currentSetpoint = rotationPid.getSetpoint();

              setRotationVoltage(
                  feedbackOutput
                      + rotationFF.calculate(currentSetpoint.position, currentSetpoint.velocity));
            })
        .beforeStarting(
            () -> {
              rotationPid.setGoal(goal.getRadians());
              rotationPid.reset(
                  getPosition(), intakeRotationMotor.getVelocity().getValueAsDouble());
            })
        .onlyIf(this::isHomed)
        .withName("SetIntakeRotationGoal");
  }

  public Command setVoltageCommand(double voltage) {
    return this.run(() -> setRotationVoltage(voltage)).withName("SetIntakeRotationVoltage");
  }

  public Command homeIntakeCommand() {
    return setVoltageCommand(-0.5)
        .until(this::isHomed)
        .beforeStarting(
            () -> {
              isHoming = true;
              isHomed = false;
            })
        .finallyDo(() -> isHoming = false)
        .withName("HomeIntakeRotation");
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return intakeRotationSysId.quasistatic(direction).beforeStarting(SignalLogger::start);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return intakeRotationSysId.dynamic(direction).beforeStarting(SignalLogger::start);
  }

  public Command addInstrumentCommand(Orchestra orchestra){
    return this.run(() -> orchestra.addInstrument(intakeRotationMotor));
  }

  @Override
  public void periodic() {
    if ((atLimit() && isHoming) || debouncer.calculate(atLimit())) {
      intakeRotationMotor.setPosition(IntakeConstants.ROTATION_UP_ANGLE);
      isHomed = true;
    }
    rotationSwitchEntry.append(atLimit());
    position.append(getPosition());
    rotationGoal.append(rotationPid.getGoal().position);
    voltageGoal.append(intakeRotationMotor.getMotorVoltage().getValueAsDouble());
    atGoal.append(rotationPid.atGoal());
    homed.append(isHomed);
  }
}
