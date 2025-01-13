package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.telemetry.tunable.TunableTelemetryProfiledPIDController;
import frc.robot.telemetry.types.EventTelemetryEntry;
import frc.robot.telemetry.wrappers.TelemetryTalonFX;
import frc.robot.utils.ConfigEquality;
import frc.robot.utils.ConfigurationUtils;
import java.util.function.DoubleSupplier;

@Logged
public class ElevatorSubsystem extends SubsystemBase {
  private final TelemetryTalonFX leftElevatorMotor =
      new TelemetryTalonFX(
          Constants.ElevatorConstants.LEFT_ID,
          "/elevator/motorleft",
          Constants.MiscConstants.TUNING_MODE);
  private final TelemetryTalonFX rightElevatorMotor =
      new TelemetryTalonFX(
          Constants.ElevatorConstants.RIGHT_ID,
          "/elevator/motorright",
          Constants.MiscConstants.TUNING_MODE);
  private final Alert rightMotorAlert =
      new Alert("right elevator motor fault", Alert.AlertType.kError);
  private final Alert leftMotorAlert =
      new Alert("left elevator motor fault", Alert.AlertType.kError);
  private final DigitalInput bottomSwitch = new DigitalInput(Constants.ElevatorConstants.BOTTOM_ID);
  private final EventTelemetryEntry rightEventEntry =
      new EventTelemetryEntry("/elevator/motorright/events");
  private final EventTelemetryEntry leftEventEntry =
      new EventTelemetryEntry("/elevator/motorleft/events");
  private boolean isHomed = false;
  private final Debouncer debouncer = new Debouncer(0.5);
  private final TunableTelemetryProfiledPIDController controller =
      new TunableTelemetryProfiledPIDController(
          "/elevator/controller",
          Constants.ElevatorConstants.PID_GAINS,
          Constants.ElevatorConstants.TRAP_GAINS);
  private final SimpleMotorFeedforward FF = Constants.ElevatorConstants.FF.createFeedforward();

  private ElevatorSubsystem() {
    configMotors();
  }

  private void configMotors() {
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
        () -> rightElevatorMotor.getConfigurator().apply(motorConfiguration),
        () -> {
          TalonFXConfiguration appliedConfig = new TalonFXConfiguration();
          rightElevatorMotor.getConfigurator().refresh(appliedConfig);
          return ConfigEquality.isTalonConfigurationEqual(motorConfiguration, appliedConfig);
        },
        faultRecorder.run("Motor configuration"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordCTRE(
        rightElevatorMotor::optimizeBusUtilization,
        () -> true,
        faultRecorder.run("Optimize bus utilization"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);

    ConfigurationUtils.postDeviceConfig(
        faultRecorder.hasFault(),
        rightEventEntry::append,
        "right elevator motor fault",
        faultRecorder.getFaultString());
    rightMotorAlert.set(faultRecorder.hasFault());

    rightElevatorMotor.setLoggingPositionConversionFactor(Constants.ElevatorConstants.GEAR_RATIO);
    rightElevatorMotor.setLoggingVelocityConversionFactor(Constants.ElevatorConstants.GEAR_RATIO);

    // Clear reset as this is on startup
    rightElevatorMotor.hasResetOccurred();

    TalonFXConfiguration leftMotorConfiguration = new TalonFXConfiguration();
    leftMotorConfiguration.CurrentLimits.SupplyCurrentLimit =
        Constants.ElevatorConstants.SUPPLY_CURRENT_LIMIT;
    leftMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    leftMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    ConfigurationUtils.StringFaultRecorder leftFaultRecorder =
        new ConfigurationUtils.StringFaultRecorder();
    ConfigurationUtils.applyCheckRecordCTRE(
        () -> leftElevatorMotor.getConfigurator().apply(motorConfiguration),
        () -> {
          TalonFXConfiguration appliedConfig = new TalonFXConfiguration();
          leftElevatorMotor.getConfigurator().refresh(appliedConfig);
          return ConfigEquality.isTalonConfigurationEqual(motorConfiguration, appliedConfig);
        },
        leftFaultRecorder.run("Motor configuration"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordCTRE(
        leftElevatorMotor::optimizeBusUtilization,
        () -> true,
        leftFaultRecorder.run("Optimize bus utilization"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);

    ConfigurationUtils.postDeviceConfig(
        leftFaultRecorder.hasFault(),
        leftEventEntry::append,
        "left elevator motor fault",
        leftFaultRecorder.getFaultString());
    leftMotorAlert.set(faultRecorder.hasFault());

    leftElevatorMotor.setLoggingPositionConversionFactor(
        Constants.ElevatorConstants.METERS_PER_REVOLUTION);
    leftElevatorMotor.setLoggingVelocityConversionFactor(
        Constants.ElevatorConstants.METERS_PER_REVOLUTION);

    leftElevatorMotor.setControl(
        new Follower(
            Constants.ElevatorConstants.RIGHT_ID, Constants.ElevatorConstants.LEFT_INVERTED));
    // Clear reset as this is on startup
    leftElevatorMotor.hasResetOccurred();
  }

  @Logged
  public double getElevatorPosition() {
    return rightElevatorMotor.getPosition().getValueAsDouble()
        * Constants.ElevatorConstants.METERS_PER_REVOLUTION;
  }

  public void setVoltage(double volts) {
    rightElevatorMotor.setVoltage(volts);
  }

  @Logged
  public boolean atLimit() {
    return bottomSwitch.get();
  }

  public Command setPosition(DoubleSupplier position) {
    return this.run(
            () -> {
              double positionClamped =
                  MathUtil.clamp(
                      position.getAsDouble(),
                      Constants.ElevatorConstants.LOW,
                      Constants.ElevatorConstants.HIGH);
              controller.setGoal(positionClamped);
              double feedback = controller.calculate(getElevatorPosition());
              TrapezoidProfile.State currentSetpoint = controller.getSetpoint();
              setVoltage(feedback + FF.calculate(currentSetpoint.velocity));
            })
        .beforeStarting(
            () ->
                controller.reset(
                    getElevatorPosition(), rightElevatorMotor.getVelocity().getValueAsDouble()))
        .onlyIf(() -> isHomed);
  }

  @Override
  public void periodic() {
    if (debouncer.calculate(atLimit())) {
      isHomed = true;
      rightElevatorMotor.setPosition(0.0);
    }
    rightElevatorMotor.logValues();
    leftElevatorMotor.logValues();
  }
}
