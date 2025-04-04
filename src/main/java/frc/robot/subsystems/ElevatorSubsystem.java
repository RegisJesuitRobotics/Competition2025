package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.telemetry.tunable.TunableTelemetryProfiledPIDController;
import frc.robot.telemetry.types.BooleanTelemetryEntry;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.types.EventTelemetryEntry;
import frc.robot.telemetry.wrappers.TelemetryTalonFX;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;
import frc.robot.utils.ConfigEquality;
import frc.robot.utils.ConfigurationUtils;
import java.util.function.DoubleSupplier;

// @Logged
public class ElevatorSubsystem extends SubsystemBase {
  private final TelemetryTalonFX leftElevatorMotor = new TelemetryTalonFX(
      Constants.ElevatorConstants.LEFT_ID,

      "/elevator/motorleft",
      Constants.MiscConstants.CANIVORE_NAME,
      Constants.MiscConstants.TUNING_MODE);
  private final SysIdRoutine elevatorSysId = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.per(Second).of(.5),
          Volts.of(5),
          null,
          (state) -> SignalLogger.writeString("elevator", state.toString())),
      new SysIdRoutine.Mechanism((voltage) -> setVoltage(voltage.in(Volts)), null, this));
  private final TelemetryTalonFX rightElevatorMotor = new TelemetryTalonFX(
      Constants.ElevatorConstants.RIGHT_ID,
      "/elevator/motorright",
      Constants.MiscConstants.CANIVORE_NAME,
      Constants.MiscConstants.TUNING_MODE);
  private final Alert rightMotorAlert = new Alert("right elevator motor fault", AlertType.ERROR);
  private final Alert leftMotorAlert = new Alert("left elevator motor fault", AlertType.ERROR);
  private final DoubleTelemetryEntry positionGoal = new DoubleTelemetryEntry("/elevator/positionGoal", true);
  private final DigitalInput bottomSwitch = new DigitalInput(Constants.ElevatorConstants.BOTTOM_ID);
  private final EventTelemetryEntry rightEventEntry = new EventTelemetryEntry("/elevator/motorright/events");
  private final EventTelemetryEntry leftEventEntry = new EventTelemetryEntry("/elevator/motorleft/events");
  private final Debouncer debouncer = new Debouncer(0.5);
  private final TunableTelemetryProfiledPIDController controller = new TunableTelemetryProfiledPIDController(
      "/elevator/controller",
      Constants.ElevatorConstants.PID_GAINS,
      Constants.ElevatorConstants.TRAP_GAINS);
  private final ElevatorFeedforward FF = Constants.ElevatorConstants.FF.createElevatorFeedforward();
  private final DoubleTelemetryEntry elevatorPosition = new DoubleTelemetryEntry("/elevator/position", true);
  private final DoubleTelemetryEntry elevatorGoal = new DoubleTelemetryEntry("/elevator/goalPos", true);
  private final BooleanTelemetryEntry topSwitch = new BooleanTelemetryEntry("/elevator/top",
      true);
  private final BooleanTelemetryEntry homed = new BooleanTelemetryEntry("/elevator/homed", true);
  private boolean isHomed = false;
  private boolean isHoming = false;

  public ElevatorSubsystem() {
    configRightMotor();
    configLeftMotor();
    controller.setTolerance(Units.inchesToMeters(.1));
    SmartDashboard.putData(forceHomeCommand().withName("Force Home"));
  }

  private void configRightMotor() {
    TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
    motorConfiguration.CurrentLimits.SupplyCurrentLimit = Constants.ElevatorConstants.SUPPLY_CURRENT_LIMIT;
    motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfiguration.MotorOutput.Inverted = Constants.ElevatorConstants.INVERTED_RIGHT;
    motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfiguration.Audio.AllowMusicDurDisable = true;

    ConfigurationUtils.StringFaultRecorder faultRecorder = new ConfigurationUtils.StringFaultRecorder();
    ConfigurationUtils.applyCheckRecordCTRE(
        () -> rightElevatorMotor.getConfigurator().apply(motorConfiguration),
        () -> {
          TalonFXConfiguration appliedConfig = new TalonFXConfiguration();
          rightElevatorMotor.getConfigurator().refresh(appliedConfig);
          return ConfigEquality.isTalonConfigurationEqual(motorConfiguration, appliedConfig);
        },
        faultRecorder.run("Motor configuration 1"),
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

    rightElevatorMotor.setLoggingPositionConversionFactor(Constants.ElevatorConstants.METERS_PER_REVOLUTION);
    rightElevatorMotor.setLoggingVelocityConversionFactor(Constants.ElevatorConstants.METERS_PER_REVOLUTION);

    // Clear reset as this is on startup
    rightElevatorMotor.hasResetOccurred();
  }

  private void configLeftMotor() {
    TalonFXConfiguration leftMotorConfiguration = new TalonFXConfiguration();
    leftMotorConfiguration.CurrentLimits.SupplyCurrentLimit = Constants.ElevatorConstants.SUPPLY_CURRENT_LIMIT;
    leftMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    leftMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftMotorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    ConfigurationUtils.StringFaultRecorder leftFaultRecorder = new ConfigurationUtils.StringFaultRecorder();
    ConfigurationUtils.applyCheckRecordCTRE(
        () -> leftElevatorMotor.getConfigurator().apply(leftMotorConfiguration),
        () -> {
          TalonFXConfiguration appliedConfig = new TalonFXConfiguration();
          leftElevatorMotor.getConfigurator().refresh(appliedConfig);
          return ConfigEquality.isTalonConfigurationEqual(leftMotorConfiguration, appliedConfig);
        },
        leftFaultRecorder.run("Motor configuration 2"),
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
    leftMotorAlert.set(leftFaultRecorder.hasFault());

    leftElevatorMotor.setLoggingPositionConversionFactor(
        Constants.ElevatorConstants.METERS_PER_REVOLUTION);
    leftElevatorMotor.setLoggingVelocityConversionFactor(
        Constants.ElevatorConstants.METERS_PER_REVOLUTION);
    leftElevatorMotor.setControl(new Follower(Constants.ElevatorConstants.RIGHT_ID, true));
    // Clear reset as this is on startup
    leftElevatorMotor.hasResetOccurred();
  }

  public double getElevatorPosition() {
    return leftElevatorMotor.getPosition().getValueAsDouble()
        * Constants.ElevatorConstants.METERS_PER_REVOLUTION;
  }

  public Command forceHomeCommand() {
    return Commands.runOnce(
        () -> {
          isHomed = true;
          leftElevatorMotor.setPosition(
              Constants.ElevatorConstants.FORCE_HOME
                  / Constants.ElevatorConstants.METERS_PER_REVOLUTION);
        })
        .ignoringDisable(true);
  }

  public void setVoltage(double volts) {
    rightElevatorMotor.setVoltage(volts);
  }

  public double getVelocityActual() {

    return leftElevatorMotor.getVelocity().getValueAsDouble()
        * Constants.ElevatorConstants.METERS_PER_REVOLUTION;
  }

  public Command setVoltageCommand(double volts) {
    return this.run(() -> setVoltage(volts));
  }

  public boolean atGoal() {
    return controller.atGoal();
  }

  // public boolean atLimit() {
  // return bottomSwitch.get();
  // }

  public boolean isHomed() {
    return isHomed;
  }

  public Command setPosition(double position) {
    return setPosition(() -> position);
  }

  public Command setPosition(DoubleSupplier position) {
    return this.run(
        () -> {
          double positionClamped = MathUtil.clamp(position.getAsDouble(), 0,
              Constants.ElevatorConstants.L4_REEF + Units.inchesToMeters(1.0));
          controller.setGoal(positionClamped);
          double feedback = controller.calculate(getElevatorPosition());
          TrapezoidProfile.State currentSetpoint = controller.getSetpoint();
          setVoltage(feedback + FF.calculate(currentSetpoint.velocity));
        })
        .beforeStarting(
            () -> controller.reset(
                getElevatorPosition(), getVelocityActual()))
        .onlyIf(() -> isHomed);
  }

  public Command homeElevatorCommand() {
    return setVoltageCommand(-2.0)
        .until(this::isHomed)
        .beforeStarting(
            () -> {
              isHoming = true;
              isHomed = false;
            })
        .finallyDo(() -> isHoming = false)
        .withName("HomeElevator");
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return elevatorSysId.quasistatic(direction).beforeStarting(SignalLogger::start);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return elevatorSysId.dynamic(direction).beforeStarting(SignalLogger::start);
  }

  public Command addInstrumentCommand(Orchestra orchestra) {
    return this.run(() -> orchestra.addInstrument(rightElevatorMotor))
        .alongWith(this.run(() -> orchestra.addInstrument(leftElevatorMotor)));
  }

  @Override
  public void periodic() {
    if ((!bottomSwitch.get() && isHoming) || debouncer.calculate(!bottomSwitch.get())) {
      isHomed = true;
      leftElevatorMotor.setPosition(0.0);
    }
    // if (controller.getGoal().position >
    // Units.inchesToMeters(Constants.ElevatorConstants.L4_REEF)){
    // controller.reset(getElevatorPosition());
    // }

    if (controller.getSetpoint().position > Constants.ElevatorConstants.L4_REEF) {
      controller.reset(getElevatorPosition(), getVelocityActual());
    }

    rightElevatorMotor.logValues();
    leftElevatorMotor.logValues();
    elevatorPosition.append(getElevatorPosition());
    elevatorGoal.append(controller.getSetpoint().position);
    positionGoal.append(controller.getGoal().position);
    topSwitch.append(!bottomSwitch.get());
    homed.append(isHomed());
    SignalLogger.writeDouble("elevatorPosition", getElevatorPosition());
  }
}
