package frc.robot.telemetry.wrappers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.types.IntegerTelemetryEntry;
import frc.robot.utils.ConfigurationUtils;
import java.util.List;

public class TelemetryTalonFX extends TalonFX {
  private final StatusSignal<Current> outputAmpsSignal;
  private final StatusSignal<Current> inputAmpsSignal;
  private final StatusSignal<Double> outputPercentSignal;
  private final StatusSignal<Temperature> temperatureSignal;
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Integer> faultsSignal;
  private final StatusSignal<Integer> stickyFaultsSignal;

  private final DoubleTelemetryEntry outputAmpsEntry;
  private final DoubleTelemetryEntry inputAmpsEntry;
  private final DoubleTelemetryEntry outputPercentEntry;
  private final DoubleTelemetryEntry temperatureEntry;
  private final DoubleTelemetryEntry positionEntry;
  private final DoubleTelemetryEntry velocityEntry;
  private final IntegerTelemetryEntry faultsEntry;
  private final IntegerTelemetryEntry stickyFaultsEntry;

  private double loggingPositionConversionFactor = 1.0;
  private double loggingVelocityConversionFactor = 1.0;

  public TelemetryTalonFX(
      int deviceNumber, String telemetryPath, String canbus, boolean tuningMode) {
    super(deviceNumber, canbus);

    outputAmpsSignal = super.getStatorCurrent();
    inputAmpsSignal = super.getSupplyCurrent();
    outputPercentSignal = super.getDutyCycle();
    temperatureSignal = super.getDeviceTemp();
    positionSignal = super.getPosition();
    velocitySignal = super.getVelocity();
    faultsSignal = super.getFaultField();
    stickyFaultsSignal = super.getStickyFaultField();

    // Set the update frequency to the default for all
    List.of(
            outputAmpsSignal,
            inputAmpsSignal,
            outputPercentSignal,
            temperatureSignal,
            positionSignal,
            velocitySignal,
            faultsSignal,
            stickyFaultsSignal)
        .forEach(ConfigurationUtils::explicitlySetSignalFrequency);

    telemetryPath += "/";
    outputAmpsEntry = new DoubleTelemetryEntry(telemetryPath + "outputAmps", tuningMode);
    inputAmpsEntry = new DoubleTelemetryEntry(telemetryPath + "inputAmps", tuningMode);
    outputPercentEntry = new DoubleTelemetryEntry(telemetryPath + "outputPercent", tuningMode);
    temperatureEntry = new DoubleTelemetryEntry(telemetryPath + "temperature", tuningMode);
    positionEntry = new DoubleTelemetryEntry(telemetryPath + "position", tuningMode);
    velocityEntry = new DoubleTelemetryEntry(telemetryPath + "velocity", tuningMode);
    faultsEntry = new IntegerTelemetryEntry(telemetryPath + "faults", tuningMode);
    stickyFaultsEntry = new IntegerTelemetryEntry(telemetryPath + "stickyFaults", tuningMode);
  }

  public TelemetryTalonFX(int deviceNumber, String logTable, boolean tuningMode) {
    this(deviceNumber, logTable, "", tuningMode);
  }

  public void setLoggingPositionConversionFactor(double loggingPositionConversionFactor) {
    this.loggingPositionConversionFactor = loggingPositionConversionFactor;
  }

  public void setLoggingVelocityConversionFactor(double loggingVelocityConversionFactor) {
    this.loggingVelocityConversionFactor = loggingVelocityConversionFactor;
  }

  public void logValues() {
    BaseStatusSignal.refreshAll(
        outputAmpsSignal,
        inputAmpsSignal,
        outputPercentSignal,
        temperatureSignal,
        positionSignal,
        velocitySignal,
        faultsSignal,
        stickyFaultsSignal);

    outputAmpsEntry.append(outputAmpsSignal.getValueAsDouble());
    inputAmpsEntry.append(inputAmpsSignal.getValueAsDouble());
    outputPercentEntry.append(outputPercentSignal.getValue());
    temperatureEntry.append(temperatureSignal.getValueAsDouble());
    positionEntry.append(positionSignal.getValueAsDouble() * loggingPositionConversionFactor);
    velocityEntry.append(velocitySignal.getValueAsDouble() * loggingVelocityConversionFactor);
    faultsEntry.append(faultsSignal.getValue());
    stickyFaultsEntry.append(stickyFaultsSignal.getValue());
  }
}
