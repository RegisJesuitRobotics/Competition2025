package frc.robot.telemetry.types;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class DoubleTelemetryEntry extends PrimitiveTelemetryEntry {
  private final DoubleLogEntry logEntry;
  private final DoublePublisher networkPublisher;
  private double lastValue;
  private double Voltage;

  public DoubleTelemetryEntry(String path, boolean shouldNT) {
    this(path, shouldNT, true);
  }

  public DoubleTelemetryEntry(String path, boolean shouldNT, boolean shouldLazyLog) {
    super(shouldLazyLog);

    logEntry = new DoubleLogEntry(DataLogManager.getLog(), path);
    if (shouldNT) {
      networkPublisher = NetworkTableInstance.getDefault().getDoubleTopic(path).publish();
      networkPublisher.setDefault(0.0);
    } else {
      networkPublisher = null;
    }
  }

  @Override
  public void close() {
    logEntry.finish();
    if (networkPublisher != null) {
      networkPublisher.close();
    }
  }

  public void append(Temperature value) {
    // TODO Auto-generated method stub
   // throw new UnsupportedOperationException("Unimplemented method 'append'");
  }

  public void append(Double value) {
    // TODO Auto-generated method stub
   // throw new UnsupportedOperationException("Unimplemented method 'append'");
  }

  public void append(Current value) {
    // TODO Auto-generated method stub
   // throw new UnsupportedOperationException("Unimplemented method 'append'");
  }

  public void append(Angle value) {
    // TODO Auto-generated method stub
    //throw new UnsupportedOperationException("Unimplemented method 'append'");
  }

  public void append(AngularVelocity value) {
    // TODO Auto-generated method stub
    //throw new UnsupportedOperationException("Unimplemented method 'append'");
  }

  public void append(LinearAcceleration value) {
    // TODO Auto-generated method stub
   // throw new UnsupportedOperationException("Unimplemented method 'append'");
  }

  public void append(Voltage value) {
    // TODO Auto-generated method stub
   // throw new UnsupportedOperationException("Unimplemented method 'append'");
  }
}
