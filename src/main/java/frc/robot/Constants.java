// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.telemetry.tunable.gains.TunableArmElevatorFFGains;
import frc.robot.telemetry.tunable.gains.TunableFFGains;
import frc.robot.telemetry.tunable.gains.TunablePIDGains;
import frc.robot.telemetry.tunable.gains.TunableTrapezoidalProfileGains;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class ElevatorConstants {
    public static final int LEFT_ID = 0;
    public static final int RIGHT_ID = 0;
    public static final int BOTTOM_ID = 0;
    public static final double SUPPLY_CURRENT_LIMIT = 0.0;
    public static final InvertedValue INVERTED_RIGHT = InvertedValue.Clockwise_Positive;
    public static final double GEAR_RATIO = 1.0;
    // TODO: inches of gear
    public static final double METERS_PER_REVOLUTION =
        (Math.PI * Units.inchesToMeters(0.0)) / GEAR_RATIO;
    public static final boolean LEFT_INVERTED = false;
    public static final TunablePIDGains PID_GAINS =
        new TunablePIDGains("/elevator/PID", 0, 0, 0, MiscConstants.TUNING_MODE);
    public static final TunableTrapezoidalProfileGains TRAP_GAINS =
        new TunableTrapezoidalProfileGains("/elevator/trap", 0, 0, MiscConstants.TUNING_MODE);
    public static final TunableFFGains FF =
        new TunableFFGains("/elevator/ff", 0, 0, 0, MiscConstants.TUNING_MODE);
    public static final double LOW = 0.0;
    public static final double HIGH = 0.0;
  }
  ;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class MiscConstants {
    public static final String CANIVORE_NAME = "canivore";

    private MiscConstants() {}

    public static final int[] USED_CONTROLLER_PORTS = {0, 1};
    public static final boolean TUNING_MODE = !DriverStation.isFMSAttached();

    public static final int CONFIGURATION_ATTEMPTS = 10;
  }

  public static class VisionConstants {
    public static final double CAMERA_MOUNT_ANGLE = 0.0;
    public static final double CAMERA_MOUNT_HEIGHT_METERS = 0.0;

    public static final double CORAL_HEIGHT = 0.0;
    public static final String APRIL_LIMELIGHT = "limelight1";
    public static final String OBJECT_LIMELIGHT = "limelight2";
    public static final double CONFIDENCE_THRESHOLD = 80.0;
  }

  public static class AutoConstants {
    public static final TunablePIDGains pointTranslationGains =
        new TunablePIDGains(
            "/drive/gains/pointTranslationController", 0, 0, 0, MiscConstants.TUNING_MODE);
    public static final TunableTrapezoidalProfileGains trapPointTranslationGains =
        new TunableTrapezoidalProfileGains(
            "/drive/gains/trapPointTranslationController", 0, 0, MiscConstants.TUNING_MODE);
    public static final TunableFFGains pointTranslationFFGains =
        new TunableFFGains("/drive/gains/pointFFController", 0, 0, 0, MiscConstants.TUNING_MODE);
    public static final double MAX_VELOCITY = 0.0;
    public static final double MAX_ACCELERATION = 0.0;
    public static final double MAX_ANGULAR_VELOCITY = 0.0;
    public static final double MAX_ANGULAR_ACCELERATION = 0.0;
    public static final double NOMINAL_VOLTAGE = 12.0;
  }

  public static class WristConstants {
    public static final int WRIST_ID = 11;
    public static final double GEAR_RATIO = 0; // idk
    public static final double SUPPLY_CURRENT_LIMIT = 0.0; // idk
    public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;
    public static final double WRIST_OFFSET = 0.0; // needs a value
    public static final double PID_TOLERANCE = 2.0; // idk
    public static final int WRIST_ENCODER_PORT = 0; // i also do not know
    // a lot of 0s
    public static final TunablePIDGains WRIST_PID_GAINS =
        new TunablePIDGains("/pid/wrist/", 0, 0.0, 0.0, MiscConstants.TUNING_MODE);

    public static final TunableTrapezoidalProfileGains WRIST_TRAP_GAINS =
        new TunableTrapezoidalProfileGains(
            "/trapezoidalprofile/wrist",
            Units.rotationsToRadians(0),
            Units.rotationsToRadians(0),
            MiscConstants.TUNING_MODE);
    public static final double DYNAMIC_OFFSET = Units.degreesToRadians(0);

    public static final TunableArmElevatorFFGains WRIST_FF_GAINS =
        new TunableArmElevatorFFGains("/feedfoward/wrist/", 0, 0, 0, 0, MiscConstants.TUNING_MODE);
  }
}
