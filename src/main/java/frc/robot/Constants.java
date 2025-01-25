// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
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

  public static class AlgaeConstants{
    public static final int ALGAE_MOTOR_ID = 13;
    public static final int STALL_MOTOR_CURRENT = 3; //m
    public static final int FREE_MOTOR_CURRENT = 3; //m
    public static final double SHOOTER_GEAR_RATIO = 3; //m
    public static final boolean INVERTED = true; //m

    public static final TunablePIDGains PID_GAINS = new TunablePIDGains(
      "algae/pid", 0, 0.0, 0.0, MiscConstants.TUNING_MODE
    );
    public static final TunableTrapezoidalProfileGains TRAP_GAINS = new TunableTrapezoidalProfileGains(
      "algae/trapezoidal profile", 0.0, 0.0, MiscConstants.TUNING_MODE);
    public static final TunableFFGains FF_GAINS = new TunableFFGains(
      "algae/ff", 0.0, 0.0, 0.0, MiscConstants.TUNING_MODE);
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
  }
}
