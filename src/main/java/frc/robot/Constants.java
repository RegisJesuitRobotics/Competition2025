// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;

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
  public static class VisionConstants {
    public static final double CAMERA_MOUNT_ANGLE = 0.0;
    public static final double CAMERA_MOUNT_HEIGHT_INCHES = 0.0;

    public static final double CORAL_HEIGHT = 0.0;
  }
}
