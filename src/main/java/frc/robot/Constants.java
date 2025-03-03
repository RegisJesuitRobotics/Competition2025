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
    public static final int LEFT_ID = 9;
    public static final int RIGHT_ID = 10;
    public static final int BOTTOM_ID = 2; 
    public static final double SUPPLY_CURRENT_LIMIT = 20.0;
    public static final InvertedValue INVERTED_RIGHT = InvertedValue.Clockwise_Positive;
    public static final double GEAR_RATIO = 9;
    public static final double METERS_PER_REVOLUTION =
        (Math.PI * Units.inchesToMeters(2.2594)) / GEAR_RATIO;
    public static final boolean LEFT_INVERTED = false;
    public static final TunablePIDGains PID_GAINS =
        new TunablePIDGains("/elevator/PID", 0, 0, 0, MiscConstants.TUNING_MODE);
    public static final TunableTrapezoidalProfileGains TRAP_GAINS =
        new TunableTrapezoidalProfileGains("/elevator/trap", 0, 0, MiscConstants.TUNING_MODE);
    public static final TunableFFGains FF =
        new TunableFFGains("/elevator/ff", 0, 0, 0, MiscConstants.TUNING_MODE);
    public static final double L2_REEF = Units.inchesToMeters(7.9736);
    public static final double L4_REEF = Units.inchesToMeters(49.706);
    public static final double L3_REEF = Units.inchesToMeters(20.75);
    public static final double L1_REEF = 0.0; 
    public static final double LOW_BALL_PICKUP = Units.inchesToMeters(12.737500);
    public static final double NET = Units.inchesToMeters(49.706);
    public static final double HUMAN = Units.inchesToMeters(15.9377);
    public static final double PROCESSOR = 0.0; 
    public static final double GROUND_INTAKE = 0.0; 
  }

  public static class IntakeConstants {
    // rotation stuff
    public static final int ROTATION_MOTOR_ID = 15;
    public static final int ROTATION_LIMIT_SWITCH_ID = 1; 
    public static final TunablePIDGains ROTATION_PID_GAINS =
        new TunablePIDGains("/intake/rotation/PID", 0, 0, 0, MiscConstants.TUNING_MODE);
    public static final TunableTrapezoidalProfileGains ROTATION_TRAP_GAINS =
        new TunableTrapezoidalProfileGains(
            "/intake/rotation/trap gains", 0, 0, MiscConstants.TUNING_MODE);
    public static final TunableArmElevatorFFGains ROTATION_FF_GAINS =
        new TunableArmElevatorFFGains(
            "/intake/rotation/ff/", 0, 0, 0, 0, MiscConstants.TUNING_MODE);

    public static final double GEAR_RATIO_ROTATION = 24.0;
    public static final double SUPPLY_CURRENT_LIMIT_ROTATION = 10.0;
    public static final InvertedValue INVERTED_ROTATION = InvertedValue.Clockwise_Positive;
    public static final double ROTATION_UP_ANGLE = 0;
    public static final double ROTATION_DOWN_ANGLE = Units.degreesToRadians(100); 

    // spinny stuff
    public static final int SPINNING_MOTOR_ID = 14; 
    public static final double SPINNING_VOLTAGE = 11.0;
    public static final double RATE_LIMIT = 1.0; 
    public static final TunablePIDGains SPINNING_PID_GAINS =
        new TunablePIDGains("/intake/spinning/PID", 0, 0, 0, MiscConstants.TUNING_MODE);
    public static final TunableFFGains SPINNING_FF_GAINS =
        new TunableFFGains("/intake/spinning/ff/", 0.0, 0.0, 0.0, MiscConstants.TUNING_MODE);
    public static final double GEAR_RATIO_SPINNING = 1;
    public static final double SUPPLY_CURRENT_LIMIT_SPINNING = 25.0; 
    public static final boolean INVERTED_SPINNING = false; 
    public static final int STALL_MOTOR_CURRENT = 30; 
    public static final int FREE_MOTOR_CURRENT = 25; 
    public static final int SWITCH_ID = 3; 
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int BUTTON_BOARD_ID = 0;
  }

  public static class MiscConstants {
    public static final String CANIVORE_NAME = "canivore";

    private MiscConstants() {} // why is there a constructor here

    public static final int[] USED_CONTROLLER_PORTS = {0, 1};
    public static final boolean TUNING_MODE = !DriverStation.isFMSAttached();

    public static final int CONFIGURATION_ATTEMPTS = 10;
  }

  public static class AlgaeConstants {
    public static final int ALGAE_MOTOR_ID = 13;
    public static final int STALL_MOTOR_CURRENT = 30; 
    public static final int FREE_MOTOR_CURRENT = 20; 
    public static final double GEAR_RATIO = 36.0/18.0;
    public static final boolean INVERTED = true; 

    public static final TunablePIDGains PID_GAINS =
        new TunablePIDGains("algae/pid", 0, 0.0, 0.0, MiscConstants.TUNING_MODE);
    public static final TunableFFGains FF_GAINS =
        new TunableFFGains("algae/ff", 0.0, 0.0, 0.0, MiscConstants.TUNING_MODE);
    public static final double RUNNING_VOLTAGE = 10.0;
    public static final int SWITCH_ID = 9; 
    public static final double RATE_LIMIT = 1.0;
    public static final double OUTPUT_VOLTAGE = -6.0;
  }

  public static class CoralConstants {
    public static final int CORAL_MOTOR_ID = 12;
    public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;
    public static final int SLEW_RATE_LIMIT = 1; 
    
    public static final double SUPPLY_CURRENT_LIMIT = 40; 
    public static final int STALL_MOTOR_CURRENT = 30; 
    public static final int FREE_MOTOR_CURRENT = 20; 
    public static final double GEAR_RATIO = 18.0 / 16.0;

    // tune
    public static final TunablePIDGains PID_GAINS =
        new TunablePIDGains("/pid/coral", 0, 0, 0, MiscConstants.TUNING_MODE);
    public static final TunableFFGains FF_GAINS =
        new TunableFFGains("/ff/coral", 0, 0, 0, MiscConstants.TUNING_MODE);
    public static final double RUNNING_VOLTAGE = 10.0;
    public static final int BEAM_ID_LEFT = 8; 
    public static final double OUTPUT_VOLTAGE = -6.0;
    public static final int BEAM_ID_RIGHT = 7;
  }

  public static class ClimberConstants {
    public static final int CLIMB_MOTOR_1_ID = 16; 
    public static final int CLIMB_MOTOR_2_ID = 17; 
    public static final TunablePIDGains CLIMB_PID_GAINS =
        new TunablePIDGains("/pid/climber", 0, 0, 0, MiscConstants.TUNING_MODE);
    public static final TunableFFGains CLIMBER_FF_GAINS =
        new TunableFFGains("/feedfoward/climber/", 0, 0, 0, MiscConstants.TUNING_MODE);
    public static final double SUPPLY_CURRENT_LIMIT = 50.0; 
    public static final InvertedValue INVERTED_2 = InvertedValue.Clockwise_Positive; 
    public static final double GEAR_RATIO = 1.0; // m
    public static final boolean INVERTED_1 = false;
    public static final double CLIMBER_UP_VOLTAGE = -2.0; 
    public static final double CLIMBER_DOWN_VOLTAGE = 0.0; // m
    public static final int LIMITER = 2; // m
  }

  public static class VisionConstants {
    public static final double CAMERA_MOUNT_ANGLE = Units.degreesToRadians(8.0);
    public static final double CAMERA_MOUNT_HEIGHT_METERS = Units.inchesToMeters(10.0);

    public static final double CORAL_HEIGHT = 0.0; //m
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
    public static final double GEAR_RATIO = 10.0; 
    public static final double SUPPLY_CURRENT_LIMIT = 40.0; 
    public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;
    public static final double WRIST_OFFSET = 0.0; // needs a value
    public static final double PID_TOLERANCE = 2.0; 
    public static final int WRIST_ENCODER_PORT = 4;
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
    public static final double L2_REEF = Units.degreesToRadians(138.2326);
    public static final double L3_REEF = Units.degreesToRadians(138.2326);
    public static final double L4_REEF = Units.degreesToRadians(118.8490);
    public static final double PROCESSOR = Units.degreesToRadians(112.5900);
    public static final double BALL_PICKUP = Units.degreesToRadians(-90.0);
    public static final double NET = Units.degreesToRadians(30.0);
    public static final double HUMAN = Units.degreesToRadians(-42.5551);
    public static final double L1_REEF = Units.degreesToRadians(4.8061);
    public static final double GROUND_INTAKE = 0.0;
  }
}
