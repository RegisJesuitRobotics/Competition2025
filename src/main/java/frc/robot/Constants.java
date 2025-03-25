// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
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
    public static final double SUPPLY_CURRENT_LIMIT = 40.0;
    public static final InvertedValue INVERTED_RIGHT = InvertedValue.CounterClockwise_Positive;
    public static final double GEAR_RATIO = 9;
    // TODO: inches of gear
    public static final double METERS_PER_REVOLUTION =
        (Math.PI * Units.inchesToMeters(2.2594)) / GEAR_RATIO;
    public static final InvertedValue LEFT_INVERTED = InvertedValue.Clockwise_Positive;
    public static final TunablePIDGains PID_GAINS =
        new TunablePIDGains("/elevator/PID", 30, 0, 0.5, MiscConstants.TUNING_MODE);
    public static final TunableTrapezoidalProfileGains TRAP_GAINS =
        new TunableTrapezoidalProfileGains("/elevator/trap", 10, 8
        , MiscConstants.TUNING_MODE);
    public static final TunableFFGains FF =
        new TunableFFGains("/elevator/ff", 0.02501, 0.12426, 0.074887, MiscConstants.TUNING_MODE);
    public static final double L2_REEF = Units.inchesToMeters(7.9736);
    public static final double L4_REEF = Units.inchesToMeters(57.50);
    public static final double L3_REEF = Units.inchesToMeters(25.75);
    public static final double L1_REEF = 0.0;
    public static final double LOW_BALL_PICKUP = Units.inchesToMeters(12.737500);
    public static final double NET = Units.inchesToMeters(49.706);
    public static final double HUMAN = Units.inchesToMeters(10.9377);
    public static final double PROCESSOR = 0.0;
    public static final double GROUND_INTAKE = Units.inchesToMeters(0.0);
    public static final double HANDOFF = Units.inchesToMeters(3.0);
    public static final double PRE_GROUND = Units.inchesToMeters(14);
    public static final double HIGH_BALL_PICKUP = Units.inchesToMeters(12.737500 + 18);
    public static final double FORCE_HOME = Units.inchesToMeters(7.0);
  }

  public static class IntakeConstants {
    // rotation stuff
    public static final int ROTATION_MOTOR_ID = 15;
    public static final int ROTATION_LIMIT_SWITCH_ID = 1;
    public static final TunablePIDGains ROTATION_PID_GAINS =
        new TunablePIDGains("/intake/rotation/PID", 6, 0, 0, MiscConstants.TUNING_MODE);
    public static final TunableTrapezoidalProfileGains ROTATION_TRAP_GAINS =
        new TunableTrapezoidalProfileGains(
            "/intake/rotation/trapgains", 25, 25, MiscConstants.TUNING_MODE);
    public static final TunableArmElevatorFFGains ROTATION_FF_GAINS =
        new TunableArmElevatorFFGains(
            "/intake/rotation/ff/", 0.52525, 0.1693, 0.0080483, 0.0020163, MiscConstants.TUNING_MODE);

    public static final double GEAR_RATIO_ROTATION = 3.0 * 4.0 * (60.0 / 30.0);
    public static final double SUPPLY_CURRENT_LIMIT_ROTATION = 30.0;
    public static final InvertedValue INVERTED_ROTATION = InvertedValue.CounterClockwise_Positive;
    public static final double ROTATION_UP_ANGLE = Units.degreesToRadians(-2); // idk
    public static final double ROTATION_DOWN_ANGLE = 1.635; // m

    // spinny stuff
    public static final int SPINNING_MOTOR_ID = 14; // idk
    public static final double SPINNING_VOLTAGE = 10.0; // m
    public static final double RATE_LIMIT = 1.0; // m
    public static final TunablePIDGains SPINNING_PID_GAINS =
        new TunablePIDGains("/intake/spinning/PID", 0, 0, 0, MiscConstants.TUNING_MODE);
    public static final TunableFFGains SPINNING_FF_GAINS =
        new TunableFFGains("/intake/spinning/ff/", 0.0, 0.0, 0.0, MiscConstants.TUNING_MODE);
    public static final double GEAR_RATIO_SPINNING = 1;
    public static final double SUPPLY_CURRENT_LIMIT_SPINNING = 25.0; // idk
    public static final InvertedValue INVERTED_SPINNING = InvertedValue.Clockwise_Positive; // m
    public static final int STALL_MOTOR_CURRENT = 40; // mF
    public static final int FREE_MOTOR_CURRENT = 30; // m
    public static final int LEFT_SWITCH = 3;
    public static final int SPINNING_VOLTAGE_OUTAKE = 12;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int BUTTON_BOARD_ID = 2;
  }

  public static class MiscConstants {
    public static final String CANIVORE_NAME = "canivore";

    private MiscConstants() {} // why is there a constructor here

    public static final int[] USED_CONTROLLER_PORTS = {0, 1, 2};
    public static final boolean TUNING_MODE = !DriverStation.isFMSAttached();

    public static final int CONFIGURATION_ATTEMPTS = 10;
    public static final double TRANSLATION_RATE_LIMIT = 15;
  }

  public static class AlgaeConstants {
    public static final int ALGAE_MOTOR_ID = 13;
    public static final int STALL_MOTOR_CURRENT = 30;
    public static final int FREE_MOTOR_CURRENT = 20;
    public static final double GEAR_RATIO = 36.0 / 18.0;
    public static final boolean INVERTED = true; // m

    public static final TunablePIDGains PID_GAINS =
        new TunablePIDGains("algae/pid", 0, 0.0, 0.0, MiscConstants.TUNING_MODE);
    public static final TunableFFGains FF_GAINS =
        new TunableFFGains("algae/ff", 0.0, 0.0, 0.0, MiscConstants.TUNING_MODE);
    public static final double RUNNING_VOLTAGE = 12.0;
    public static final double RATE_LIMIT = 1.0;
    public static final double OUTPUT_VOLTAGE = -6.0;
    public static final int SWITCH_ID_RIGHT = 0;
    public static final DoubleSupplier LEAVING = () -> Units.degreesToRadians(95);
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
    public static final double RUNNING_VOLTAGE = 12.0;
    public static final int SWITCH_ID_LEFT = 4; 
    public static final double OUTPUT_VOLTAGE = -10.0;
    public static final int SWITCH_ID_RIGHT = 9; 
  }

  public static class ClimberConstants {
    public static final int CLIMB_MOTOR_1_ID = 13; // m
    public static final TunablePIDGains CLIMB_PID_GAINS =
        new TunablePIDGains("/pid/climber", 0, 0, 0, MiscConstants.TUNING_MODE);
    public static final TunableFFGains CLIMBER_FF_GAINS =
        new TunableFFGains("/feedfoward/climber/", 0, 0, 0, MiscConstants.TUNING_MODE);
    public static final double SUPPLY_CURRENT_LIMIT = 60.0; // m
    public static final InvertedValue INVERTED_2 = InvertedValue.Clockwise_Positive; // m
    public static final InvertedValue INVERTED_1 = InvertedValue.Clockwise_Positive;
    public static final double CLIMBER_UP_VOLTAGE = 4.0; // m
    public static final double CLIMBER_DOWN_VOLTAGE = -4.0; // m
    public static final int LIMITER = 0; // m
  }

  public static class ChassisConstants{
    public static final Distance WHEEL_RADIUS =  Distance.ofBaseUnits(0.048, Meters);
    public static final LinearVelocity MAX_SPEED = LinearVelocity.ofBaseUnits(5.450, MetersPerSecond);
    public static final Current MAX_CURRENT = Current.ofBaseUnits(60, Amps);
    public static final RobotConfig PP_CONFIG = new RobotConfig(
      Mass.ofBaseUnits(74, Kilogram),
      MomentOfInertia.ofBaseUnits(6.883, KilogramSquareMeters),

      new ModuleConfig(
        WHEEL_RADIUS,
        MAX_SPEED,
         1.2,
          DCMotor.getKrakenX60(1),
          MAX_CURRENT ,
           1),
    new Translation2d(0.273,0.273),
    new Translation2d(0.273,-0.273),
    new Translation2d(-0.273,0.273),
    new Translation2d(-0.273,-0.273)
    
      
    );
  }

  public static class VisionConstants {
    public static final double CAMERA_MOUNT_ANGLE = 0.0;
    public static final double CAMERA_MOUNT_HEIGHT_METERS = 0.0;

    public static final double CORAL_HEIGHT = 0.0;
    public static final String APRIL_LIMELIGHT = "limelight-coral";
    public static final String OBJECT_LIMELIGHT = "limelight-april";
    public static final double CONFIDENCE_THRESHOLD = 80.0;
  }

  public static class AutoConstants {

    public static final TunablePIDGains pointTranslationGains =
        new TunablePIDGains(
            "/drive/gains/pointTranslationController", 5, 0, 0, MiscConstants.TUNING_MODE);
    public static final TunableTrapezoidalProfileGains trapPointTranslationGains =
        new TunableTrapezoidalProfileGains(
            "/drive/gains/trapPointTranslationController", 0, 0, MiscConstants.TUNING_MODE);
    public static final TunableFFGains pointTranslationFFGains =
        new TunableFFGains("/drive/gains/pointFFController", 0, 0.124, 0, MiscConstants.TUNING_MODE);
    public static final double MAX_VELOCITY = 3.0;
    public static final double MAX_ACCELERATION = 0.5;
    public static final double MAX_ANGULAR_VELOCITY = Units.degreesToRadians(100);
    public static final double MAX_ANGULAR_ACCELERATION = Units.degreesToRadians(80);
    public static final double NOMINAL_VOLTAGE = 12.0;
    public static final PIDConstants ROTATION_PID_GAINS = new PIDConstants(5, 0, 0);
    public static final double RATE_LIMIT = 10.0;
  }

  public static class WristConstants {
    public static final int WRIST_ID = 11;
    public static final double GEAR_RATIO = 9.0;
    public static final double SUPPLY_CURRENT_LIMIT = 60.0;
    public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;
    public static final double WRIST_OFFSET = Units.degreesToRadians(90 - 85.55177013879425);
    public static final double PID_TOLERANCE = 2.0; // idk
    public static final int WRIST_ENCODER_PORT = 22; // needs a value
    // a lot of 0s
    public static final TunablePIDGains WRIST_PID_GAINS =
        new TunablePIDGains("/pid/wrist/", 17, 0.0, 0, MiscConstants.TUNING_MODE);

    public static final TunableTrapezoidalProfileGains WRIST_TRAP_GAINS =
        new TunableTrapezoidalProfileGains(
            "/trapezoidalprofile/wrist",
            Units.rotationsToRadians(5.0),
            Units.rotationsToRadians(5),
            MiscConstants.TUNING_MODE);
    public static final double DYNAMIC_OFFSET = Units.degreesToRadians(0);

    public static final TunableArmElevatorFFGains WRIST_FF_GAINS =
        new TunableArmElevatorFFGains(
            "/feedfoward/wrist/", 0.38155, 0.42528, 0.26833, 0.11142, MiscConstants.TUNING_MODE);

    public static final double L2_REEF = Units.degreesToRadians(145.2326);

    public static final double L3_REEF = Units.degreesToRadians(147.2326);
    public static final double L4_REEF = Units.degreesToRadians(125.8490);
    public static final double PROCESSOR = -Units.degreesToRadians(112.5900);
    public static final double BALL_PICKUP = -Units.degreesToRadians(70.0);
    public static final double NET = Units.degreesToRadians(20.0);
    public static final double HUMAN = -0.62;
    public static final double L1_REEF = Units.degreesToRadians(4.8061);
    public static final double GROUND_INTAKE = 0.0;
    public static final TunablePIDGains WRIST_PID_ALGAE_GAINS = new TunablePIDGains("/wrist/pidAlgae", 10.0, 0, .1, true);
    public static final TunableTrapezoidalProfileGains WRIST_TRAP_ALGAE = new TunableTrapezoidalProfileGains("/wrist/trap", 10, 5, true);
    public static final TunableArmElevatorFFGains WRIST_ALGAE_FF_GAINS = new TunableArmElevatorFFGains("/wristAlgae/ff", 0.35886, 4.556, 0.13355, 0.024112, true);
  }
}