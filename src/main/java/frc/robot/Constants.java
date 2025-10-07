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
    public static final double METERS_PER_REVOLUTION =
        (Math.PI * Units.inchesToMeters(2.2594)) / GEAR_RATIO;
    public static final InvertedValue LEFT_INVERTED = InvertedValue.Clockwise_Positive;

    public static final TunablePIDGains PID_GAINS =
        new TunablePIDGains("/elevator/PID", 38, 1, 0.5, MiscConstants.TUNING_MODE);
    public static final TunableTrapezoidalProfileGains TRAP_GAINS =
        new TunableTrapezoidalProfileGains("/elevator/trap", 10, 8
        , MiscConstants.TUNING_MODE);
    public static final TunableFFGains FF =
        new TunableFFGains("/elevator/ff", 0.02501, 0.12426, 0.074887, MiscConstants.TUNING_MODE);

    public static final double L2_REEF = Units.inchesToMeters(28.5); //7.9736
    public static final double L4_REEF = Units.inchesToMeters(71.35507394660516);
    public static final double L3_REEF = Units.inchesToMeters(44.77526942901589);
    public static final double L1_REEF = Units.inchesToMeters(16.345548826546363);
    public static final double INTAKE_POSITION = Units.inchesToMeters(1);
    public static final double HUMAN = Units.inchesToMeters(10.9377);
    public static final double FORCE_HOME = Units.inchesToMeters(7.0);
  }

 
  public static class CoralConstants {

    public static final int CORAL_MOTOR_ID = 12;
    public static final int SWITCH_ID_RIGHT = 1; 
    public static final int SWITCH_ID_LEFT = 0; 

    public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;
    public static final double SUPPLY_CURRENT_LIMIT = 40; 
    public static final int STALL_MOTOR_CURRENT = 30; 
    public static final int FREE_MOTOR_CURRENT = 20;

    public static final double GEAR_RATIO = 18.0 / 16.0;
    public static final double RUNNING_VOLTAGE = 12.0;
    public static final double OUTPUT_VOLTAGE = 11.0;
    public static final double INTAKE_VOLTAGE = 2.5;
    
  }


  public static class MiscConstants {
    public static final String CANIVORE_NAME = "canivore";

    private MiscConstants() {} // why is there a constructor here

    public static final int[] USED_CONTROLLER_PORTS = {0, 1, 2};
    public static final boolean TUNING_MODE = !DriverStation.isFMSAttached();

    public static final int CONFIGURATION_ATTEMPTS = 10;
    public static final double TRANSLATION_RATE_LIMIT = 15;
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
    public static final String APRIL_LIMELIGHT = "limelight-april";
    public static final String OBJECT_LIMELIGHT = "limelight-coral";
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

}