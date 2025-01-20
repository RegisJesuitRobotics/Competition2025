// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

import org.opencv.features2d.SimpleBlobDetector;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.MiscConstants;
import frc.robot.telemetry.tunable.TunableTelemetryPIDController;
import frc.robot.telemetry.types.BooleanTelemetryEntry;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.types.EventTelemetryEntry;
import frc.robot.telemetry.wrappers.TelemetryCANSparkFlex;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;
import frc.robot.utils.ConfigurationUtils;
import frc.robot.utils.ConfigurationUtils.StringFaultRecorder;

public class CoralHolderSubsystem extends SubsystemBase {
  
  /* I put null for anything I didn't know */
  private final TelemetryCANSparkFlex coralHolderMotor = new TelemetryCANSparkFlex(CoralConstants.CORAL_HOLDER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless, "/coral/motor",MiscConstants.TUNING_MODE);
  private final Alert coralMotorAlert = new Alert("you're so incompetent that you broke my motor (coral holder motor fault)", AlertType.ERROR);  //maybe cycle through a list of insults randomly but not rn
  private final SlewRateLimiter rateLimiter = new SlewRateLimiter(CoralConstants.SLEW_RATE_LIMIT);
  private final RelativeEncoder coralHolderMotorEncoder = coralHolderMotor.getEncoder();
  private final SysIdRoutine sysIdRoutine = new SysIdRoutine(null, null);
  private final SimpleMotorFeedforward coralHolderMotorPID = CoralConstants.FF_GAINS.createFeedforward();


  void configMotor() {
    ;   //haha not my problem
  }

  /** Creates a new CoralHolderSubsystem. (constructor) */
  public CoralHolderSubsystem() {
    configMotor();

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
