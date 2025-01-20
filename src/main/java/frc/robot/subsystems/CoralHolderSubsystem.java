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

import edu.wpi.first.math.controller.PIDController;
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
  new Alert("Haii~\nI have a sniper on the building next to this one you better win :3", AlertType.INFO).set(true);
  
  /* I put null for anything I didn't know */
  private final TelemetryCANSparkFlex coralHolderMotor = new TelemetryCANSparkFlex(CoralConstants.CORAL_HOLDER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless, "/coralHolder/motor", MiscConstants.TUNING_MODE);
  private final Alert coralMotorAlert = new Alert("you're so incompetent that you broke my motor (coral holder motor fault)", AlertType.ERROR);
  private final SlewRateLimiter rateLimiter = new SlewRateLimiter(CoralConstants.SLEW_RATE_LIMIT);
  private final RelativeEncoder coralHolderMotorEncoder = coralHolderMotor.getEncoder();
  private final SysIdRoutine sysIDRoutine = new SysIdRoutine(null, null);
  private final SimpleMotorFeedforward coralHolderMotorFF = CoralConstants.FF_GAINS.createFeedforward();

  /* telemetry stuff */
  private final EventTelemetryEntry coralHolderMotorTelemetry = new EventTelemetryEntry("/coralHolder/events");
  private final DoubleTelemetryEntry coralHolderDoubleEntries = new DoubleTelemetryEntry("/coralHolder/voltageReq", null);
  private final BooleanTelemetryEntry coralHolderBooleanEntries = new BooleanTelemetryEntry("/coralHolder/boolean", null);
  private final TunableTelemetryPIDController coralHolderMotorPID = new TunableTelemetryPIDController("/coralHolder/pid", );

  void configMotor() {
    ;   //haha not my problem
  }

  public CoralHolderSubsystem() {
    configMotor();
  }

  public void setVoltage(double voltage) {
    coralHolderMotor.setVoltage(voltage);
  }

  public double getVelocity() {
    return coralHolderMotorEncoder.getVelocity();
  }

  public double getSetpoint() {
    return coralHolderMotorPID.getSetpoint();
  }

  public boolean inTolerance() {
    return Math.abs(getVelocity() - getSetpoint()) / (getSetpoint()) < 0.05;
  }

  public void runVelocity(double setpointRadiansPerSecond) {
    double limitedRate = rateLimiter.calculate(setpointRadiansPerSecond);
    setVoltage(coralHolderMotorPID.calculate(getVelocity(), limitedRate) + coralHolderMotorFF.calculate(limitedRate));
  }

  public Command sysIDQuasistatic(SysIdRoutine.Direction direction) {
    return sysIDRoutine.quasistatic(direction);
  }

  public Command sysIDDynamic(SysIdRoutine.Direction direction) {
    return sysIDRoutine.dynamic(direction);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
