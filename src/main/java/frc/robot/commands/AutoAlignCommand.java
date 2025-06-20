// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.autoCommands.ToPointCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
import frc.robot.telemetry.types.Pose2dEntry;
public class AutoAlignCommand extends Command {
 
  Translation2d robotTranslation;
 public CommandSwerveDrivetrain drivetrain;
 private ToPointCommand toPointCommand;
 public int leftOrRight;
 public static Pose2dEntry desiredPoseEntry = new Pose2dEntry("/drive/desiredpose", true);
 public static DoubleTelemetryEntry rotationEntry = new DoubleTelemetryEntry("drive/rotationstuffig", true);

 private final Translation2d[][] fieldZones = new Translation2d[6][3];
  private final Pose2d[][] branchPoints = new Pose2d[6][2];
 
   public AutoAlignCommand(CommandSwerveDrivetrain drivetrain, int leftOrRight) {
     this.drivetrain = drivetrain;
     this.leftOrRight = leftOrRight;


     addRequirements(drivetrain);
     initializeFieldZones();
     makeBranchPlacements();
  }

  //TODO: like actually get values for these
  public void initializeFieldZones(){
    //top
    fieldZones[0][0] = AutoConstants.TOP_RIGHT; //top right
    fieldZones[0][1] = AutoConstants.TOP_LEFT; // top left
    fieldZones[0][2] = AutoConstants.CENTER; //center

    // top right
    fieldZones[1][0] =  AutoConstants.MID_RIGHT; //mid right
    fieldZones[1][1] =  AutoConstants.TOP_RIGHT; // top right
    fieldZones[1][2] =  AutoConstants.CENTER; //center

    //bottom right
    fieldZones[2][0] =  AutoConstants.BOTTOM_RIGHT; //bottom right
    fieldZones[2][1] =  AutoConstants.MID_RIGHT; //mid right
    fieldZones[2][2] =  AutoConstants.CENTER; //center

    //bottom
    fieldZones[3][0] =  AutoConstants.BOTTOM_LEFT; // bottom left
    fieldZones[3][1] =  AutoConstants.CENTER; //center
    fieldZones[3][2] =  AutoConstants.BOTTOM_RIGHT; // bottom right

    //bottom left
    fieldZones[4][0] =  AutoConstants.MID_LEFT; //mid left
    fieldZones[4][1] =  AutoConstants.CENTER; //center
    fieldZones[4][2] =  AutoConstants.BOTTOM_LEFT; //bottom left

    //top left 
    fieldZones[5][0] =  AutoConstants.TOP_LEFT; // top left
    fieldZones[5][1] =  AutoConstants.CENTER; // center
    fieldZones[5][2] =  AutoConstants.MID_LEFT; //mid left
  }

  public void makeBranchPlacements(){
//0 = left
//1 = right

    branchPoints[0][0] = new Pose2d(5.696, 3.987, new Rotation2d(Units.radiansToDegrees(90)) //180
    ); //12 
    branchPoints[0][1] = new Pose2d(5.404, 3.619, new Rotation2d(Units.radiansToDegrees(90))); //1

    branchPoints[1][0] = new Pose2d(4.994, 3.024, new Rotation2d(Units.radiansToDegrees(60)) //120
    ); //2
    branchPoints[1][1] = new Pose2d(4.735, 2.813, new Rotation2d(Units.radiansToDegrees(60))); //3

    branchPoints[2][0] = new Pose2d(3.81, 3.044, new Rotation2d(Units.radiansToDegrees(-30)) //60
    ); //4
    branchPoints[2][1] = new Pose2d(3.561, 3.321, new Rotation2d(Units.radiansToDegrees(-30))); //5

    branchPoints[3][1] = new Pose2d(3.363, 4.098, new Rotation2d(Units.radiansToDegrees(-90)) //0
    ); //6
    branchPoints[3][0] = new Pose2d(3.411, 4.334, new Rotation2d(Units.radiansToDegrees(-90))); //7

    branchPoints[4][1] = new Pose2d(3.964, 5.006, new Rotation2d(Units.radiansToDegrees(210)) //-60
    ); //8
    branchPoints[4][0] = new Pose2d(4.31, 5.075, new Rotation2d(Units.radiansToDegrees(210))); //9

    branchPoints[5][0] = new Pose2d(5.119, 5.026, new Rotation2d(Units.radiansToDegrees(-55)) //-125
    ); //10
    branchPoints[5][1] = new Pose2d(5.364, 4.803, new Rotation2d(Units.radiansToDegrees(-55))); //11

  }

  
  public static Command createAutoAlignCommand(CommandSwerveDrivetrain drivetrain, int leftOrRight) {
    return Commands.defer(() -> {
      AutoAlignCommand autoAlignCommand = new AutoAlignCommand(drivetrain, leftOrRight);
      int currentZone = autoAlignCommand.determineCurrentZone();
      
      // && leftOrRight >= 0 && leftOrRight <= 1 && currentZone>= 0
      if (leftOrRight >= 0 && leftOrRight <= 1 && currentZone >= 0) {
        Pose2d targetPose = autoAlignCommand.branchPoints[currentZone][leftOrRight];
        Supplier<Pose2d> targetPoseSupplier = () -> targetPose;
        desiredPoseEntry.append(targetPose);  
      
        System.out.println("WORKING???????????????????????????????????????????????????????????????????????????????????????????" + currentZone);
        return new ToPointCommand(drivetrain, targetPoseSupplier);
    
      }
      else {
        System.out.println("NOTWORKING!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        return Commands.print("AutoAlign invalid zone or left/right");
      }
    }, Set.of(drivetrain));
  }

  @Override
  public void initialize() {
  //   int currentZone = determineCurrentZone();

  // if (currentZone >= 0 && leftOrRight >= 0 && leftOrRight <= 1) {

  //     Pose2d targetPose = branchPoints[currentZone][leftOrRight];
     
  //     Supplier<Pose2d> targetPoseSupplier = () -> targetPose;
  
  //   toPointCommand = new ToPointCommand(drivetrain, targetPoseSupplier);
  //     toPointCommand.initialize();
  // }
  // else{
  //   System.out.println("Invalid zone: " + currentZone + " or left/right: " + leftOrRight);
  //   toPointCommand = null;
  // }
  }

  private int determineCurrentZone(){
     Pose2d pose = drivetrain.getPose();
     double poseRotation = Math.atan2(pose.getY(), pose.getX());

      
     rotationEntry.append(poseRotation);

   
     if ( -0.524<= poseRotation && poseRotation < .524){
      return 0;
     }
     if ( .524 <= poseRotation && poseRotation < 1.571 ){
      return 5;
     }
     if (1.571 <= poseRotation && poseRotation <2.618 ){
      return 4;
     }
     if ( 2.618 <= poseRotation && poseRotation < -2.618){
      return 3;
     }
     if (-2.618 <= poseRotation && poseRotation < -1.571){
      return 2;
     }
     if (-1.571 <= poseRotation && poseRotation < -0.524){
      return 1;
     }
    else{
      return -1;
    }
   
  }
  
  @Override
  public void execute() {
    
  }

  
  @Override
  public void end(boolean interrupted) {
    
  }

 
}
