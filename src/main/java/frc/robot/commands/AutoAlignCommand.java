// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.autoCommands.ToPointCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignCommand extends Command {
 
  Translation2d robotTranslation;
 public CommandSwerveDrivetrain drivetrain;
 private ToPointCommand toPointCommand;
 public int leftOrRight;

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

  @Override
  public void initialize() {
    int currentZone = determineCurrentZone();

  if (currentZone >= 0) {

      Pose2d targetPose = branchPoints[currentZone][leftOrRight];
     
      Supplier<Pose2d> targetPoseSupplier = () -> targetPose;
  
    toPointCommand = new ToPointCommand(drivetrain, targetPoseSupplier);
      toPointCommand.schedule();
  }
  }

  private double calculateTriangleArea(Translation2d v1, Translation2d v2, Translation2d v3) {
    return Math.abs((v1.getX() * (v2.getY() - v3.getY()) + 
                     v2.getX() * (v3.getY() - v1.getY()) + 
                     v3.getX() * (v1.getY() - v2.getY())) / 2.0);
  }

  private boolean isPointInTriangle(Translation2d point, Translation2d v1, Translation2d v2, Translation2d v3) {
        double totalArea = calculateTriangleArea(v1, v2, v3);
    
    double area1 = calculateTriangleArea(point, v1, v2);
    double area2 = calculateTriangleArea(point, v2, v3);
    double area3 = calculateTriangleArea(point, v3, v1);
    
    // Check if sum of the three areas equals the total area (with tolerance)
    return Math.abs(totalArea - (area1 + area2 + area3)) < 0.01;
  }

  private int determineCurrentZone() {
    Translation2d robotPosition = drivetrain.getPose().getTranslation();
    
    for (int i = 0; i < 6; i++) {
      if (isPointInTriangle(robotPosition, fieldZones[i][0], fieldZones[i][1], fieldZones[i][2])) {
        return i;
      }
    }

    
    return -1;
  }
  @Override
  public void execute() {
    
  }

  
  @Override
  public void end(boolean interrupted) {
    if (toPointCommand != null && toPointCommand.isScheduled()) {
      toPointCommand.cancel();
    }
  }

  
  @Override
  public boolean isFinished() {
    return toPointCommand != null && !toPointCommand.isScheduled();
  }
}
