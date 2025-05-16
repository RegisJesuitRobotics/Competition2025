// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    fieldZones[0][0] = new Translation2d(); //top right
    fieldZones[0][1] = new Translation2d(); // top left
    fieldZones[0][2] = new Translation2d(); //center

    // top right
    fieldZones[1][0] = new Translation2d(); //mid right
    fieldZones[1][1] = new Translation2d(); // top right
    fieldZones[1][2] = new Translation2d(); //center

    //bottom right
    fieldZones[2][0] = new Translation2d(); //bottom right
    fieldZones[2][1] = new Translation2d(); //mid right
    fieldZones[2][2] = new Translation2d(); //center

    //bottom
    fieldZones[3][0] = new Translation2d(); // bottom left
    fieldZones[3][1] = new Translation2d(); //center
    fieldZones[3][2] = new Translation2d(); // bottom right

    //bottom left
    fieldZones[4][0] = new Translation2d(); //mid left
    fieldZones[4][1] = new Translation2d(); //center
    fieldZones[4][2] = new Translation2d(); //bottom left

    //top left 
    fieldZones[5][0] = new Translation2d(); // top left
    fieldZones[5][1] = new Translation2d(); // center
    fieldZones[5][2] = new Translation2d(); //mid left
  }

  public void makeBranchPlacements(){

    branchPoints[0][0] = new Pose2d(); //12
    branchPoints[0][1] = new Pose2d(); //1

    branchPoints[1][0] = new Pose2d(); //2
    branchPoints[1][1] = new Pose2d(); //3

    branchPoints[2][0] = new Pose2d(); //4
    branchPoints[2][1] = new Pose2d(); //5

    branchPoints[3][0] = new Pose2d(); //6
    branchPoints[3][1] = new Pose2d(); //7

    branchPoints[4][0] = new Pose2d(); //8
    branchPoints[4][1] = new Pose2d(); //9

    branchPoints[5][0] = new Pose2d(); //10
    branchPoints[5][1] = new Pose2d(); //11

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
