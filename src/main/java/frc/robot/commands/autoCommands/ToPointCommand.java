
package frc.robot.commands.autoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.function.Supplier;


public  class ToPointCommand extends Command
{
private final CommandSwerveDrivetrain drive;
private Supplier<Pose2d> desiredPoseSupplier;
private Pose2d desiredPoseCurrent = new Pose2d();

 public ToPointCommand(CommandSwerveDrivetrain drive)
    {
       this.drive = drive;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(drive);
    }

    @Override
    public void initialize()
    {
        
    }

    @Override
    public void execute()
    {
        
    }
    
    @Override
    public boolean isFinished()
    {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted)
    {
        
    }
}
