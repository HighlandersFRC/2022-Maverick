package frc.robot.commands.defaults;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.Drive;

public class DriveDefault extends CommandBase {
  /** Creates a new DriveDefault. */
  private Drive drive;  

  public DriveDefault(Drive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    addRequirements(this.drive);
    // System.out.println("INSIDE DEFAULT CONSTRUCTOR");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      // System.out.println("INSIDE DRIVE DEFAULT!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.teleopDrive();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}