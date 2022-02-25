package frc.robot.commands.defaults;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.OI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.MagIntake;

public class MagIntakeDefault extends CommandBase {
  /** Creates a new MagIntakeDefault. */
  private static MagIntake magIntake;  

  public MagIntakeDefault(MagIntake magIntake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.magIntake = magIntake;
    addRequirements(this.magIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    magIntake.setIntakeUp();
    magIntake.setIntakePercent(0);
    magIntake.setBackMagazine(0);
    magIntake.setFrontMagazine(0);
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