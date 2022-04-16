package frc.robot.commands.defaults;

import java.lang.invoke.ConstantBootstraps;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Shooter;

public class ShooterDefault extends CommandBase {
  /** Creates a new ShooterDefault. */
  private Shooter shooter;  
  private Peripherals peripherals;

  public ShooterDefault(Shooter shooter, Peripherals peripherals) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.peripherals = peripherals;
    addRequirements(this.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("SD", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(OI.operatorB.get()) {
    //   shooter.setShooterRPM(Constants.getShooterValues(peripherals.getLimeLightDistanceToTarget())[1]);
    // }
    shooter.setShooterPercent(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("SD", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}