package frc.robot.commands.defaults;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Lights.LEDMode;

public class LightsDefault extends CommandBase {
  private Lights lights;
  private double startTime = Timer.getFPGATimestamp();
  public LightsDefault(Lights lights) {
    this.lights = lights;
    addRequirements(this.lights);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println("In lights default");
    if (Timer.getFPGATimestamp() - startTime > 0.5){
      startTime = Timer.getFPGATimestamp();
      // lights.switchAmericaLights();
      lights.setMode(LEDMode.BLUE);
    }
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