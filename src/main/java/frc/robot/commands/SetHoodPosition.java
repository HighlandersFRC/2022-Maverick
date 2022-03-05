// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;

public class SetHoodPosition extends CommandBase {
  /** Creates a new SetHoodPosition. */
  private Hood hood;
  private double position;
  private double distance;
  private double startTime = Timer.getFPGATimestamp();

  public SetHoodPosition(Hood hood, double position, double distance) {
    this.hood = hood;
    this.position = position;
    this.distance = distance;
    addRequirements(hood);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // position = (0.0000001 * Math.pow(distance, 4)) + (-0.00008 * Math.pow(distance, 3)) + (0.0206 * Math.pow(distance, 2)) + (-2.0225 * distance) + 75;
    // if(position > 25) {
    //   position = 25;
    // }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hood.setHoodPosition(position);
    // hood.setHoodPercent(position);
    // hood.setHoodPercent(-0.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(hood.getHoodPosition() - position) < 0.5 || Timer.getFPGATimestamp() - startTime > 1){
      // System.out.println("`````````````````````````");
      return true;
    }
    return false;
  }
}
