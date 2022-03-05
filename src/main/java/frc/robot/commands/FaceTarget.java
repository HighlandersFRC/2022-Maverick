package frc.robot.commands;

import org.json.JSONArray;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive;

public class FaceTarget extends SequentialCommandGroup {

  private Drive drive;
  private double turnAngle;
  private JSONArray turnPath;

  public FaceTarget(Drive drive) {

    this.turnPath = drive.testGetCameraTurn();
    this.drive = drive;
    // this.turnAngle = turnAngle;
    addRequirements(this.drive);

    //this.turnPath = this.drive.getJSONTurnPath(this.turnAngle);
    
    addCommands(new ContinuousAccelerationInterpolation(drive, turnPath));
  }
}