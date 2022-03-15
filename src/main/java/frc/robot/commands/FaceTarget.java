package frc.robot.commands;

import org.json.JSONArray;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Peripherals;

public class FaceTarget extends SequentialCommandGroup {

  private Drive drive;
  private JSONArray turnPath;

  private Peripherals peripherals;

  public FaceTarget(Drive drive, Peripherals peripherals, double offset) {

    // this.turnPath = drive.testGetCameraTurn();
    // System.out.println("????????????????????????????");
    // System.out.println("TURN PATH: " + this.turnPath);
    // System.out.println("????????????????????????????");
    this.drive = drive;
    this.peripherals = peripherals;
    // this.turnAngle = turnAngle;
    addRequirements(this.drive, this.peripherals);
    this.turnPath = drive.getJSONTurnPath(offset);

    System.out.println("!!!!!!!!!!!!!!!!!!!!");
    System.out.println(turnPath);
    
    addCommands(new TurnOnLightRing(peripherals), new ContinuousAccelerationInterpolation(drive, this.turnPath, true));
  }
}