package frc.robot.commands;

import org.json.JSONArray;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Peripherals;

public class FaceTarget extends SequentialCommandGroup {

  private Drive drive;
  private JSONArray turnPath;

  private Peripherals peripherals;

  private double turn;

  public FaceTarget(Drive drive, Peripherals peripherals, double offset) {

    // System.out.println("????????????????????????????");
    // System.out.println("TURN PATH: " + this.turnPath);
    // System.out.println("????????????????????????????");
    this.drive = drive;
    this.peripherals = peripherals;
    // this.turnAngle = turnAngle;
    addRequirements(this.drive, this.peripherals);    
    addCommands(new TurnOnLightRing(peripherals), new TurnDriveTrain(drive, peripherals));
  }
}