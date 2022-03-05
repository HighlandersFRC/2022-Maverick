// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.io.File;
import java.io.FileReader;

import org.json.JSONArray;
import org.json.JSONTokener;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ContinuousAccelerationInterpolation;
import frc.robot.commands.FaceTarget;
import frc.robot.commands.FireBalls;
import frc.robot.commands.IntakeBalls;
import frc.robot.commands.IntakeDown;
import frc.robot.commands.ResetAutoOdometry;
import frc.robot.commands.defaults.DriveDefault;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.MagIntake;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FiveBallAuto extends SequentialCommandGroup {
  /** Creates a new FiveBallAuto. */
  private File pathingFile;
  private JSONArray pathJSON;

  private File pathingFile2;
  private JSONArray pathJSON2;

  public FiveBallAuto(Drive drive, MagIntake magIntake, Shooter shooter, Hood hood, Peripherals peripherals, Lights lights) {
    try {
      pathingFile = new File("/home/lvuser/deploy/Adj3Ball.json");
      FileReader scanner = new FileReader(pathingFile);
      pathJSON = new JSONArray(new JSONTokener(scanner));
    }
    catch(Exception e) {
      System.out.println("ERROR WITH PATH FILE " + e);
    }

    try {
        pathingFile2 = new File("/home/lvuser/deploy/5BallPart2.json");
        FileReader scanner = new FileReader(pathingFile2);
        pathJSON2 = new JSONArray(new JSONTokener(scanner));
    }
    catch(Exception e) {
        System.out.println("ERROR WITH PATH FILE " + e);
    }
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(drive, magIntake);
    addCommands(new IntakeDown(magIntake),
        new FireBalls(drive, magIntake, shooter, hood, peripherals, lights, 15, 2300, 0.5, 0.01), 
        new ParallelRaceGroup(
            new ContinuousAccelerationInterpolation(drive, pathJSON, false),
            new IntakeBalls(magIntake, lights)), 
        new FireBalls(drive, magIntake, shooter, hood, peripherals, lights, 22, 2500, 0.5, 1),
        new ParallelRaceGroup(
            new ContinuousAccelerationInterpolation(drive, pathJSON2, false),
            new IntakeBalls(magIntake, lights)), 
        new FireBalls(drive, magIntake, shooter, hood, peripherals, lights, 23, 2500, 0.5, 1));
  }
}

