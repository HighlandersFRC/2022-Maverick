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
import frc.robot.commands.CancelMagazine;
import frc.robot.commands.ContinuousAccelerationInterpolation;
import frc.robot.commands.FireBalls;
import frc.robot.commands.FireBallsNoVision;
import frc.robot.commands.FireOneBall;
import frc.robot.commands.IntakeBalls;
import frc.robot.commands.IntakeDown;
import frc.robot.commands.SetHoodPosition;
import frc.robot.commands.SpinShooter;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.MagIntake;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Shooter;
import frc.robot.tools.ShotAdjuster;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Chezy5BallAuto extends SequentialCommandGroup {
  /** Creates a new Chezy5BallAuto. */
  private File pathingFile;
  private JSONArray pathJSON;

  private File pathingFile2;
  private JSONArray pathJSON2;

  private File pathingFile3;
  private JSONArray pathJSON3;

  private ShotAdjuster adjuster = new ShotAdjuster();

  public Chezy5BallAuto(Drive drive, MagIntake magIntake, Shooter shooter, Hood hood, Peripherals peripherals, Lights lights) {
    try {
      pathingFile = new File("/home/lvuser/deploy/Chezy5Pt1.json");
      FileReader scanner = new FileReader(pathingFile);
      pathJSON = new JSONArray(new JSONTokener(scanner));
    }
    catch(Exception e) {
      System.out.println("ERROR WITH PATH FILE " + e);
    }
    try {
      pathingFile3 = new File("/home/lvuser/deploy/5BallPart3.json");
      FileReader scanner2 = new FileReader(pathingFile3);
      pathJSON3 = new JSONArray(new JSONTokener(scanner2));
    }
    catch(Exception e) {
        System.out.println("ERROR WITH PATH FILE " + e);
    }
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(drive, magIntake);
    addCommands(
    new FireBalls(drive, magIntake, shooter, hood, peripherals, lights, 10, 1400, 0.75, 0.75, adjuster, 0, true),
    new ParallelRaceGroup(
        new ContinuousAccelerationInterpolation(drive, pathJSON, false),
        new IntakeBalls(magIntake, lights)),
        // new SpinShooter(shooter, peripherals, 1600, adjuster, true),
        // new SetHoodPosition(hood, peripherals, 24, adjuster, true),
        // new IntakeBalls(magIntake, lights)),
    // new WaitCommand(0.25),
    // new CancelMagazine(magIntake),
    // new ParallelRaceGroup(new IntakeBalls(magIntake, lights), new WaitCommand(0.35)),
    // new FireBalls(drive, magIntake, shooter, hood, peripherals, lights, 24, 1600, 0.5, 0.01, adjuster, 0, true),
    // new ParallelRaceGroup(
    //     new ContinuousAccelerationInterpolation(drive, pathJSON2, false)),
        // new IntakeBalls(magIntake, lights)),
    // new CancelMagazine(magIntake),
    // new WaitCommand(0.15),
    // new FaceTarget(drive, peripherals, lights, 0),
    new FireOneBall(drive, magIntake, shooter, hood, peripherals, lights, 16, 1460, 0.5, 1, adjuster, 0.1, true),
    new ParallelRaceGroup(
        new ContinuousAccelerationInterpolation(drive, pathJSON3, false),
        new IntakeBalls(magIntake, lights)),
    new WaitCommand(0.25),
    new FireBalls(drive, magIntake, shooter, hood, peripherals, lights, 10, 1400, 0.75, 0.75, adjuster, 0, true));
    // new ParallelRaceGroup(
    //     new ContinuousAccelerationInterpolation(drive, pathJSON3, false)),
    //     // new IntakeBalls(magIntake, lights)),
    // new CancelMagazine(magIntake),
    // new WaitCommand(0.2),
    // new FireBalls(drive, magIntake, shooter, hood, peripherals, lights, 27, 1500, 0.5, 1, adjuster, 0.1, true));
  }
}

