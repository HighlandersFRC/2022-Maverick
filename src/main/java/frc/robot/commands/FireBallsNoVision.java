// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.MagIntake;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Shooter;
import frc.robot.tools.ShotAdjuster;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FireBallsNoVision extends SequentialCommandGroup {
  /** Creates a new FireBallsNoVision. */
  public FireBallsNoVision(Drive drive, MagIntake magIntake, Shooter shooter, Hood hood, Peripherals peripherals, Lights lights, double hoodPosition, double shooterRPM, double firstBallTimeout, double secondBallTimeout, ShotAdjuster adjuster) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(drive, magIntake, shooter, hood);
    double distance = drive.getDistanceToTarget();
    // System.out.println("")
    // shooterRPM = shooterRPM + adjuster.getRPMAdjustment();
    // hoodPosition = hoodPosition + adjuster.getHoodAdjustment();
    addCommands(
      new ParallelCommandGroup(
          new SpinShooter(shooter, peripherals, shooterRPM, adjuster, false),
        //   new FaceTarget(drive, peripherals),
        //   new VisionAlignment(drive, peripherals),
          new SetHoodPosition(hood, peripherals, hoodPosition, adjuster, false)
      ),
      // new LockDriveWheels(drive),
      new TurnBackMag(magIntake, 360),
      //new EjectBalls(magIntake, lights, 0.55, 0.85, firstBallTimeout),
      //new EjectBalls(magIntake, 0.0, 0, 0.001),
      new WaitCommand(0.25),
      new TurnBackMag(magIntake, 720),
      new WaitCommand(0.25),
      //new EjectBalls(magIntake, lights, 0.35, 0.45, secondBallTimeout)
      //new EjectBalls(magIntake, 0.0, 0, 0.1)
      //new TurnBackMag(magIntake)
      new TurnBackMag(magIntake, 360)
      // new WaitCommand(0.5),
      // new TurnBackMag(magIntake, 270)
    );
  }
}
