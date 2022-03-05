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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FireBalls extends SequentialCommandGroup {
  /** Creates a new FireBalls. */
  public FireBalls(Drive drive, MagIntake magIntake, Shooter shooter, Hood hood, Peripherals peripherals, Lights lights, double hoodPosition, double shooterRPM, double firstBallTimeout, double secondBallTimeout) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(drive, magIntake, shooter, hood);
    double distance = drive.getDistanceToTarget();
    addCommands(
      new ParallelCommandGroup(
          new SpinShooter(shooter, shooterRPM, distance),
          new FaceTarget(drive),
          new SetHoodPosition(hood, hoodPosition, distance)
      ),
      new TurnBackMag(magIntake, 360),
      //new EjectBalls(magIntake, lights, 0.55, 0.85, firstBallTimeout),
      //new EjectBalls(magIntake, 0.0, 0, 0.001),
      new WaitCommand(0.5),
      //new EjectBalls(magIntake, lights, 0.35, 0.45, secondBallTimeout)
      //new EjectBalls(magIntake, 0.0, 0, 0.1)
      //new TurnBackMag(magIntake)
      new TurnBackMag(magIntake, 1080)
      // new WaitCommand(0.5),
      // new TurnBackMag(magIntake, 270)
    );
  }
}
