// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbSequence extends SequentialCommandGroup {
  /** Creates a new ClimbSequence. */
  public ClimbSequence(Climber climber, Carriage carriage) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(climber, carriage);
    addCommands(
      new PositionVerticalClimber(carriage, climber, 21),
      new PositionRotatingClimber(climber, 30),
      new PositionVerticalClimber(carriage, climber, 23),
      new PositionRotatingClimber(climber, 43),
      new PositionVerticalClimber(carriage, climber, 30)
      // new PositionRotatingClimber(climber, 15),
      // new ParallelCommandGroup(new PositionVerticalClimber(carriage, 30), new PositionRotatingClimber(climber, 25)),
      // new PositionVerticalClimber(carriage, 25)
    );
  }
}
