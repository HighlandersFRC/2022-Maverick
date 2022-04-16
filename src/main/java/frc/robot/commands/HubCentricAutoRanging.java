// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Shooter;
import frc.robot.tools.ShotAdjuster;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HubCentricAutoRanging extends SequentialCommandGroup {
  /** Creates a new HubCentricAutoRanging. */
  
  public HubCentricAutoRanging(Drive drive, Hood hood, Shooter shooter, ShotAdjuster adjuster, Peripherals peripherals) {
    
    addRequirements(drive, hood, shooter);

    addCommands(new ParallelCommandGroup(new HubCentricDrive(drive, peripherals), new SetHoodPosition(hood, peripherals, 0, adjuster, true), new SpinShooter(shooter, peripherals, 1400, adjuster, true)));
  }
}
