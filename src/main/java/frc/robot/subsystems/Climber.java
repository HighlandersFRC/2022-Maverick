// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.commands.defaults.ClimberDefault;
import frc.robot.tools.PneumaticsControl;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  private final TalonFX leftClimber = new TalonFX(20);
  private final TalonFX rightClimber = new TalonFX(21);

  private final DutyCycleEncoder climberEncoder = new DutyCycleEncoder(3);

  private double climberGearRatio = 30.0/27.0;

  private PneumaticsControl pneumatics;

  public Climber(PneumaticsControl pneumaticsControl) {
    this.pneumatics = pneumaticsControl;
  }

  public void init() {
    // setDefaultCommand(new ClimberDefault(this));
    leftClimber.config_kF(0, 0);
    leftClimber.config_kP(0, 0);
    leftClimber.config_kI(0, 0);
    leftClimber.config_kD(0, 0);

    rightClimber.config_kF(0, 0);
    rightClimber.config_kP(0, 0);
    rightClimber.config_kI(0, 0);
    rightClimber.config_kD(0, 0);
  }

  public double getClimberEncoder() {
      return climberEncoder.get();
  }

  public void setClimberPercents(double percent) {
    leftClimber.set(ControlMode.PercentOutput, percent);
    rightClimber.set(ControlMode.PercentOutput, percent);
  }

//   public void engageClimberBrake() {
//     pneumatics.engageClimberBrake();
//   }

//   public void releaseClimberBrake() {
//     pneumatics.releaseClimberBrake();
//   }

  public void setClimberPosition(double position) {
    leftClimber.set(ControlMode.Position, getClimberTics(position));
    rightClimber.set(ControlMode.Position, getClimberTics(position));
  }

  public double getClimberTics(double rotations) {
    return rotations * climberGearRatio;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
