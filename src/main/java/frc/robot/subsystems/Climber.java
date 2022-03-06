// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.defaults.ClimberDefault;
import frc.robot.tools.PneumaticsControl;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  private final TalonFX leftClimber = new TalonFX(20);
  private final TalonFX rightClimber = new TalonFX(21);
  private final TalonFX rotatingMotor = new TalonFX(22);

  private final DutyCycleEncoder climberEncoder = new DutyCycleEncoder(3);

  private double climberGearRatio = 30.0/27.0;

  private PneumaticsControl pneumatics;

  public Climber(PneumaticsControl pneumaticsControl) {
    this.pneumatics = pneumaticsControl;
  }

  public void init() {
    setDefaultCommand(new ClimberDefault(this));

    rightClimber.setSelectedSensorPosition(0);
    leftClimber.setSelectedSensorPosition(0);
    rotatingMotor.setSelectedSensorPosition(climberEncoder.get());

    rotatingMotor.configPeakOutputForward(1);
    rotatingMotor.configPeakOutputReverse(-1);

    leftClimber.configPeakOutputForward(1);
    leftClimber.configPeakOutputReverse(-1);
    leftClimber.setInverted(true);

    leftClimber.config_kF(0, 0);
    leftClimber.config_kP(0, 1);
    leftClimber.config_kI(0, 0);
    leftClimber.config_kD(0, 0);

    rightClimber.configPeakOutputForward(1);
    rightClimber.configPeakOutputReverse(-1);

    rightClimber.config_kF(0, 0);
    rightClimber.config_kP(0, 1);
    rightClimber.config_kI(0, 0);
    rightClimber.config_kD(0, 0);
  }

  public double getRightClimberPosition() {
    return rightClimber.getSelectedSensorPosition();
  }

  public double getLeftClimberPosition() {
    return leftClimber.getSelectedSensorPosition();
  }

  public double getRotatingEncoder() {
    return climberEncoder.get();
  }

  public void zeroRotatingEncoder() {
    climberEncoder.reset();
  }

  public void lockExtendingClimber() {
    pneumatics.engageClimberBrake();
  }

  public void unlockExtendingClimber() {
    pneumatics.releaseClimberBrake();
  }

  public void setRotatingClimberPercent(double percent) {
    rotatingMotor.set(ControlMode.PercentOutput, percent);
  }

  public void setClimberPercents(double leftPercent, double rightPercent) {
    leftClimber.set(ControlMode.PercentOutput, leftPercent);
    rightClimber.set(ControlMode.PercentOutput, rightPercent);
  }

  public double getLeftClimberCurrent() {
    return leftClimber.getStatorCurrent();
  }

  public double getRightClimberCurrent() {
    return rightClimber.getStatorCurrent();
  }

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
