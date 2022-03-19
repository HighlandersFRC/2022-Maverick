// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.defaults.ClimberDefault;
import frc.robot.tools.PneumaticsControl;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  private final TalonFX climberFalcon1 = new TalonFX(20);
  private final TalonFX climberFalcon2 = new TalonFX(21);
  private final CANSparkMax rotatingMotor = new CANSparkMax(22, MotorType.kBrushless);
  private double maxPos = 100;
  private PneumaticsControl pneumatics;

  public Climber(PneumaticsControl pneumaticsControl) {
    this.pneumatics = pneumaticsControl;
  }

  public void init() {
    setDefaultCommand(new ClimberDefault(this));

    climberFalcon1.configFactoryDefault();
    climberFalcon2.configFactoryDefault();

    rotatingMotor.getEncoder().setPosition(0);
    rotatingMotor.setSmartCurrentLimit(20, 20);

    climberFalcon2.setSelectedSensorPosition(0);
    climberFalcon1.setSelectedSensorPosition(0);
    rotatingMotor.getEncoder().setPosition(0);

    climberFalcon1.configPeakOutputForward(1);
    climberFalcon1.configPeakOutputReverse(-1);
    climberFalcon2.configPeakOutputForward(1);
    climberFalcon2.configPeakOutputReverse(-1);
    climberFalcon1.setInverted(true);

    climberFalcon1.config_kF(0, 0);
    climberFalcon1.config_kP(0, 0);
    climberFalcon1.config_kI(0, 0);
    climberFalcon1.config_kD(0, 0);

    climberFalcon1.configForwardSoftLimitEnable(true);
    climberFalcon1.configReverseSoftLimitEnable(true);
    climberFalcon1.configForwardSoftLimitThreshold(maxPos);
    climberFalcon1.configReverseSoftLimitThreshold(0);

    climberFalcon2.configForwardSoftLimitEnable(true);
    climberFalcon2.configReverseSoftLimitEnable(true);
    climberFalcon2.configForwardSoftLimitThreshold(maxPos);
    climberFalcon2.configReverseSoftLimitThreshold(0);
    climberFalcon2.set(ControlMode.Follower, 20);
  }

  public double getRotatingMotorPosition() {
    return rotatingMotor.getEncoder().getPosition();
  }

  public void zeroRotatingMotor(double position) {
    rotatingMotor.getEncoder().setPosition(position);
  }

  public void setRotatingMotorPosition(double position) {
    rotatingMotor.getPIDController().setReference(position, ControlType.kPosition);
  }

  public void setRotatingMotorPercent(double percent) {
    rotatingMotor.set(percent);
  }

  public double getclimberFalcon2Position() {
    return climberFalcon2.getSelectedSensorPosition();
  }

  public double getclimberFalcon1Position() {
    return climberFalcon1.getSelectedSensorPosition();
  }

  public void setClimberFalconsPosition(double position) {
    // if (position < 0) {
    //   position = 0;
    // } else if (position > maxPos) {
    //   position = maxPos;
    // }

    climberFalcon1.set(ControlMode.Position, position);
    climberFalcon2.set(ControlMode.Position, position);
  }

  public void lockExtendingClimber() {
    pneumatics.engageClimberBrake();
  }

  public void unlockExtendingClimber() {
    pneumatics.releaseClimberBrake();
  }

  public void setClimberPercents(double percentOne, double percentTwo) {
    climberFalcon1.set(ControlMode.PercentOutput, percentOne);
    climberFalcon2.set(ControlMode.PercentOutput, percentTwo);
  }

  public double getclimberFalcon1Current() {
    return climberFalcon1.getStatorCurrent();
  }

  public double getclimberFalcon2Current() {
    return climberFalcon2.getStatorCurrent();
  }

  public Boolean getClimberLimitSwitch() {
    if(climberFalcon1.getSensorCollection().isRevLimitSwitchClosed() == 1 || climberFalcon2.getSensorCollection().isRevLimitSwitchClosed() == 1) {
      return false;
    }
    return true;
  }

  public void zeroClimberFalcons() {
    climberFalcon1.setSelectedSensorPosition(0);
    climberFalcon2.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {}
}
