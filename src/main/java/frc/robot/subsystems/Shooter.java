// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.defaults.ShooterDefault;

public class Shooter extends SubsystemBase {

 private final TalonFX leftShooter = new TalonFX(10);
 private final TalonFX rightShooter = new TalonFX(9);

  /** Creates a new Intake. */
  public Shooter() {
    
  }

  // method run to set PID values and sets motors to Coast
  public void init() {
     leftShooter.setNeutralMode(NeutralMode.Coast);
     rightShooter.setNeutralMode(NeutralMode.Coast);
     rightShooter.setInverted(InvertType.InvertMotorOutput);

    rightShooter.configStatorCurrentLimit(new StatorCurrentLimitConfiguration());
    rightShooter.config_kF(0, 0.02);
    rightShooter.config_kP(0, 0.18);
    rightShooter.config_kI(0, 0.0001);
    rightShooter.config_kD(0, 0.025);
    leftShooter.set(ControlMode.Follower, 9);
    setDefaultCommand(new ShooterDefault(this));
  }

  // zeroes shooter encoder
  public void zeroShooterEncoder() {
    leftShooter.setSelectedSensorPosition(0);
}

  // sets shooter motor to a specified percent
  public void setShooterPercent(double percent) {
    rightShooter.set(ControlMode.PercentOutput, percent);
  }

  // sets shooter to run a velocity PID at given RPM
  public void setShooterRPM(double rpm) {
    rightShooter.set(ControlMode.Velocity, Constants.shooterRPMToUnitsPer100MS(rpm));
}

// converts shooter RPM to units/100 ms
public double getShooterRPM(){
  return Constants.unitsPer100MsToRPM(rightShooter.getSelectedSensorVelocity());
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}