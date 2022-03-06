// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.defaults.HoodDefaultCommand;

public class Hood extends SubsystemBase {
  /** Creates a new Hood. */

  private TalonFX hoodMotor = new TalonFX(13);

  private DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(2);

  private double absoluteEncoderOffset = 0.54;

  private double originalAbsolutePosition = 0;

  private double ticksOffset = 0;

  public Hood() {}

  // method run to set PID values and configure motor settings
  public void init() {
    hoodMotor.config_kP(0, 0.2);
    hoodMotor.config_kI(0, 0.00001);
    hoodMotor.config_kD(0, 1);
    hoodMotor.config_IntegralZone(0, 0.01);

    hoodMotor.configPeakOutputForward(1);
    hoodMotor.configPeakOutputReverse(-1);
    hoodMotor.configVoltageCompSaturation(11.7);
    hoodMotor.setSelectedSensorPosition(0);
    hoodMotor.config_IntegralZone(0, 0.01);

    hoodMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

    setDefaultCommand(new HoodDefaultCommand(this));

  }

  public void setHoodPercent(double percent) {
    // System.out.println("PERCENT: " + percent);
    hoodMotor.set(ControlMode.PercentOutput, percent);
  }

  public boolean getLowerLimitSwitch() {
    if(hoodMotor.getSensorCollection().isRevLimitSwitchClosed() == 1) {
      return false;
    }
    return true;
  }

  public double getSensorPosition() {
    return hoodMotor.getSelectedSensorPosition();
  }

  public double getHoodPosition() {
    double encoderTics = getSensorPosition();
    double hoodPosition = (encoderTics * 360.0 * 24.0)/(2048.0 * 81.0 * 42.0);
    return hoodPosition;
  }

  public void zeroHood() {
    hoodMotor.setSelectedSensorPosition(0);
  }

  public void setHoodPosition(double position) {
    double positionEncoderTics = (2048.0 * 81.0 * (42.0/24.0)) * (position/360.0);
    hoodMotor.set(ControlMode.Position, positionEncoderTics);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
