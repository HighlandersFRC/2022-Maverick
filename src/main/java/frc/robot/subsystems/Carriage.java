// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.ejml.dense.block.decomposition.chol.InnerCholesky_DDRB;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.defaults.CarriageDefault;
import frc.robot.commands.defaults.ClimberDefault;
import frc.robot.tools.PneumaticsControl;

public class Carriage extends SubsystemBase {
  /** Creates a new Carriage. */

  private final TalonFX climberFalcon1 = new TalonFX(20);
  private final TalonFX climberFalcon2 = new TalonFX(21);
  // private final CANSparkMax rotatingMotor = new CANSparkMax(22, MotorType.kBrushless);
  private double maxPos = Constants.getClimberFalconTics(30);
  private PneumaticsControl pneumatics;

  public Carriage(PneumaticsControl pneumaticsControl) {
    this.pneumatics = pneumaticsControl;
  }

  public void init() {
    setDefaultCommand(new CarriageDefault(this));

    climberFalcon1.configFactoryDefault();
    climberFalcon2.configFactoryDefault();

    climberFalcon2.setSelectedSensorPosition(0);
    climberFalcon1.setSelectedSensorPosition(0);

    // climberFalcon1.s

    climberFalcon1.configPeakOutputForward(0.8);
    climberFalcon1.configPeakOutputReverse(-0.8);
    climberFalcon2.configPeakOutputForward(0.8);
    climberFalcon2.configPeakOutputReverse(-0.8);
    climberFalcon1.setInverted(false);

    // rotatingMotor.set

    climberFalcon1.config_kF(0, 0);
    climberFalcon1.config_kP(0, 0.05);
    climberFalcon1.config_kI(0, 0.000005);
    climberFalcon1.config_kD(0, 0.005);

    climberFalcon1.config_kF(1, 0);
    climberFalcon1.config_kP(1, 1);
    climberFalcon1.config_kI(1, 0);
    climberFalcon1.config_kD(1, 0);

    // climberFalcon1.configAllowableClosedloopError(0, Constants.getClimberFalconTics(0.1), 10);
    climberFalcon1.configForwardSoftLimitEnable(false);
    climberFalcon1.configReverseSoftLimitEnable(false);
    climberFalcon1.configForwardSoftLimitThreshold(maxPos);
    climberFalcon1.configReverseSoftLimitThreshold(0);

    climberFalcon2.configForwardSoftLimitEnable(false);
    climberFalcon2.configReverseSoftLimitEnable(false);
    climberFalcon2.configForwardSoftLimitThreshold(maxPos);
    climberFalcon2.configReverseSoftLimitThreshold(0);
    climberFalcon2.set(ControlMode.Follower, 20);

    climberFalcon1.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 80, 80, 500));
    climberFalcon1.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 35, 500));

    climberFalcon1.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100);
    climberFalcon1.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 100);
    climberFalcon1.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 1000);
    climberFalcon1.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 10000);
    climberFalcon1.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 1000);

    climberFalcon2.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100);
    climberFalcon2.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 100);
    climberFalcon2.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 1000);
    climberFalcon2.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 10000);
    climberFalcon2.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 1000);
  }

  public double getclimberFalcon1Position() {
    return Constants.getVerticalClimberInches(climberFalcon1.getSelectedSensorPosition());
  }

  public void setClimberFalconsPosition(double inches) {
    // if (position < 0) {
    //   position = 0;
    // } else if (position > maxPos) {
    //   position = maxPos;
    // }
    climberFalcon1.selectProfileSlot(0, 0);
    double ticks = Constants.getClimberFalconTics(inches);

    climberFalcon1.set(ControlMode.Position, ticks);
  }

  public void setClimberPercents(double percentOne) {
    climberFalcon1.set(ControlMode.PercentOutput, percentOne);
  }

  public void loadedRobotClimb(double inches) {
    climberFalcon1.selectProfileSlot(1, 0);
    double ticks = Constants.getClimberFalconTics(inches);

    climberFalcon1.set(ControlMode.Position, ticks);
  }

  public double getCarriageFalconCurrent() {
    return climberFalcon1.getSupplyCurrent();
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
