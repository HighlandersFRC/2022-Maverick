// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.defaults.MagIntakeDefault;
import frc.robot.tools.PneumaticsControl;

public class MagIntake extends SubsystemBase {
  /** Creates a new MagIntake. */

  private final PneumaticsControl pneumatics;

  private final TalonFX backMagazine = new TalonFX(15);
  private final TalonFX frontMagazine = new TalonFX(14);
  private final TalonFX intakeMotor = new TalonFX(12);

  public MagIntake(PneumaticsControl pneumatics) {
    this.pneumatics = pneumatics;
  }
  public void init() {
    backMagazine.configFactoryDefault();
    frontMagazine.configFactoryDefault();
    intakeMotor.configFactoryDefault();

    // backMagazine.config_kP(0, 0.75);
    // backMagazine.config_kI(0, 0);
    // backMagazine.config_kD(0, 0);
    // backMagazine.config_IntegralZone(0, 0);

    backMagazine.config_kP(0, 0.25);
    backMagazine.config_kI(0, 0.0000025);
    backMagazine.config_kD(0, 0);
    backMagazine.config_IntegralZone(0, 0);

    frontMagazine.config_kP(0, 0.2);
    frontMagazine.config_kI(0, 0.0);
    frontMagazine.config_kD(0, 0.0);
    frontMagazine.config_IntegralZone(0, 0);

    backMagazine.setNeutralMode(NeutralMode.Brake);
    frontMagazine.setNeutralMode(NeutralMode.Brake);

    setDefaultCommand(new MagIntakeDefault(this));
    backMagazine.setNeutralMode(NeutralMode.Brake);
    intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100);
    intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 100);
    intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 1000);
    intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 10000);
    intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 1000);
    intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 1000);
    intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 1000);
    intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 1000);
    intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 1000);

    frontMagazine.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100);
    frontMagazine.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 100);
    frontMagazine.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 1000);
    frontMagazine.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 10000);
    frontMagazine.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 1000);
    frontMagazine.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 1000);
    frontMagazine.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 1000);
    frontMagazine.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 1000);
    frontMagazine.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 1000);
  
    backMagazine.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100);
    backMagazine.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 100);
    backMagazine.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 1000);
    backMagazine.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 10000);
    backMagazine.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 1000);
    backMagazine.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 1000);
    backMagazine.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 1000);
    backMagazine.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 1000);
    backMagazine.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 1000);
  }

  public double rotateBackMag(double degrees) {
    double motorTics = (degrees / 360.0) * (68.0 / 18.0) * 2048.0 + backMagazine.getSelectedSensorPosition();
    backMagazine.set(ControlMode.Position, motorTics);
    return motorTics;
  }

  public double rotateFrontMag(double degrees) {
    double motorTics = (degrees / 360.0) * (4.0 / 3.0) * 2048.0 + frontMagazine.getSelectedSensorPosition();
    frontMagazine.set(ControlMode.Position, motorTics);
    return motorTics;
  }

  public double getBackMagPosition() {
    return backMagazine.getSelectedSensorPosition();
  }

  public double getFrontMagPosition() {
    return frontMagazine.getSelectedSensorPosition();
  }

  public void setFrontMagRPM(double rpm) {
    frontMagazine.set(ControlMode.Velocity, Constants.shooterRPMToUnitsPer100MS(rpm));
  }

  public void setBackMagRPM(double rpm) {
    backMagazine.set(ControlMode.Velocity, Constants.shooterRPMToUnitsPer100MS(rpm));
  }

  public double getFrontMagRPM() {
    return Constants.unitsPer100MsToRPM(frontMagazine.getSelectedSensorVelocity());
  }

  public double getBackMagRPM() {
    return Constants.unitsPer100MsToRPM(backMagazine.getSelectedSensorVelocity());
  }

  private final DigitalInput lowerBackBeamBreak = new DigitalInput(2);
  private final DigitalInput upperBeamBreak = new DigitalInput(1);

  public Boolean getLowerBackBeamBreak() {
      return lowerBackBeamBreak.get();
  }

  public Boolean getUpperBeamBreak() {
      return upperBeamBreak.get();
  }

  public void moveMagazine() {  
      if(!getUpperBeamBreak()){
        backMagazine.set(ControlMode.PercentOutput, 0.0);
        frontMagazine.set(ControlMode.PercentOutput, 0.0);
      } else {
          frontMagazine.set(ControlMode.PercentOutput, -0.5);
          if(!getLowerBackBeamBreak()){
            backMagazine.set(ControlMode.PercentOutput, 0.3);
          } else{
            backMagazine.set(ControlMode.PercentOutput, 0.0);
          }
      }
  }

  public void setMagazinePercents(double backPercent, double frontPercent) {
    backMagazine.set(ControlMode.PercentOutput, backPercent);
    frontMagazine.set(ControlMode.PercentOutput, frontPercent);
  }

  public void postGreenWheelVelocity() {
    SmartDashboard.putNumber("GW VELOCITY", getBackMagRPM());
  }

  public void setBackMagazine(double percent) {
    backMagazine.set(ControlMode.PercentOutput, percent);
  } 
public void setFrontMagazine(double percent) {
    frontMagazine.set(ControlMode.PercentOutput, -percent);
  }

  public void setIntakeUp() {
    pneumatics.setIntakeUp();
  }

  public void setIntakeDown() {
    pneumatics.setIntakeDown();
  }

  public void setIntakePercent(double percent) {
    intakeMotor.set(ControlMode.PercentOutput, -percent);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
