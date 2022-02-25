// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.defaults.MagIntakeDefault;
import frc.robot.tools.PneumaticsControl;

public class MagIntake extends SubsystemBase {
  /** Creates a new MagIntake. */

  private final PneumaticsControl pneumatics;

  public MagIntake(PneumaticsControl pneumatics) {
    this.pneumatics = pneumatics;
  }

  private final TalonFX backMagazine = new TalonFX(15);
  private final TalonFX frontMagazine = new TalonFX(14);
  private final TalonFX intakeMotor = new TalonFX(12);

  public void init() {
    setDefaultCommand(new MagIntakeDefault(this));
    backMagazine.setNeutralMode(NeutralMode.Brake);
  }

  private final DigitalInput lowerBackBeamBreak = new DigitalInput(0);
  private final DigitalInput upperBeamBreak = new DigitalInput(1);

  public Boolean getLowerBackBeamBreak() {
      return lowerBackBeamBreak.get();
  }

  public Boolean getUpperBeamBreak() {
      return upperBeamBreak.get();
  }

  public void moveMagazine() {  
    System.out.println("BOTTOM: " + getLowerBackBeamBreak() + " UPPER: " + getUpperBeamBreak());
      if(!getUpperBeamBreak()){
        backMagazine.set(ControlMode.PercentOutput, 0.0);
      } else {
        if(!getLowerBackBeamBreak()){
          backMagazine.set(ControlMode.PercentOutput, 0.3);
          frontMagazine.set(ControlMode.PercentOutput, 0.5);
        } else{
          backMagazine.set(ControlMode.PercentOutput, 0.0);
        }
      }
         
  }

  public void setMagazinePercents(double backPercent, double frontPercent) {
    backMagazine.set(ControlMode.PercentOutput, backPercent);
    frontMagazine.set(ControlMode.PercentOutput, frontPercent);
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
    frontMagazine.set(ControlMode.PercentOutput, percent);
    intakeMotor.set(ControlMode.PercentOutput, percent);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
