// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.defaults.PeripheralsDefault;
import frc.robot.sensors.Navx;
import frc.robot.sensors.VisionCamera;

public class Peripherals extends SubsystemBase {
  private final AHRS ahrs = new AHRS(Port.kMXP);

  private final Navx navx = new Navx(ahrs);

  private final VisionCamera visionCamera = new VisionCamera();

  private MqttSubscribe mqttSubscribe;

  private final PowerDistribution m_pdh = new PowerDistribution(1, ModuleType.kRev);
  /** Creates a new Peripherals. */
  public Peripherals(MqttSubscribe mqttSubscribe) {
   this.mqttSubscribe = mqttSubscribe;
  }

  public void init() {
    System.out.print("INSIDE PERIPHERALS INIT");
    zeroNavx();
    setDefaultCommand(new PeripheralsDefault(this));
  }

  public double[] getVisionArray() {
    return visionCamera.getVisionArray(mqttSubscribe.getLatestMessage());
  }

  public void turnLightRingOn() {
    m_pdh.setSwitchableChannel(false);
  }

  public void turnLightRingOff() {
    m_pdh.setSwitchableChannel(true);
  }

  public double getNavxAngle() {
    // System.out.println("ANGLE: " + navx.getRawAngle() + " YAW: " + navx.getRawYaw() + " PITCH: " + navx.getRawPitch() + " ROLL: " + navx.getRawRoll());
    return navx.currentYaw();
}

  public double getRawNavxAngle() {
    return navx.getRawAngle();
  }

  public void zeroNavx() {
    navx.softResetYaw();
}

  public double getNavxYaw(){
    return navx.currentYaw();
  }

  public double getNavxPitch(){
    return navx.currentPitch();
  }

  public double getNavxRoll(){
    return navx.currentRoll();
  }

  public double getNavxRate() {
    return navx.getAngleRate();
  }

  public void setNavxAngle(double angle) {
    navx.setNavxAngle(angle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
