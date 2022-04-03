// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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

  private Lights lights;

  private final VisionCamera visionCamera = new VisionCamera();

  private NetworkTable limeLightTable = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry tableX = limeLightTable.getEntry("tx");
  private NetworkTableEntry tableY = limeLightTable.getEntry("ty");

  private double limeLightX = -1.0;
  private double limeLightY = -1.0;

  private MqttSubscribe mqttSubscribe;

  private final PowerDistribution m_pdh = new PowerDistribution(1, ModuleType.kRev);
  /** Creates a new Peripherals. */
  public Peripherals(MqttSubscribe mqttSubscribe, Lights lights) {
   this.mqttSubscribe = mqttSubscribe;
   this.lights = lights;
  }

  public void init() {
    System.out.print("INSIDE PERIPHERALS INIT");
    zeroNavx();
    turnLightRingOn();
    setDefaultCommand(new PeripheralsDefault(this));
  }

  public double getLimeLightX() {
    limeLightX = Math.PI * (tableX.getDouble(-1.0))/180;
    return limeLightX;
  }

  public double getLimeLightY() {
    limeLightY = tableY.getDouble(-100);
    return limeLightY;
  }

  public double getLimeLightDistanceToTarget() {
    if(getLimeLightY() != -100) {
      return (102.75 * (Math.pow(Math.E, -0.026 * getLimeLightY())));
    }
    return -1.0;
  }

  public boolean hasLock() {
    return visionCamera.hasLock();
  }

  public double[] getVisionArray() {
    return visionCamera.getVisionArray(mqttSubscribe.getLatestMessage());
  }

  public void turnLightRingOn() {
    m_pdh.setSwitchableChannel(true);
  }

  public void turnLightRingOff() {
    m_pdh.setSwitchableChannel(false);
  }

  public double getNavxAngle() {
    // System.out.println("ANGLE: " + navx.getRawAngle() + " YAW: " + navx.getRawYaw() + " PITCH: " + navx.getRawPitch() + " ROLL: " + navx.getRawRoll());
    return navx.currentYaw();
}

  public double getRawNavxAngle() {
    return navx.getRawAngle();
  }

  public double getNavxAngleRotating() {
    return navx.currentAngle();
  }

  public void zeroNavx() {
    navx.softResetYaw();
    navx.softResetPitch();
    // navx.softResetAngle();
}

  public double getNavxYaw(){
    return navx.currentYaw();
  }

  public double getNavxPitch(){
    return -navx.currentPitch();
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
