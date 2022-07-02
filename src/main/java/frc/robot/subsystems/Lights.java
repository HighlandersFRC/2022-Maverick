package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.defaults.LightsDefault;

import com.fasterxml.jackson.databind.deser.impl.CreatorCandidate;

import edu.wpi.first.wpilibj.PWM;

public class Lights extends SubsystemBase {
  private PWM ledPWM;
  private PWM ledPWM2;
  private double currentLedMode;
  private int counter;

  public Lights() {
    ledPWM = new PWM(1);
    ledPWM2 = new PWM(0);
    ledPWM.setBounds(2.0, 1.6, 1.5, 1.4, 1.0);
    currentLedMode = LEDMode.REDONBLUE.value;
    counter = 0;
  }

  public void switchAmericaLights(){
    if (counter % 3 == 0){
      currentLedMode = LEDMode.RED.value;
    } else if (counter % 3 == 1){
      currentLedMode = LEDMode.WHITE.value;
    } else {
      currentLedMode = LEDMode.BLUE.value;
    }
    counter ++;
  }
  // setMode method uses constants from the LEDMode enum
  public void setMode(LEDMode mode){
    currentLedMode = mode.value;
  }
  
  public enum LEDMode{
    BLUE(0.87), RED(0.61), GREEN(0.75), YELLOW(0.67), RAINBOW(-0.97), OFF(0.99), ORANGE(0.65), REDFLASH(-0.11), VIOLET(0.91), REDONBLUE(0.53), BLUEONRED(0.41), WHITE(0.93);

    public final double value;
    private LEDMode(double value){
        this.value = (value + 1.0) / 2.0;
    }
  }

  public void periodic() {
   ledPWM.setPosition(currentLedMode);
   ledPWM2.setPosition(currentLedMode);
  }
  
  public void init() {
    setDefaultCommand(new LightsDefault(this));
  }

  public void autoInit() {

  }

  public void teleopInit() {
    setDefaultCommand(new LightsDefault(this));
  }
}
