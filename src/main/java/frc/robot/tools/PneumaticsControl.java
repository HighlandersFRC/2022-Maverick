package frc.robot.tools;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class PneumaticsControl {
    
    private final PneumaticsControlModule hub = new PneumaticsControlModule();

    private final DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    // private final DoubleSolenoid leftClimberBrake = new DoubleSolenoid(PneumaticsModuleType.REVPH, 10, 11);

    public void setIntakeUp() {
        intakeSolenoid.set(Value.kForward);
    }

    public void setIntakeDown() {
        intakeSolenoid.set(Value.kReverse);
    }

    // public void engageClimberBrake() {
    //     leftClimberBrake.set(Value.kReverse);
    // }

    // public void releaseClimberBrake() {
    //     leftClimberBrake.set(Value.kForward);
    // }

}
