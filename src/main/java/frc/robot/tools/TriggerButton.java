// Copyrights (c) 2018-2019 FIRST, 2020 Highlanders FRC. All Rights Reserved.

package frc.robot.tools;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Button;

public class TriggerButton extends Button {
    private final double THRESHOLD = 0.5;
    private GenericHID joystick;
    private int axis;

    public TriggerButton(GenericHID joystick, int axisNumber) {
        this.joystick = joystick;
        axis = axisNumber;
    }

    public boolean get() {
        return joystick.getRawAxis(axis) >= THRESHOLD;
    }
}