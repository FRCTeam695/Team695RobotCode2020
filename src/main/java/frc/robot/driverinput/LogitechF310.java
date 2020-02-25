/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.driverinput;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/*
 * COntroller button indicies:
 * A: 1
 * B: 2
 * X: 3
 * Y: 4
 * 
 * Left X-axis: 0
 * Left Y-axis: 1
 * 
 * POV:
 *      0
 *      ^
 *      |
 * 270<- -> 90
 *      |
 *      v
 *     180
 * Purpose of this class: Unpack all the buttons and axes out of logitech f310 so that it never has to be done again ever
 **/
public class LogitechF310 {
    public final Joystick WrappedJoystick;
    public final JoystickButton AButton;
    public final JoystickButton BButton;
    public final JoystickButton XButton;
    public final JoystickButton YButton;
    public final Supplier<Vector2d> LeftJoystick, RightJoystick;
    public final DoubleSupplier LeftTrigger,RightTrigger;
    public final ControllerAxisToggle RightTriggerAsButton,LeftTriggerAsButton;
    public final POVButton POVTop,POVTopRight,POVRight,POVBottomRight,POVBottom,POVBottomLeft,POVLeft,POVTopLeft;
    public final JoystickButton LeftBumper,RightBumper;

    public LogitechF310(Joystick WrappedJoystick) {
        this.WrappedJoystick = WrappedJoystick;
        AButton = new JoystickButton(WrappedJoystick,1);
        BButton = new JoystickButton(WrappedJoystick,2);
        XButton = new JoystickButton(WrappedJoystick,3);
        YButton = new JoystickButton(WrappedJoystick,4);
        LeftJoystick = () -> {return new Vector2d(WrappedJoystick.getRawAxis(0),WrappedJoystick.getRawAxis(1));};
        RightJoystick = () -> {return new Vector2d(WrappedJoystick.getRawAxis(4),WrappedJoystick.getRawAxis(5));};
        LeftTrigger = () -> {return WrappedJoystick.getRawAxis(2);};
        RightTrigger = () -> {return WrappedJoystick.getRawAxis(3);};
        LeftBumper = new JoystickButton(WrappedJoystick, 5); //TODO: FIGURE OUT BUMPER BUTTON MAPS and trigger axis
        RightBumper = new JoystickButton(WrappedJoystick, 6); //TODO: FIGURE OUT BUMPER BUTTON MAPS
        RightTriggerAsButton = new ControllerAxisToggle(WrappedJoystick, 2);
        LeftTriggerAsButton = new ControllerAxisToggle(WrappedJoystick, 3);

        //povs 
        POVTop = new POVButton(WrappedJoystick,0);
        POVTopRight = new POVButton(WrappedJoystick, 45);
        POVRight = new POVButton(WrappedJoystick, 90);
        POVBottomRight = new POVButton(WrappedJoystick, 135);
        POVBottom = new POVButton(WrappedJoystick, 180);
        POVBottomLeft = new POVButton(WrappedJoystick, 225);
        POVLeft = new POVButton(WrappedJoystick, 270);
        POVTopLeft = new POVButton(WrappedJoystick, 315);

        
    }
}
