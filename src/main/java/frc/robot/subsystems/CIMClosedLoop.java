/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class CIMClosedLoop extends SubsystemBase {

	private VictorSPX ControlledMotor; 
    public CIMClosedLoop(VictorSPX MotorToControl) {
        this.ControlledMotor = MotorToControl;
    }
    //StringBuilder _sb = new StringBuilder();
    //ON GITHUB

        /* Velocity Closed Loop */



        /**

         * Convert 500 RPM to units / 100ms.

         * 4096 Units/Rev * 500 RPM / 600 100ms/min in either direction:

         * velocity setpoint is in units/100ms

         */
    public void setVelocity(double velocity) {
        double targetVelocity_UnitsPer100ms = (velocity * 500.0 * 4096 / 600);

        /* 500 RPM in either direction */

        ControlledMotor.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);



        /* Append more signals to print when in speed mode. */
        /*
        _sb.append("\terr:");

        _sb.append(motorS1.getClosedLoopError(Constants.kPIDLoopIdx));

        _sb.append("\ttrg:");

        _sb.append(targetVelocity_UnitsPer100ms);*/

    }
}