/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class FalconClosedLoop extends SubsystemBase {

    private TalonFX ControlledMotor; 
    private int timeoutMs = 30;
    private int PIDLoopId = 0;
    public FalconClosedLoop(int talonId) {
        this.ControlledMotor = new TalonFX(talonId);
        ControlledMotor.configFactoryDefault();
        ControlledMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 
        PIDLoopId,
        timeoutMs);
        ControlledMotor.setSensorPhase(true);
        ControlledMotor.configNominalOutputForward(0, timeoutMs);
		ControlledMotor.configNominalOutputReverse(0, timeoutMs);
		ControlledMotor.configPeakOutputForward(1, timeoutMs);
        ControlledMotor.configPeakOutputReverse(-1, timeoutMs);
        //ControlledMotor.configAllowableClosedloopError(0, PIDLoopId, timeoutMs);

		ControlledMotor.config_kP(PIDLoopId, .25, timeoutMs);
		ControlledMotor.config_kI(PIDLoopId, 0.001, timeoutMs);
        ControlledMotor.config_kD(PIDLoopId, 20, timeoutMs);
        ControlledMotor.config_kF(PIDLoopId, 1023.0/7200.0, timeoutMs);


    }
    //StringBuilder _sb = new StringBuilder();
    //ON GITHUB

        /* Velocity Closed Loop */
       //copy paste from https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java/VelocityClosedLoop/src/main/java/frc/robot/Robot.java


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

        _sb.append(motorS1.getClosedLoopError(PIDLoopId));

        _sb.append("\ttrg:");

        _sb.append(targetVelocity_UnitsPer100ms);*/

    }
}