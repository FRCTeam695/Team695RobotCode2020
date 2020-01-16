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
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class FalconClosedLoop extends SubsystemBase {

    private TalonFX Talon; 
    private int timeoutMs = 30;
    private int PIDLoopId = 0;
    public FalconClosedLoop(int talonId) {
        this.Talon = new TalonFX(talonId);
        Talon.configFactoryDefault();
        Talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 
        PIDLoopId,
        timeoutMs);
        Talon.setSensorPhase(true);
        Talon.configNominalOutputForward(0, timeoutMs);
		Talon.configNominalOutputReverse(0, timeoutMs);
		Talon.configPeakOutputForward(1, timeoutMs);
        Talon.configPeakOutputReverse(-1, timeoutMs);
        //ControlledMotor.configAllowableClosedloopError(0, PIDLoopId, timeoutMs);

		Talon.config_kP(PIDLoopId, .23, timeoutMs);
		Talon.config_kI(PIDLoopId, 0.0004, timeoutMs);
        Talon.config_kD(PIDLoopId, 7, timeoutMs);
        Talon.config_kF(PIDLoopId, 0, timeoutMs);


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
        double targetVelocity_UnitsPer100ms = (velocity * 4096 / 600);
        Talon.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);


    }
}