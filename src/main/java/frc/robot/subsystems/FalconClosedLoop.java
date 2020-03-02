/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

class PIDCoefficients {
    double kP,kI,kD,kF;
    PIDCoefficients(double kP,double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }
}

public class FalconClosedLoop extends SubsystemBase {

    private TalonFX Talon; 
    private int PIDLoopId = 0;
    private ControlMode CurrentControlMode;
    private static PIDCoefficients VelocityPIDCoefficients = new PIDCoefficients(.23,0.0004,7,0);
    private static PIDCoefficients PositionPIDCoefficients = new PIDCoefficients(1,0,0,0);
    public double distancePerPulse = 2048;
    public FalconClosedLoop(TalonFX TalonUsed,int PIDLoopId,ControlMode ClosedLoopMode) {
        this.PIDLoopId = PIDLoopId;
        this.Talon = TalonUsed;
        Talon.configFactoryDefault();
        Talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 
        PIDLoopId,
        DriveConstants.MOTOR_TIMEOUT);
        Talon.setSensorPhase(true);
        Talon.configNominalOutputForward(0, DriveConstants.MOTOR_TIMEOUT);
		Talon.configNominalOutputReverse(0, DriveConstants.MOTOR_TIMEOUT);
		Talon.configPeakOutputForward(1, DriveConstants.MOTOR_TIMEOUT);
        Talon.configPeakOutputReverse(-1, DriveConstants.MOTOR_TIMEOUT);
        Talon.getSensorCollection().setIntegratedSensorPositionToAbsolute(DriveConstants.MOTOR_TIMEOUT);
        applyPIDCoefficients(getPIDCoefficientsForControlMode(ClosedLoopMode));
    }
    public PIDCoefficients getPIDCoefficientsForControlMode(ControlMode ControlModeToUse) {
        switch (ControlModeToUse) {
            case Velocity:
                return VelocityPIDCoefficients;
            case Position:
                Talon.getSensorCollection().setIntegratedSensorPositionToAbsolute(DriveConstants.MOTOR_TIMEOUT);
                return PositionPIDCoefficients;
            case PercentOutput:
                return new PIDCoefficients(0, 0, 0, 0);
            default:
                throw new IllegalArgumentException("Did not specify a supported closed loop mode.");
        } 
    }
    private void applyPIDCoefficients(PIDCoefficients CoefficientsToApply) {
		Talon.config_kP(PIDLoopId, CoefficientsToApply.kP, DriveConstants.MOTOR_TIMEOUT);
		Talon.config_kI(PIDLoopId, CoefficientsToApply.kI, DriveConstants.MOTOR_TIMEOUT);
        Talon.config_kD(PIDLoopId, CoefficientsToApply.kD, DriveConstants.MOTOR_TIMEOUT);
        Talon.config_kF(PIDLoopId, CoefficientsToApply.kF, DriveConstants.MOTOR_TIMEOUT);
    }




    public void setClosedLoopMode(ControlMode ClosedLoopMode) {
        this.CurrentControlMode = ClosedLoopMode;
        applyPIDCoefficients(getPIDCoefficientsForControlMode(ClosedLoopMode));


    }

        /* Velocity Closed Loop */
       //copy paste from https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java/VelocityClosedLoop/src/main/java/frc/robot/Robot.java


        /**

         * Convert 500 RPM to units / 100ms.

         * 4096 Units/Rev * 500 RPM / 600 100ms/min in either direction:

         * velocity setpoint is in units/100ms

         */
    public void setVelocity(double velocity) {
        if (!this.CurrentControlMode.equals(ControlMode.Velocity))
            throw new IllegalArgumentException("Cannot set control mode to velocity when motor control mode is not velocity.");
        double targetVelocity_UnitsPer100ms = (velocity * 600/4096);
        Talon.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
    }

    public void setVelocityPercent(double percent) {
        setVelocity(percent*142600.0);
    }
    public void setMotor(double value) {
        Talon.set(CurrentControlMode,value);
    }

    public double getRPM() {
        return Talon.getSelectedSensorVelocity(0) * 600 / 4096;
    }

    public void immediateStop() {
        Talon.set(ControlMode.PercentOutput,0);
    }

    public void setDistancePerPulse(double distancePerPulse){
        this.distancePerPulse = distancePerPulse;
      }
    public void resetEncoder(){
        Talon.setSelectedSensorPosition(0);
      }
      public double getEncoderDistance(){
        return Talon.getSelectedSensorPosition()*distancePerPulse;
      }
      public double getEncoderRate(){
        return Talon.getSelectedSensorVelocity()*distancePerPulse;
      }
      public int getClosedLoopError() {
          return Talon.getClosedLoopError();
      }
}