/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package edu.wpi.first.wpilibj;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.SpeedController;

/**
 * Interface for speed controlling devices.
 */
public interface falconSpeedController extends SpeedController {
  private final FalconClosedLoop motor;
  public falconSpeedController(int talonId, int PIDLoopId, int timeoutMs){
   this.motor = new FalconClosedLoop(talonId, PIDLoopId, timeoutMs, ControlMode.Velocity)
  }
  /**
   * Common interface for setting the speed of a speed controller.
   *
   * @param speed The speed to set. Value should be between -1.0 and 1.0.
 * @return 
   */
  @Override
  double Velocity;
  public default void set(double speed){
    Velocity = speed;
    motor.setVelocity(Velocity);
  };

  /**
   * Sets the voltage output of the SpeedController.  Compensates for the current bus
   * voltage to ensure that the desired voltage is output even if the battery voltage is below
   * 12V - highly useful when the voltage outputs are "meaningful" (e.g. they come from a
   * feedforward calculation).
   *
   * <p>NOTE: This function *must* be called regularly in order for voltage compensation to work
   * properly - unlike the ordinary set function, it is not "set it and forget it."
   *@deprecated
   
   * @param outputVolts The voltage to output.
   */
  void setVoltage(double outputVolts);

  /**
   * Common interface for getting the current set speed of a speed controller.
   *
   * @return The current set speed. Value is between -1.0 and 1.0.
   */
  @Override
  default double get() {
    return Velocity;
  }

  /**
   * Common interface for inverting direction of a speed controller.
   *
   * @param isInverted The state of inversion true is inverted.
   */
  @Override
  void setInverted(boolean isInverted);

  /**
   * Common interface for returning if a speed controller is in the inverted state or not.
   *
   * @return isInverted The state of the inversion true is inverted.
   */
  @Override
  boolean getInverted();

  /**
   * Disable the speed controller.
   */
  @Override
  void disable();

  /**
   * Stops motor movement. Motor can be moved again by calling set without having
   * to re-enable the motor.
   */
  @Override
  default void stopMotor() {
    motor.immediateStop();
  };
}