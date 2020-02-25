/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AuxiliaryMotorIds;
import frc.robot.enums.RotationDirection;

public class IntakeRake extends SubsystemBase {
  private Solenoid RakePneumaticCylinderLeft = new Solenoid(0); 
  private Solenoid RakePneumaticCylinderRight = new Solenoid(1);
  private AdjustableVictor IntakeDriverMotor = new AdjustableVictor(AuxiliaryMotorIds.INTAKE_VICTOR_ID,RotationDirection.CLOCKWISE);
  /**
   * Creates a new IntakeRake.
   */
  public IntakeRake() {
  }

  public void setDirectionClockwise() {
    IntakeDriverMotor.setDirection(RotationDirection.CLOCKWISE);
  }

  public void setDirectionCounterClockwise() {
    IntakeDriverMotor.setDirection(RotationDirection.COUNTER_CLOCKWISE);
  }
  public void enableMotor() {
    //direction is dynamically controlled behind the scenes.
    IntakeDriverMotor.setPower(1);
  }
  public void disableMotor() {
    
    IntakeDriverMotor.setPower(0);
  }

  public void raiseRake() {
    RakePneumaticCylinderLeft.set(true);
    RakePneumaticCylinderRight.set(true);

  }

  public void lowerRake() {
    RakePneumaticCylinderLeft.set(false);
    RakePneumaticCylinderRight.set(false);

  }

  public void enableRake() {
    lowerRake();
    enableMotor();
  }

  public void disableRake() {
    disableMotor();
    raiseRake();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
