/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo;

public class ModelTurret extends SubsystemBase {
  private Servo XServo;
  private Servo YServo;
  /**
   * Creates a new ModelTurret.
   */
  public ModelTurret(int xServoChannel,int yServoChannel) {
    this.XServo = new Servo(xServoChannel);
    this.YServo = new Servo(yServoChannel);
  }

  public void setXServoAngle(double angle) {
    XServo.setAngle(angle);
  }

  public void setYServoAngle(double angle) {
    YServo.setAngle(angle);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
