/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretMotor;

import java.lang.Math;
import frc.robot.Constants;
public class AutoTurretRotation extends CommandBase {
  //private ModelTurret TurretControlled;
  private TurretMotor motor;
  private int t = 0;
  private int i = 0;
  private final int finalLimit = 1000;
  private final double velocity = .5;

  //private int panDirection = 1;
  /**
   * Creates a new SetTurretRotation.
   */
  public AutoTurretRotation(TurretMotor motor) {
    this.motor = motor;
    addRequirements(motor);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initialize() {
    
  }

  //second version of execute
  @Override
  public void execute() {
    if(!motor.isTooFarLeft() && !motor.isTooFarRight() && i==0)
      motor.setPower(velocity);
    if(!motor.isTooFarLeft() && !motor.isTooFarRight() && i==1)
      motor.setPower(-velocity);
    if(motor.isTooFarLeft()) {
      i=1;
      motor.setPower(-velocity);
    }
    if(motor.isTooFarRight()) {
      i=0;
      motor.setPower(velocity);
    }
    
  }

  public boolean endCommand() {
    System.out.println(motor.getAzimuth());
    return motor.getContourArea() >= 0.02;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return endCommand();
  }
}
