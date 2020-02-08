/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ModelTurret;
import frc.robot.subsystems.TurretMotor;

import java.lang.Math;
import frc.robot.Constants;
public class AutoTurretRotation extends CommandBase {
  //private ModelTurret TurretControlled;
  private TurretMotor motor;
  private int t = 0;
  private int i = 0;
  private final int finalLimit = 500;
  private final double velocity = .25;

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
    if(i == 0) {
      motor.setPower(velocity);
      t += 20;
    }
    if(i == 1) {
      motor.setPower(-velocity);
      t -= 20;
    }
    if(t == 0) {
      i = 0;
      motor.setPower(velocity);
    }
    if(t == finalLimit) {
      i = 1;
      motor.setPower(-velocity);
    }
    
  }

  public boolean endCommand() {
    return motor.getContourArea() > 0.001;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endCommand();
  }
}
