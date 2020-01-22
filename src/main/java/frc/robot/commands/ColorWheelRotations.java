/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

//java.util.Date javaDate = new java.util.Date();
//FixedColorWheelConstants Constants = new FixedColorWheelConstants();

import frc.robot.Constants;
import static frc.robot.Constants.FixColorConst;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FalconClosedLoop;


public class ColorWheelRotations extends CommandBase {
  /**
   * Creates a new EnableCIMClosedLoop.
   */
  FalconClosedLoop closedLoop;
  public ColorWheelRotations(FalconClosedLoop loop,double velocity) {
    this.closedLoop = loop;
    this.velocity = 0;
    loop.setClosedLoopMode(ControlMode.Velocity);
    addRequirements(loop);
  }

  double finalRotationsThreshold = FixColorConst.finalRotationsThreshold;
  double motorCircumference = 2*Math.PI*FixColorConst.motorWheelRadius;//cm enter radius
  double colorWheelCircumference = 2*Math.PI*FixColorConst.colorWheelRadius;//enter radius cm
  double mechAdvantage = colorWheelCircumference/motorCircumference;
  double motorRotationsThreshold = (finalRotationsThreshold*mechAdvantage);
  double motorRotations = 0;
  double velocity = 1;
  double maxVelocity = 10*mechAdvantage;//number is color wheel rpm
  double a = -1*(velocity - maxVelocity)/(Math.pow(motorRotationsThreshold,2)/4);

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    closedLoop.setVelocity(velocity); //Determine Velocity
    motorRotations += velocity/3000;
    velocity = -1*a*Math.pow((motorRotations-(motorRotationsThreshold/2)),2)+maxVelocity;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (motorRotations >= motorRotationsThreshold){
      return true;
    } else {
      return false;
    }
  }
}
