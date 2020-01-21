/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FalconClosedLoop;


public class ColorWheelRotations extends CommandBase {
  /**
   * Creates a new EnableCIMClosedLoop.
   */
  FalconClosedLoop closedLoop;
  double velocity = 0;
  public ColorWheelRotations(FalconClosedLoop loop,double velocity) {
    this.closedLoop = loop;
    this.velocity = velocity;
    loop.setClosedLoopMode(ControlMode.Velocity);
    addRequirements(loop);
  }
  int count;

  double finalRotationsThreshold = 4;
  double motorCircumference = 2*Math.PI*2.5;//cm enter radius
  double colorWheelCircumference = 2*Math.PI*41.5;//enter radius cm
  double mechAdvantage = colorWheelCircumference/motorCircumference;
  double motorRotationsThreshold = finalRotationsThreshold*mechAdvantage;
  double motorRotations = 0.0;
  velocity = 1;
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
