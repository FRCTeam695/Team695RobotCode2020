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


public class EnableFalconPositionClosedLoop extends CommandBase {
  /**
   * Creates a new EnableCIMClosedLoop.
   */
  FalconClosedLoop closedLoop;
  double position;
  public EnableFalconPositionClosedLoop(FalconClosedLoop loop) {
    this.closedLoop = loop;
    loop.setClosedLoopMode(ControlMode.Position);
    addRequirements(loop);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  public void setPosition(double newPosition) {
    this.position = newPosition;
  }

  public void incrementPosition(double increment) {
    position += increment;
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    closedLoop.setMotor(position); //Determine Velocity
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
