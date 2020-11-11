/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Driving.ConventionalDrive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.driverinput.LogitechF310;
import frc.robot.subsystems.ConventionalDriveTrain;

public class ConventionalArcadeDrive extends CommandBase {
  private LogitechF310 ControllerDrive;
  private double rightPower = 0;
  private double leftPower = 0;
  private ConventionalDriveTrain ConventionalDriveTrain_Inst;
  /**
   * Creates a new ConventionalTankDrive.
   */
  public ConventionalArcadeDrive(ConventionalDriveTrain ConventionalDriveTrain_Inst,LogitechF310 ControllerDrive) {
    this.ControllerDrive = ControllerDrive;
    this.ConventionalDriveTrain_Inst = ConventionalDriveTrain_Inst;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ConventionalDriveTrain_Inst);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double throttle = ControllerDrive.LeftJoystick.get().x;
    double steer = ControllerDrive.RightJoystick.get().y;

    rightPower = throttle + steer;
    leftPower = throttle - steer;
    
    ConventionalDriveTrain_Inst.setPowers(rightPower,leftPower);

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
