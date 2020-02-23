/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ControllerConst;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

enum DriveState {
  TANK_DRIVE,
  ARCADE_DRIVE,
  CURVE_DRIVE;
}

//make sure class names are uppercase on the first letter
public class Driving extends CommandBase {
  /**
   * Creates a new driving.
   */
  private Drivetrain Drive;
  private Joystick Controller;
  private int leftSideAxisID = ControllerConst.leftSideAxisId;
  private int rightSideAxisID = ControllerConst.rightSideAxisId;
  private int curveButtonId = ControllerConst.curveDriveButton;
  private int driveSwitchButtonId = ControllerConst.curveDriveButton;

  private DriveState CurrentDriveState = DriveState.TANK_DRIVE;


  private ArcadeDrive Arcade;
  private CurveDrive Curve;
  private TankDrive Tank;

  public Driving(Drivetrain Drive, Joystick Controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Drive = Drive;
    this.Controller = Controller;
    addRequirements(Drive);
  }
  public void toggleQuickTurn(){
    Curve.toggleQuickTurn();
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Arcade = new ArcadeDrive(Drive, Controller, leftSideAxisID, rightSideAxisID);
    Curve = new CurveDrive(Drive, Controller, leftSideAxisID, rightSideAxisID);
    Tank = new TankDrive(Drive, Controller, leftSideAxisID, rightSideAxisID);
    Arcade.initialize();
    Curve.initialize();
    Tank.initialize();
  }

  public void toggleDrive(){
    switch (CurrentDriveState){
      case TANK_DRIVE:
        CurrentDriveState = DriveState.ARCADE_DRIVE;
      case ARCADE_DRIVE:
        CurrentDriveState = DriveState.CURVE_DRIVE;
      case CURVE_DRIVE:
        CurrentDriveState = DriveState.TANK_DRIVE;
      default:
        CurrentDriveState = DriveState.ARCADE_DRIVE;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (CurrentDriveState){
      case TANK_DRIVE:
        Tank.execute();
      case ARCADE_DRIVE:
        Arcade.execute();
      case CURVE_DRIVE:
        Curve.execute();
      default:
        Tank.execute();/**this is good */
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //no condition where it should end, as Drive will continue throughout teleop.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; //ditto.
  }
}
