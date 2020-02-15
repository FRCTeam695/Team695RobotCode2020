/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.dash;
import frc.robot.Constants.DriveConstants;

public class gyroTest extends CommandBase {
  /**
   * Creates a new gyroTest.
   */

  private final Gyro m_gyro = new ADXRS450_Gyro();
  
  private dash dashboard;
  public gyroTest(dash dashboard) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.dashboard = dashboard;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  public double head(){
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
  public double rate(){
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    dashboard.getGyroBox().set(head());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
  public Gyro getGyro() {
    return m_gyro;
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
