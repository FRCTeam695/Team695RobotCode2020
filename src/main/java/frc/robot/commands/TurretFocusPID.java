/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.TurretMotor;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TurretFocusPID extends PIDCommand {
  private TurretMotor Motor;
  private static double timeOutOfSetpoint = 0;
  /**
   * Creates a new TurretFocusPID.
   */
  public TurretFocusPID(TurretMotor Motor,PIDController PID) {
    super(
      // The controller that the command will use
      PID,
      // This should return the measurement
      () -> Motor.getAzimuth(),
      // This should return the setpoint (can also be a constant)
      0.0,
      // This uses the output
      output -> {
        if (!Motor.isTooFarLeft() && !Motor.isTooFarRight()) {
          try {Motor.setPower(-output);}
          catch(IllegalArgumentException percentageOverFlException) {}
        }
        if (PID.atSetpoint()) { 
          System.out.println("at set point, Bottom percent: " + ((Double) Motor.determineBottomMotorPercent()).toString() + " Distance in feet : " +((Double) Motor.getDistanceToContourInFeet()).toString());
          timeOutOfSetpoint = 0;
          //Motor.setMotorPowers(Motor.determineTopMotorPercent(), Motor.determineBottomMotorPercent());
        } else {
          timeOutOfSetpoint += 0.02;
          if (timeOutOfSetpoint >= 0.25) {
            Motor.setMotorPowers(0, 0); //stop motors if out of setpoint for too long.
          }   
        }        // Use the output here
        }
    );
    this.getController().setTolerance(1.2);
    this.Motor = Motor;
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
