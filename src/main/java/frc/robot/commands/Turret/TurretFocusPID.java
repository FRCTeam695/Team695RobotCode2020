/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TurretFocusPID extends PIDCommand {
  private Turret Turret_Inst;
  private static double timeOutOfSetpoint = 0;
  //private 
  private boolean status = false;
  
  /**
   * Creates a new TurretFocusPID.
   */
  public TurretFocusPID(Turret Turret_Inst,PIDController PID) {
    super(
      // The controller that the command will use
      PID,
      // This should return the measurement
      () -> Turret_Inst.getAzimuth(),
      // This should return the setpoint (can also be a constant)
      0.0,
      // This uses the output
      output -> {

        if (PID.atSetpoint()) { 
          System.out.println(Turret_Inst.getDistanceToContourInFeet() );
          System.out.print(" ");

          System.out.print(Turret_Inst.determineBottomMotorPercent());
          System.out.print(" ");

          System.out.print(Turret_Inst.determineTopMotorPercent());
          timeOutOfSetpoint = 0;
          Turret_Inst.setShooterWheelPowers(Turret_Inst.determineTopMotorPercent(), Turret_Inst.determineBottomMotorPercent());
        } 
        else {
          if (!(Turret_Inst.isTooFarLeft())  && !(Turret_Inst.isTooFarRight())) { //ensure motor cannot turn more than fully left if headed left. Need to determine what this direction is.
            try {
              Turret_Inst.setPower(-output);}
            catch(IllegalArgumentException percentageOverFlException) {}
          }
          timeOutOfSetpoint += 0.02;
          if (timeOutOfSetpoint >= 0.25) {
            Turret_Inst.setShooterWheelPowers(0, 0); //stop motors if out of setpoint for too long.
            
          }   
        }        // Use the output here
      }
    );
    this.getController().setTolerance(1.2);
    this.Turret_Inst = Turret_Inst;
    addRequirements(Turret_Inst);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  //public void stopCommand() {
  //  status = true;
  //}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
