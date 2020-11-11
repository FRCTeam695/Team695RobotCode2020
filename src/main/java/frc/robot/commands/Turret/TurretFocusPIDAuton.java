/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Turret;

import java.util.LinkedList;
import java.util.Queue;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.AdjustableVictor;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TurretFocusPIDAuton extends PIDCommand {
  private Turret Turret_Inst;
  private static double timeOutOfSetpoint = 0;
  private static double timeInSetpoint = 0;
  private static Queue<Double> DistanceReadings = new LinkedList<Double>();
  private static double readingSum = 0;
  //private 
  private boolean status = false;
  
  /**
   * Creates a new TurretFocusPID.
   */
  public TurretFocusPIDAuton(Turret Turret_Inst,AdjustableVictor Hopper_Inst,PIDController PID) {
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
          System.out.print(" B PERCENT ");

          System.out.print(Turret_Inst.determineBottomMotorPercent());
          System.out.print(" TOP PERCE ");

          System.out.print(Turret_Inst.determineTopMotorPercent());
          timeOutOfSetpoint = 0;
          timeInSetpoint += 0.02;
          //calculate definite sum from a second ago up to now and then average it/
          double CurrentDistanceReading = Turret_Inst.getDistanceToContourInFeet();
          DistanceReadings.add(CurrentDistanceReading);
          int ReadingsTaken = DistanceReadings.size();
          //if there are more than 50 readings (1 second's worth of data, the last reading gets removed.)
          if (ReadingsTaken > 50) {
            readingSum -= DistanceReadings.peek();
            DistanceReadings.remove();
            ReadingsTaken -= 1;
          }
          readingSum += CurrentDistanceReading;
          double averageDistance = (1.0/(ReadingsTaken))*readingSum;
          Turret_Inst.setShooterWheelPowers(Turret_Inst.determineTopMotorPercent(averageDistance), Turret_Inst.determineBottomMotorPercent(averageDistance));
          System.out.print(" TOP ERROR: ");
          System.out.print(Turret_Inst.getTopError());
          System.out.print(" BOTOP ERROR: ");
          System.out.print(Turret_Inst.getBottomError());
          System.out.print(Math.abs(Turret_Inst.getTopError()) < 4000 && Math.abs(Turret_Inst.getBottomError()) < 2000);
          System.out.println(averageDistance);

          if (Math.abs(Turret_Inst.getTopError()) < 500 && Math.abs(Turret_Inst.getBottomError()) < 500) {
            Hopper_Inst.setPower(1);
          }
        } 
        else {
          if (!(Turret_Inst.isTooFarLeft())  && !(Turret_Inst.isTooFarRight())) { //ensure motor cannot turn more than fully left if headed left. Need to determine what this direction is.
            try {
              Turret_Inst.setPower(-output);}
            catch(IllegalArgumentException percentageOverFlException) {}
          }
          timeOutOfSetpoint += 0.02;
          if (timeOutOfSetpoint >= 0.25) {
            Turret_Inst.setShooterWheelPowers(0, 0); //stop motors if out of setpoint for too long and also reset integral data.
            DistanceReadings.clear();
            readingSum = 0;
            
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
