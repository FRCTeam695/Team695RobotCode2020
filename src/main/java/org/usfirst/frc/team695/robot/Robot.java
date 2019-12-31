package org.usfirst.frc.team695.robot;





import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.cscore.UsbCamera;

import edu.wpi.first.cameraserver.CameraServer;
/*
XBOX controls:

axis:
0 left X
1 left Y
2 left trigger test change
3 right trigger
4 right X
5 right Y

buttons:
1 A
2 B
3 X
4 Y
5 left bump
6 right bump
7 multi screen
8 options
9 left push
10 right push

*/

public class Robot extends SampleRobot
{
	//button ids
	private int buttonBlueX = 3;
	private int buttonGreenA = 1;
	private int buttonRedB = 2;
	private int buttonYellowY = 4;
	//axis ids
	private int leftXAxis = 0;
	private int leftYAxis = 1;

	private int rightXAxis = 4;
	private int rightYAxis = 5;

	private int leftTriggerAxis = 2;
	private int rightTriggerAxis = 3;
	// network communications
	private NetworkTableInstance inst;
	private NetworkTable table;
	//private NetworkTableEntry pidx;
	private NetworkTableEntry ringop;
	private NetworkTableEntry tabhatchleft;
	private NetworkTableEntry tabhatchright;
	private NetworkTable limeLightValues;
	private NetworkTableEntry limeTx;
	private NetworkTableEntry limeTy;
	private NetworkTableEntry limeTa;
	// pneumatic objects
	private Compressor comp = new Compressor(0);
	private Solenoid hatch = new Solenoid(0);
	private Solenoid lift = new Solenoid(1);
	private Solenoid boostPiston = new Solenoid(2);

	// user controller objects
	private Joystick controllerLiftJack = new Joystick(1);
	private Joystick controllerDrive = new Joystick(0);

	// jack in limit switch
	private DigitalInput jackin = new DigitalInput(1);
	
	// hatch detection switches
	private DigitalInput hatchleft = new DigitalInput(6);
	private DigitalInput hatchright = new DigitalInput(5);
	// motor controllers
	private VictorSPX motorL1 = new VictorSPX(1);
	private VictorSPX motorL2 = new VictorSPX(2);
	private VictorSPX motorR1 = new VictorSPX(3);
	private VictorSPX motorR2 = new VictorSPX(4);
	private VictorSPX motorJack = new VictorSPX(8); 


	/****************************************************************/
	/****************************************************************/
	/****************************************************************/
	private void retractJackUnlessAlreadyRetracted(double movejack) {
		movejack = -movejack; //so negative inputs correspond to downwards jack movement
		//switch is false when pushed in but true when not pushed in
		if (movejack > 0 && !jackin.get()) { //prevent retracting (moving upwards) if pushed in 
			movejack = 0;
		}
		motorJack.set(ControlMode.PercentOutput, movejack);
	}
	
	
	
	
	
	
	/****************************************************************/
	/****************************************************************/
	/****************************************************************/

	public String getAlliance()
	{
		DriverStation.Alliance color = DriverStation.getInstance().getAlliance();
		
		if (color == DriverStation.Alliance.Blue)
		{
			return("B");
			//return("BLUE" + DriverStation.getInstance().getLocation());
		}
		else if (color == DriverStation.Alliance.Red)
		{
			return("R");
		}
		return("?");
	}
	
	/****************************************************************/
	/****************************************************************/
	/****************************************************************/
	
	public void robotInit()
	{
		System.out.println("695:  robotInit()");
		//this allows for the usb camera plugged into the robot to be shown on the dashboard
		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		camera.setResolution(160, 120);


		inst = NetworkTableInstance.getDefault();
		table = inst.getTable("SmartDashboard");
		//pidx = table.getEntry("pidx");
		ringop = table.getEntry("ringop");
		tabhatchleft = table.getEntry("tabhatchleft");
		tabhatchright = table.getEntry("tablehatchright");
		limeLightValues = inst.getTable("limelight");
		limeTx = limeLightValues.getEntry("tx");
		limeTy = limeLightValues.getEntry("ty");
		limeTa = limeLightValues.getEntry("ta"); 		
		comp.enabled();
		boostPiston.set(false);
		
	}

	/****************************************************************/
	/****************************************************************/
	/****************************************************************/

	public void disabled()
	{		
		long cnt = 0;
		long tickcnt = 0;
		String ringstr;
		
		for(;;)
		{
				
			if (getAlliance() == "R")
			{
				ringop.setNumber(1);
				ringstr = "RED";
			}
			else
			{
				ringop.setNumber(2);
				ringstr = "BLUE";
			}

			if (!isDisabled())
			{
				return;
			}
			
			if (++tickcnt == 100)
			{
				System.out.println("695:  disabled(" + (++cnt) + "):  Alliance is " + ringstr);
				tickcnt = 0;
			}

			Timer.delay(0.01);

		}
		
		//}
	}
	
	/****************************************************************/
	/****************************************************************/
	/****************************************************************/
	
	public void autonomous()
	{
		System.out.println("695:  autonomous()");
		operatorControl();
	}

	
	/****************************************************************/
	/****************************************************************/
	/****************************************************************/
	
	public void operatorControl()
	{
		long hatchdebounce = 0;
		long liftdebounce = 0;
		long povdebounce = 0;
		long boostPistonDebounce = 0;
		
		
		long tickcnt = 0;
		
		double err, pgain = 0.3;
		
		double driveleft;
		double driveright;
		double drivesteer;
		
		double azimuthToTarget;// = limeTx.getDouble(0.0);
		double error;
		double copolarToTarget;// = limeTy.getDouble(0.0);
		double areaOfContour;// = limeTa.getDouble(0.0);
		double Kp = 0.03;  // Proportional control constant
		double steeringAdjust = 0;
		double minCommand = -0.015;
		double driveSensitivity = 1;
		System.out.println("695:  operatorControl()...");
		System.out.println("Ring is green!");
		ringop.setNumber(3);
		
		while(isEnabled())
		{
			// *********
			// boost piston code
			// *********
			
			if (controllerLiftJack.getRawButton(buttonBlueX) == true)
			{
				if (boostPistonDebounce == 0)
				{
					boostPistonDebounce = 1;
					boostPiston.set(!boostPiston.get()); //switch fork state
				}
			}
			else
			{
				boostPistonDebounce = 0;
			}
			
			retractJackUnlessAlreadyRetracted(controllerLiftJack.getRawAxis(1));
			if ((hatchleft.get()) && (hatchright.get()))
			{
				hatch.set(true);
			}
			/*
			if (controllerDrive.getPOV() != -1)
			{
				if (povdebounce == 0)
				{
					povdebounce = 1;
					if (controllerDrive.getPOV() == 0)
					{
						if (pgain < 1)
						{
							pgain = pgain + 0.1;
						}
					}
					if (controllerDrive.getPOV() == 180)
					{
						if (pgain > 0.1)
						{
							pgain = pgain - 0.1;
						}
					}
				}
			}
			else
			{
				povdebounce = 0;
			}
			*/						
			// lift
			if (controllerDrive.getRawButton(buttonRedB) == true)
			{
				if (liftdebounce == 0)
				{
					liftdebounce = 1;
					if (lift.get() == true)
					{
						lift.set(false);
					}
					else
					{
						lift.set(true);
					}
				}
			}
			else
			{
				liftdebounce = 0;
			}
			
			// hatch
			if (controllerDrive.getRawButton(buttonYellowY) == true)
			{
				if (hatchdebounce == 0)
				{
					hatchdebounce = 1;
					hatch.set(!hatch.get());
				}
			}
			else
			{
				hatchdebounce = 0;
			}
			
			if (hatchleft.get() == true)
			{
				tabhatchleft.setNumber(1);
			}
			else
			{
				tabhatchleft.setNumber(0);
			}
			
			if (hatchright.get() == true)
			{
				tabhatchright.setNumber(1);
			}
			else
			{
				tabhatchright.setNumber(0);
			}
			
			//***********
			// drive code
			//***********

			// drive speed
			driveSensitivity = 1-(controllerDrive.getRawAxis(rightTriggerAxis)); 
			if (driveSensitivity <= 0.25) {//do not let it become zero
				driveSensitivity = 0.25;
			} 
			driveleft = driveright = controllerDrive.getRawAxis(leftYAxis)*(driveSensitivity); //speed is scales by how much controller drive is compressed
			drivesteer = controllerDrive.getRawAxis(rightXAxis);

			if ((driveleft >= -0.1) && (driveleft <= 0.1))
			{
				driveleft = driveright = 0;
			}
			/*
			// auto dock
			err = pidx.getDouble(0) / 100;

			if (controllerDrive.getRawButton(7) == true)
			{

				drivesteer = err * pgain;

				if (driveleft > 0.25)
				{
					driveleft = driveright = 0.25;
				}
				if (driveleft < -0.25)
				{
					driveleft = driveright = -0.25;
				}

			}
			*/
				
		// apply steering to move
		if (drivesteer > 0)
		{
			driveleft = driveleft * (1 - drivesteer);
		}
		if (drivesteer < 0)
		{
			driveright = driveright * (1 + drivesteer);
		}
				
		azimuthToTarget = error = limeTx.getDouble(0.0);
		copolarToTarget = limeTy.getDouble(0.0);
		areaOfContour = limeTa.getDouble(0.0);

		//System.out.println("LIME DATA: X: " + Double.toString(azimuthToTarget) + " Y: " + Double.toString(y) + " AREA: " + Double.toString(area));
		double forwardModifier = 0;
		//driveleft  = controllerDrive.getRawAxis(5);
		//driveright = controllerDrive.getRawAxis(1);
		if (controllerDrive.getRawButton(buttonGreenA)) {
			//System.out.println("PBUTTON DOWN");
			steeringAdjust = Kp*error;
			if (error > 3.0)
			{
				steeringAdjust = Kp*error - minCommand;
			}
			else if (error < 3.0)
			{
				steeringAdjust = Kp*error + minCommand;
				forwardModifier = 0;//-0.5;
			}
			//driveleft = driveright; //disable tank drive by ignoring right stick, left becomes the throttle
			driveleft += steeringAdjust;
			driveright -= steeringAdjust;
			driveright += forwardModifier;
			driveleft += forwardModifier;
			// drive speed
		}
			
			motorL1.set(ControlMode.PercentOutput, .7*driveleft);
			motorL2.set(ControlMode.PercentOutput, .7*driveleft);

			motorR1.set(ControlMode.PercentOutput, -1 * .7*driveright);
			motorR2.set(ControlMode.PercentOutput, -1 * .7*driveright);

						
			//******************************
			// diagnostic print every second
			//******************************
			if (++tickcnt == 100)
			{
				tickcnt = 0;
				//System.out.println("raw button 2: " + controllerDrive.getRawButton(2));
				//System.out.println("driveleft=" + driveleft + ", driveright=" + driveright + ", drivesteer=" + drivesteer);
				//System.out.println("GAIN: " + pgain);
				//System.out.println("JACK: " + countJack.get());
				//System.out.println("695:  operatorControl(" + (++cnt) + ")");
				//System.out.println(hatchleft.get() + " / " + hatchright.get());
				//System.out.println("hatch: " + hatch.get() + ":  " + hatchleft.get() + " / " + hatchright.get());
				//System.out.println("   jack count: " + countJack.get());
				//System.out.println("   jack distance: " + getLidar());
				//System.out.println("   jack in: " + jackin.get());
				//System.out.println("   movejack: " + movejack);
				//leds();
		
			}
			
			//***********************
			// time delay for roborio
			//***********************
			Timer.delay(0.01);
			
		}		
	}
	
	/****************************************************************/
	/****************************************************************/
	/****************************************************************/

	public void test()
	{
		long cnt = 0;

		System.out.println("695:  test()");
		while (isTest() && isEnabled())
		{
			Timer.delay(1);
			System.out.println("695:  test tick() " + (++cnt));
		}
	}
	
	/****************************************************************/
	/****************************************************************/
	/****************************************************************/

}



/**************************************************************************************************************************************************/
/**************************************************************************************************************************************************/
/**************************************************************************************************************************************************/
/**************************************************************************************************************************************************/
/**************************************************************************************************************************************************/
/**************************************************************************************************************************************************/
/**************************************************************************************************************************************************/
/**************************************************************************************************************************************************/
/**************************************************************************************************************************************************/

//public class Robot extends IterativeRobot
//{
	
	/****************************************************************/
	/****************************************************************/
	/****************************************************************/
	
	//public void robotInit()
	//{
		//System.out.println("robotInit()");		
	//}

	/****************************************************************/
	/****************************************************************/
	/****************************************************************/

	//public void disabledInit()
	//{
		//System.out.println("disabledInit()");
	//}
	
	//public void disabledPeriodic()
	//{
		
	//}
	
	//public void disabledContinuous()
	//{
		
	//}
	
	/****************************************************************/
	/****************************************************************/
	/****************************************************************/

	//public void autonomousInit()
	//{
		//System.out.println("autonomousInit()");
	//}
	
	//public void autonomousPeriodic()
	//{
		
	//}

	//public void autonomousContinuous()
	//{
		
	//}

	/****************************************************************/
	/****************************************************************/
	/****************************************************************/
	
	//public void teleopInit()
	//{
		//System.out.println("teleopInit()");		
	//}
	
	//public void teleopPeriodic()
	//{
		
	//}
	
	//public void teleopContinuous()
	//{
		
	//}
	
	/****************************************************************/
	/****************************************************************/
	/****************************************************************/
		
