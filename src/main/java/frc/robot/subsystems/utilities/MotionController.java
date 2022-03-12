package frc.robot.subsystems.utilities;


import frc.robot.subsystems.DriveTrainMotionControl;
import frc.robot.subsystems.utilities.AdjustSpeedAsTravelMotionControlHelper;
import frc.robot.subsystems.utilities.MotionControlPIDController;

//import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj.PIDSource;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MotionController 
{
	private DriveTrainMotionControl m_DriveTrain;
	private AdjustSpeedAsTravelHelper m_AdustsSpeedAsTravelStraightHelper;
	private AdjustSpeedAsTravelMotionControlHelper m_AdjustRpmAsTurnHelper;
	private AdjustSpeedAsTravelHelper m_AdjustSpeedAsTravelArcHelper;
	
	private MotionControlPIDController m_StraightDistancePIDController;
	private PIDOutputStraightMotion    m_StraightRotationPIDOutput;
	
	private MotionControlPIDController m_TurnPIDController;
	
	private MotionControlPIDController m_ArcDistancePIDController;
	private PIDOutputArcMotion         m_ArcRotationPIDOutput;
	
	private double m_DistanceToExceed;
	private double m_targetAngle;
	private double m_StraightTolerance;
	private double m_TurnTolerance;
	private double m_AngularVelocityTolerance;
//	private boolean m_PIDEnabled;// no longer valid, code must now do it self by calling calculate()_ in like a command.execute
	
	private final double TurnKp = 0.005;
	private final double TurnKi = 0.0020;
	private final double TurnKd = 0.0;
	private final double TurnMaxPower = 1;
	
//for ref	private final double SwingKp = 0.07;
//for ref	private final double SwingKi = 0.0;
//for ref	private final double SwingKd = 0.0;

	
	private final double StraightKp = 0.001;
	private final double StraightKi = 0.0;
	private final double StraightKd = 0.0;
	private final double StraightMaxPower = 1;

	private final double ArcKp = StraightKp; //0.002;
	private final double ArcKi = StraightKi; //0.001;
	private final double ArcKd = StraightKd; //0.0;
	private final double arcMaxPower = 0.5;
	
	//PIDSource m_LineSource;
	//DoubleSupplier m_LineSource;
    Encoder m_LineSource;
	//PIDSource m_TurnSource;
	//DoubleSupplier m_TurnSource;
	Gyro m_TurnSource;
	
	boolean isArcMovingForward = true;
	boolean isStraightMovingForward = true;
	
	
	
	public MotionController(DriveTrainMotionControl driveTrainMotionControl, Encoder distanceSource, Gyro rotationSource)
	{
		m_DriveTrain = driveTrainMotionControl;
		m_LineSource = distanceSource;
		m_TurnSource = rotationSource;
		
		m_StraightDistancePIDController = null;
		m_StraightRotationPIDOutput = null;
//		m_ControlledAngleDrivePIDOutput = null;
		m_TurnPIDController = null;
		
		
		m_DistanceToExceed = 0;
		m_targetAngle = 0;
		m_StraightTolerance = 0.5;
		m_TurnTolerance = 5;// had been 0.5
		
		m_AngularVelocityTolerance = 15;
//		m_PIDEnabled = false;
		
	}
	//TODO Execute is even more missleading, change to startStraightMotion, using START instead of EXECUTE think shoudl work.
	public boolean StartStraightMotion(double distance, double maxspeed, double ramp) {
		double targetAngle = m_DriveTrain.GetAngle();
		return StartStraightMotionProvideAngle( distance,  maxspeed,  ramp, targetAngle);
	}

	//TODO Execute missleading, use START instead of EXECUTE think shoudl work.
	public boolean StartControlledAngleDriveMotion(double distance, double maxspeed, double ramp, double targetAngle) {
		return StartStraightMotionProvideAngle( distance,  maxspeed,  ramp, targetAngle);
	}
	
	//TODO Execute missleading, use START instead of EXECUTE think shoudl work.
	private boolean StartStraightMotionProvideAngle(double distance, double maxspeed, double ramp, double targetAngle)
	{
//		if (!m_PIDEnabled)
//		{
			m_targetAngle =  targetAngle;
			m_DistanceToExceed = distance;
			m_DriveTrain.ResetEncoders();
			
			double start = 0;
			
			double convertedDistance = distance;	// Inches
			double convertedSpeed = maxspeed * 12; 	// Converted from Feet/Second to Inches/Second
			double convertedRamp = ramp;			// Inches/Second
			
			if (!(Math.abs(m_DriveTrain.GetLeftDistance()) > Math.abs(m_DistanceToExceed)))
			{
				//Instantiates a new AdjustSpeedAsTravelMotionControlHelper() object for the driveStraightDistance we are going to traverse
				m_StraightRotationPIDOutput = new PIDOutputStraightMotion(m_DriveTrain, m_TurnSource, m_targetAngle);
				m_AdustsSpeedAsTravelStraightHelper = new AdjustSpeedAsTravelMotionControlHelper(convertedDistance, convertedRamp, convertedSpeed, 
				                                                                                 start, new EncoderDistenceAsDouble(m_LineSource), 
																								 m_StraightRotationPIDOutput);
				
				//Instantiates a new MotionControlPIDController() object for the new drive segment using the previous MotionControlHelper()
				m_StraightDistancePIDController = new MotionControlPIDController(StraightKp, StraightKi, StraightKd, m_AdustsSpeedAsTravelStraightHelper);
//				m_StraightDistancePIDController.setAbsoluteTolerance(m_StraightTolerance);
				m_StraightDistancePIDController.setTolerance(m_StraightTolerance);// there is also a setTolerance that takes position and velocity acceptable tolerance
//				m_StraightDistancePIDController.setOutputRange(-StraightMaxPower, StraightMaxPower);// This is removed from PIDController docs say use clamp to get this if needed
				
				//OLD Turns the MotionControlPID ON and it will continue to execute by itself until told otherwise.
				//OLD m_StraightDistancePIDController.enable();
				//OLD m_PIDEnabled = true;
				//TODO get calculate being called from som loop probably a command in the execute method RGT 20220227
				return true;
			}
			return false;
//		}
//		return true;
	}
	
	public boolean StartTurnMotion(double turnToAngle)
	{
		//TODO to really have it turn on a Dime should monitor left to right wheel and make sure adding them goes to Zero
		// create a forward motion PID control on that then you can get precise turning.
//		if (!m_PIDEnabled)
//		{
//			m_DriveTrain.ResetGyro();
			double start = m_DriveTrain.GetAngle();
			
			//TODO Magic numbers need fixing
			//TODO What are the units?
			double maxRPM = 60/*30*/;			// Rotations/Minute
			double ramp = 45/* 3.5 * maxRPM*/;	//angle off from target to start slowing down.
			
			double maxSpeed = maxRPM * 6; // 360 Degrees/60 seconds to convert RPM to speed or degrees per second
			m_targetAngle = turnToAngle;
//			+ start;
			
			if (!(Math.abs(m_DriveTrain.GetAngle()-m_targetAngle) < m_TurnTolerance))
			{
				
				//Instantiates a new MotionControlHelper() object for the new turn segment
				m_AdjustRpmAsTurnHelper = new AdjustSpeedAsTravelMotionControlHelper(m_targetAngle, ramp, maxSpeed, start, 
				                                                                      new GyroAngleAsDouble(m_TurnSource), 
																					  m_StraightRotationPIDOutput);
//DontThinkNeeded				m_TurnControl.setTargetDistance(m_targetAngle);
				
				//Instantiates a new MotionControlPIDController() object for the new turn segment using the previous MotionControlHelper()
				m_TurnPIDController = new MotionControlPIDController(TurnKp, TurnKi, TurnKd, m_AdjustRpmAsTurnHelper);
				//m_TurnPIDController.setOutputRange(-TurnMaxPower, TurnMaxPower);// No longer available would have to code ourselves, they recommoned using a function clamp
				
				//Turns the MotionControlPID ON and it will continue to execute by itself until told otherwise.
				//m_TurnPIDController.enable();	
				//m_PIDEnabled = true;
				
				return true;
			}
			return false;
//		}
//		return true;
	}
	


	
	/**
	 * 
	 * @param distance  to travel in inches
	 * @param maxSpeed  in ft/sec
	 * @param ramp      in inches
	 * @param radiusOfArc  The radius of the arc travel path of the robot in inches
	 * @return true if it has completed the arc path
	 */
	public boolean ExecuteArcMotion(double distance, double maxSpeed, double ramp, double radiusOfArc)
	{
		//TODO have it pay attention to current position and calc based on the differance
		if(m_DistanceToExceed>0){
			isArcMovingForward = true;
		}
		else {
			isArcMovingForward = false;
		}
		m_DistanceToExceed = distance;//inches
//		m_DriveTrain.ResetEncoders();
		
		double start = 0;

		
//OLD		if (!isPIDEnabled())
//OLD		{
			double convertedDistance = m_DistanceToExceed; 	// In inches
			double convertedSpeed = maxSpeed * 12; 	// convert from feet to inches/second
			double convertedRamp = ramp; 			// in inches
			

//			//Instantiates a new MotionControlHelper() object for the new Arch segment
//			//Instantiates a new AdjustSpeedAsTravelMotionControlHelper() object for the driveStraightDistance we are going to traverse
//			m_StraightRotationPIDOutput = new PIDOutputStraightMotion(m_DriveTrain, m_TurnSource, m_targetAngle);
//			m_AdustsSpeedAsTravelStraightHelper = new AdjustSpeedAsTravelMotionControlHelper(convertedDistance, convertedRamp, convertedSpeed, start, m_StraightSource, m_StraightRotationPIDOutput);
			// motionControlForwardSpeed
			m_ArcRotationPIDOutput         = new PIDOutputArcMotion(m_DriveTrain, new GyroAngleAsDouble(m_TurnSource), radiusOfArc);
			m_AdjustSpeedAsTravelArcHelper = new AdjustSpeedAsTravelMotionControlHelper(convertedDistance, convertedRamp, convertedSpeed, start, 
			                                                                            new EncoderDistenceAsDouble(m_LineSource),
																						 m_ArcRotationPIDOutput);
			
//			//Instantiates a new MotionControlPIDController() object for the new drive segment using the previous MotionControlHelper()
//			m_StraightDistancePIDController = new MotionControlPIDController(StraightKp, StraightKi, StraightKd, m_AdustsSpeedAsTravelStraightHelper);
//			m_StraightDistancePIDController.setAbsoluteTolerance(m_StraightTolerance);
//			m_StraightDistancePIDController.setOutputRange(-StraightMaxPower, StraightMaxPower);
//			
			//Instantiates a new MotionControlPIDController() object for the new turn segment using the previous MotionControlHelper()
			m_ArcDistancePIDController = new MotionControlPIDController(ArcKp, ArcKi, ArcKd, m_AdjustSpeedAsTravelArcHelper);
//			m_ArcDistancePIDController.setAbsoluteTolerance(m_StraightTolerance);
			m_ArcDistancePIDController.setTolerance(m_StraightTolerance);
			//OLD m_ArcDistancePIDController.setOutputRange(-arcMaxPower, arcMaxPower);//No longer availalbe code has to do it itself, docs suggest using clamp
			
//			//Turns the MotionControlPID ON and it will continue to execute by itself until told otherwise.
//			m_StraightDistancePIDController.enable();
//			m_PIDEnabled = true;
			//Turns the MotionControlPID ON and it will continue to execute on a seperate thread by itself until told otherwise.
//OLD TODO: get calulate called and do something with the next output value			m_ArcDistancePIDController.enable();
			return true;
//OLD		}
//OLD		return true;
	}
	
	public boolean isStraightMotionFinished()
	{
		/*
		 * Called while waiting for the MotionControlPID to finish. The PID will be disabled when the end condition is met, and
		 * the return value indicates you can proceed to the next step.
		 * */
		SmartDashboard.putNumber("Distance Left", m_DriveTrain.GetLeftDistance());
		SmartDashboard.putNumber("Target distance", m_DistanceToExceed);
		SmartDashboard.putNumber("Straight Tolerance", m_StraightTolerance);
		
		//TODO Verify this tolerance works... it should...
		SmartDashboard.putNumber("Average Distance", m_DriveTrain.GetAverageDistance());
		SmartDashboard.putNumber("Target", Math.abs(m_DistanceToExceed - m_StraightTolerance));

		boolean didExceedDistance = false;
		if(isStraightMovingForward) {
			// Traveling Forward
			if (m_DriveTrain.GetAverageDistance() > m_DistanceToExceed) {
				didExceedDistance = true;
			}else {
				didExceedDistance = false;
			}
		}else {
			// Traveling Backward
			if (m_DriveTrain.GetAverageDistance() < m_DistanceToExceed) {
				didExceedDistance = true;
			}else {
				didExceedDistance = false;
			}
		}
		if(didExceedDistance){
			if(m_StraightDistancePIDController != null) {
//OLD TODO: get calulate called and do something with the next output value					m_StraightDistancePIDController.disable();
//OLD TODO: get calulate called and do something with the next output value					m_StraightRotationPIDOutput.disableRotationPIDController();
			}
//OLD TODO: get calulate called and do something with the next output value				m_PIDEnabled = false;
			return true;
		}
		return false;
	}
	
	public boolean isTurnMotionFinished()
	{
		/*
		 * Called while waiting for the MotionControlPID to finish. The PID will be disabled when the end condition is met, and
		 * the return value indicates you can proceed to the next step.
		 * */
		if (Math.abs(m_DriveTrain.GetAngle()-m_targetAngle) < m_TurnTolerance && Math.abs(m_DriveTrain.getAngularVelocity()) < m_AngularVelocityTolerance)
		{
//OLD TODO: get calulate called and do something with the next output value				m_TurnPIDController.disable();
			//m_DriveTrain.ArcadeDrive(0, 0);
//OLD TODO: get calulate called and do something with the next output value				m_PIDEnabled = false;
			return true;
		}
		return false;
	}
	
	public boolean isArcMotionFinished()
	{
		/*
		 * Called while waiting for the MotionControlPID to finish. The PID will be disabled when the end condition is met, and
		 * the return value indicates you can proceed to the next step.
		 * */
		SmartDashboard.putNumber("Arc - Distance", m_DriveTrain.GetAverageDistance());
		System.out.println("Arc - Distance = "+ m_DriveTrain.GetAverageDistance());
		SmartDashboard.putNumber("ARc - Distance to Exceed", m_DistanceToExceed);
		System.out.println("Arc - Distance to Exceed = "+ m_DistanceToExceed);
		System.out.println("Arc - isArcMovingForward = "+ isArcMovingForward);
//		SmartDashboard.putNumber("Straight Tolerance", m_StraightTolerance);
		
		boolean didExceedDistance = false;
		if(isArcMovingForward) {
			// Traveling Forward
			if (m_DriveTrain.GetAverageDistance() > m_DistanceToExceed) {
				didExceedDistance = true;
			}else {
				didExceedDistance = false;
			}
		}else {
			// Traveling Backward
			if (m_DriveTrain.GetAverageDistance() < m_DistanceToExceed) {
				didExceedDistance = true;
			}else {
				didExceedDistance = false;
			}
		}
		System.out.println("Arc - didExceedDistance = "+ didExceedDistance);
		
		if(didExceedDistance) {
//OLD TODO: get calulate called and do something with the next output value				m_ArcDistancePIDController.disable();
//OLD TODO: get calulate called and do something with the next output value				m_PIDEnabled = false;
			return true;
		}
        //Dont stop, let motion flow to next if desired			m_DriveTrain.ArcadeDrive(0, 0);
		return false;
	}

//OLD TODO: get calulate called and do something with the next output value		public boolean isPIDEnabled()
//OLD TODO: get calulate called and do something with the next output value		{
//OLD TODO: get calulate called and do something with the next output value			return m_PIDEnabled;
//OLD TODO: get calulate called and do something with the next output value		}
	
	public void DisablePIDControls()
	{
		if(m_TurnPIDController != null)
		{
			//m_TurnPIDController.disable();
			m_TurnPIDController.disableContinuousInput();
		}
		
		if(m_StraightDistancePIDController != null)
		{
			//m_StraightDistancePIDController.disable();
			m_StraightDistancePIDController.disableContinuousInput();
			//m_StraightRotationPIDOutput.disableRotationPIDController();
		}
		
		if(m_ArcDistancePIDController != null) {
//			m_ArcDistancePIDController.disable();
			m_ArcDistancePIDController.disableContinuousInput();
			//TODO
		}
		
	}
}
