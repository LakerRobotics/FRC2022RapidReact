package frc.robot.commands;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.utilities.AdjustSpeedAsTravelMotionControlHelper;
import frc.robot.subsystems.utilities.EncoderAvgLeftRight;
import frc.robot.subsystems.utilities.GyroAngleAsDouble;
import frc.robot.subsystems.utilities.MotionControlPIDController;
import frc.robot.subsystems.utilities.PIDOutputArcMotion;
/**
 *
 */
public class DriveTrainTurnOnCircleToAngle extends DriveTrainMoveStraight {
    private double m_distance;
    private double m_DistanceToExceed; //TODO Check if can Eliminate this redudent variable

//    private AdjustSpeedAsTravelMotionControlHelper m_AdjustRpmAsTurnHelper;
	private AdjustSpeedAsTravelMotionControlHelper m_AdjustSpeedAsTravelArcHelper;
    private MotionControlPIDController m_ArcDistancePIDController; 


    private PIDOutputArcMotion m_ArcRotationPIDOutput;

//    private double m_TurnTolerance = 5;// had been 0.5		
//    private double m_AngularVelocityTolerance = 15;

//    private final double TurnKp = 0.005;
//    private final double TurnKi = 0.0020;
//    private final double TurnKd = 0.0;
//    private final double TurnMaxPower = 1;

    private final double ArcKp = 0.002;//StraightKp; //0.002;
	private final double ArcKi = 0.001; //StraightKi; //0.001;
	private final double ArcKd = 0.0; //StraightKd; //0.0;
    private final double m_DistanceTolerance = 0.5;

 
/**
 * This is the Old existing Constuctore
 * @param theDriveTrain
 * @param distance  in inches
 * @param maxSpeed in ft/sec
 * @param ramp, inches to go from start speed (e.g. 0 ft/sec) to the maxSpeed
 * @param radiusOfArc, positive is clockwise, negative is counter clockwise
 */
/*     public DriveTrainTurnOnCircleToAngle(DriveTrain theDriveTrain, double distance, double maxSpeed, double ramp, double radiusOfArc){
        m_DriveTrain = theDriveTrain;
        addRequirements(m_DriveTrain);
        m_leftEncoder    = theDriveTrain.getEncoderLeft();
        m_rightEncoder   = theDriveTrain.getEncoderRight();
        m_rotationSource = theDriveTrain.getGyro();
     
        m_LineSource = new EncoderAvgLeftRight(m_leftEncoder, m_rightEncoder);
//        m_TurnSource = m_rotationSource;
        m_TurnSource = theDriveTrain.getGyro();
        m_distance = distance;
        m_DistanceToExceed = m_distance; //TODO check if can eliminate this duplicate varable
        m_maxspeed = maxSpeed;
        m_ramp = ramp;
        m_targetAngle = targetAngle;

        //m_StraightTolerance = 0.5;

    }
*/
/**
 *  This is the new Constructor we want to code Up 
 * @param theDriveTrain
 * @param rampUp in inches distance to get to the maxSpeed
 * @param maxSpeed   ft/sec
 * @param rampDown in inches distance to get from maxSpeed down to slow speed at target distance (calculate from target angle & circleRadius)
 * @param turnToAngle postive from current Angle would mean clockwise rotation
 * @param circleRadius in inches
 */   public DriveTrainTurnOnCircleToAngle(DriveTrain theDriveTrain, double rampUp, double maxSpeed, double rampdown, double turnToAngle, double circleRadius){

    // Determine:
    //    the distance to travel and 
    //    the direction (forward or backward) and 
    //    clockwise or counter clockwise
    //=============================================================================
        double currentAngle = theDriveTrain.getGyro().getAngle();

        //Calculate length of arc on the cirle we are to traverse
        double archAngle = turnToAngle-currentAngle;
        double distanceToTravel = circleRadius *2*java.lang.Math.PI*(archAngle/360);

        if(maxSpeed>0){
            if(archAngle>0){
                //So we are being asked to go clockwise (maxSpeed positive), and increase our Angle, which we can do by going forwar and turning to the right
                distanceToTravel = -circleRadius *2*java.lang.Math.PI; //TODO not have negative but for this robot negative is forward
            }else{
               //So we are being asked to go clockwise (maxSpeed positive), and decrease our Angle, which we cant do by going forwar and turning to the right
 
            }

        }
        else{

        }
     
        m_LineSource = new EncoderAvgLeftRight(m_leftEncoder, m_rightEncoder);
//        m_TurnSource = m_rotationSource;
        m_TurnSource = theDriveTrain.getGyro();

        // 
        double currentAngle = m_TurnSource.getAngle();

        double percentOfCircle = 0;
        if (maxSpeed >0) {            
            percentOfCircle = (turnToAngle - currentAngle)/360;
        }
        else{
            percentOfCircle = (currentAngle-turnToAngle)/360;
        }
        // Calculate distance based on percent of diameter traveled
        m_distance = circleRadius*java.lang.Math.PI*percentOfCircle;
        m_DistanceToExceed = m_distance; //TODO check if can eliminate this duplicate varable
        m_maxspeed = maxSpeed;
        m_ramp = 12; // default to 12 inches, maybe provide another constuctor that allwows it to be passed in
        m_radiusOfArc = circleRadius;

//        m_DistanceTolerance = 0.5;


    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
	{
        m_LineSource.reset();
			
        double start = 0;
        
        double convertedDistance = m_distance;	// Inches
        double convertedSpeed = m_maxspeed * 12; 	// Converted from Feet/Second to Inches/Second
        double convertedRamp = m_ramp;			// Inches/Second
        
//			if (!(Math.abs(m_LineSource.getDistance()) > Math.abs(m_DistanceToExceed)))
//			{
            // TURN POWER to drive straight, Setup PID to constantly adjust the turn to follow the gyro
            m_StraightRotationPIDOutput = new PIDOutputStraightMotion(m_DriveTrain, m_TurnSource, m_targetAngle);

            // FORWARD POWER, will have two parts, a guess of the motor power needed plus PID control to try and get to actaul speed requesting
            // so 20% min power to move (deadzone)
            // assuming max speed robot is 13 ft/sec which 156 in/sec need to get to 100% that calculcat e0.0059rr for the second number, but it was too low
            m_simpleMotorFeedForward = new SimpleMotorFeedforward(0.20, 0.008);//0.005944);

            //Instantiates a new AdjustSpeedAsTravelMotionControlHelper() object for the driveStraightDistance we are going to traverse
            m_AdustsSpeedAsTravelStraightHelper = new AdjustSpeedAsTravelMotionControlHelper(convertedDistance, convertedRamp, convertedSpeed, 
                                                                                             start, m_LineSource/*m_StraightRotationPIDOutput*/);// Not needed to be passed in, this is don here in exeute, this is just here for historical reasons and should be eliminated
            //Instantiates a new MotionControlPIDController() object for the new drive segment using the AdustSpeedAsTravelMotionControlHelper to very the speed
            m_StraightDistancePIDController = new MotionControlPIDController(StraightKp, StraightKi, StraightKd, m_AdustsSpeedAsTravelStraightHelper);
            m_StraightDistancePIDController.setTolerance(m_StraightTolerance);// there is also a setTolerance that takes position and velocity acceptable tolerance
//			}

	}

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        		//TODO have it pay attention to current position and calc based on the differance
		if(m_DistanceToExceed>0){
			isArcMovingForward = true;
		}
		else {
			isArcMovingForward = false;
		}
		m_DistanceToExceed = m_distance;//inches
//		m_DriveTrain.ResetEncoders();
		
		double start = 0;

		
//OLD		if (!isPIDEnabled())
//OLD		{
			double convertedDistance = m_DistanceToExceed; 	// In inches
			double convertedSpeed = m_maxspeed * 12; 	// convert from feet to inches/second
			double convertedRamp = m_ramp; 			// in inches
			

//			//Instantiates a new MotionControlHelper() object for the new Arch segment
//			//Instantiates a new AdjustSpeedAsTravelMotionControlHelper() object for the driveStraightDistance we are going to traverse
//			m_StraightRotationPIDOutput = new PIDOutputStraightMotion(m_DriveTrain, m_TurnSource, m_targetAngle);
//			m_AdustsSpeedAsTravelStraightHelper = new AdjustSpeedAsTravelMotionControlHelper(convertedDistance, convertedRamp, convertedSpeed, start, m_StraightSource, m_StraightRotationPIDOutput);
			// motionControlForwardSpeed
			m_ArcRotationPIDOutput         = new PIDOutputArcMotion(m_DriveTrain, new GyroAngleAsDouble(m_TurnSource), m_radiusOfArc);
            m_AngleRotationPID
			m_AdjustSpeedAsTravelArcHelper = new AdjustSpeedAsTravelMotionControlHelper(convertedDistance, convertedRamp, convertedSpeed, start, 
			                                                                            m_LineSource//,
																						 //m_ArcRotationPIDOutput
                                                                                         );
			
//			//Instantiates a new MotionControlPIDController() object for the new drive segment using the previous MotionControlHelper()
//			m_StraightDistancePIDController = new MotionControlPIDController(StraightKp, StraightKi, StraightKd, m_AdustsSpeedAsTravelStraightHelper);
//			m_StraightDistancePIDController.setAbsoluteTolerance(m_StraightTolerance);
//			m_StraightDistancePIDController.setOutputRange(-StraightMaxPower, StraightMaxPower);
//			
			//Instantiates a new MotionControlPIDController() object for the new turn segment using the previous MotionControlHelper()
            
			m_ArcDistancePIDController = new MotionControlPIDController(ArcKp, ArcKi, ArcKd, m_AdjustSpeedAsTravelArcHelper);
//			m_ArcDistancePIDController.setAbsoluteTolerance(m_StraightTolerance);
			m_ArcDistancePIDController.setTolerance(m_DistanceTolerance);
			//OLD m_ArcDistancePIDController.setOutputRange(-arcMaxPower, arcMaxPower);//No longer availalbe code has to do it itself, docs suggest using clamp
			
//			//Turns the MotionControlPID ON and it will continue to execute by itself until told otherwise.
//			m_StraightDistancePIDController.enable();
//			m_PIDEnabled = true;
			//Turns the MotionControlPID ON and it will continue to execute on a seperate thread by itself until told otherwise.
//OLD TODO: get calulate called and do something with the next output value			m_ArcDistancePIDController.enable();
//			return true;

        }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
 		/*
		 * Called while waiting for the MotionControlPID to finish. The PID will be disabled when the end condition is met, and
		 * the return value indicates you can proceed to the next step.
		 * */
		SmartDashboard.putNumber("DriveTrainTurnOnCircleToAngle - Distance", m_LineSource.getDistance());//
		System.out.println("DriveTrainTurnOnCircleToAngle - Distance = "+ m_LineSource.getDistance());
		SmartDashboard.putNumber("DriveTrainTurnOnCircleToAngle - Distance to Exceed", m_DistanceToExceed);
		System.out.println("DriveTrainTurnOnCircleToAngle - Distance to Exceed = "+ m_DistanceToExceed);
		System.out.println("DriveTrainTurnOnCircleToAngle - isArcMovingForward = "+ isArcMovingForward);
//		SmartDashboard.putNumber("Straight Tolerance", m_StraightTolerance);
		
		boolean didExceedDistance = false;
		if(isArcMovingForward) {
			// Traveling Forward
			if (m_LineSource.getDistance() > m_DistanceToExceed) {
				didExceedDistance = true;
			}else {
				didExceedDistance = false;
			}
		}else {
			// Traveling Backward
			if (m_LineSource.getDistance() < m_DistanceToExceed) {
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


    @Override
    public boolean runsWhenDisabled() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
        return false;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    }
}
