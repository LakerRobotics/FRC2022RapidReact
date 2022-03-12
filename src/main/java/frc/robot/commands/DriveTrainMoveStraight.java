package frc.robot.commands;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.utilities.AdjustSpeedAsTravelHelper;
import frc.robot.subsystems.utilities.AdjustSpeedAsTravelMotionControlHelper;
import frc.robot.subsystems.utilities.EncoderAvgLeftRight;
import frc.robot.subsystems.utilities.EncoderDistenceAsDouble;
import frc.robot.subsystems.utilities.MotionControlPIDController;
import frc.robot.subsystems.utilities.PIDOutputStraightMotion;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class DriveTrainMoveStraight extends CommandBase {
    private final DriveTrain m_DriveTrain;

    RelativeEncoder m_leftEncoder; 
    RelativeEncoder m_rightEncoder; 
    Gyro m_rotationSource;

	//PIDSource m_LineSource;
	//DoubleSupplier m_LineSource;
    private Encoder m_LineSource;
	//PIDSource m_TurnSource;
	//DoubleSupplier m_TurnSource;
	private Gyro m_TurnSource;
    private double m_distance;
    private double m_DistanceToExceed; //TODO Check if can Eliminate this redudent variable
    private double m_maxspeed;
    private double m_ramp;
    private double m_targetAngle;
    
	private double m_StraightTolerance;
	private AdjustSpeedAsTravelHelper m_AdustsSpeedAsTravelStraightHelper;
    private PIDOutputStraightMotion    m_StraightRotationPIDOutput;
	private MotionControlPIDController m_StraightDistancePIDController;


    private boolean isStraightMovingForward = true;
    private final double StraightKp = 0.08;
    private final double StraightKi = 0.0001;
    private final double StraightKd = 0.0;
    private final double StraightMaxPower = 1;

 

/** 
    * @param theDriveTrain the drivetrain subsystem
    * @param distanceSource the wheel encoders
    * @param rotationSource the Gryro
    * @param distance  to travel in inches
    * @param maxSpeed  in ft/sec
    * @param ramp      in inches
    * @param targetAngle assume it want Degrees
    ------------------------------------------------*/
   public DriveTrainMoveStraight(DriveTrain theDriveTrain, double distance, double maxspeed, double ramp, double targetAngle){


        m_DriveTrain = theDriveTrain;
        addRequirements(m_DriveTrain);
        m_leftEncoder    = theDriveTrain.getEncoderLeft();
        m_rightEncoder   = theDriveTrain.getEncoderRight();
        m_rotationSource = theDriveTrain.getGyro();
     
        m_LineSource = new EncoderAvgLeftRight(m_leftEncoder, m_rightEncoder);
        m_TurnSource = m_rotationSource;
        m_distance = distance;
        m_DistanceToExceed = m_distance; //TODO check if can eliminate this duplicate varable
        m_maxspeed = maxspeed;
        m_ramp = ramp;
        m_targetAngle = targetAngle;

        m_StraightTolerance = 0.5;
    

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
	{
//			m_DriveTrain.ResetEncoders();
            m_LineSource.reset();
			
			double start = 0;
			
			double convertedDistance = m_distance;	// Inches
			double convertedSpeed = m_maxspeed * 12; 	// Converted from Feet/Second to Inches/Second
			double convertedRamp = m_ramp;			// Inches/Second
			
			if (!(Math.abs(m_LineSource.getDistance()) > Math.abs(m_DistanceToExceed)))
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
				//return true;
			}
			//return false;
//		}
//		return true;
	}

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //get 
        double distanceSoFar = m_LineSource.getDistance();
        double targetSpeed = m_AdustsSpeedAsTravelStraightHelper.getTargetSpeed(distanceSoFar);
        double currentSpeed = m_LineSource.getRate();
        double forwardPower = m_StraightDistancePIDController.calculate(currentSpeed, targetSpeed);

        double angleRightNow = m_TurnSource.getAngle();
        double turnPower = m_StraightRotationPIDOutput.calculate(angleRightNow,m_targetAngle);

            m_DriveTrain.arcadeDrive(forwardPower, turnPower);
            SmartDashboard.putNumber("DriveStraign Target distance", m_DistanceToExceed);
            SmartDashboard.putNumber("DriveStraight distanceSoFar", distanceSoFar );
            SmartDashboard.putNumber("DriveStraight targetSpeed", targetSpeed);
            SmartDashboard.putNumber("DriveStraight forwardPower", forwardPower);
            SmartDashboard.putNumber("DriveStraight angleRightNow", angleRightNow);
            SmartDashboard.putNumber("DriveStraight turnPower", turnPower);
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
//            SmartDashboard.putNumber("Distance Left", m_DriveTrain.GetLeftDistance());
            SmartDashboard.putNumber("Target distance", m_DistanceToExceed);
            SmartDashboard.putNumber("Straight Tolerance", m_StraightTolerance);
            
            //TODO Verify this tolerance works... it should...
            SmartDashboard.putNumber("Average Distance", m_LineSource.getDistance());
            SmartDashboard.putNumber("Target", Math.abs(m_DistanceToExceed - m_StraightTolerance));
    
            boolean didExceedDistance = false;
            if(isStraightMovingForward) {
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

    @Override
    public boolean runsWhenDisabled() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
        return false;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    }
}
