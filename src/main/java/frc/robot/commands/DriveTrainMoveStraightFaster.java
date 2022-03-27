
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.utilities.AdjustSpeedAsTravelHelper;
//import frc.robot.subsystems.utilities.AdjustSpeedAsTravelMotionControlHelper;
import frc.robot.subsystems.utilities.AdjustSpeedAsTravelMotionControlHelperWithMinSpeed;
import frc.robot.subsystems.utilities.EncoderAvgLeftRight;
import frc.robot.subsystems.utilities.MotionControlPIDController;
import frc.robot.subsystems.utilities.PIDOutputStraightMotion;

/**
 *
 */
public class DriveTrainMoveStraightFaster  extends CommandBase {
    private final DriveTrain m_DriveTrain;

    RelativeEncoder m_leftEncoder; 
    RelativeEncoder m_rightEncoder; 
    Gyro m_rotationSource;

    private EncoderAvgLeftRight m_LineSource;
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

    private SimpleMotorFeedforward m_simpleMotorFeedForward;

    private boolean isStraightMovingForward = true;
    private final double StraightKp = 0.006;// 0.020;
    private final double StraightKi = 0.008;//0.001;
    private final double StraightKd = 0.0;
//    private final double StraightMaxPower = 1;

/** 
    * @param theDriveTrain the drivetrain subsystem
    * @param distanceSource the wheel encoders
    * @param rotationSource the Gryro
    * @param distance  to travel in inches
    * @param maxSpeed  in ft/sec
    * @param ramp      in inches
    * @param targetAngle assume it want Degrees
    * @param targetToleracne provided in inches, the speed will be adjusted based on this target tolerance, so should hit the target
    ------------------------------------------------*/
   public DriveTrainMoveStraightFaster(DriveTrain theDriveTrain, double distance, double maxspeed, double ramp, double targetAngle, double targetTolerance){

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

        m_StraightTolerance = targetTolerance;

        if(distance > 0){
            isStraightMovingForward = true;
        }
        else{
            isStraightMovingForward = false;
        }

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
	{
        System.out.println("DriveTrainMoveStraight.initialize Entered");
            m_LineSource.reset();
			
			double start = 0;
			
			double convertedDistance = m_distance;	// Inches
			double convertedSpeed = m_maxspeed * 12; 	// Converted from Feet/Second to Inches/Second
			double convertedRamp = m_ramp;			// Inches/Second
            double minSpeed = m_StraightTolerance/0.02; /*seconds the typical sampeling speed or time between when measurements are taken*/
            minSpeed = minSpeed/4; // Add a factor of safty so should hit target withinspecified tolerance
			
//			if (!(Math.abs(m_LineSource.getDistance()) > Math.abs(m_DistanceToExceed)))
//			{
                // TURN POWER to drive straight, Setup PID to constantly adjust the turn to follow the gyro
				m_StraightRotationPIDOutput = new PIDOutputStraightMotion(m_DriveTrain, m_TurnSource, m_targetAngle);

                // FORWARD POWER, will have two parts, a guess of the motor power needed plus PID control to try and get to actaul speed requesting
                // so 20% min power to move (deadzone)
                // assuming max speed robot is 13 ft/sec which 156 in/sec need to get to 100% that calculcat e0.0059rr for the second number, but it was too low
                m_simpleMotorFeedForward = new SimpleMotorFeedforward(0.20, 0.008);//0.005944);

                //Instantiates a new AdjustSpeedAsTravelMotionControlHelper() object for the driveStraightDistance we are going to traverse
                m_AdustsSpeedAsTravelStraightHelper = new AdjustSpeedAsTravelMotionControlHelperWithMinSpeed(convertedDistance, convertedRamp, convertedSpeed, 
				                                                                                 start, m_LineSource,/*m_StraightRotationPIDOutput*/// Not needed to be passed in, this is don here in exeute, this is just here for historical reasons and should be eliminated
                                                                                                 minSpeed);
                                                                                                 //Instantiates a new MotionControlPIDController() object for the new drive segment using the AdustSpeedAsTravelMotionControlHelper to very the speed
				m_StraightDistancePIDController = new MotionControlPIDController(StraightKp, StraightKi, StraightKd, m_AdustsSpeedAsTravelStraightHelper);
				m_StraightDistancePIDController.setTolerance(m_StraightTolerance);// there is also a setTolerance that takes position and velocity acceptable tolerance
//			}
    	}
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
        
        double distanceSoFar = m_LineSource.getDistance();
        double targetSpeed   = m_AdustsSpeedAsTravelStraightHelper.getTargetSpeed(distanceSoFar);
        double currentSpeed  = m_LineSource.getRate();

        double firstGuessAtMotorPower = m_simpleMotorFeedForward.calculate(targetSpeed);
        double pidTuneForwardPower = m_StraightDistancePIDController.calculate(currentSpeed, targetSpeed);
        double forwardPower = firstGuessAtMotorPower + pidTuneForwardPower;
    
        double angleRightNow = m_TurnSource.getAngle();
        double turnPower = m_StraightRotationPIDOutput.calculate(angleRightNow,m_targetAngle);

        m_DriveTrain.arcadeDrive(forwardPower, turnPower);

        
        SmartDashboard.putNumber("DriveStraight distanceSoFar", distanceSoFar );
        SmartDashboard.putNumber("DriveStraight targetSpeed", targetSpeed);
        SmartDashboard.putNumber("DriveStraight currentSpeed", currentSpeed);
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
        SmartDashboard.putNumber("Target distance", m_DistanceToExceed);
        SmartDashboard.putNumber("Straight Tolerance", m_StraightTolerance);
            
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
            }
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
