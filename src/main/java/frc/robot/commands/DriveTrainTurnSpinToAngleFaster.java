package frc.robot.commands;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
//import frc.robot.subsystems.utilities.AdjustSpeedAsTravelMotionControlHelper;
import frc.robot.subsystems.utilities.AdjustSpeedAsTravelMotionControlHelperWithMinSpeed;
import frc.robot.subsystems.utilities.GyroAngleAsDouble;
import frc.robot.subsystems.utilities.MotionControlPIDController;

/**
 *
 */
public class DriveTrainTurnSpinToAngleFaster  extends CommandBase {
    private final DriveTrain m_DriveTrain;

    RelativeEncoder m_leftEncoder; 
    RelativeEncoder m_rightEncoder; 
    Gyro m_rotationSource;

	private Gyro m_TurnSource;
    private double m_targetAngle;
    
    private AdjustSpeedAsTravelMotionControlHelperWithMinSpeed m_AdjustRpmAsTurnHelper;

	private MotionControlPIDController m_TurnPIDController;

    private double m_TurnTolerance;// = 5;// had been 0.5		
    private double m_AngularVelocityTolerance = 15;

	private final double TurnKp = 0.005;
	private final double TurnKi = 0.0020;
	private final double TurnKd = 0.0;
//	private final double TurnMaxPower = 1;
 

/** 
    * @param theDriveTrain the drivetrain subsystem
    * @param turnToAngle In Degrees, in some field reference fram, like lookig downfield is 0 derees
    * @param turnAccuracy in degrees
    ------------------------------------------------*/
   public DriveTrainTurnSpinToAngleFaster(DriveTrain theDriveTrain, double turnToAngle, double accuracy){
        m_DriveTrain = theDriveTrain;
        m_TurnSource = theDriveTrain.getGyro();
        m_targetAngle = turnToAngle;
        m_TurnTolerance = accuracy;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //TODO to really have it turn on a Dime should monitor left to right wheel and make sure adding them goes to Zero
		// create a forward motion PID control on that then you can get precise turning.
		double start = m_TurnSource.getAngle();
		
		double maxRPM = 120/*30*/;			// Rotations/Minute
		double ramp = 45/* 3.5 * maxRPM*/;	//angle off from target to start slowing down.			
		double maxSpeed = maxRPM * 6; // 360 Degrees/60 seconds to convert RPM to speed or degrees per second
        double minRPM = (m_TurnTolerance/360)/(0.02/60);// so accuracy (converting Degrees to Revolutions) divide by 0.02 or 20 milliseconds, converted to Min by dividing by 60, which is how ofter the robot does updates
               minRPM = m_TurnTolerance/4;// Factor of safty to get us to detect and stop on the the design Turn Tolerance
		
//		if (!(Math.abs(m_TurnSource.getAngle()-m_targetAngle) < m_TurnTolerance)){
			//Instantiates a new MotionControlHelper() object for the new turn segment
			m_AdjustRpmAsTurnHelper = new AdjustSpeedAsTravelMotionControlHelperWithMinSpeed(m_targetAngle, ramp, maxSpeed, start, 
			                                                                      new GyroAngleAsDouble(m_TurnSource),/*new PIDOutputDriveTurn(m_DriveTrain)*/
                                                                                  minRPM);
			//Instantiates a new MotionControlPIDController() object for the new turn segment using the previous MotionControlHelper()
			m_TurnPIDController = new MotionControlPIDController(TurnKp, TurnKi, TurnKd, m_AdjustRpmAsTurnHelper);
				
			//Turns the MotionControlPID ON and it will continue to execute by itself until told otherwise.
			//m_TurnPIDController.enable();	
//		}

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double angleRightNow = m_TurnSource.getAngle();
        double targetRotationSpeed = m_AdjustRpmAsTurnHelper.getTargetSpeed(angleRightNow);
        double currentRotationSpeed = m_TurnSource.getRate();
        double turnPower = m_TurnPIDController.calculate(currentRotationSpeed,targetRotationSpeed);

        m_DriveTrain.arcadeDrive(0, turnPower);

        SmartDashboard.putNumber("DriveTurn angleRightNow", angleRightNow);
        SmartDashboard.putNumber("DriveTurn targetRotationSpeed", targetRotationSpeed);
        SmartDashboard.putNumber("DriveTurn currentRotationSpeed",currentRotationSpeed);
        SmartDashboard.putNumber("DriveTurn turnPower", turnPower);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
//        m_DriveTrain.tankDrive(0, 0);// ArcadeDrive(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
            if (Math.abs(m_TurnSource.getAngle()-m_targetAngle) < m_TurnTolerance  &&
                Math.abs(m_TurnSource.getRate()               ) < m_AngularVelocityTolerance)
            {
                //Stop
                m_DriveTrain.tankDrive(0, 0);// ArcadeDrive(0, 0);
                return true;
            }
            return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }

}
