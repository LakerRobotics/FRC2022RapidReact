package frc.robot.commands;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.utilities.AdjustAngleAsTravelHelper;
/**
 *
 */
public class DriveTrainTurnOnCircleToAngle extends DriveTrainMoveStraight {
//    private double m_distance;
//    private double m_DistanceToExceed; //TODO Check if can Eliminate this redudent variable

//    private AdjustSpeedAsTravelMotionControlHelper m_AdjustRpmAsTurnHelper;
	protected AdjustAngleAsTravelHelper m_AdjustAngleAsTravelArcHelper;

//    private double m_TurnTolerance = 5;// had been 0.5		
//    private double m_AngularVelocityTolerance = 15;

//    private final double TurnKp = 0.005;
//    private final double TurnKi = 0.0020;
//    private final double TurnKd = 0.0;
//    private final double TurnMaxPower = 1;


 
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
 * @param turnToAngle Degrees, absolute. Example, we could be starting at 90 and going to 180, so 180 would be specified here.
 * @param circleRadius in inches, positive is clockwise, negative is counterclockwise
 */   
public DriveTrainTurnOnCircleToAngle(DriveTrain theDriveTrain, double rampUp, double maxSpeed, double rampdown, double turnToAngle, double circleRadius, boolean isTurnRight, boolean isForward){
    super( theDriveTrain,  rampUp, 0, maxSpeed, turnToAngle);//Note, distance will bet set in a seperate call turnToAngle will be overwriden by the .getTargetAngle(..)
    // Determine:
    //    the distance to travel and 
    //    the direction (forward or backward) and 
    //    clockwise or counter clockwise
    //=============================================================================
        double currentAngle = theDriveTrain.getGyro().getAngle();

        double currentDistance = m_LineSource.getDistance(); // not m_LineSource is created in the call to super above.

        m_AdjustAngleAsTravelArcHelper = new AdjustAngleAsTravelHelper(currentDistance, circleRadius, currentAngle, turnToAngle);

        double distance = m_AdjustAngleAsTravelArcHelper.getLengthOfArc();

        // Assign distance as either forward or backward.
        if(isForward){
            // We are going forward
            //nothing to do distance = distance
            if(isTurnRight){
                //clockwise
            }
            else{
                //counterclockwise
            }
        }else{
            // We are going backwards
            distance = -distance;
            if(isTurnRight){
                //counterclockwise
            }
            else{
                //clockwise
            }
        }

        setDistance(distance);
//        m_AdjustAngleAsTravelArcHelper = new AdjustAngleAsTravelHelper(); 
    }
/**
 * 
 * @param currentDistance, 
 * @return the target Angle in degree for the provided distance, note in the base drive straight this is always just the passed in straight angle
 *         but in a derived class like turn on a circle it would but updated for each distance
 * 
 * NOTE: this shoul overide the derive straight (which has the same function but always returns a constant Angle, but this changes angle as moves on arc)
 */
@Override
protected double getTargetAngle(double currentDistance){
    return m_AdjustAngleAsTravelArcHelper.getTargetAngle(currentDistance);
};


    // Returns true when the command should end.
    public boolean isFinished() {
        if(super.isFinished()){
            return true;
        //TODO  Check angle make sure we are within tolerance
        }
        else{
            return false;
        }
    }

}
