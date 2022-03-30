package frc.robot.commands;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.utilities.AdjustAngleAsTravelHelper;
/**
 *
 */
public class DriveTrainTurnOnCircleToAngle extends DriveTrainMoveStraight {
	protected AdjustAngleAsTravelHelper m_AdjustAngleAsTravelArcHelper;

 
/**
 * @param theDriveTrain
 * @param rampUp in inches distance to get to the maxSpeed
 * @param maxSpeed   ft/sec
 * @param rampDown in inches distance to get from maxSpeed down to slow speed at target distance (calculate from target angle & circleRadius)
 * @param turnToAngle Degrees, absolute. Example, we could be starting at 90 and going to 180, so 180 would be specified here.
 * @param circleRadius in inches, positive is clockwise, negative is counterclockwise
 * @param 
 */   
public DriveTrainTurnOnCircleToAngle(DriveTrain theDriveTrain, double rampUp, double maxSpeed, double rampdown, double turnToAngle, double circleRadius, boolean isForward){
    super( theDriveTrain,  0,  maxSpeed, rampUp, turnToAngle);//Note, distance will bet set in a seperate call turnToAngle will be overwriden by the .getTargetAngle(..)
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
            //distance = distance;// nothing to do
        }else{
            // We are going backwards, put distance to negative
            distance = -distance;
        }

        super.setDistance(distance);
//        m_AdjustAngleAsTravelArcHelper = new AdjustAngleAsTravelHelper(); 
    }
/**
 * 
 * @param currentDistance, 
 * @return the target Angle in degree for the provided distance, note in the base drive straight this is always just the passed in straight angle
 *         but in a derived class like turn on a circle it would but updated for each distance
 * 
 * NOTE: this should overide the drive straight (which has the same function but always returns a constant Angle, but this changes angle as moves on arc)
 */
@Override
protected double getTargetAngle(double currentDistance){
    return m_AdjustAngleAsTravelArcHelper.getTargetAngle(currentDistance);
};


    // Returns true when the command should end.
/*    public boolean isFinished() {
        if(super.isFinished()){
            return true;
        //TODO  Check angle make sure we are within tolerance
        }
        else{
            return false;
        }
    }
*/
}
