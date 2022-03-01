package frc.robot.subsystems;


//import edu.wpi.first.wpilibj.PIDSource;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.shuffleboard.*;
//import edu.wpi.first.wpilibj.PIDSourceType;`

/*
 * This is to wrap the drive train to provide the wheel encoders as PIDInput sources
 */
public class PIDSourceDistance implements DoubleSupplier {

	DriveTrainMotionControl m_driveTrainMotionControl;
	boolean returnDistance = true;
	
	public PIDSourceDistance(DriveTrainMotionControl the_driveTrainMotionControl) {
		m_driveTrainMotionControl = the_driveTrainMotionControl;
	}


	
	
//	@Override
	public void setPIDSourceTypeToDistance(boolean isDistance) {
		if(isDistance == true) {
			returnDistance = true;
		}
		else {
			returnDistance = false; // so return speed
		}
	}

//	@Override
	public boolean /*PIDSourceType*/ isDistance/*getPIDSourceType*/() {
		if(returnDistance) {
			return true;/*PIDSourceType.kDisplacement;*/
		}else {
			return false;/*PIDSourceType.kRate;*/
		}		
	}

//	@Override
	public double pidGet() {
		return getAsDouble();
	}
	public double getAsDouble(){
		if(returnDistance) {
			return m_driveTrainMotionControl.GetAverageDistance();
		}
		else {
			return m_driveTrainMotionControl.GetAverageSpeed();			
		}

	}

}
