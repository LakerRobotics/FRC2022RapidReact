package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import frc.robot.subsystems.utilities.EncoderAvgLeftRight;

/*
 * This is to wrap the drive train to provide the wheel encoders as PIDInput sources
 */
public class PIDSourceDistance implements DoubleSupplier {

	DriveTrain m_driveTrain;
	EncoderAvgLeftRight m_encoderAvg;
	boolean returnDistance = true;
	
	public PIDSourceDistance(DriveTrain the_driveTrain) {
		m_driveTrain = the_driveTrain;
        m_encoderAvg = new EncoderAvgLeftRight(m_driveTrain.getEncoderLeft(), m_driveTrain.getEncoderRight());
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
			return m_encoderAvg.getDistance();// m_driveTrain.GetAverageDistance();
		}
		else {
			return m_encoderAvg.getRate();//m_driveTran.GetAverageSpeed();			
		}

	}

}
