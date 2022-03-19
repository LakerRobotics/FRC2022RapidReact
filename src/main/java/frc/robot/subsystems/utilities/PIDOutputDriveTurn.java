package frc.robot.subsystems.utilities;


import java.util.function.DoubleConsumer;

import frc.robot.subsystems.DriveTrain;
//import frc.robot.subsystems.DriveTrainMotionControl;

//import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class PIDOutputDriveTurn implements DoubleConsumer/*PIDOutput*/ {


	protected DriveTrain m_DriveTrain;
	
	public PIDOutputDriveTurn(DriveTrain m_DriveTrain2) {
	    SmartDashboard.putString("PIDOutputDriveTurn", "constructor called");
	    m_DriveTrain = m_DriveTrain2;
	}

	@Override

	//public void pidWrite(double output) {
	public void accept(double output) {
			m_DriveTrain.tankDrive(-output,output); 

		//System.out.println("DriveSpinPIDOutput Rotation Motor Output:"+output);
		SmartDashboard.putNumber("PIDOutputDriveTurn Rotation Motor Output",output); 
	}


}