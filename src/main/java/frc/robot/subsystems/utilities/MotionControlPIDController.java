package frc.robot.subsystems.utilities;

import java.util.TimerTask;

import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.interfaces.Gyro;
//import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj.PIDOutput;
import java.util.function.DoubleConsumer;
//import edu.wpi.first.wpilibj.PIDSource;
import java.util.function.DoubleSupplier;
//import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//?import edu.wpi.first.wpilibj.util.BoundaryException;

public class MotionControlPIDController extends PIDController {
	AdjustSpeedAsTravelHelper m_motionControlHelper; 
	/**
	 * 
	 * @return
	 * @override
	 * @throws Exception
	 */
	public double getRate() throws Exception
	{
		// Set the PIDSource to return Rate and then get the Rate.
//OLD not needed		m_motionControlHelper.getM_source().setPIDSourceType(PIDSourceType.kRate);
// This looks like a recusive loop which is bad so 		SmartDashboard.putNumber("Motion Control Rate", this.getRate());
//		SmartDashboard.putNumber("MotionControlPIDController Rate", m_motionControlHelper.getM_source().pidGet());
		SmartDashboard.putNumber("MotionControlPIDController Rate", m_motionControlHelper.getM_source().getAsDouble());
//		return m_motionControlHelper.getM_source().pidGet();
		return m_motionControlHelper.getM_source().getAsDouble();
	}
	
//	public MotionControlHelper getMotionControlHelper()
//	{
//		return m_motionControlHelper;
//	}

	public MotionControlPIDController(double Kp, double Ki, double Kd, AdjustSpeedAsTravelHelper motionControl) 
	{
		//super(Kp, Ki, Kd, motionControl.getM_source(), motionControl.getM_output());
		super(Kp, Ki, Kd);
		//TODO code needs to call calculate(motionControl.getM_source(), motionControl.getM_output());
		m_motionControlHelper = motionControl;
		motionControl.setRegularPIDControl(this);// to let the motionControl adjust the Rate, ie do the motion control
	}
	
//OLD	public MotionControlPIDController(double Kp, double Ki, double Kd, AdjustSpeedAsTravelHelper motionControl, double period) 
//OLD	{
//OLD		//super(Kp, Ki, Kd, motionControl.getM_source(), motionControl.getM_output(), period);
//OLD		super(Kp, Ki, Kd);
//OLD		//TODO code needs to call calculate(motionControl.getM_source(), motionControl.getM_output(), period);
//OLD		m_motionControlHelper = motionControl;
//OLD		motionControl.setRegularPIDControl(this);// to let the motionControl adjust the Rate, ie do the motion control
//OLD	}

//OLD	public MotionControlPIDController(double Kp, double Ki, double Kd, double Kf, AdjustSpeedAsTravelHelper motionControl) 
//OLD	{
//OLD		//super(Kp, Ki, Kd, Kf, motionControl.getM_source(), motionControl.getM_output());
//OLD		super(Kp, Ki, Kd, Kf, motionControl.getM_source(), motionControl.getM_output());
//OLD		m_motionControlHelper = motionControl;
//OLD		motionControl.setRegularPIDControl(this);// to let the motionControl adjust the Rate, ie do the motion control
//OLD	}

//OLD	public MotionControlPIDController(double Kp, double Ki, double Kd, double Kf, double period, AdjustSpeedAsTravelHelper motionControl) 
//OLD	{
//OLD		super(Kp, Ki, Kd, Kf, motionControl.getM_source(), motionControl.getM_output(), period);
//OLD		m_motionControlHelper = motionControl; 
//OLD		motionControl.setRegularPIDControl(this);// to let the motionControl adjust the Rate, ie do the motion control
//OLD	}
}
