package frc.robot.subsystems.utilities;

import frc.robot.subsystems.DriveTrain;
import java.util.function.DoubleConsumer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

	/**
	 * 
	 * @author Rich Topolewski
	 * 
	 * Used to take the speed calculated from the PID control and pass it to the drive train, 
	 * and also adjust the speed going to the wheels to drive straight
	 *
	 */

	public class PIDOutputStraightMotion implements DoubleConsumer /*PIDOutput*/ {

		double maxRotationPower = 1;

		private DriveTrain m_driveTrain;
		private Gyro m_TurnSource;
		private double m_targetAngle = 0.0d;
		private double rotationPower = 0.0d;
		private MotionControlPIDController m_RotationController;
		

		public PIDOutputStraightMotion(DriveTrain/*DriveTrainMotionControl*/ drivetrain, Gyro turnSource, double targetAngle) 
		{
			m_targetAngle = targetAngle;
			m_driveTrain = drivetrain;
			m_TurnSource = turnSource;
			
			double slowRotation = m_targetAngle + 90;// because we use motion control to start somewhere, and go to straight
			WrapRotationPIDOutput wrappedRotationPIDOutput =  new WrapRotationPIDOutput(this);
			
			m_RotationController = createRotationPIDController(m_targetAngle, slowRotation, (DoubleConsumer) wrappedRotationPIDOutput);
			
		}

		/**
		 * OLD came whne brought 2018 code in. should be eliminated i Think once the commands are working RGT 20220305 TODO 
		 * @param drivetrain
		 * @param turnSource
		 * @param targetAngle
		 */
/* 		public PIDOutputStraightMotion(DriveTrainMotionControl drivetrain, Gyro turnSource, double targetAngle) 
		{
			m_targetAngle = targetAngle;
//			m_driveTrain = drivetrain; TODO GET THIS TO WORK
			m_TurnSource = turnSource;
			
			double slowRotation = m_targetAngle + 90;// because we use motion control to start somewhere, and go to straight
			WrapRotationPIDOutput wrappedRotationPIDOutput =  new WrapRotationPIDOutput(this);
			
			m_RotationController = createRotationPIDController(m_targetAngle, slowRotation, (DoubleConsumer) wrappedRotationPIDOutput);
			
			//WrapRotationPIDInput  wrapRotationPIDInput = new WrapRotationPIDOutput(rotationPID, (PIDSource) m_gyro);
		}
*/

		public double calculate(double currentValue, double targetValue){
			rotationPower = m_RotationController.calculate(currentValue, targetValue);
			return rotationPower;
		}

		protected synchronized double getRotationPower(){
			return rotationPower;
		}


		protected synchronized void setRotationPower(double rotationPower) {
			this.rotationPower = rotationPower;
		}


		//@Override
		//public synchronized void pidWrite(double motorPower) 
		@Override
		public void accept(double motorPower) {
		    //rotationPower
		   	//double rotationPower = 0;
		   	//RobotMap.driveTrainRobotDrive21.arcadeDrive(/*moveValue*/ motorPower, /*rotateValue*/ rotationPower); 
		    SmartDashboard.putNumber("RobotDriveStraightPIDOoutput Motor Output",motorPower);
		    SmartDashboard.putNumber("RobotDriveStraightPIDOoutput RotationPower", rotationPower);
		    
	    	double leftPower; 
	    	double rightPower;
	    	
	    	// Reduce joystick power so can get full turning effect, and hopefully avoid a jerk in the rotation
	    	// if quickly taken off full throttle.
	    	if (motorPower + rotationPower > 1.0){
	    		motorPower = 1 - rotationPower;
	    	}
	    	
	    	leftPower = motorPower-rotationPower;
	    	rightPower = motorPower+rotationPower;
	    	
	    	m_driveTrain.tankDrive(-leftPower ,  -rightPower );
	    	
	    	//System.out.println("Left Power: " + leftPower);
	    	//System.out.println("Right power: " + rightPower);

		}
		
//		public  MotionControlPIDController createRotationPIDController(double targetAngle, double start, PIDOutput pidOutput) 
		public  MotionControlPIDController createRotationPIDController(double targetAngle, double start, DoubleConsumer pidOutput) {
			
		    double ramp 	=  30; //degrees
		    double maxspeed = 20.0*(360/60) ; //60/360 converts the first numbers which is in RPM to degrees/second
			
			// This is just a simple P control, Proportional control of the line follow
			// if we assume angle is in degrees and if we were off by 20 Degrees then we would want how much correction
			// for example id Kp is 0.025 at 20 degrees we would have 0.5 or half the power toward rotating the robot 
//			private double Kp = 1d/200d; //0.025;// 
		    //TODO change Kp to 1/20 or 0.025 so it is better at staying on the line.
			final double Kp = 0.025; // so at denominator off in the spin-Rate the power will reach the max
		    final double Ki = 0.0001;
		    final double Kd = 0.0;
		 
		    MotionControlPIDController localRotationSpeedPID;

		    AdjustSpeedAsTravelHelper rotationSpeedProfile; 
			GyroAngleAsDouble gyroAngleAsDouble = new GyroAngleAsDouble(m_TurnSource);
	        rotationSpeedProfile = new AdjustSpeedAsTravelMotionControlHelper(targetAngle, ramp, maxspeed, start, gyroAngleAsDouble/*,  pidOutput*/);
	        localRotationSpeedPID = new MotionControlPIDController(Kp,Ki,Kd, rotationSpeedProfile );
//	        localRotationSpeedPID.setOutputRange(-maxRotationPower, maxRotationPower);
	        //localRotationSpeedPID.setPID(Kp, Ki, Kd, 0);
			localRotationSpeedPID.setP(Kp);
			localRotationSpeedPID.setI(Ki);
			localRotationSpeedPID.setD(Kd);
	        //localRotationSpeedPID.enable();
//TODO			localRotationSpeedPID.;//Is there no enableing on the PID Controller, have to do that in some execute call calculat eth on the PIDController
		    return localRotationSpeedPID;
		}
		
	
//OLD	    public void disableRotationPIDController()
//OLD	    {
//OLD	    	m_RotationController.disable(); // no longer available, since new PIDController has to have its calculate method called.
//OLD	    	//m_RotationController.free();
//OLD	    }
	    
	    private class WrapRotationPIDOutput implements DoubleConsumer 
	    {

	        private PIDOutputStraightMotion m_RotationPowerDestination;

	        public WrapRotationPIDOutput(PIDOutputStraightMotion rotationPowerDesintation) 
	        {
	            if (rotationPowerDesintation == null) {
	                throw new NullPointerException("Given rotationPowerDestination was null");
	            }
	            else{
	                m_RotationPowerDestination = rotationPowerDesintation;            	
	            }
	        }

			@Override
//			public void pidWrite(double rotationPower) 
			public void accept(double rotationPower) {
				this.m_RotationPowerDestination.setRotationPower(-rotationPower); // Inverted because if your off in the positive direction then need to bring it back the other way
			}
	    }
	}
