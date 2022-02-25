// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;


import frc.robot.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.revrobotics.CANSparkMax; 
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
    import com.revrobotics.CANSparkMax; 
    import com.revrobotics.CANSparkMaxLowLevel.MotorType;
    import com.revrobotics.SparkMaxPIDController;
    import com.revrobotics.RelativeEncoder;
    import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 *
 */
public class Intake extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
private CANSparkMax intakeSpark;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private SparkMaxPIDController m_pidController;
    private RelativeEncoder m_encoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
    
    /**
    *
    */
    public Intake() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
intakeSpark = new CANSparkMax(3, MotorType.kBrushless);
 
 


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    initializeSpeedControl(intakeSpark);

    
    }

    private void initializeSpeedControl(CANSparkMax intakeSpark){
        
        /**
         * In order to use PID functionality for a controller, a SparkMaxPIDController object
         * is constructed by calling the getPIDController() method on an existing
         * CANSparkMax object
         */
        m_pidController = intakeSpark.getPIDController();
    
        // Encoder object created to display position values
        m_encoder = intakeSpark.getEncoder();
    
        // PID coefficients
        //kP = 6e-5;
        kP = 0.001; 
        kI = 0;
        kD = 0; 
        kIz = 0; 
        kFF = 0.000015; 
        kMaxOutput = 1; 
        kMinOutput = -1;
        maxRPM = 1200;
    
        // set PID coefficients
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);
    
        // display PID coefficients on SmartDashboard
        
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);
    
        };



    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public void movePower(double power){
        intakeSpark.set(power);
    }

    public void moveSpeed(double speed){
        double rpmSpeed = speed*maxRPM;
        m_pidController.setReference(rpmSpeed, CANSparkMax.ControlType.kVelocity);
        
        SmartDashboard.putNumber("IntakeSetSpeed", rpmSpeed);
        SmartDashboard.putNumber("IntakeActualSpeed", m_encoder.getVelocity());
        SmartDashboard.putNumber("IntakeMotorOutputCurrent", intakeSpark.getOutputCurrent());

    }

}

