package frc.robot.commands;

// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Command.

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

/**
 *
 */
public class ShooterMoveMed  extends CommandBase {
    private final Shooter m_shooter;
    private static final double m_curbShootRPM = 0.65*5000;

    public ShooterMoveMed(Shooter subsystem) {
        m_shooter = subsystem;
        addRequirements(m_shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //m_shooter.move(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_shooter.moveSpeed(m_curbShootRPM);
        if(java.lang.Math.abs((m_shooter.getSpeed()-m_curbShootRPM)/m_curbShootRPM) < 0.1){
            // Rumble power is 100% less 10% for every 1% off of the target speed, so will be at max rumbel when at speed
            double rumblePower = 1-10*java.lang.Math.abs((m_shooter.getSpeed()-m_curbShootRPM)/m_curbShootRPM);
            RobotContainer.getInstance().getOperatorController().setRumble(RumbleType.kRightRumble,rumblePower);
            RobotContainer.getInstance().getDriverController().setRumble(  RumbleType.kRightRumble,rumblePower);
        }
        else{
            RobotContainer.getInstance().getOperatorController().setRumble(RumbleType.kRightRumble, 0.0);
            RobotContainer.getInstance().getDriverController().setRumble(  RumbleType.kRightRumble, 0.0);
        }
        SmartDashboard.putNumber("Shooter Low Speed", m_shooter.getSpeed());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_shooter.move(0);
        RobotContainer.getInstance().getOperatorController().setRumble(RumbleType.kRightRumble, 0.0);
        RobotContainer.getInstance().getDriverController().setRumble(  RumbleType.kRightRumble, 0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
    
}