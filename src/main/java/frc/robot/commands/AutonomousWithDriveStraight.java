package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class AutonomousWithDriveStraight extends SequentialCommandGroup {
    public AutonomousWithDriveStraight(Intake theIntake, Shooter shooter,Conveyor theConveyor, DriveTrain theDriveTrain){
//         Encoder leftEncoder, Encoder rightEncoder, Gyro rotationSource){

//did work runtime error        addCommands(new ShooterMoveLow(RobotContainer.getInstance().m_shooter));
//addCommands(new ShooterMoveLow(RobotContainer.getInstance().m_shooter));
/**TEMP 
 * CommandGroupBase spinAndShootAndintake = SequentialCommandGroup.parallel(
                                              new ShooterMoveLow(shooter),
                                              new IntakeMove(theIntake),
                                              new ConveyorMove(theConveyor)  ).withTimeout(5);
TEMP**/                                              

        //addCommands(new DriveTrainMoveStraight(theDriveTrain, leftEncoder, rightEncoder, rotationSource, distance, maxspeed, ramp, targetAngle));                                     
        addCommands(new DriveTrainMoveStraight(theDriveTrain, 200, 10, 12, 0));                                     
       
    }

   
}
