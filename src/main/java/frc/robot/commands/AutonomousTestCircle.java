package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 *
 */
public class AutonomousTestCircle extends SequentialCommandGroup {
    public AutonomousTestCircle(Intake theIntake, Shooter shooter,Conveyor theConveyor, DriveTrain theDriveTrain){

        addCommands(new GyroReset(theDriveTrain));

//        addCommands(new DriveTrainTurnOnCircleToAngle(theDriveTrain,
//                                                         10    /* inches ramp Up*/,
//                                                         3,    /* ft/sec Max Speed*/
//                                                         10,   /* inches ramp Down */
//                                                         360,  /* Target angle, i.e. when it will end traveling on the circle*/
//                                                         36,   /* inch radius of the Circle*/
//                                                         true  /* forward (if false goes backward*/
//                                                    )
//                    );                                     
addCommands(new DriveTrainTurnSpinToAngle(theDriveTrain, 180));
addCommands(new DriveTrainTurnSpinToAngle(theDriveTrain, 360));
addCommands(new DriveTrainTurnSpinToAngle(theDriveTrain, 360+180));
addCommands(new DriveTrainTurnSpinToAngle(theDriveTrain, 360));
addCommands(new DriveTrainTurnSpinToAngle(theDriveTrain, 180));
addCommands(new DriveTrainTurnSpinToAngle(theDriveTrain,   0));

       
    }
   
}