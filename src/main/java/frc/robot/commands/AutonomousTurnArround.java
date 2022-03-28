package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 *
 */
public class AutonomousTurnArround extends SequentialCommandGroup {
    public AutonomousTurnArround(Intake theIntake, Shooter shooter,Conveyor theConveyor, DriveTrain theDriveTrain){

        addCommands(new GyroReset(theDriveTrain));

        addCommands(new DriveTrainTurnSpinToAngle(theDriveTrain, 180));                                     
       
    }
   
}
