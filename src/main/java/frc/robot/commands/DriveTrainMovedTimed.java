package frc.robot.commands;
//import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.DriveTrain;

public class DriveTrainMovedTimed extends ParallelRaceGroup {
    
        public DriveTrainMovedTimed(DriveTrain the_drivetrain) {
            super(new DriveTrainMove(the_drivetrain).withTimeout(10));
        }
        
}


