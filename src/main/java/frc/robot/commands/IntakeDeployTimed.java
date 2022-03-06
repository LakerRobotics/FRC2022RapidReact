package frc.robot.commands;
//import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;

public class IntakeDeployTimed extends ParallelRaceGroup {

    public IntakeDeployTimed(Intake theIntake, Conveyor theConveyor) {
        super(new IntakeDeploy(theIntake, theConveyor).withTimeout(2));
    }
}