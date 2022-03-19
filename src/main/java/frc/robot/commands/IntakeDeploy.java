package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;

/**
 *
 */
public class IntakeDeploy extends SequentialCommandGroup {
        public IntakeDeploy(Intake theIntake, Conveyor theConveyor){
CommandGroupBase spinIntakeAndConvayor = SequentialCommandGroup.parallel(
                                            new IntakeMove(theIntake),
                                            new ConveyorMove(theConveyor));

       addCommands(spinIntakeAndConvayor);
    }

}
