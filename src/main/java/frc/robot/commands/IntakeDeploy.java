package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class IntakeDeploy extends SequentialCommandGroup {
        public IntakeDeploy(Intake theIntake, Conveyor theConveyor){
//did work runtime error        addCommands(new ShooterMoveLow(RobotContainer.getInstance().m_shooter));
//addCommands(new ShooterMoveLow(RobotContainer.getInstance().m_shooter));
CommandGroupBase spinIntakeAndConvayor = SequentialCommandGroup.parallel(
                                            new IntakeMove(theIntake),
                                            new ConveyorMove(theConveyor));

       // addCommands(spinAndShoot);
 //       addCommands(new ShooterMoveLow(shooter));
       // addCommands(new ConveyorMove(theConveyor));
       addCommands(spinIntakeAndConvayor);
    }

}
