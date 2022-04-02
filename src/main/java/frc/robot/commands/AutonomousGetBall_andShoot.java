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

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
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
public class AutonomousGetBall_andShoot extends SequentialCommandGroup {

    public AutonomousGetBall_andShoot(Intake theIntake, Shooter shooter,Conveyor theConveyor, DriveTrain theDriveTrain){

        //TODO create a startShooterLow command (so it will keep running, all during auton) and 
        //TODO create a StartIntake command
        addCommands(new GyroReset(theDriveTrain));
   // Turn on Intake
   // go forward 50"
   // pick up the ball
   ParallelRaceGroup driveForwardWithIntakeShooter = new ParallelRaceGroup(        
    new DriveTrainMoveStraight(theDriveTrain, -75 /*Distance*/, 5 /*maxSpeed ft/sec*/, 10 /*inch to get to maxSpeed*/, 0 /*Angle to drive straight on*/),
    new IntakeMove(theIntake),
    new ShooterMoveLow(shooter)
    );  
    addCommands(driveForwardWithIntakeShooter);

   // turn around to face hub
   //addCommands(new DriveTrainTurnSpinToAngle(theDriveTrain, 180/*TurnToAngle*/));

   ParallelRaceGroup turnWithFlywheel = new ParallelRaceGroup(
    new DriveTrainTurnSpinToAngle(theDriveTrain, 180)
    //, new ShooterMoveLow(shooter)
   );
   addCommands(turnWithFlywheel);

   // go forward 50"
   // turn on shooter 
   // wait for shooter to charge up then move ball up on the conveytor
   ParallelRaceGroup driveForwardWithShooter = new ParallelRaceGroup(        
    new ShooterMoveLow(shooter),
    new IntakeMove(theIntake),
    new DriveTrainMoveStraight(theDriveTrain, -75 /*Distance*/, 5 /*maxSpeed ft/sec*/, 10 /*inch to get to maxSpeed*/, 180 /*Angle to drive straight on*/)
    );
    addCommands(driveForwardWithShooter);

   //shoot the ball 
   CommandGroupBase spinAndShootAndintake = SequentialCommandGroup.parallel(
    new ShooterMoveLow(shooter),
    new IntakeMove(theIntake),
    new ConveyorMove(theConveyor)).withTimeout(5);
addCommands(spinAndShootAndintake);
    } 

}
