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

import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;


/**
 *
 */
public class Autonomous3BallRightFaster  extends SequentialCommandGroup {

    public Autonomous3BallRightFaster(Intake theIntake, Shooter shooter,Conveyor theConveyor, DriveTrain theDriveTrain){


        //TODO create a startShooterLow command (so it will keep running, all during auton) and 
        //TODO create a StartIntake command
        addCommands(new GyroReset(theDriveTrain));
        // Turn on Intake
        // go forward 50"
        // pick up the ball
         ParallelRaceGroup driveForwardWithIntakeShooter = new ParallelRaceGroup(        
            new DriveTrainMoveStraightFaster(theDriveTrain, -70 /*Distance*/, 5 /*maxSpeed ft/sec*/, 10 /*inch to get to maxSpeed*/, 0 /*Angle to drive straight on*/, 0.5/*inchs Accuracy*/),
            new IntakeMove(theIntake),
            new ShooterMoveLow(shooter)
        );  
        addCommands(driveForwardWithIntakeShooter);

        // turn around to face hub  
        //addCommands(new DriveTrainTurnSpinToAngle(theDriveTrain, 180/*TurnToAngle*/));

        ParallelRaceGroup turnWithFlywheel = new ParallelRaceGroup(
            new DriveTrainTurnSpinToAngleFaster(theDriveTrain, 180, 1 /*Degrees Accuracy*/),
            new ShooterMoveLow(shooter)
        );
        addCommands(turnWithFlywheel);

        // go forward 50"
        // turn on shooter 
        // wait for shooter to charge up then move ball up on the conveytor
        ParallelRaceGroup driveForwardWithShooter70 = new ParallelRaceGroup(        
            new ShooterMoveLow(shooter),
            new IntakeMove(theIntake),
            new DriveTrainMoveStraightFaster(theDriveTrain, -70 /*Distance*/, 5 /*maxSpeed ft/sec*/, 10 /*inch to get to maxSpeed*/, 180 /*Angle to drive straight on*/, 0.5/*inchs Accuracy*/)
        );
        addCommands(driveForwardWithShooter70);

        //shoot the ball 
        CommandGroupBase spinAndShootAndintake = SequentialCommandGroup.parallel(
            new ShooterMoveLow(shooter),
            new IntakeMove(theIntake),
            new ConveyorMove(theConveyor)
        ).withTimeout(2);
        addCommands(spinAndShootAndintake);

        //Turn clockwise (180 + 50) degrees
        addCommands(new DriveTrainTurnSpinToAngleFaster(theDriveTrain, 250/*TurnToAngle*/, 1 /*Degree accuracy*/));

        //Drive 120 inches with intake
        ParallelRaceGroup driveForwardWithIntake120 = new ParallelRaceGroup(
            new DriveTrainMoveStraightFaster(theDriveTrain, -120 /*Distance*/, 5 /*maxSpeed ft/sec*/, 10 /*inch to get to maxSpeed*/, 180+250 /*Angle to drive straight on*/, 0.5/*inchs Accuracy*/),
            new IntakeMove(theIntake)
        );
        addCommands(driveForwardWithIntake120);
    
        //Turn around clockwise (230 + 150) degrees
        addCommands(new DriveTrainTurnSpinToAngleFaster(theDriveTrain, (180+250 + 150), 1 /* Degree accuracy*/));
    
        //Drive 50 inches with shooter for long shot
        ParallelRaceGroup driveForwardWithShooter50 = new ParallelRaceGroup(        
            new ShooterMoveLow(shooter),
            new IntakeMove(theIntake),
            new DriveTrainMoveStraightFaster(theDriveTrain, -70 /*Distance*/, 5 /*maxSpeed ft/sec*/, 10 /*inch to get to maxSpeed*/, (180 +250 + 150) /*Angle to drive straight on*/, 0.5/*inchs Accuracy*/)
        );
        addCommands(driveForwardWithShooter50);
    
        //Engage conveyor and keep shooter at consistent speed
        ParallelRaceGroup spinAndShootLong = new ParallelRaceGroup(
            new ShooterMoveMed(shooter),
            new ConveyorMove(theConveyor)
        );
        addCommands(spinAndShootLong);

    }

}
