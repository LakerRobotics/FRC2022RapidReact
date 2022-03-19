// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: RobotContainer.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.utilities.JoystickAxisAsButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.*;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private static RobotContainer m_robotContainer = new RobotContainer();

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
// The robot's subsystems
    public final Climber m_climber = new Climber();
    public final Conveyor m_conveyor = new Conveyor();
    public final Intake m_intake = new Intake();
    public final Shooter m_shooter = new Shooter();
    public final DriveTrain m_driveTrain = new DriveTrain();

// Joysticks
private final XboxController driverController = new XboxController(0);
private final XboxController operatorController = new XboxController(1);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

  
  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
  * The container for the robot.  Contains subsystems, OI devices, and commands.
  */
  private RobotContainer() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SMARTDASHBOARD
    // Smartdashboard Subsystems


    // SmartDashboard Buttons
    SmartDashboard.putData("AutonomousCommand", new AutonomousShootandBackupStraight(m_intake,m_shooter,m_conveyor,m_driveTrain));
    SmartDashboard.putData("DriveTrainArcade", new DriveTrainArcade( m_driveTrain ));
    SmartDashboard.putData("ConveyorControlPower", new ConveyorControlPower( m_conveyor ));
    SmartDashboard.putData("ShooterControlPower", new ShooterControlPower( m_shooter ));
    SmartDashboard.putData("ShooterMove", new ShooterMove( m_shooter ));
    SmartDashboard.putData("IntakeMove", new IntakeMove( m_intake ));
    SmartDashboard.putData("ConveyorMove", new ConveyorMove( m_conveyor ));
    SmartDashboard.putData("ClimberMove", new ClimberMove( m_climber ));
    SmartDashboard.putData("IntakeStop", new IntakeStop( m_intake ));
    SmartDashboard.putData("IntakeControlPower", new IntakeControlPower( m_intake ));
    SmartDashboard.putData("IntakeControlSpeed", new IntakeControlSpeed( m_intake ));
    //SmartDashboard.putData("DriveTrainMoveStraight", new DriveTrainMoveStraight(m_driveTrain, 100, 3, 10, 10));

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SMARTDASHBOARD
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SUBSYSTEM_DEFAULT_COMMAND
//    m_climber.setDefaultCommand(new ClimberMove( m_climber ) );
    m_intake.setDefaultCommand(new IntakeControlPower( m_intake ) );
    m_conveyor.setDefaultCommand(new ConveyorControlPower( m_conveyor ) );
    m_driveTrain.setDefaultCommand(new DriveTrainArcade( m_driveTrain ) );


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SUBSYSTEM_DEFAULT_COMMAND

    // Configure autonomous sendable chooser
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS

    //m_chooser.setDefaultOption("$command.getName()", new ${name.replace(' ', '')}( m_${name.substring(0,1).toLowerCase()}${name.substring(1).replace(' ', '')} ));
m_chooser.setDefaultOption("AutnomouseShootAndBackup",new AutonomousShootandBackup(m_intake,m_shooter,m_conveyor, m_driveTrain));
m_chooser.addOption(       "AutnomouseShootAndBackup",new AutonomousShootandBackup(m_intake,m_shooter,m_conveyor, m_driveTrain));
m_chooser.addOption(       "AutnomouseShootAndBackupStraight",new AutonomousShootandBackupStraight(m_intake,m_shooter,m_conveyor, m_driveTrain));

m_chooser.addOption("AutnomouseStraight",new AutonomousWithDriveStraight(m_intake, m_shooter, m_conveyor, m_driveTrain));
m_chooser.addOption("AutnomouseTurnArround",new AutonomousTurnArround(m_intake, m_shooter, m_conveyor, m_driveTrain));
m_chooser.addOption("Autonomous2Ball_AlignToSecondBall",new Autonomous2Ball_AlignToSecondBall(m_intake, m_shooter, m_conveyor, m_driveTrain));
//m_chooser.addOption("AutonomousTurnArroundGetBallAndShoot",new AutonomousTurnArroundGetBallAndShoot(m_intake, m_shooter, m_conveyor, m_driveTrain));

                                                                             // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS
  
    SmartDashboard.putData("Auto Mode", m_chooser);
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=BUTTONS
// Create some buttons

// INTAKE 
final JoystickButton intakeMovePower = new JoystickButton(operatorController, XboxController.Button.kRightStick.value);        
intakeMovePower.whileHeld(new IntakeMove( m_intake ) ,true);
    SmartDashboard.putData("IntakeMovePower",new IntakeMove( m_intake ) );

// CONVEYOR
//   Operator Controller
final JoystickButton conveyorMovePower = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);        
                     conveyorMovePower.whileHeld(new ConveyorMove( m_conveyor ) ,true);
final JoystickButton conveyorMovePowerBackwards = new JoystickButton(operatorController, XboxController.Button.kRightBumper.value);
                     conveyorMovePowerBackwards.whileHeld(new ConveyorMoveBackwards( m_conveyor ) ,true);
//   Driver Controller

final JoystickButton conveyorMovePowerDriver = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);        
                     conveyorMovePowerDriver.whileHeld(new ConveyorMove( m_conveyor ) ,true);

final JoystickButton conveyorMovePowerDriverBackwards = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
                     conveyorMovePowerDriverBackwards.whileHeld(new ConveyorMoveBackwards( m_conveyor ) ,true);
    SmartDashboard.putData("ConveyorMovePower",new ConveyorMove( m_conveyor ) );

// SHOOTER Curb Shoot
//   Operator Controlller
final JoystickButton shooterMovePowerShort = new JoystickButton(operatorController, XboxController.Button.kA.value);        
                     shooterMovePowerShort.whileHeld(new ShooterMoveLow( m_shooter ) ,true);
//   Driver Controller
//        same as Operator
final JoystickButton shooterMovePowerShortDriver = new JoystickButton(driverController, XboxController.Button.kA.value);        
                     shooterMovePowerShortDriver.whileHeld(new ShooterMoveLow( m_shooter ) ,true);
//        access without interupting driving, with free trigger Finger
//GrantRequest final JoystickButton shooterMovePowerShortDriverEasyAccess = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);        
//GrantRequest                     shooterMovePowerShortDriverEasyAccess.whileHeld(new ShooterMoveLow( m_shooter ) ,true);
//  Smartdashboard
    SmartDashboard.putData("ShooterMovePowerShort",new ShooterMove( m_shooter ) );

// SHOOTER fieldPermiter Shoot
//   Operator Controlller
final JoystickButton shooterMovePowerLong = new JoystickButton(operatorController, XboxController.Button.kB.value);        
                     shooterMovePowerLong.whileHeld(new ShooterMoveHigh( m_shooter ) ,true);
//   Driver Controller
//        same as Operator
final JoystickButton shooterMovePowerLongDriver = new JoystickButton(driverController, XboxController.Button.kB.value);        
                     shooterMovePowerLongDriver.whileHeld(new ShooterMoveHigh( m_shooter ) ,true);
                     
final JoystickButton driveTrainForward = new JoystickButton(driverController, XboxController.Button.kY.value);        
                     driveTrainForward.whileHeld(new DriveTrainMoveForward( m_driveTrain ) ,true);
                     
//        access without interupting driving, with free trigger Finger
//GrantRequest final JoystickAxisAsButton shooterMovePowerLongDriverEasyAccess = new JoystickAxisAsButton(driverController, XboxController.Button.kRightStick.value);        
//GrantRequest                           shooterMovePowerLongDriverEasyAccess.whileHeld(new ShooterMoveHigh( m_shooter ) ,true);
SmartDashboard.putData("shooterMovePowerLongDriver",new ShooterMove( m_shooter ) );




    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=BUTTONS
  }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
public XboxController getDriverController() {
      return driverController;
    }

public XboxController getOperatorController() {
      return operatorController;
    }


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
  */
  public Command getAutonomousCommand() {
    // The selected command will be run in autonomous
    return m_chooser.getSelected();
  }
  

}

