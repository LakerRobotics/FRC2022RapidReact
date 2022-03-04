package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.Shooter;

public class ShooterMoveLowTimed extends ParallelRaceGroup {

    public ShooterMoveLowTimed(Shooter subsystem) {
        super(new ShooterMoveLow(subsystem).withTimeout(3));

    }
}
