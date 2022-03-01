package frc.robot.subsystems.utilities;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.interfaces.Gyro;

public class GyroAngleAsDouble implements DoubleSupplier{
    
    Gyro m_gyro;

    public GyroAngleAsDouble(Gyro aGyro){
        m_gyro = aGyro;
    }
    

    @Override
    public double getAsDouble() {
        return m_gyro.getAngle();
    }
}
