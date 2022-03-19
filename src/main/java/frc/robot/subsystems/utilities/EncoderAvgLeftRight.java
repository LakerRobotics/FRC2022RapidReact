package frc.robot.subsystems.utilities;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;


public class EncoderAvgLeftRight implements DoubleSupplier{
    RelativeEncoder m_leftEncoder;
    RelativeEncoder m_rightEncoder;

    public EncoderAvgLeftRight(RelativeEncoder theLeftEncoder, RelativeEncoder theRightEncoder){
       // super(0, 1);// junk parameters should never be used unless underthe covers the Encoder does something which it problably does
        m_leftEncoder = theLeftEncoder;
        m_rightEncoder = theLeftEncoder;

    }
    /**@overide
    */
    public void reset(){
        m_leftEncoder.setPosition(0);
        m_rightEncoder.setPosition(0);

    }
public double getRate(){
    return (m_leftEncoder.getVelocity()+m_rightEncoder.getVelocity())/2;
}
    /**
     * @overide
     */
    public double getDistance(){
        return (m_leftEncoder.getPosition()+m_rightEncoder.getPosition())/2;
    }
    @Override
    public double getAsDouble() {
        return getDistance();
    }
}
