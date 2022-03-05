package frc.robot.subsystems.utilities;

import edu.wpi.first.wpilibj.Encoder;

public class EncoderAvgLeftRight extends Encoder{
    Encoder m_leftEncoder;
    Encoder m_rightEncoder;

    public EncoderAvgLeftRight(Encoder theLeftEncoder, Encoder theRightEncoder){
        super(99, 98);// junk parameters should never be used unless underthe covers the Encoder does something which it problably does
        m_leftEncoder = theLeftEncoder;
        m_rightEncoder = theLeftEncoder;

    }
    /**@overide
    */
    public void reset(){
        m_leftEncoder.reset();
        m_rightEncoder.reset();
    }

    /**
     * @overide
     */
    public double getDistance(){
        return (m_leftEncoder.getDistance()+m_rightEncoder.getDistance())/2;
    }
}
