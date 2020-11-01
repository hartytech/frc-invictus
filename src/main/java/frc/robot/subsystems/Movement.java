package frc.robot.subsystems;

import javax.xml.namespace.QName;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

public class Movement extends Subsystem {
    
    public CANSparkMax leftMaster; 
    public CANSparkMax leftSlave; 
    public CANSparkMax rightMaster; 
    public CANSparkMax rightSlave;
    public WPI_TalonSRX shooter = new WPI_TalonSRX(6);;


    public MecanumDrive myRobot;
    public PigeonIMU imu = new PigeonIMU(shooter);
    public PIDController turnController;

    public final double kP = 0.1;
    public final double kI = 0;
    public final double kD = 0;

    public Movement() {
         
        leftMaster = new CANSparkMax(4,MotorType.kBrushless);
        leftSlave = new CANSparkMax(3,MotorType.kBrushless);
        rightMaster = new CANSparkMax(2,MotorType.kBrushless);
        rightSlave = new CANSparkMax(1,MotorType.kBrushless);
       


        myRobot = new MecanumDrive(leftMaster, leftSlave, rightMaster, rightSlave);
    


    }


    public void set(final double strafe, final double forward, final double turn) {
        myRobot.setSafetyEnabled(true);
        myRobot.driveCartesian(strafe*0.7, forward*0.7, turn*0.7);
    
    }

    public void deg(double angle, double power) {
        

    }

    @Override
    protected void initDefaultCommand() {
        // TODO Auto-generated method stub

    }

}