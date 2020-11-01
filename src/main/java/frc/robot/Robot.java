/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonImuJNI;
import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import frc.robot.commands.turnDegree;
import frc.robot.libs.LIB_Enc;
import frc.robot.subsystems.*;

public class Robot extends TimedRobot {

  //Drive Setup
  ;

  public static Movement movement;
//Variable Setup
  public boolean toggle;
  public boolean detectToggle;
  public boolean shooterToggle;
  public boolean mechToggle;
  public boolean eleToggle;

  //Motors/Servos
  
  public WPI_TalonSRX imuMotor =  new WPI_TalonSRX(8);
  public CANSparkMax belt = new CANSparkMax(5,MotorType.kBrushless); 
  public CANSparkMax sucker = new CANSparkMax(6,MotorType.kBrushless);
  public CANSparkMax leftElevator = new CANSparkMax(9,MotorType.kBrushless);
  public CANSparkMax rightElevator = new CANSparkMax(10,MotorType.kBrushless);
  //5 Vertical
  //6 Horizonal
  //7 Intake
  //8 & 9 Victors & Talon
  public WPI_TalonFX shooter1 = new WPI_TalonFX(13);
  public WPI_TalonFX shooter2 = new WPI_TalonFX(14);
  public WPI_TalonFX feeder = new WPI_TalonFX(15);


  //Encoders
  /*CANEncoder leftMasterEnc = new CANEncoder(leftMaster);
  CANEncoder leftSlaveEnc = new CANEncoder(leftSlave);
  CANEncoder rightMasterEnc = new CANEncoder(rightMaster);
  CANEncoder rightSlaveEnc = new CANEncoder(rightSlave);
  -----------------Built-In-To-Library-----------------
  */
  //Command Init
  public Command Auto;
  //Pneumatics
  public Compressor c = new Compressor(11);
  public DoubleSolenoid solenoid1 = new DoubleSolenoid(11, 1, 2);
  public DoubleSolenoid solenoid2 = new DoubleSolenoid(11, 3, 4);
  public DoubleSolenoid solenoid3 = new DoubleSolenoid(11, 5, 6);
  public DoubleSolenoid solenoid4 = new DoubleSolenoid(11, 7, 0);
  //Controller
  public XboxController m_stick = new XboxController(0);

  //Gyro/AHRS
  public PigeonIMU imu = new PigeonIMU(imuMotor);


  //LimeLight 2.0
  public NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  public NetworkTableEntry cameraSelection = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");
  public NetworkTableEntry tx = table.getEntry("tx");
  public NetworkTableEntry ty = table.getEntry("ty");
  public NetworkTableEntry ta = table.getEntry("ta");
  public NetworkTableEntry tv = table.getEntry("tv");

  //Other Libraries
  public LIB_Enc enc = new LIB_Enc();
  
  //Other Functions
  public static Movement move = new Movement();

  //Camera
  public UsbCamera camera1 = CameraServer.getInstance().startAutomaticCapture(0);

  //Lights
  public PWM blinkin;

  //Timer
  public Timer shoot = new Timer();
  public Timer angle = new Timer();

  //Doubles
  public double baseAngleTarget = 0;
  public double ballDistance = 0; 
  
  //Color Sensor
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
  
  public Robot() {

      camera1.setResolution(1280, 720);
      blinkin = new PWM(4);
      Auto = new turnDegree(0.4, -270);
      
  }


  @Override
  public void robotInit() {

    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);

    //Motor Configuration
    //customTest.configFactoryDefault();
    //customTest.setInverted(false);
    //customTest.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    
  };

  @Override
  public void robotPeriodic() {
    
    Color detectedColor = m_colorSensor.getColor();

    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      SmartDashboard.putString("Color From ColorSensor", "Blue");
    } else if (match.color == kRedTarget) {
      SmartDashboard.putString("Color From ColorSensor", "Red");
    } else if (match.color == kGreenTarget) {
      SmartDashboard.putString("Color From ColorSensor", "Green");
    } else if (match.color == kYellowTarget) {
      SmartDashboard.putString("Color From ColorSensor", "Yellow");
    } else {
      SmartDashboard.putString("Color From ColorSensor", "Unknown");
    }

  }

  @Override
  public void teleopInit() {
    SmartDashboard.putNumber("EncoderCheck", enc.adjustNEO(enc.getPos(move.rightMaster)));
    toggle = false;
    detectToggle = false;
    shooterToggle = false;
    mechToggle = false;
    eleToggle = false;
    feeder.set(0);
    shoot.start();
    imu.setAccumZAngle(0);
    enc.reset(move.leftMaster);
    enc.reset(move.rightMaster);
    leftElevator.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    leftElevator.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    rightElevator.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    rightElevator.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    leftElevator.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -90);
    leftElevator.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 0);
    rightElevator.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 90);
    rightElevator.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);

  };

  @Override
  public void teleopPeriodic() {
    leftElevator.set(-m_stick.getRawAxis(3) + m_stick.getRawAxis(2));
    rightElevator.set(m_stick.getRawAxis(3)-m_stick.getRawAxis(2)); 
    SmartDashboard.putNumber("NewElevator", enc.getPos(rightElevator));
    shooter1.set(-1);
    shooter2.set(1);
    
    
    double[] accumGyro = new double[3];
    imu.getAccumGyro(accumGyro);
    SmartDashboard.putNumber("Pigeon", accumGyro[2]);
    SmartDashboard.putNumber("leftMaster Encoder", enc.adjustNEO(enc.getPos(move.leftMaster)));
    SmartDashboard.putNumber("rightMaster Encoder", enc.adjustNEO(enc.getPos(move.rightMaster)));
    feeder.set(0);
    
    if (eleToggle) {
      
      solenoid1.set(Value.kForward);

    }

    else {
      
      solenoid1.set(Value.kReverse);
      solenoid4.set(Value.kForward);
    
    }

    if (toggle) {
      
      belt.set(1);
      sucker.set(-0.6);

    }

    else {
      
      belt.set(0);
      sucker.set(0);
    
    }

    if (mechToggle) {

      if (Math.abs(m_stick.getRawAxis(1)) > 0.2 || Math.abs(m_stick.getRawAxis(4)) > 0.2 || Math.abs(m_stick.getRawAxis(0)) > 0.2) {
        move.set(-m_stick.getRawAxis(0),m_stick.getRawAxis(1),-m_stick.getRawAxis(4));
        solenoid1.set(Value.kReverse);
      }

      else {
        move.set(0,0,0);
        solenoid1.set(Value.kReverse);
      }

    }
    else {

      if (m_stick.getRawAxis(4) > 0.2 || m_stick.getRawAxis(4) < -0.2) {

        move.set(-m_stick.getRawAxis(4),m_stick.getRawAxis(1),-m_stick.getRawAxis(0));
        solenoid1.set(Value.kReverse);

      } 

      else if (Math.abs(m_stick.getRawAxis(1)) > 0.2 || Math.abs(m_stick.getRawAxis(4)) > 0.2 || Math.abs(m_stick.getRawAxis(0)) > 0.2) {

        move.set(0,m_stick.getRawAxis(1),-m_stick.getRawAxis(0));
        solenoid1.set(Value.kForward);
    
      } 

      else {
        move.set(0,0,0);
        solenoid1.set(Value.kForward);
      }

    }
    while (detectToggle && isEnabled()) {
      
      if (tv.getDouble(1) == 1) {
        double forward;
        double ortargetx = tx.getDouble(1);
        double ortargety = ty.getDouble(1);
        if (ortargety > 1){
          forward = 0.27;
          shoot.reset();
        }
        else if (ortargety < -1){
          forward = -0.27;
          shoot.reset();
        }
        else if (ortargety < 1 && ortargety > -1) {
          forward = 0;

        }
        else {
          forward = 0; 
          shoot.reset();
        }
        double targety = (ty.getDouble(1)/22) * 0.1;
        if (ortargetx < 1.5 && ortargetx > -1.5) {
          move.set(0,forward,0);  
          if (shoot.get() > 1) {
            feeder.set(1);
            blinkin.setSpeed(0.63);
          } 
          else {
            feeder.set(0);
            blinkin.setSpeed(0.77);
          } 
       }
        else if (ortargetx > 1.5) {
          move.set(0,0,-0.3);
          shoot.reset();
        }
        else if (ortargetx < -1.5) {
          move.set(0,0,0.3);
          shoot.reset();
        }
        
        //System.out.println(ortargetx);
      }

      if(m_stick.getAButtonReleased()) {
        detectToggle = !detectToggle;
      };
    }
    if (m_stick.getStartButton() == true) {
      feeder.set(1);
    }
    else {
      feeder.set(0);
    }

    if(m_stick.getBButtonReleased()) {
      mechToggle = !mechToggle;
      
    };
    if(m_stick.getXButtonReleased()) {
      toggle = !toggle;
      
    };
    if(m_stick.getBumperReleased(Hand.kRight)) {
      eleToggle = !eleToggle;
      
    };

    if(m_stick.getAButtonReleased()) {
      if (detectToggle == false) {
        if (tv.getDouble(1) == 1) {
          detectToggle = !detectToggle;
        }
      }
      
    };

    if (tv.getDouble(1) == 1) {
      blinkin.setSpeed(0.77);
    }
    else {
      blinkin.setSpeed(0.95);
    }

  }

  @Override
  public void autonomousInit() {
    solenoid2.set(Value.kForward);
    belt.set(1);
    shooter1.set(-1);
    shooter2.set(1);

    findTarget();
    shooter1.set(0);
    shooter2.set(0);
    belt.set(0);
    //turnDeg(calcAngleToBall(baseAngleTarget,17.5)-baseAngleTarget, 0.4);
    //driveInch(40);
    
    SmartDashboard.putNumber("BaseAngle", baseAngleTarget);
    SmartDashboard.putNumber("newAngle", calcAngleToBall(baseAngleTarget,15.5));
  }

  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putNumber("EncoderCheck", enc.getPos(move.leftMaster));
  }
  @Override
  public void disabledPeriodic() {
    
  }
  public void turnDeg(double deg, double power) {
    if (isAutonomous()) {
      angle.reset();
      power = Math.abs(power);
      imu.setAccumZAngle(0);
      double[] accumGyro = new double[3];
      imu.getAccumGyro(accumGyro);
      double initDeg = accumGyro[2];
      double constDeg = accumGyro[2];
      while ((constDeg > (deg + 0.3) || constDeg < (deg - 0.3)) && angle.get() < 2 && isEnabled() && isAutonomous()) {
        if (constDeg < (deg + 10) && constDeg > (deg - 10)) {
          if (deg - constDeg > 0) {
            move.set(0,0,0.3);
          }
          else {
            move.set(0,0,-0.3);
          }
        }
        else {
          if (deg - constDeg > 0) {
            move.set(0,0,power);
          }
          else {
            move.set(0,0,-power);
            
          }
        }
        accumGyro = new double[3];
        imu.getAccumGyro(accumGyro);
        constDeg = accumGyro[2];
        SmartDashboard.putNumber("Pigeon", accumGyro[2]);
      }
      accumGyro = new double[3];
        imu.getAccumGyro(accumGyro);
        constDeg = accumGyro[2];
        SmartDashboard.putNumber("Pigeon", accumGyro[2]);
        move.set(0,0,0);
    }
  }
  public void findTarget() {
    if (isAutonomous()) {
      Timer finderTimer = new Timer();
      Boolean finderToggle = true;
      double[] accumGyro;
      imu.setAccumZAngle(0);
      shoot.start();
      finderTimer.start();
      while (isEnabled() && finderToggle && isAutonomous()) {
        accumGyro = new double[3];
        imu.getAccumGyro(accumGyro);
        baseAngleTarget = accumGyro[2];
        if (tv.getDouble(1) == 1) {
          double forward;
          double ortargetx = tx.getDouble(1);
          double ortargety = ty.getDouble(1);
          if (ortargety > 1){
            forward = 0.27;
            shoot.reset();
          }
          else if (ortargety < -1){
            forward = -0.27;
            shoot.reset();
          }
          else if (ortargety < 1 && ortargety > -1) {
            forward = 0;
          }
          else {
            forward = 0; 
            shoot.reset();
          }
          double targety = (ty.getDouble(1)/22) * 0.1;
          if (ortargetx < 2 && ortargetx > -2) {
            move.set(0,forward,0);  
            if (shoot.get() > 1) {
              feeder.set(1);
              blinkin.setSpeed(0.63);
              finderTimer.reset();
              while (finderTimer.get() < 7) {}
              feeder.set(0);
              finderToggle = false;
              blinkin.setSpeed(0.99);
            } 
            else {
              feeder.set(0);
              blinkin.setSpeed(0.77);
            } 
        }
          else if (ortargetx > 2) {
            move.set(0,0,-0.3);
            shoot.reset();
          }
          else if (ortargetx < -2) {
            move.set(0,0,0.3);
            shoot.reset();
          }
        }
      }
    }
  }
  public void driveInch(double distance) {
      Timer testStop = new Timer();
      boolean loop = true;
      double kP = 0.2;
      double encPosition;
      double error;
      double outputSpeed;
      imu.setAccumZAngle(0);
      enc.reset(move.rightMaster);
      double[] accumGyro;
      double errorDeg;
      double constDeg;
      double turnOutput;
      testStop.start();
      while (loop && isEnabled() && isAutonomous()) {
        encPosition = enc.adjustNEO(enc.getPos(move.rightMaster));
        error = distance - encPosition;
        outputSpeed = kP * error;
        outputSpeed = Math.min(outputSpeed, 0.4);
        outputSpeed = Math.max(outputSpeed, -0.4);
        accumGyro = new double[3];
        imu.getAccumGyro(accumGyro);
        constDeg = accumGyro[2];
        turnOutput = -constDeg;
        turnOutput = Math.min(turnOutput, 0.05);
        turnOutput = Math.max(turnOutput, -0.05);
        move.set(0,-outputSpeed,turnOutput);
        SmartDashboard.putNumber("error", outputSpeed);
        if (outputSpeed < .2 && outputSpeed > -.2) {
          if (testStop.get() > 1) {
            loop = false;
          }
        }
        else {
          testStop.reset();
        }
      }
    move.set(0,0,0);
  }
  public final double calcAngleToBall(double needAngle, double halfLength /*15.5 length*/ /*13.5in width*/ ) {
    double trueAngle; 
    double xFromBall =  66.91;
    double yFromBall =  100 - halfLength;
    double newAngle = 90 - Math.abs(needAngle);
    double distanceFromTarget = 120 + halfLength; 
    double xDistanceToMid = (distanceFromTarget / Math.tan(Math.toRadians(newAngle))) ;
    if (baseAngleTarget < 0) {
      xDistanceToMid = xDistanceToMid * 1 ;
    }
    else {
      xDistanceToMid = xDistanceToMid * -1;
    }
    double xDistanceToBall = Math.abs(xFromBall + xDistanceToMid);
    SmartDashboard.putNumber("TAN", Math.tan(Math.toRadians(newAngle)));
    SmartDashboard.putNumber("TRI_ANGLE", newAngle);
    double yDistanceToBall = yFromBall;
    trueAngle = Math.toDegrees(Math.atan(yDistanceToBall/xDistanceToBall)) + 90;
    trueAngle = -trueAngle;
    return trueAngle;
  }
}

