/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.libs;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

public final class LIB_Enc {
  public void reset(WPI_TalonSRX motor1) {
    motor1.setSelectedSensorPosition(0);
  } 
  public void reset(WPI_TalonSRX motor1, WPI_TalonSRX motor2) {
    motor1.setSelectedSensorPosition(0);
    motor2.setSelectedSensorPosition(0);
  } 
  public void reset(WPI_TalonSRX motor1, WPI_TalonSRX motor2, WPI_TalonSRX motor3) {
    motor1.setSelectedSensorPosition(0);
    motor2.setSelectedSensorPosition(0);
    motor3.setSelectedSensorPosition(0);
  } 
  public void reset(WPI_TalonSRX motor1, WPI_TalonSRX motor2, WPI_TalonSRX motor3, WPI_TalonSRX motor4) {
    motor1.setSelectedSensorPosition(0);
    motor2.setSelectedSensorPosition(0);
    motor3.setSelectedSensorPosition(0);
    motor4.setSelectedSensorPosition(0);
  } 
  public void reset(WPI_TalonFX motor1) {
    motor1.setSelectedSensorPosition(0);
  } 
  public void reset(WPI_TalonFX motor1, WPI_TalonFX motor2) {
    motor1.setSelectedSensorPosition(0);
    motor2.setSelectedSensorPosition(0);
  } 
  public void reset(WPI_TalonFX motor1, WPI_TalonFX motor2, WPI_TalonFX motor3) {
    motor1.setSelectedSensorPosition(0);
    motor2.setSelectedSensorPosition(0);
    motor3.setSelectedSensorPosition(0);
  } 
  public void reset(WPI_TalonFX motor1, WPI_TalonFX motor2, WPI_TalonFX motor3, WPI_TalonFX motor4) {
    motor1.setSelectedSensorPosition(0);
    motor2.setSelectedSensorPosition(0);
    motor3.setSelectedSensorPosition(0);
    motor4.setSelectedSensorPosition(0);
  }
  public void reset(CANSparkMax motor1) {
    CANEncoder motor1enc = new CANEncoder(motor1);
    motor1enc.setPosition(0);
  } 
  public void reset(CANSparkMax motor1, CANSparkMax motor2) {
    CANEncoder motor1enc = new CANEncoder(motor1);
    CANEncoder motor2enc = new CANEncoder(motor2);
    motor1enc.setPosition(0);
    motor2enc.setPosition(0);
  } 
  public void reset(CANSparkMax motor1, CANSparkMax motor2, CANSparkMax motor3) {
    CANEncoder motor1enc = new CANEncoder(motor1);
    CANEncoder motor2enc = new CANEncoder(motor2);
    CANEncoder motor3enc = new CANEncoder(motor3);
    motor1enc.setPosition(0);
    motor2enc.setPosition(0);
    motor3enc.setPosition(0);
  } 
  public void reset(CANSparkMax motor1, CANSparkMax motor2, CANSparkMax motor3, CANSparkMax motor4) {
    CANEncoder motor1enc = new CANEncoder(motor1);
    CANEncoder motor2enc = new CANEncoder(motor2);
    CANEncoder motor3enc = new CANEncoder(motor3);
    CANEncoder motor4enc = new CANEncoder(motor4);
    motor1enc.setPosition(0);
    motor2enc.setPosition(0);
    motor3enc.setPosition(0);
    motor4enc.setPosition(0);
  }
  public double getPos(WPI_TalonSRX motor1) {
    return motor1.getSelectedSensorPosition(0);
  }
  public double getPos(WPI_TalonFX motor1) {
    return motor1.getSelectedSensorPosition(0);
  }
  public double getPos(CANSparkMax motor1) {
    CANEncoder motor1enc = new CANEncoder(motor1);
    return motor1enc.getPosition();
  }
  public double adjustSRX(double x) {
    double eqn = ((x/(420))*(3.14*4));
    return eqn;
  }
  public double adjustFX(double x) {
    double eqn = ((x/2048));
    return eqn;
  }
  public double adjustNEO(double x) {
    double eqn = (x/6.2) * ((3.14*6));
    return eqn;
  }
}
