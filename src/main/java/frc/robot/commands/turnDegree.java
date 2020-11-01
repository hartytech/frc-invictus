package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class turnDegree extends Command {
	private double goalAngle = 0.0;
	private boolean isDone = false;
	private double speed;
	private double tolerance = 5;
  private double currentAngle;
  private double[] accumGyro = new double[3];
	
    public turnDegree(double speed, double givenAngle) {
    	requires(Robot.move);
    	goalAngle = givenAngle;
    	this.speed = speed;
    	isDone = false;
    }

    protected void initialize() {
      Robot.move.imu.setAccumZAngle(0);
      isDone = false;
      Robot.move.imu.getAccumGyro(accumGyro);
      goalAngle = goalAngle + accumGyro[2];

    }

    protected void execute() {
      Robot.move.imu.getAccumGyro(accumGyro);
      if ((accumGyro[2] < goalAngle + 2) && (accumGyro[2] > goalAngle - 2)) {
        if (goalAngle > accumGyro[2]) {
          Robot.move.set(0,0,-speed);
        }
        else if (goalAngle < accumGyro[2]) {
          Robot.move.set(0,0,speed);
        }
        else {
          isDone = true;
          System.out.print("Error");
        }
      }
      else {
        System.out.print("Done");
        isDone = true;
      }
    }

    protected boolean isFinished() {
    	return isDone;
    }

    protected void end() {
    }

 
    protected void interrupted() {
    	Robot.move.set(0,0, 0);
    	isDone = true;
    }
}