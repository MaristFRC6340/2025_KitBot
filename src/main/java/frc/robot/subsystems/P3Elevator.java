package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;

public class P3Elevator extends SubsystemBase{
    
    private SparkMax elevatorMotor;
    private SparkMaxConfig motorConfig;
    private SparkClosedLoopController loopController;
    private RelativeEncoder encoder;
    private int targetPosition;
    private ProfiledPIDController pid;
    private ElevatorFeedforward elevatorFeedforward;


    public P3Elevator() {

        // init
        elevatorMotor = new SparkMax(RollerConstants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        loopController = elevatorMotor.getClosedLoopController();
        encoder = elevatorMotor.getEncoder();
        motorConfig = new SparkMaxConfig();

        // config doing config things
        motorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40);

        pid = new ProfiledPIDController(.5, 0, .6, new TrapezoidProfile.Constraints(1.5, .75));
        elevatorFeedforward = new ElevatorFeedforward(.18166, 0.20667, 0.0021352,0.0002943);
        pid.setGoal(0);
        
    }

    public void periodic() {
         SmartDashboard.putNumber("Elevator/encoderposition",encoder.getPosition());
    SmartDashboard.putNumber("Elevator/busvoltage",RobotController.getBatteryVoltage());

    SmartDashboard.putNumber("Elevator/voltage",elevatorMotor.get());
    SmartDashboard.putNumber("Elevator/appliedoutput",elevatorMotor.getAppliedOutput());
    
    //SmartDashboard.putNumber("Elevator/PID/vel");

    }

    public Command goToPosition(double position) {
        pid.setGoal(position);
        pid.reset(encoder.getPosition());
    
        return this.runEnd(() -> {

            
    SmartDashboard.putNumber("Elevator/PID/goal",pid.getGoal().position);
    SmartDashboard.putNumber("Elevator/PID/goalvel",pid.getGoal().velocity);

    SmartDashboard.putNumber("Elevator/PID/curSetPoint",pid.getSetpoint().position);
    SmartDashboard.putNumber("Elevator/PID/feedforward",elevatorFeedforward.calculate(pid.getSetpoint().velocity));

    SmartDashboard.putNumber("Elevator/PID/profiledpid",pid.calculate(encoder.getPosition(), position));
    SmartDashboard.putString("Elevator/PID/PIDState","Active");

            elevatorMotor.set(
                pid.calculate(encoder.getPosition(),position)
                // calculates next output of PID based on position of encoder + parameter,
                // adds it to the feedforward based on pid's vel
                );
        },()->{
            elevatorMotor.set(0);
            SmartDashboard.putString("Elevator/PID/PIDState","Active");

        });
    }
    public Command deltaPositionCommand(int deltaPos) {
    
        return this.runOnce( () -> {
          targetPosition += deltaPos;
          // Limits - Software stop
          if (targetPosition < 0) { // Bottom of the Elevator
            targetPosition = 0;
          }
          if (targetPosition > 200) { // Top of Elevator
            targetPosition = 200;
          }
          this.setPosition(targetPosition);
        });
        
    
      }
      public void setPosition(double position) {
    loopController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }




}
