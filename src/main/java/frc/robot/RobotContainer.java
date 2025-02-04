// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RollerConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.CANRollerSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.P3Elevator;
import frc.robot.subsystems.SimulationSubsystem;
import frc.robot.subsystems.WristSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import java.util.List;
import java.util.function.DoubleSupplier;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems

  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final CANRollerSubsystem rollerSubsystem = new CANRollerSubsystem();
  private final WristSubsystem wristSubsystem = new WristSubsystem();
  private final Elevator elevatorSubsystem = new Elevator();
  //private final SimulationSubsystem sim = new SimulationSubsystem(()->elevatorSubsystem.getPosition(), ()->wristSubsystem.getPosition());
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  // Create Triggers for Bindings
  Trigger driverLTrigger = new Trigger(() -> m_driverController.getLeftTriggerAxis()>OIConstants.kDriverLTriggerDeadband);
  Trigger driverRTrigger = new Trigger(() -> m_driverController.getRightTriggerAxis()>OIConstants.kDriverRTriggerDeadband);

  Trigger driverA = new JoystickButton(m_driverController, XboxController.Button.kA.value);
  Trigger driverY = new JoystickButton(m_driverController, XboxController.Button.kY.value);

  Trigger driverLButton = new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);
  Trigger driverRButton = new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value);

  Trigger driverDpadUp = new Trigger(() -> m_driverController.getPOV()==0);
  Trigger driverDpadDown = new Trigger(() -> m_driverController.getPOV() == 180);


  // Other Fields
  // Speed Control
  private double SPEED_CONTROL = 0.25; // Super Slow Mode : 0.25

 //op

  

  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {




    // Configure the button bindings
    configureButtonBindings();

    
    // Configure default commands
    // Added Speed Control for testing. Local Field in this class
    m_robotDrive.setDefaultCommand(
        
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY()*SPEED_CONTROL, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX()*SPEED_CONTROL, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX()*SPEED_CONTROL, OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  
  private void configureButtonBindings() {
    /* 
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    */
    // Intake Controls
    driverLTrigger.whileTrue(rollerSubsystem.getSetSpeedCommand(0.9));
    driverRTrigger.whileTrue(rollerSubsystem.getSetSpeedCommand(-0.9));

    //driverLTrigger.whileTrue(rollerSubsystem.getSetSpeedCommand( () -> m_driverController.getLeftTriggerAxis() ));
    //driverRTrigger.whileTrue(rollerSubsystem.getSetSpeedCommand( () -> m_driverController.getRightTriggerAxis() ));
    
    // Wrist Controls
    driverY.onTrue(wristSubsystem.deltaPositionCommand(1));
    driverA.onTrue(wristSubsystem.deltaPositionCommand(-1));

    
    // new JoystickButton(m_driverController, XboxController.Button.kB.value).whileTrue(elevatorSubsystem.goToPosition(50));
    // new JoystickButton(m_driverController, XboxController.Button.kX.value).whileTrue(elevatorSubsystem.goToPosition(0));

    driverLButton.onTrue(elevatorSubsystem.deltaPositionCommand(10));
    driverRButton.onTrue(elevatorSubsystem.deltaPositionCommand(-10));

    // Presets
    // Dpad Down: Elevator to 0 Position, Intake to Reef Station Position
    // // Make Constants Later . . .
    driverDpadDown.onTrue(new ParallelCommandGroup(
      elevatorSubsystem.getSetPositionCommand(20),  // down Position
      wristSubsystem.getSetPositionCommand(0)      // Reef Intake Position
    ));

    // Dpad Up: L2 Position
    driverDpadUp.onTrue(new ParallelCommandGroup(
      elevatorSubsystem.getSetPositionCommand(90), // L2 Position
      wristSubsystem.getSetPositionCommand(-13)             // Angle for L2 and L3
    ));

    //TODO: Presets for L3 Position, L1 Position

     //new JoystickButton(m_driverController, XboxController.Button.kB.value).whileTrue(elevatorSubsystem.routine.quasistatic(Direction.kForward));
     //new JoystickButton(m_driverController, XboxController.Button.kX.value).whileTrue(elevatorSubsystem.routine.quasistatic(Direction.kReverse));
     //new JoystickButton(m_driverController, XboxController.Button.kY.value).whileTrue(elevatorSubsystem.routine.dynamic(Direction.kForward));
     //new JoystickButton(m_driverController, XboxController.Button.kA.value).whileTrue(elevatorSubsystem.routine.dynamic(Direction.kReverse));

    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }

  
}
