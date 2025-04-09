// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import frc.robot.subsystems.vision.VisionIOPhotonVision;
import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import frc.robot.subsystems.SuperStructure;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.LightsIOAddressable;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.PhoenixUtil.ReefTarget;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.util.PhoenixUtil;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import java.util.function.Supplier;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Elevator elevator;
  private final Lights lights;
  private final Intake intake;
  private final SuperStructure superstructure;
  private final Climber climber;
  // Controller
  // private final CommandJoystick controller = new CommandJoystick(0);
  private final CommandJoystick controller = new CommandJoystick(0);
  private final CommandXboxController codriver = new CommandXboxController(1);
  private boolean isLeft = false;
  private boolean isClimbing = false;
  

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement, 
                drive::getPose,
                new VisionIOLimelight("limelight-front", drive::getRotation)
             //   new VisionIOPhotonVision("cameraRight", VisionConstants.robotToCameraRight),
             //   new VisionIOPhotonVision("cameraLeft", VisionConstants.robotToCameraLeft)
            ); 
            
        climber =
            new Climber(
                new ClimberIOTalonFX()
            );

        Logger.addDataReceiver(new WPILOGWriter("/media/sda1"));        

        elevator = 
            new Elevator(new ElevatorIOTalonFX());
        
        intake = 
            new Intake(new IntakeIOTalonFX());

        superstructure = new SuperStructure(elevator, intake);

        lights = new Lights(new LightsIOAddressable(), superstructure::supplyLED);


        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        // we do not need vision simulation, disable IO

        vision = new Vision(drive::addVisionMeasurement, drive::getPose, new VisionIO() {});

        elevator = new Elevator(new ElevatorIO() {});

        intake = new Intake(new IntakeIO() {});
                
        climber =
            new Climber(
                new ClimberIO() {}
            );


        superstructure = new SuperStructure(elevator, intake);

        lights = new Lights(new LightsIOAddressable(), superstructure::supplyLED);

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        vision = new Vision(drive::addVisionMeasurement, drive::getPose, new VisionIO() {});

        elevator = new Elevator(new ElevatorIO() {});

        intake = new Intake(new IntakeIO() {});

        superstructure = new SuperStructure(elevator, intake);
        
        climber =
            new Climber(
                new ClimberIO() {}
            );


        lights = new Lights(new LightsIOAddressable(), superstructure::supplyLED);

        break;
    }
    NamedCommands.registerCommand("L4_Ready", superstructure.readyToScore(ReefTarget.L4));
    NamedCommands.registerCommand("L3_Ready", superstructure.readyToScore(ReefTarget.L3));
    NamedCommands.registerCommand("Idle", superstructure.robotIdle());
    NamedCommands.registerCommand("Intake", superstructure.intake());
    NamedCommands.registerCommand("Score", new SequentialCommandGroup(superstructure.score(ReefTarget.L4), new WaitCommand(0.4)));
    NamedCommands.registerCommand("Score_L3", new SequentialCommandGroup(superstructure.score(ReefTarget.L3), new WaitCommand(0.4)));


    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Voltage Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive Torque Characterization", drive.sysIDFullRoutine());

    autoChooser.addOption(
        "2 L4 Left", AutoBuilder.buildAuto("2-L4-Left"));

    autoChooser.addOption(
        "Taxi", AutoBuilder.buildAuto("Taxi"));

    autoChooser.addOption(
        "1 L3 Mid", AutoBuilder.buildAuto("1-L3-Mid"));

    autoChooser.addOption("Taxi Right", AutoBuilder.buildAuto("TaxiRight"));
    
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    
    SignalLogger.setPath("/home/lvuser/logs/");

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getY(),
            () -> -controller.getX(),
            () -> -controller.getZ(),
            () -> superstructure.getSlow()));

    controller
        .button(16)
        .onTrue(
            new InstantCommand(() -> this.isLeft = true)
        );


    controller
        .button(17)
        .onTrue(
            new InstantCommand(() -> this.isLeft = false)
        );

    controller
        .button(18)
        .whileTrue(
            new DriveToPose(
                drive,
                () -> PhoenixUtil.getClosestPose(drive.getPose(), () -> getIsLeft())
            )
        );
 
    controller
        .button(1)
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getY(),
                () -> -controller.getX(),
                () -> PhoenixUtil.returnSourceRotatedPose(drive.getPose()).getRotation()
            )
        );

    controller
        .button(2)
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getY(),
                () -> -controller.getX(),
                () -> new Rotation2d(0.0)
            )
        );

    codriver
        .b()
        .onTrue(
            superstructure.algaeRemove()
        );

    codriver
        .a()
        .onTrue(
            superstructure.intake()
        );

    codriver
        .x()    
        .onTrue(
            
            new ConditionalCommand(
                //climber.setClimberVoltage(-4),
                climber.setPosition(0),
                climber.setPosition(ClimberConstants.swivelClimb),
                //climber.setClimberVoltage(4),
                () -> getIsClimbing()
            )
                
            //climber.setClimberVoltage(4)
        );

    codriver
        .y()
        .onTrue(
            superstructure.robotIdle()
           // climber.setClimberVoltage(0)
        );


    codriver.leftBumper()
        .onTrue(
            superstructure.scoreStateMachine(ReefTarget.L4)
        );

    codriver.rightBumper()
        .onTrue(
            superstructure.scoreStateMachine(ReefTarget.L3)
        );

    codriver.leftTrigger()
        .onTrue(
            superstructure.scoreStateMachine(ReefTarget.L2)
        );

    codriver.rightTrigger()
        .onTrue(
            superstructure.scoreStateMachine(ReefTarget.L1)
        );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public boolean getIsClimbing() {
    boolean temp = this.isClimbing;
    this.isClimbing = !this.isClimbing;
    return temp;
  }

  public boolean getIsLeft() {
    SmartDashboard.putBoolean("IsLeft", isLeft);
    return isLeft;
  }


}