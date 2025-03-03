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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import com.pathplanner.lib.auto.NamedCommands;
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
import frc.robot.util.PhoenixUtil;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
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

  // Controller
  // private final CommandJoystick controller = new CommandJoystick(0);
  private final CommandJoystick controller = new CommandJoystick(0);
  private final CommandXboxController codriver = new CommandXboxController(1);

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
                new VisionIOLimelight("limelight-front", drive::getRotation),
                new VisionIOLimelight("limelight-back", drive::getRotation),
                new VisionIOPhotonVision("cameraRight", VisionConstants.robotToCameraRight),
                new VisionIOPhotonVision("cameraLeft", VisionConstants.robotToCameraLeft)
            ); 

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

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});

        elevator = new Elevator(new ElevatorIO() {});

        intake = new Intake(new IntakeIO() {});

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

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});

        elevator = new Elevator(new ElevatorIO() {});

        intake = new Intake(new IntakeIO() {});

        superstructure = new SuperStructure(elevator, intake);

        lights = new Lights(new LightsIOAddressable(), superstructure::supplyLED);

        break;
    }

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
        "Test Auto12", AutoBuilder.buildAuto("autotest"));

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
    
    NamedCommands.registerCommand("L4_Ready", superstructure.readyToScore(ReefTarget.L4));
    NamedCommands.registerCommand("Idle", superstructure.robotIdle());
    NamedCommands.registerCommand("Intake", superstructure.intake());
    NamedCommands.registerCommand("Score", superstructure.score(ReefTarget.L4));

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
            () -> superstructure.isSlow()));

    controller
        .button(18)
        .whileTrue(
            new DriveToPose(
                drive,
                () -> PhoenixUtil.getClosestPose(drive.getPose())
            )
        );

    codriver
        .a()
        .onTrue(
            superstructure.intake()
        );

    codriver
        .y()
        .onTrue(
            superstructure.robotIdle()
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

    // Lock to 0° when A button is held
    /* 
    controller
        .button(3)
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getY(),
                () -> -controller.getX(),
                () -> new Rotation2d()));
    */
    // Switch to X pattern when X button is pressed
    /* 
    controller.button(3).onTrue(Commands.runOnce(drive::stopWithX, drive));
    */

    // Reset gyro to 0° when B button is pressed
    /* 
    controller
        .button(5)
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
    */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }


}