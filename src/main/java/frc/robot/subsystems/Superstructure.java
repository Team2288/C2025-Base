package frc.robot.subsystems;

import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.PhoenixUtil.ReefTarget;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeConstants;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.lights.Lights;

import org.littletonrobotics.junction.AutoLogOutput;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;

public class SuperStructure {
    Elevator elevator;
    Intake intake;
    Lights lights;
    SuperState state;

    public static enum SuperState {
        SCORING_L4,
        SCORING_L3,
        SCORING_L2,
        SCORING_L1,
        HAS_CORAL,
        HAS_ALGAE,
        INTAKING,
        SCORED,
        IDLE
    }

    public SuperStructure(Elevator elevator, Intake intake)
    {
        this.elevator = elevator;
        this.intake = intake;
        this.state = SuperState.IDLE;
    }

    public double getSlow() {
        if (this.state == SuperState.SCORING_L4) {
            return 2;
        } else if (this.state == SuperState.SCORING_L3) {
            return 1.3;
        } else {
            return 1;
        }
    }

    public Command readyToScore(ReefTarget target) {
        return new SequentialCommandGroup(
            intake.setIntakePosition(IntakeConstants.intakeIdle),
            elevator.setElevatorPosition(target.elevatorHeight),
            intake.setIntakePosition(target.wristPosition)
        )
        .beforeStarting(() -> {
            if (target.equals(ReefTarget.L4)) {
                this.state = SuperState.SCORING_L4;
            } else if (target.equals(ReefTarget.L3)) {
                this.state = SuperState.SCORING_L3;
            } else if (target.equals(ReefTarget.L2)) {
                this.state = SuperState.SCORING_L2;
            } else {
                this.state = SuperState.SCORING_L1;
            }
        });
    }

    public Command score(ReefTarget target) {
        return intake.setIntakeVoltage(target.outtakeSpeed)
               .beforeStarting(() -> {
                    this.state = SuperState.SCORED;
               });
    }

    public Command robotIdle() {
        return new SequentialCommandGroup(
            this.intake.setIntakePositionAndVoltageNoBeambreak(IntakeConstants.intakeIdle, -1),
            this.elevator.fastZero()
        ).andThen(() -> {
            this.state = SuperState.IDLE;
        });
    }

    public boolean supplyLED() {
        return this.state == SuperState.SCORING_L4 || this.state == SuperState.SCORING_L3 || this.state == SuperState.SCORING_L2 || this.state == SuperState.SCORING_L1 || this.state == SuperState.SCORED || this.state == SuperState.INTAKING;
    }

    public Command intake() {
        return intake.intake()
               .beforeStarting(() -> this.state = SuperState.INTAKING);
              // .andThen(new WaitCommand(0.1))
              // .andThen(robotIdle());
    }

    
    public Command scoreStateMachine(ReefTarget target) {
        return new ConditionalCommand(
            readyToScore(target),
            score(target),
            () -> this.state == SuperState.IDLE
        );
    }
        
/* 
    public Command scoreStateMachine(ReefTarget target) {
        return new SequentialCommandGroup(
            readyToScore(target),
            score(target)
        );
    }
        */

    @AutoLogOutput(key = "Superstructure/State")
    public SuperState getState() {
        return this.state;
    }

}