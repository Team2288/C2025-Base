package frc.robot.subsystems;

import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.PhoenixUtil.ReefTarget;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeConstants;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.littletonrobotics.junction.Logger;

public class SuperStructure {
    Elevator elevator;
    Intake intake;
    SuperState state;

    public static enum SuperState {
        SCORING_CORAL,
        SCORING_ALGAE,
        HAS_CORAL,
        HAS_ALGAE,
        IDLE
    }

    public SuperStructure(Elevator elevator, Intake intake)
    {
        this.elevator = elevator;
        this.intake = intake;
    }

    public Command scoreCoral(ReefTarget target) {
        return new SequentialCommandGroup(
            intake.setIntakePosition(IntakeConstants.intakeIdle),
            elevator.setElevatorPosition(target.elevatorHeight),
            intake.setIntakePosition(target.wristPosition),
            robotIdle()
        )
        .beforeStarting(() -> {
            this.state = SuperState.SCORING_CORAL;
            Logger.recordOutput("SuperStructure/State", this.state);
        })
        .andThen(robotIdle());
    }

    public Command robotIdle() {
        return new SequentialCommandGroup(
            this.intake.setIntakePosition(IntakeConstants.intakeIdle),
            this.elevator.fastZero()
        ).beforeStarting(() -> {
            this.state = SuperState.IDLE;
            Logger.recordOutput("SuperStructure/State", this.state);
        });
    }

    public SuperState getState() {
        return this.state;
    }
}