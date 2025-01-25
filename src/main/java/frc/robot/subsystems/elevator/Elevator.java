package frc.robot.subsystems.elevator;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PhoenixUtil;

public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final Alert leaderConnected, followerConnected;
    private ElevatorState currState;
    private double elevatorGoal;

    public enum ElevatorState {
        IDLING,
        RETRACTING,
        EXTENDING
    }

    public Elevator(ElevatorIO io) {
        this.io = io;
        this.leaderConnected = new Alert("Leader Kraken disconnected for Elevator", AlertType.kError);
        this.followerConnected = new Alert("Follower Kraken disconnected for Elevator", AlertType.kError);

        this.currState = ElevatorState.IDLING;
        this.elevatorGoal = 0.0;

        this.io.resetEncoder(0.0);
        setDefaultCommand(setElevatorPosition(0.0));
    }

    public boolean supplyLED() {
        return this.currState == ElevatorState.RETRACTING || this.currState == ElevatorState.EXTENDING;
    }

    public Command setElevatorPosition(double position) {
        return new FunctionalCommand(
            () -> {
                if (position == this.elevatorGoal) this.currState = ElevatorState.IDLING;
                else if (position > this.elevatorGoal) this.currState = ElevatorState.EXTENDING;
                else this.currState = ElevatorState.RETRACTING;

                this.elevatorGoal = position;
                Logger.recordOutput("Elevator/Setpoint", elevatorGoal);
            },
            () -> io.setPositionMotionMagic(position),
            interrupted -> {
                this.currState = ElevatorState.IDLING;
            },
            () -> PhoenixUtil.epsilonEquals(inputs.leaderPosition, position, 10),
            this
        );
    }

    public Command currentZero() {
        return run(() -> io.setVoltage(-1.0))
               .until(() -> inputs.leaderCurrentAmps > 40.0)
               .finallyDo(() -> io.resetEncoder(0.0));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        leaderConnected.set(inputs.leaderConnected);
        followerConnected.set(inputs.followerConnected);

        Logger.processInputs("Elevator/", inputs);
        Logger.recordOutput("Elevator/ElevatorState", currState);
    }

}
