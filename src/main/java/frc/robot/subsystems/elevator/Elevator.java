package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.PhoenixUtil;

public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final Alert leaderConnected, followerConnected;
    private ElevatorState currState;
    private double elevatorGoal;
    private final SysIdRoutine elevatorRoutine;

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

        elevatorRoutine =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    null, // Default ramp rate is acceptable
                    Volts.of(4),
                    null, // Default timeout is acceptable
                    (state) -> SignalLogger.writeString("state", state.toString())),
                new SysIdRoutine.Mechanism(
                    volts -> io.setVoltage(volts.in(Volts)), null, this)
                );

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
            () -> io.setPosition(position),
            interrupted -> {
                this.currState = ElevatorState.IDLING;
            },
            () -> PhoenixUtil.epsilonEquals(inputs.leaderPosition, position, 0.1),
            this
        );
    }

    public Command fastZero() {
        return setElevatorPosition(0.0)
               .andThen(currentZero());
    }

    public Command currentZero() {
        return this.run(() -> io.setVoltage(-1.0))
               .until(() -> inputs.leaderCurrentAmps > 40.0)
               .finallyDo(() -> {
                    io.resetEncoder(0.0);
                    io.setVoltage(0.0);
               });
    }

    public Command sysIDCharacterize() {
        return Commands.sequence(
            this.currentZero(),

            this.runOnce(() -> SignalLogger.start()),

            elevatorRoutine
                .quasistatic(Direction.kForward)
                .until(() -> inputs.leaderPosition > ElevatorConstants.maxEncoderTicks - 0.8),

            this.runOnce(() -> io.setVoltage(0.0)),

            Commands.waitSeconds(1.0),

            // Stop when we get close to max to avoid hitting hard stop
            elevatorRoutine
                .quasistatic(Direction.kReverse)
                .until(() -> inputs.leaderPosition < 1),

            this.runOnce(() -> io.setVoltage(0.0)),

            Commands.waitSeconds(1.0),

            // Stop when we get close to max to avoid hitting hard stop
            elevatorRoutine
                .dynamic(Direction.kForward)
                .until(() -> inputs.leaderPosition > ElevatorConstants.maxEncoderTicks - 0.8),

            this.runOnce(() -> io.setVoltage(0.0)),

            Commands.waitSeconds(1.0),

            // Stop when we get close to max to avoid hitting hard stop
            elevatorRoutine
                .dynamic(Direction.kReverse)
                .until(() -> inputs.leaderPosition < 1),

            this.runOnce(() -> SignalLogger.stop())
        );

    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        leaderConnected.set(inputs.leaderConnected);
        followerConnected.set(inputs.followerConnected);

        Logger.processInputs("Elevator/Inputs", inputs);
        Logger.recordOutput("Elevator/ElevatorState", currState);
    }
}
