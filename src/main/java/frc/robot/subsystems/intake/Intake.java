package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.util.PhoenixUtil;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private final Alert swivelConnected, intakeConnected;

    private double swivelGoal, intakeGoalRotPerSec;
    private final SysIdRoutine swivelRoutine;
    
    private DigitalInput beambreak;

    public Intake(IntakeIO io) {
        this.io = io;
        this.swivelConnected = new Alert("Leader Kraken disconnected for Intake", AlertType.kError);
        this.intakeConnected = new Alert("Follower Kraken disconnected for Intake", AlertType.kError);

        this.swivelGoal = 0.0;
        this.intakeGoalRotPerSec = 0.0;

        this.beambreak = new DigitalInput(8);

        swivelRoutine =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    null, // Default ramp rate is acceptable
                    Volts.of(4),
                    null, // Default timeout is acceptable
                    (state) -> SignalLogger.writeString("state", state.toString())),
                new SysIdRoutine.Mechanism(
                    volts -> io.swivelSetVoltage(volts.in(Volts)), null, this)
                );

        this.io.swivelResetEncoder(0.0);
       // setDefaultCommand(setIntakePositionAndVelocity(0.0, 0.0));
    }

    public Command setIntakePositionAndVoltageBeambreak(double position, double volts) {
        return new FunctionalCommand(
            () -> {
                this.swivelGoal = position;
                Logger.recordOutput("Intake/SwivelGoal", swivelGoal);
            },
            () -> {io.swivelSetPosition(position); io.intakeSetVoltage(volts);},
            interrupted -> {
            },
            () -> getBeambreak(),
            this
        );

    }

    public Command setIntakePositionAndVoltageNoBeambreak(double position, double volts) {
        return new FunctionalCommand(
            () -> {
                this.swivelGoal = position;
                Logger.recordOutput("Intake/SwivelGoal", swivelGoal);
            },
            () -> {io.swivelSetPosition(position); io.intakeSetVoltage(volts);},
            interrupted -> {
            },
            () -> true,
            this
        );
    }

    public Command setIntakePosition(double position) {
        return new FunctionalCommand(
            () -> {
                this.swivelGoal = position;
                Logger.recordOutput("Intake/SwivelGoal", swivelGoal);
            },
            () -> io.swivelSetPosition(position),
            interrupted -> {

            },
            () -> PhoenixUtil.epsilonEquals(inputs.swivelPosition, position, 0.1),
            this
        );
    }

    public Command setIntakeVelocity(double velocityRotPerSec) {
        return new FunctionalCommand(
            () -> {
                this.intakeGoalRotPerSec = velocityRotPerSec;
                Logger.recordOutput("Intake/IntakeGoalVelocity", this.intakeGoalRotPerSec);
            },
            () -> io.intakeSetVelocity(velocityRotPerSec),
            interrupted -> {
            },
            () -> PhoenixUtil.epsilonEquals(inputs.intakeVelocityRotPerSec, velocityRotPerSec, 0.1),
            this
        );
    }

    public Command setIntakeVoltage(double voltage) {
        return runOnce(() -> io.intakeSetVoltage(voltage));
    }

    public Command characterizeSwivel() {
        return Commands.sequence(
            this.runOnce(() -> SignalLogger.start()),

            swivelRoutine
                .quasistatic(Direction.kForward)
                .until(() -> inputs.swivelPosition > 20),

            this.runOnce(() -> io.swivelSetVoltage(0.0)),

            Commands.waitSeconds(1.0),

            // Stop when we get close to max to avoid hitting hard stop
            swivelRoutine
                .quasistatic(Direction.kReverse)
                .until(() -> inputs.swivelPosition < 5),

            this.runOnce(() -> io.swivelSetVoltage(0.0)),

            Commands.waitSeconds(1.0),

            // Stop when we get close to max to avoid hitting hard stop
            swivelRoutine
                .dynamic(Direction.kForward)
                .until(() -> inputs.swivelPosition > 20),

            this.runOnce(() -> io.swivelSetVoltage(0.0)),

            Commands.waitSeconds(1.0),

            // Stop when we get close to max to avoid hitting hard stop
            swivelRoutine
                .dynamic(Direction.kReverse)
                .until(() -> inputs.swivelPosition < 5),

            this.runOnce(() -> SignalLogger.stop())
        );

    }

    public Command intake() {
        return setIntakePositionAndVoltageBeambreak(IntakeConstants.intakeIntake, -8)
               .until(() -> getBeambreak());
    }

    public boolean getBeambreak(){
        return !beambreak.get();
    }


    @Override
    public void periodic() {
        io.updateInputs(inputs);

        swivelConnected.set(inputs.swivelConnected);
        intakeConnected.set(inputs.intakeConnected);

        Logger.processInputs("Intake/Inputs", inputs);
    }
}
