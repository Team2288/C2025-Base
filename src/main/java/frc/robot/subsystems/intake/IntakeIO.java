package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public boolean swivelConnected = false;
        public boolean intakeConnected = false;

        public double intakeVelocityRotPerSec = 0.0;
        public double intakeAppliedVolts = 0.0;
        public double intakeCurrentAmps = 0.0;

        public double swivelPosition = 0.0;
        public double swivelVelocityRadPerSec = 0.0;
        public double swivelAppliedVolts = 0.0;
        public double swivelCurrentAmps = 0.0;
    }   

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void swivelSetPosition(double position) {}
    public default void swivelSetVoltage(double voltage) {}
    public default void swivelResetEncoder(double ticks) {}

    public default void intakeSetVelocity(double velocity) {}
    public default void intakeSetVoltage(double voltage) {}
}
