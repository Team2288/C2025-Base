package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.drive.ModuleIO.ModuleIOInputs;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public boolean leaderConnected = false;
        public double leaderPosition = 0.0;
        public double leaderVelocityRadPerSec = 0.0;
        public double leaderAppliedVolts = 0.0;
        public double leaderCurrentAmps = 0.0;

        public boolean followerConnected = false;
        public double followerPosition = 0.0;
        public double followerVelocityRadPerSec = 0.0;
        public double followerAppliedVolts = 0.0;
        public double followerCurrentAmps = 0.0;
    }

    public default void updateInputs(ModuleIOInputs inputs) {}

    public default void setPosition(double position) {}

}
