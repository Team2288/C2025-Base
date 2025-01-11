package frc.robot.subsystems.lights;
import frc.robot.subsystems.lights.LightsConstants.LightStatesEnum;
import org.littletonrobotics.junction.AutoLog;

public interface LightsIO {
    @AutoLog
    public static class LightsIOInputs {
        public boolean connected = false;
        public LightStatesEnum currentState = LightStatesEnum.kIdle;
    }

    public default void updateInputs(LightsIOInputs inputs) {}

    public default void setLED(LightStatesEnum state) {}
}
