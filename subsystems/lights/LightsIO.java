package frc.robot.subsystems.lights;
import frc.robot.subsystems.lights.LightsConstants.LightStatesEnum;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.wpilibj.LEDPattern;

public interface LightsIO {
    @AutoLog
    public static class LightsIOInputs {
        public LightStatesEnum currentState = LightStatesEnum.kIdle;
    }

    public default void updateInputs(LightsIOInputs inputs) {}

    public default void setLEDPattern(LightStatesEnum state, boolean isFlashing) {}

    public default void setLEDColor(LEDPattern color, boolean isFlashing) {}
 
    public default void setLEDData() {}
}
