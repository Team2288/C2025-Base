package frc.robot.subsystems.lights;
import java.util.Map;

public class LightsConstants {
    public static enum LightStatesEnum {
        kIdle,
        kDriving,
        kActing,
        kHasCoral,
        kIntaking,
        kHasAlgae,
        kHasBoth
    }

    public static final Map<LightStatesEnum, Double> LightStates = Map.of(
        LightStatesEnum.kIdle, -0.95, // rainbow ocean
        LightStatesEnum.kDriving, 0.91, // purple
        LightStatesEnum.kActing, 0.63, // red orange
        LightStatesEnum.kHasCoral, 0.81, // aqua
        LightStatesEnum.kHasAlgae, 0.73, // lime
        LightStatesEnum.kHasBoth, -0.93 // rainbow lava
    );

    public static final int kBlinkinPWMPort = 20;
}
