package frc.robot.subsystems.lights;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Map;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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

    public static Map<LightStatesEnum, LEDPattern> LightStates = Map.of(
        LightStatesEnum.kIdle, LEDPattern.steps(
                                    Map.of(0, 
                                    Color.kDarkViolet,
                                    0.5, 
                                    DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue 
                                        ? Color.kBlue 
                                        : Color.kRed
                                     )).scrollAtAbsoluteSpeed(MetersPerSecond.of(.3), Meters.of(1 / 60.0)), 
        LightStatesEnum.kDriving, LEDPattern.solid(Color.kPurple), // purple
        LightStatesEnum.kActing, LEDPattern.solid(Color.kOrangeRed), // red orange
        LightStatesEnum.kHasCoral, LEDPattern.solid(Color.kCoral), // aqua
        LightStatesEnum.kHasAlgae, LEDPattern.solid(Color.kLime), // lime
        LightStatesEnum.kHasBoth, LEDPattern.solid(Color.kAzure) // coral
    );

    public static final int kPWMPort = 1;
}