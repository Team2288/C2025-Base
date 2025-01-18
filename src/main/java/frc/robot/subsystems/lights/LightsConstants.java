package frc.robot.subsystems.lights;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;

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
        LightStatesEnum.kDriving, LEDPattern.steps(
                                    Map.of(0, 
                                    Color.kDarkViolet,
                                    0.5, 
                                    DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue 
                                        ? Color.kBlue 
                                        : Color.kRed
                                    )).scrollAtAbsoluteSpeed(MetersPerSecond.of(.3), Meters.of(1 / 60.0)), 
        LightStatesEnum.kIdle, LEDPattern.solid(Color.kPurple), // purple
        LightStatesEnum.kActing, LEDPattern.gradient(
                                    GradientType.kContinuous,
                                    Color.kOrange,
                                    Color.kDarkRed
                                    ).scrollAtAbsoluteSpeed(MetersPerSecond.of(.3), Meters.of(1/60.0)), // orange to red gradient
        LightStatesEnum.kHasCoral, LEDPattern.solid(Color.kCoral), // coral
        LightStatesEnum.kHasAlgae, LEDPattern.solid(Color.kLime), // lime
        LightStatesEnum.kHasBoth, LEDPattern.gradient(
                                    GradientType.kContinuous,
                                    Color.kAzure,
                                    Color.kLime
                                    ).scrollAtAbsoluteSpeed(MetersPerSecond.of(.3), Meters.of(1/60.0)) // azure to lime gradient
    );

    public static final int kPWMPort = 1;
}