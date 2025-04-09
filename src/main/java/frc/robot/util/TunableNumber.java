package frc.robot.util;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TunableNumber {
    private String key;
    private double defaultValue;

    public TunableNumber(String dashboardKey) {
        this.key = dashboardKey;
    }

    public double getDefault() {
        return defaultValue;
    }

    public void setDefault(double defaultValue) {
        this.defaultValue = defaultValue;
        if (Constants.tuningMode) {
            SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, defaultValue));
        }
    }

    public double get() {
        return Constants.tuningMode ? SmartDashboard.getNumber(key, defaultValue) : defaultValue;
    }
}
