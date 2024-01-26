package com.gos.lib.rev;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.frc2023.util.Alert;

import java.util.function.Supplier;

public class AutoRetryUtil {

    private static final StringBuilder ALERT_BUILDER = new StringBuilder(100); // NOPMD(AvoidStringBufferField)
    private static final Alert CONFIG_FAILED_ALERT = new Alert("Rev CAN config failure", Alert.AlertType.ERROR);
    private static boolean configFailed;

    public static void autoRetry(Supplier<REVLibError> command) {
        autoRetry("", command);
    }

    public static void autoRetry(String functionName, Supplier<REVLibError> command) {
        autoRetry(functionName, command, 10);
    }

    public static void autoRetry(String functionName, Supplier<REVLibError> command, int maxRetries) {
        int ctr = 0;
        REVLibError error;

        do {
            error = command.get();
            ++ctr;
        }
        while (error != REVLibError.kOk && ctr < maxRetries);

        configFailed &= error != REVLibError.kOk;

        if (ctr != 1) {
            ALERT_BUILDER.append("Took ").append(ctr).append(" times to retry command");
            DriverStation.reportError("Took " + ctr + " times to retry command", false);
        }

        CONFIG_FAILED_ALERT.set(configFailed);
        CONFIG_FAILED_ALERT.setText(ALERT_BUILDER.toString());
    }

    public static void setP(SparkPIDController pidController, double gain, int slot) {
        AutoRetryUtil.autoRetry("setP", () -> pidController.setP(gain, slot));
    }

    public static void setI(SparkPIDController pidController, double gain, int slot) {
        AutoRetryUtil.autoRetry("setI", () -> pidController.setI(gain, slot));
    }

    public static void setD(SparkPIDController pidController, double gain, int slot) {
        AutoRetryUtil.autoRetry("setD", () -> pidController.setD(gain, slot));
    }

    public static void setFF(SparkPIDController pidController, double gain, int slot) {
        AutoRetryUtil.autoRetry("setFF", () -> pidController.setFF(gain, slot));
    }

    public static void setSmartMotionMaxVelocity(SparkPIDController pidController, double gain, int slot) {
        AutoRetryUtil.autoRetry("setSmartMotionMaxVelocity", () -> pidController.setSmartMotionMaxVelocity(gain, slot));
    }

    public static void setSmartMotionMaxAccel(SparkPIDController pidController, double gain, int slot) {
        AutoRetryUtil.autoRetry("setSmartMotionMaxAccel", () -> pidController.setSmartMotionMaxAccel(gain, slot));
    }

    public static void restoreFactoryDefaults(CANSparkBase motorController) {
        AutoRetryUtil.autoRetry("restoreFactoryDefaults", motorController::restoreFactoryDefaults);
    }

    public static void setIdleMode(CANSparkBase motorController, CANSparkMax.IdleMode idleMode) {
        AutoRetryUtil.autoRetry("setIdleMode", () -> motorController.setIdleMode(idleMode));
    }

    public static void setSmartCurrentLimit(CANSparkBase motorController, int currentLimit) {
        AutoRetryUtil.autoRetry("setSmartCurrentLimit", () -> motorController.setSmartCurrentLimit(currentLimit));
    }

    public static void follow(CANSparkBase motorController, CANSparkBase controllerToFollow) {
        AutoRetryUtil.autoRetry("follow", () -> motorController.follow(controllerToFollow));
    }
}
