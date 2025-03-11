package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.math.MathHelper;
import frc.robot.math.PID;

import static frc.robot.Constants.*;
import static frc.robot.Constants.Coralizer.*;

public class CoralizerSubsystem extends LoggedSubsystem{

    private SparkMax intakeMotor;
    private SparkMax wristMotor;
    private boolean shouldHold;

    private PID pid;

    private double wristTarget;

    private Timer wristEnableTimer;

    public CoralizerSubsystem() {

        shouldHold = false;
        intakeMotor = new SparkMax(4, SparkLowLevel.MotorType.kBrushless);
        wristMotor = new SparkMax(3, SparkLowLevel.MotorType.kBrushless);
        pid = new PID(cCoralizerP, cCoralizerI, cCoralizerD, cCoralizerMax, cCoralizerMin, 0, this::getRotation);
        wristTarget = 30;
        pid.setGoal(wristTarget);
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        config.inverted(false);
        config.smartCurrentLimit(40);
        config.signals.absoluteEncoderPositionPeriodMs(20);
//        config.absoluteEncoder.inverted(true);
//        config.absoluteEncoder.zeroCentered(true);
        config.absoluteEncoder.zeroOffset(304 / 360.0);
        wristMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        SmartDashboard.putData("coralizer/PID", pid);
        SmartDashboard.putNumber("coralizer/Encoder", 0.0);
        SmartDashboard.putNumber("coralizer/Output", 0.0);

        wristEnableTimer = new Timer();
        wristEnableTimer.reset();
        wristEnableTimer.stop();
    }

    @Override
    public void periodic() {
        if (shouldHold){
            if (hasCoral()){
                setIntakeSpeed(0.1);
            }
            else {
                setIntakeSpeed(1);
            }
        }

        double speed = pid.calculate() + cCoralizerG * Math.cos(getRotation() * (Math.PI / 180));
        wristMotor.set(speed);
        SmartDashboard.putNumber("coralizer/Output", speed);

        SmartDashboard.putNumber("coralizer/Encoder", getRotation());
        SmartDashboard.putBoolean("coralizer/HasCoral", hasCoral());
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public void setWristTarget(double wristTarget) {
        pid.setGoal(MathHelper.clamp(wristTarget, cCoralizerMinRotation, cCoralizerMaxRotation));
        this.wristTarget = wristTarget;
    }

    public double getWristTarget() {
        return wristTarget;
    }

    public double getRotation() {
        double position = wristMotor.getAbsoluteEncoder().getPosition() * 360;
        if (position >= 180){
            position -= 360;
        }
        return position;
    }

    public void startWristTimer(){
        wristEnableTimer.restart();
    }

    public boolean hasCoral(){
        return intakeMotor.getForwardLimitSwitch().isPressed();
    }

    public boolean isShouldHold() {
        return shouldHold;
    }

    public void setShouldHold(boolean shouldHold) {
        this.shouldHold = shouldHold;
    }
}
