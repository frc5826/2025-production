package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.math.MathHelper;
import frc.robot.math.PID;

import static frc.robot.Constants.*;
import static frc.robot.Constants.Coralizer.*;

public class CoralizerSubsystem extends LoggedSubsystem{

    private SparkMax intakeMotor;
    private SparkMax wristMotor;

    private PID pid;

    private double wristTarget;

    public CoralizerSubsystem() {

        intakeMotor = new SparkMax(4, SparkLowLevel.MotorType.kBrushless);
        wristMotor = new SparkMax(3, SparkLowLevel.MotorType.kBrushless);
        pid = new PID(cCoralizerP, cCoralizerI, cCoralizerD, cCoralizerMax, cCoralizerMin, cCoralizerDeadband, this::getRotation);
        pid.setGoal(0);
        wristTarget = 0;
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        config.inverted(false);
        config.smartCurrentLimit(40);
        wristMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        SmartDashboard.putData("coralizer/PID", pid);
        SmartDashboard.putNumber("coralizer/Encoder", 0.0);
        SmartDashboard.putNumber("coralizer/Output", 0.0);
        //SmartDashboard.putNumber("coralizer/Current", 0.0);
    }

    @Override
    public void periodic() {
        double speed = pid.calculate() + cCoralizerG * Math.cos(getRotation() * (Math.PI/180));
        wristMotor.set(speed);
        SmartDashboard.putNumber("coralizer/Encoder", getRotation());
        SmartDashboard.putNumber("coralizer/Output", speed);
        SmartDashboard.putBoolean("coralizer/HasCoral", hasCoral());
        //SmartDashboard.putNumber("coralizer/Current", cPowerDistribution.getCurrent(2));
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

    public boolean hasCoral(){
        return intakeMotor.getReverseLimitSwitch().isPressed();
    }

}
