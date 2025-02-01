package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.math.HelperMath;
import frc.robot.math.PID;

import static frc.robot.Constants.Coralizer.*;

public class CoralizerSubsystem extends LoggedSubsystem{

    private SparkMax intakeMotor;
    private SparkMax wristMotor;

    private PID pid;

    public CoralizerSubsystem() {

        intakeMotor = new SparkMax(4, SparkLowLevel.MotorType.kBrushless);
        wristMotor = new SparkMax(3, SparkLowLevel.MotorType.kBrushless);
        pid = new PID(cCoralizerP, cCoralizerI, cCoralizerD, cCoralizerMax, cCoralizerMin, cCoralizerDeadband, this::getRotation);
        pid.setGoal(0);
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        wristMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        SmartDashboard.putData("coralizer/PID", pid);
        SmartDashboard.putNumber("coralizer/Encoder", 0.0);

    }

    @Override
    public void periodic() {
        wristMotor.set(pid.calculate());
        SmartDashboard.putNumber("coralizer/Encoder", getRotation());
    }

    public void setIntakeSpeed(double speed){
        intakeMotor.set(speed);
    }

    public void setWristTarget(double wristTarget){
        pid.setGoal(HelperMath.clamp(wristTarget, cCoralizerMinRotation, cCoralizerMaxRotation));
    }

    public double getRotation(){
        double position = wristMotor.getAbsoluteEncoder().getPosition() * 360;
        if (position >= 180){
            position -= 360;
        }
        return position;
    }

}
