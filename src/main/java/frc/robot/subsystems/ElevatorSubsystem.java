package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.math.ElevatorController;
import frc.robot.math.MathHelper;
import frc.robot.math.PID;
import frc.robot.positioning.ReefPosition;

import static frc.robot.Constants.Elevator.*;
import static frc.robot.Constants.Elevator.ConversionFactor.*;

public class ElevatorSubsystem extends LoggedSubsystem {

    private PID elevatorPID = new PID(cElevatorP, cElevatorI, cElevatorD, cElevatorMaxOutput, cElevatorMinOutput, 0, this::getPos);
    private ElevatorController upController = new ElevatorController(cElevatorVUp, cElevatorG, cElevatorMaxVelocity, cElevatorMaxAcceleration, cElevatorMinOutput, cElevatorMaxOutput, elevatorPID );
    private ElevatorController downController = new ElevatorController(cElevatorVDown, cElevatorG, cElevatorMaxVelocity, cElevatorMaxAcceleration, cElevatorMinOutput, cElevatorMaxOutput, elevatorPID );
    private ElevatorController currentController;
    private SparkMax motor, motorFollower;
    private Encoder encoder;
    private double desiredPos;

    public ElevatorSubsystem(){

        motor = new SparkMax(cElevatorMotor1ID, SparkLowLevel.MotorType.kBrushless);
        motorFollower = new SparkMax(cElevatorMotor2ID, SparkLowLevel.MotorType.kBrushless);
        encoder = new Encoder(cElevatorEncoder1IDA, cElevatorEncoder1IDB);
        SmartDashboard.putData("elevator/PID", elevatorPID);
        SmartDashboard.putData("elevator/UpController", upController);
        SmartDashboard.putData("elevator/DownController", downController);
        SmartDashboard.putNumber("elevator/Encoder", 0.0);
        SmartDashboard.putNumber("elevator/Goal", 0.0);
        SmartDashboard.putNumber("elevator/Velocity", 0.0);
        SmartDashboard.putString("elevator/CurrentFF", "Up");
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(false);
        SparkMaxConfig configFollower = new SparkMaxConfig();
        configFollower.follow(motor, true);
        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        motorFollower.configure(configFollower, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        currentController = upController;
    }

    @Override
    public void periodic() {

        double speed = currentController.calculate(0.02);
        motor.set(speed);

        SmartDashboard.putNumber("elevator/Encoder", getPos());

        SmartDashboard.putNumber("elevator/Velocity", motor.getEncoder().getVelocity()* cElevatorRPMtoMPS);

    }

    public void setDesiredPosition(double position) {

        desiredPos = MathHelper.clamp(position, cElevatorHeightMin, cElevatorHeightMax);

        if(getPos() < desiredPos){
            currentController = upController;
            SmartDashboard.putString("elevator/CurrentFF", "Up");
        }
        else {
            currentController = downController;
            SmartDashboard.putString("elevator/CurrentFF", "Down");
        }

        upController.setGoal(getPos(), desiredPos, motor.getEncoder().getVelocity() * cElevatorRPMtoMPS);
        downController.setGoal(getPos(), desiredPos, motor.getEncoder().getVelocity() * cElevatorRPMtoMPS);

        SmartDashboard.putNumber("elevator/Goal", desiredPos);

    }

    public double getDesiredPosition(){
        return desiredPos;
    }


    public double getPos(){

        double pos = encoder.get() / cElevatorClicksPerMeter;

        if(pos >= cElevatorOverlapPosition){
            pos = cElevatorOverlapPosition + ((pos - cElevatorOverlapPosition) * cElevatorOverlapConversion);
        }

        return pos;

    }

}
