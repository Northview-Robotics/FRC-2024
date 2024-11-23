package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import edu.wpi.first.wpilibj2.command.button.JoystickButton; // unused rn
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;

public class arm extends SubsystemBase {
    private WPI_TalonSRX ArmMotor;// Sets up object representing the real arm motor
    private Encoder armEncoder;
    private PIDController armPID;
    private double encoderDPP = 100; //Distance per pulse aka counts per rotation in the encoder
    
    private static arm arm = null;

   
    // Note that the current arm won't work as intended because it isn't utilizing PID to get into states
    //We don't want to use a toggle because if you hold it will be weird not to mention we are setting arm speed not position
    //to work around this problem I'm using two buttons instead of a toggle
    //Ideally we use enums and PIDs when possible 
    //constructor
    public arm() {
        ArmMotor = new WPI_TalonSRX(5);
        armEncoder = new Encoder(0,3);
        armPID = new PIDController(0.001, 0 ,0); //tune later
    }

    public void runArm(boolean button1, boolean button2){ 
        if(button1){
            double AngleUP = 45;
            setArmPos(AngleUP);
        }
        else if(button2){
            double AngleDown = 0;
            setArmPos(AngleDown);
        }
    }

    //encoderDPP changes depending on the motor you are using (encoderDPP is ticks per rotation)

    public void setArmPos(double setAngle){
        double calcAngle = (armEncoder.get()/encoderDPP) * 360;
        double calcPID = armPID.calculate(calcAngle, setAngle);
        ArmMotor.set(calcPID);
    }

    //Methods
    
    private void stopMotor() {
        ArmMotor.set(0);
    }

    public static arm getInstance(){
        if (arm == null){
            arm = new arm();
        }
        return arm;
    }
}