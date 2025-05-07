package configs;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoConfig {

    public Servo outLeft;
    public Servo outRight;
    public Servo outLink;
    public Servo outY;
    public Servo outClaw;

    public Servo ptoRot;
    public Servo ptoLeft;
    public Servo ptoRight;

    public Servo intRot;
    public Servo intY;
    public Servo intClawRot;
    public Servo intClaw;

    public ServoConfig(HardwareMap hardwareMap){
        //pto
        ptoLeft = hardwareMap.get(Servo.class, "ptoLeft");
        ptoRight = hardwareMap.get(Servo.class, "ptoRight");
        ptoRot = hardwareMap.get(Servo.class, "ptoRot");

        //outtake
        outLeft = hardwareMap.get(Servo.class, "outLeft");
        outRight = hardwareMap.get(Servo.class, "outRight");
        outLink = hardwareMap.get(Servo.class, "outLink");
        outY = hardwareMap.get(Servo.class, "outY");
        outClaw = hardwareMap.get(Servo.class, "outClaw");

        //intake
        intRot = hardwareMap.get(Servo.class, "intRot");
        intY = hardwareMap.get(Servo.class, "intY");
        intClawRot = hardwareMap.get(Servo.class, "intClawRot");
        intClaw = hardwareMap.get(Servo.class, "intClaw");

    }
}
