package configs;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import consts.IntConst;
import consts.OutConst;

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
//        ptoLeft = hardwareMap.get(Servo.class, "ptoLeft");
//        ptoRight = hardwareMap.get(Servo.class, "ptoRight");
//        ptoRot = hardwareMap.get(Servo.class, "ptoRot");
//
//        //outtake
//        outLeft = hardwareMap.get(Servo.class, "outLeft");
//        outRight = hardwareMap.get(Servo.class, "outRight");
//        outLink = hardwareMap.get(Servo.class, "outLink");
//        outY = hardwareMap.get(Servo.class, "outY");
//        outClaw = hardwareMap.get(Servo.class, "outClaw");

        //intake
        intRot = hardwareMap.get(Servo.class, "intRot");
        intY = hardwareMap.get(Servo.class, "intY");
        intClawRot = hardwareMap.get(Servo.class, "intClawRot");
        intClaw = hardwareMap.get(Servo.class, "intClaw");

    }
    public void setIntakePos(double rot, double y, double clawRot, double claw){
        intRot.setPosition(rot);
        intY.setPosition(y);
        intClawRot.setPosition(clawRot);
        intClaw.setPosition(claw);
    }

    public void setOuttakePos(double lr, double y, double link, double claw){
        outRight.setPosition(lr);
        outLeft.setPosition(lr);
        outY.setPosition(y);
        outLink.setPosition(link);
        outClaw.setPosition(claw);
    }

    public void setInitPos(){
        intRot.setPosition(IntConst.intRot_INIT);
        intY.setPosition(IntConst.intY_INIT);
        intClawRot.setPosition(IntConst.intClawRot_INIT);
        intClaw.setPosition(IntConst.intClaw_INIT);

        outRight.setPosition(OutConst.outLeftRight_INIT);
        outLeft.setPosition(OutConst.outLeftRight_INIT);
        outY.setPosition(OutConst.outY_INIT);
        outLink.setPosition(OutConst.outLink_INIT);
        outClaw.setPosition(OutConst.outClaw_INIT);

        ptoRot.setPosition(IntConst.ptoUnlock);
    }

    public void lowerLegs(){
        ptoLeft.setPosition(IntConst.ptoLegsDown);
        ptoRight.setPosition(IntConst.ptoLegsDown);
        ptoRot.setPosition(IntConst.ptoLock);
    }
    public void raiseLegs(){
        ptoLeft.setPosition(IntConst.ptoLegsUp);
        ptoRight.setPosition(IntConst.ptoLegsUp);
    }



}
