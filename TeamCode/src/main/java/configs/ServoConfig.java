package configs;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

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

    public ServoConfig(HardwareMap hardwareMap) {
//        //pto
       ptoLeft = hardwareMap.get(Servo.class, "ptoLeft");
        ptoRight = hardwareMap.get(Servo.class, "ptoRight");
       ptoRot = hardwareMap.get(Servo.class, "ptoRot");
//
//        //outtake
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

        outRight.setDirection(Servo.Direction.REVERSE);

    }

    public void setIntakePos(double rot, double y, double clawRot, double claw) {
        intRot.setPosition(rot);
        intY.setPosition(y);
        intClawRot.setPosition(clawRot);
        intClaw.setPosition(claw);
    }



    public void setOuttakePos(double lr, double y, double link, double claw) {
        outRight.setPosition(lr);
        outLeft.setPosition(lr);
        outY.setPosition(y);
        outLink.setPosition(link);
        outClaw.setPosition(claw);
    }

    public void setInitPos() {
        intRot.setPosition(IntConst.rot_INIT);
        intY.setPosition(IntConst.y_INIT);
        intClawRot.setPosition(IntConst.clawRot_INIT);
        intClaw.setPosition(IntConst.claw_OPEN);

        outRight.setPosition(OutConst.lr_INIT);
        outLeft.setPosition(OutConst.lr_INIT);
        outY.setPosition(OutConst.y_INIT);
        outLink.setPosition(OutConst.link_INIT);
        outClaw.setPosition(OutConst.claw_OPEN);
//
        ptoRot.setPosition(IntConst.ptoUnlock);

        ptoLeft.setPosition(IntConst.ptoLegsUp);
        ptoRight.setPosition(IntConst.ptoLegsUp);
    }



}
