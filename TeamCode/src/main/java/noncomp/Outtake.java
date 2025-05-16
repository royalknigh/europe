package noncomp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import com.pedropathing.util.CustomPIDFCoefficients;
import com.pedropathing.util.PIDFController;

import configs.MotorConfig;
import consts.IntConst;
import consts.OutConst;

@Config
@TeleOp(name = "Outtake Dashboard", group = "NonComp")
public class Outtake extends OpMode {

    private FtcDashboard dashboard;
    private MotorConfig motorConfig;
    private PIDFController pidfController;

    public Servo outLeft;
    public Servo outRight;
    public Servo outLink;
    public Servo outY;
    public Servo outClaw;

    public static double lr;
    public static double link;
    public static double y;
    public static double claw;

    private boolean areSlidesDown = true;

    private static final int TOLERANCE = 10;

    public static double P = 0.01;
    public static double I = 0.0001;
    public static double D = 0.0003;
    public static double F = 0.008;

    public static double targetPosition = 0;

    public Servo intRot;
    public Servo intY;
    public Servo intClawRot;
    public Servo intClaw;


    public static double iP = 0, iI = 0, iD = 0, iF = 0;
    private PIDFController intPID;
    public static int intTargetPosition = OutConst.slidesDown;
    private boolean isIntSlideDown;

    public static double intakeRot = 0.5;
    public static double clawRot;
    public static double intakeY;
    public static double intakeClaw;

    @Override
    public void init() {
        motorConfig = new MotorConfig(hardwareMap);

        outLeft = hardwareMap.get(Servo.class, "outLeft");
        outRight = hardwareMap.get(Servo.class, "outRight");
        outLink = hardwareMap.get(Servo.class, "outLink");
        outY = hardwareMap.get(Servo.class, "outY");
        outClaw = hardwareMap.get(Servo.class, "outClaw");

        intRot = hardwareMap.get(Servo.class, "intRot");
        intY = hardwareMap.get(Servo.class, "intY");
        intClawRot = hardwareMap.get(Servo.class, "intClawRot");
        intClaw = hardwareMap.get(Servo.class, "intClaw");

        updatePIDFController();

        // Initialize FTC Dashboard
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        telemetry = dashboard.getTelemetry();
    }

    @Override
    public void loop() {
        setIntPID();
        updatePIDFController1();
        updatePIDFController();

        intClaw.setPosition(intakeClaw);
        intRot.setPosition(intakeRot);
        intY.setPosition(intakeY);
        intClawRot.setPosition(clawRot);

        setServoPositions();

        pidfController.setTargetPosition(targetPosition);
        pidfController.updatePosition(motorConfig.frontSlideMotor.getCurrentPosition());

        double powerLeft = pidfController.runPIDF();

        if (Math.abs(motorConfig.frontSlideMotor.getCurrentPosition() - targetPosition) <= TOLERANCE) {
            powerLeft = 0;
        }
        if (motorConfig.frontSlideMotor.getCurrentPosition() > 10)
            areSlidesDown = false;
        if (motorConfig.frontSlideMotor.getVelocity() < 0.1 && motorConfig.frontSlideMotor.getCurrentPosition() < 10) {
            MotorConfig.frontSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            MotorConfig.backSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            MotorConfig.frontSlideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            MotorConfig.backSlideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            areSlidesDown = true;
        } else {
            motorConfig.frontSlideMotor.setPower(powerLeft);
            motorConfig.backSlideMotor.setPower(powerLeft);
        }

        if (-gamepad1.left_stick_y > 0.1) {
            targetPosition += 40;
        } else if (-gamepad1.left_stick_y < -0.1) {
            targetPosition -= 40;
        }

        telemetry.addData("velocity", MotorConfig.frontSlideMotor.getVelocity());
        telemetry.addData("areSlidesDown", areSlidesDown);
        telemetry.addData("Target Position", targetPosition);
        telemetry.addData("Left Motor Position", motorConfig.frontSlideMotor.getCurrentPosition());

        telemetry.addData("Left Motor Error", motorConfig.frontSlideMotor.getCurrentPosition() - targetPosition);
        telemetry.addData("Right Motor Error", motorConfig.backSlideMotor.getCurrentPosition() - targetPosition);

        telemetry.addData("Right Motor Power", motorConfig.backSlideMotor.getPower());
        telemetry.addData("Left Motor Power", motorConfig.frontSlideMotor.getPower());


        telemetry.update();
    }

    private void updatePIDFController1() {
        CustomPIDFCoefficients coefficients = new CustomPIDFCoefficients(P, I, D, F);
        pidfController = new PIDFController(coefficients);
    }

    public void setServoPositions() {
        outLeft.setPosition(lr);
        outRight.setPosition(lr);
        outLink.setPosition(link);
        outY.setPosition(y);
        outClaw.setPosition(claw);
    }

    public void setIntPID() {
        intPID.setTargetPosition(intTargetPosition);
        intPID.updatePosition(motorConfig.intakeMotor.getCurrentPosition());

        double IntakePower = intPID.runPIDF();

        if (Math.abs(motorConfig.intakeMotor.getCurrentPosition() - intTargetPosition) < 10)
            IntakePower = 0;

        if (motorConfig.intakeMotor.getCurrentPosition() > 10) isIntSlideDown = false;

        if (motorConfig.intakeMotor.getCurrentPosition() < 60 && motorConfig.intakeMotor.getVelocity() < 0.05 && intTargetPosition == IntConst.slideRetracted) {
            MotorConfig.intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            MotorConfig.intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            isIntSlideDown = true;
        }

        if (intTargetPosition == IntConst.slideRetracted && isIntSlideDown)
            motorConfig.intakeMotor.setPower(0);

        if (intTargetPosition == IntConst.slideRetracted && motorConfig.intakeMotor.getCurrentPosition() > 10)
            motorConfig.intakeMotor.setPower(-1);

        else motorConfig.intakeMotor.setPower(IntakePower);

        motorConfig.intakeMotor.setPower(IntakePower);

    }

    public void updatePIDFController() {
        CustomPIDFCoefficients intCoefficients = new CustomPIDFCoefficients(iP, iI, iD, iF);
        intPID = new PIDFController(intCoefficients);
    }
}