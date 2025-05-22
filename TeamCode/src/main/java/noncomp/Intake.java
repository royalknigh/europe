package noncomp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.pedropathing.util.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import configs.MotorConfig;
import configs.ServoConfig;
import consts.IntConst;
import consts.OutConst;


@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Intake Dashboard", group = "NonComp")
public class Intake extends LinearOpMode {
    private FtcDashboard dashboard;

    private ServoConfig servoConfig;
    private MotorConfig motorConfig;

    public Servo intRot;
    public Servo intY;
    public Servo intClawRot;
    public Servo intClaw;


    public static double iP = 0, iI = 0, iD = 0, iF = 0;
    private PIDFController intPID;
    public static int intTargetPosition = 0;
    private boolean isIntSlideDown;

    public static double intakeRot = 0.5;
    public static double clawRot;
    public static double intakeY;
    public static double intakeClaw;

    @Override
    public void runOpMode() {
        motorConfig = new MotorConfig(hardwareMap);

        intRot = hardwareMap.get(Servo.class, "intRot");
        intY = hardwareMap.get(Servo.class, "intY");
        intClawRot = hardwareMap.get(Servo.class, "intClawRot");
        intClaw = hardwareMap.get(Servo.class, "intClaw");

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        telemetry = dashboard.getTelemetry();

        waitForStart();

        while (opModeIsActive()) {
            //setIntPID();
           // updatePIDFController();

            intClaw.setPosition(intakeClaw);
            intRot.setPosition(intakeRot);
            intY.setPosition(intakeY);
            intClawRot.setPosition(clawRot);

            telemetry.addData("intakeRot", intakeRot);

            telemetry.update();
        }
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
