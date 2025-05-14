package comp.tele;

import com.pedropathing.util.CustomPIDFCoefficients;
import com.pedropathing.util.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import configs.MotorConfig;
import configs.ServoConfig;
import consts.OutConst;

@TeleOp(name="Basic: Omni Linear OpMode", group="Linear OpMode")
@Disabled
public class Merman extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    //outPID and intPID
    public static double oP = 0, oI = 0, oD=0, oF=0;
    public static double iP = 0, iI = 0, iD = 0, iF = 0;

    private MotorConfig motorConfig;
    private ServoConfig servoConfig;
    
    private PIDFController outPID;
    private PIDFController intPID;
    
    private int fraction;
    private boolean areSlidesDown;

    private int targetPosition = OutConst.slidesDown;

    @Override
    public void runOpMode() {
        motorConfig = new MotorConfig(hardwareMap);
        servoConfig = new ServoConfig(hardwareMap);
        fraction =1;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        updatePIDFController();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            movement();
            setOutPID();
            updatePIDFController();


        }

    }

    public void movement(){
        double y = -gamepad2.left_stick_y;
        double x = gamepad2.left_stick_x;
        double rx = gamepad2.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        motorConfig.setMotorPowers(
                frontLeftPower/fraction,
                backLeftPower/fraction,
                frontRightPower/fraction,
                backRightPower/fraction
        );

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }

    public void updatePIDFController() {
        CustomPIDFCoefficients outCoefficients = new CustomPIDFCoefficients(oP, oI, oD, oF);
        CustomPIDFCoefficients intCoefficients = new CustomPIDFCoefficients(iP, iI, iD, iF);
        outPID = new PIDFController(outCoefficients);
        intPID = new PIDFController(intCoefficients);
    }

    public void setOutPID(){
        outPID.setTargetPosition(targetPosition);
        outPID.updatePosition(motorConfig.frontSlideMotor.getCurrentPosition());

        double OuttakePower = outPID.runPIDF();

        if (Math.abs(motorConfig.frontSlideMotor.getCurrentPosition() - targetPosition) < 10) {
            OuttakePower = 0;
        }
        if(motorConfig.frontSlideMotor.getCurrentPosition()>10)
            areSlidesDown = false;
        if(motorConfig.frontSlideMotor.getCurrentPosition()<60 && motorConfig.frontSlideMotor.getVelocity()<0.05 && targetPosition== OutConst.slidesDown){
            MotorConfig.frontSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            MotorConfig.backSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            MotorConfig.frontSlideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            MotorConfig.backSlideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            areSlidesDown = true;
        }
        if(targetPosition == OutConst.slidesDown && areSlidesDown){
            motorConfig.frontSlideMotor.setPower(0);
            motorConfig.backSlideMotor.setPower(0);
        }
        if(targetPosition == OutConst.slidesDown && motorConfig.frontSlideMotor.getCurrentPosition()>10){
            motorConfig.frontSlideMotor.setPower(-1);
            motorConfig.backSlideMotor.setPower(-1);

        }
        else {
            motorConfig.frontSlideMotor.setPower(OuttakePower);
            motorConfig.backSlideMotor.setPower(OuttakePower);
        }
        motorConfig.frontSlideMotor.setPower(OuttakePower);
        motorConfig.backSlideMotor.setPower(OuttakePower);
    }

    public void setIntPID(){

    }
    }

