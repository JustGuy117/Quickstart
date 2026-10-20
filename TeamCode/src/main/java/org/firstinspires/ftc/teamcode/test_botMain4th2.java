package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class test_botMain4th2 extends OpMode {


    public DcMotor b_l_drive;
    public DcMotor b_r_drive;
    public DcMotor f_l_drive;
    public DcMotor f_r_drive;
    public DcMotorEx shoota;
    public DcMotor intake;
    public DcMotor transfer;
    public Servo gate;
    double vertical;
    double horizontal;
    double pivot;
    boolean takeIn;
    boolean takeOut;
    double shootaPowwa;
    double shootaSpeedy = 1550;
    double b_l_drivePower;
    double b_r_drivePower;
    double f_l_drivePower;
    double f_r_drivePower;
    double maxDriveSpeed = 1.0;
    double intakePower;
    double transferPower;
    double gatePosition;








    double F = 11.6;
    double P = 0;
    double[] stepsizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    int stepindex = 1;


    @Override
    public void init() {
        b_l_drive = hardwareMap.get(DcMotor.class, "b-l-drive");
        b_l_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        b_l_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        b_r_drive = hardwareMap.get(DcMotor.class, "b-r-drive");
        b_r_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        b_r_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        b_l_drive.setDirection(DcMotor.Direction.REVERSE);


        f_l_drive = hardwareMap.get(DcMotor.class, "f-l-drive");
        f_l_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        f_l_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        f_r_drive = hardwareMap.get(DcMotor.class, "f-r-drive");
        f_r_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        f_r_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        f_l_drive.setDirection(DcMotor.Direction.REVERSE);




        intake = hardwareMap.get(DcMotor.class, "intakeMotor");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        transfer = hardwareMap.get(DcMotor.class, "transferMotor");
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setDirection(DcMotor.Direction.REVERSE);


        shoota = hardwareMap.get(DcMotorEx.class, "shoota");
        shoota.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoota.setDirection(DcMotor.Direction.FORWARD);
        shoota.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        PIDFCoefficients shootaPID = new PIDFCoefficients(P, 0, 0, F);
        shoota.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shootaPID);


        gate = hardwareMap.get(Servo.class, "gateServo");


    }


    @Override
    public void loop() {


        vertical = gamepad1.left_stick_y;
        horizontal = gamepad1.left_stick_x;
        pivot = gamepad1.right_stick_x;
        takeIn = gamepad1.right_bumper;
        takeOut = gamepad1.left_bumper;






        if (gamepad1.aWasPressed()) {
            if ( maxDriveSpeed < 1.0) {
                maxDriveSpeed = 1.0;
            }
            else {
                maxDriveSpeed = 0.5;
            }
        }




        b_l_drivePower = (vertical - pivot  + horizontal) * maxDriveSpeed;
        b_r_drivePower = (vertical + pivot  - horizontal) * maxDriveSpeed;
        f_l_drivePower = (vertical - pivot  - horizontal) * maxDriveSpeed;
        f_r_drivePower = (vertical + pivot  + horizontal) * maxDriveSpeed;




        if (gamepad1.dpadDownWasPressed()) {
            if (!(shootaPowwa == shootaSpeedy)) {
                shootaPowwa = shootaSpeedy;
            }
            else {
                shootaPowwa = 0.0;
            }
        }






        if (takeIn) {
            intakePower = 0.7;
            transferPower = 0.7;
        }
        else if (takeOut) {
            intakePower = -0.7;
            transferPower = -0.7;
        }
        else {
            intakePower = 0;
            transferPower = 0;
        }


        if (gamepad1.right_trigger > .2){
            gatePosition =.25;
            intakePower = 0.5;
            transferPower = 0.4;
        }
        else{
            gatePosition =.62;
        }




        if (gamepad2.bWasPressed()) {
            stepindex = (stepindex + 1) % stepsizes.length;
        }


        if (gamepad2.dpadLeftWasPressed()) {
            F -= stepsizes[stepindex];
        }


        if (gamepad2.dpadRightWasPressed()) {
            F += stepsizes[stepindex];
        }


        if (gamepad2.dpadUpWasPressed()) {
            P += stepsizes[stepindex];
        }


        if (gamepad2.dpadDownWasPressed()) {
            P -= stepsizes[stepindex];
        }


        if (gamepad2.yWasPressed()) {
            shootaPowwa += 10;
        }


        if (gamepad2.aWasPressed()) {
            shootaPowwa -= 10;
        }




        PIDFCoefficients shootaPID = new PIDFCoefficients(P, 0, 0, F);
        shoota.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shootaPID);


        b_l_drive.setPower(b_l_drivePower);
        b_r_drive.setPower(b_r_drivePower);
        f_l_drive.setPower(f_l_drivePower);
        f_r_drive.setPower(f_r_drivePower);
        intake.setPower(intakePower);
        transfer.setPower(transferPower);
        gate.setPosition(gatePosition);
        shoota.setVelocity(shootaPowwa);




        double curShootaPowwa = shoota.getVelocity();
        double shootaError = shootaPowwa - curShootaPowwa;


        telemetry.addData("target powwa", shootaPowwa);
        telemetry.addData("current powwa", "%.2f", curShootaPowwa);
        telemetry.addData("Error", "%.2f", shootaError);
        telemetry.addLine("-----------------------------------");
        telemetry.addData("P tune", "%.4f (D-Pad U/D)", P);
        telemetry.addData("F tune", "%.4f (D-Pad L/R)", F);
        telemetry.addData("step size", "%.4f (B Button)", stepsizes[stepindex]);
        telemetry.addLine("-----------------------------------");
        telemetry.addData("gate position", gate.getPosition());
        telemetry.addData("intake speed", intakePower);
        telemetry.addData("transfer speed", transferPower);
        telemetry.addLine("-----------------------------------");
        telemetry.update();
    }
}




