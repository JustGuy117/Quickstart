package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class test_botMain4th extends OpMode {

    public DcMotor b_l_drive;
    public DcMotor b_r_drive;
    public DcMotor f_l_drive;
    public DcMotor f_r_drive;
    public DcMotor shoota;
    public DcMotor intake;
    public DcMotor transferMotor;
    public Servo gate;
    double vertical;
    double horizontal;
    double pivot;
    boolean takeIn;
    boolean shootaToggle;
    boolean takeOut;
    double shootaPowwa;
    double b_l_drivePower;
    double b_r_drivePower;
    double f_l_drivePower;
    double f_r_drivePower;
    double maxDriveSpeed = 1.0;
    double intakePower;
    double transferPower;
    double gatePosition;


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

        transferMotor = hardwareMap.get(DcMotor.class, "transferMotor");
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transferMotor.setDirection(DcMotor.Direction.REVERSE);

        shoota = hardwareMap.get(DcMotor.class, "shoota");
        shoota.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoota.setDirection(DcMotor.Direction.REVERSE);

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



        if ((!shootaToggle) && gamepad1.dpad_down)
        {
            if (shootaPowwa < 0.9)
            {
                shootaPowwa = 0.9;
            }
            else
            {
                shootaPowwa = 0.0;
            }
        }

        shootaToggle = gamepad1.dpad_down;



        b_l_drivePower = (vertical - pivot  + horizontal) * maxDriveSpeed;
        b_r_drivePower = (vertical + pivot  - horizontal) * maxDriveSpeed;
        f_l_drivePower = (vertical - pivot  - horizontal) * maxDriveSpeed;
        f_r_drivePower = (vertical + pivot  + horizontal) * maxDriveSpeed;


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

        if (gamepad1.right_trigger < .8){
            gatePosition =.62;
        }
        else{
            gatePosition =.25;
        }

        b_l_drive.setPower(b_l_drivePower);
        b_r_drive.setPower(b_r_drivePower);
        f_l_drive.setPower(f_l_drivePower);
        f_r_drive.setPower(f_r_drivePower);
        shoota.setPower(shootaPowwa);
        intake.setPower(intakePower);
        transferMotor.setPower(transferPower);
        gate.setPosition(gatePosition);

    }
}
