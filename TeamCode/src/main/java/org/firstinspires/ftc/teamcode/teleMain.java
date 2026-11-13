package org.firstinspires.ftc.teamcode;


import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;


@TeleOp
public class teleMain extends OpMode {


    robotConstats rConstats = new robotConstats();



    public DcMotor b_l_drive;
    public DcMotor b_r_drive;
    public DcMotor f_l_drive;
    public DcMotor f_r_drive;
    double vertical;
    double horizontal;
    double pivot;
    double b_l_drivePower;
    double b_r_drivePower;
    double f_l_drivePower;
    double f_r_drivePower;
    double maxDriveSpeed = 1.0;

    double[] stepsizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    int stepindex = 1;

    private Follower follower;




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

        rConstats.init(hardwareMap);

        follower = Constants.create(hardwareMap);

    }

    @Override
    public void start(){
        follower.setPose(OpModeStorage.autonomousEndPose);
        follower.update();
    }


    @Override
    public void loop() {




        if (gamepad1.dpadRightWasPressed()){
            if (!rConstats.getFlowerMech()) {
                rConstats.flowerMechDown(true);
            }
            else {
                rConstats.flowerMechDown(false);
            }
        }


        if (gamepad1.dpadDownWasPressed()) {
            if (!rConstats.shooterOn) {
                rConstats.shooterOn(true);
            }
            else {
                rConstats.shooterOn(false);
            }
        }

        if (gamepad1.right_bumper) {
            rConstats.intakeOn(true);
        }
        else {
            rConstats.intakeOn(false);
        }
        rConstats.transferStaller();


        if (gamepad1.left_bumper) {
            rConstats.intakeOut();
        }

        if (gamepad1.right_trigger > .2){
            rConstats.shooting(true);
        }
        else{
            rConstats.shooting(false);
        }




        if (gamepad2.bWasPressed()) {
            stepindex = (stepindex + 1) % stepsizes.length;
        }


        if (gamepad2.dpadLeftWasPressed()) {
            rConstats.F -= stepsizes[stepindex];
        }


        if (gamepad2.dpadRightWasPressed()) {
            rConstats.F += stepsizes[stepindex];
        }


        if (gamepad2.dpadUpWasPressed()) {
            rConstats.P += stepsizes[stepindex];
        }


        if (gamepad2.dpadDownWasPressed()) {
            rConstats.P -= stepsizes[stepindex];
        }


        if (gamepad2.yWasPressed()) {
            rConstats.shooterSpeed += 10;
        }


        if (gamepad2.aWasPressed()) {
            rConstats.shooterSpeed -= 10;
        }



        vertical = -gamepad1.left_stick_y;
        horizontal = -gamepad1.left_stick_x;
        pivot = -gamepad1.right_stick_x;

        b_l_drivePower = (vertical - pivot  + horizontal) * maxDriveSpeed;
        b_r_drivePower = (vertical + pivot  - horizontal) * maxDriveSpeed;
        f_l_drivePower = (vertical - pivot  - horizontal) * maxDriveSpeed;
        f_r_drivePower = (vertical + pivot  + horizontal) * maxDriveSpeed;

        b_l_drive.setPower(b_l_drivePower);
        b_r_drive.setPower(b_r_drivePower);
        f_l_drive.setPower(f_l_drivePower);
        f_r_drive.setPower(f_r_drivePower);




        if (gamepad1.dpadUpWasPressed()) {
            Pose cornerPose = new Pose(8.5, 133, Math.toRadians(90));
            follower.setPose(cornerPose); // overrides our pose
        }

        follower.update();
        Pose robotPose = follower.pose();
        rConstats.callUpdate(robotPose);


        double curShootaPowwa = rConstats.shooter.getVelocity();
        double shootaError = rConstats.shooterSpeed - curShootaPowwa;

        telemetry.addData("target powwa", rConstats.shooterSpeed);
        telemetry.addData("current powwa", "%.2f", curShootaPowwa);
        telemetry.addData("Error", "%.2f", shootaError);
        telemetry.addLine("-----------------------------------");
        telemetry.addData("P tune", "%.4f (D-Pad U/D)", rConstats.P);
        telemetry.addData("F tune", "%.4f (D-Pad L/R)", rConstats.F);
        telemetry.addData("step size", "%.4f (B Button)", stepsizes[stepindex]);
        telemetry.addLine("-----------------------------------");
        telemetry.addData("gate position", rConstats.gate.getPosition());
        telemetry.addData("intake speed", rConstats.intakePower);
        telemetry.addData("transfer speed", rConstats.transferPower);
        telemetry.addLine("-----------------------------------");
        telemetry.addData("transfer over current", rConstats.transfer.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
        telemetry.update();
    }
}




