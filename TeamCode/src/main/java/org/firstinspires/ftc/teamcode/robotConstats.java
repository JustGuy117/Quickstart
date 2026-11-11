package org.firstinspires.ftc.teamcode;


import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class robotConstats {


    public DcMotorEx shooter;
    public DcMotorEx intake;
    public DcMotorEx transfer;
    public Servo gate;
    boolean shooterOn;
    double shooterSpeed = 1550;
    double intakePower;
    double transferPower;
    double gatePos;

    boolean transferStall;



    public Servo flowerMech;
    double flowerMDown = 0.95;
    double flowerMUp = 0.25;
    double flowerMCur = flowerMUp;



    double F = 12.0;
    double P = 0.9;

    private Pose curPose = new Pose(0,0,0);
    public void init(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        transfer = hardwareMap.get(DcMotorEx.class, "transferMotor");
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setDirection(DcMotor.Direction.REVERSE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        shooter = hardwareMap.get(DcMotorEx.class, "shoota");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        PIDFCoefficients shootaPID = new PIDFCoefficients(P, 0, 0, F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shootaPID);


        gate = hardwareMap.get(Servo.class, "gateServo");

        flowerMech = hardwareMap.get(Servo.class, "flowerServo");
    }

    public void intakeOn(boolean on) {
        if (on) {
            intakePower = 0.7;
            transferPower = 0.7;
        }
        else {
            intakePower = 0.0;
            transferPower = 0.0;
            transferStall = false;
        }
    }
    public void transferStaller() {
        if (transfer.getCurrent(CurrentUnit.AMPS) > 3.5 || transferStall) {
            transferPower = 0;
            transferStall = true;
        }
    }
    public void intakeOut() {
        intakePower = -0.7;
        transferPower = -0.7;
    }
    public void flowerMechDown(boolean down) {
        if (down) {
            flowerMCur = flowerMDown;
        }
        else {
            flowerMCur = flowerMUp;
        }
        flowerMech.setPosition(flowerMCur);
    }
    public boolean getFlowerMech() {
        if (flowerMCur == flowerMDown) {
            return true;
        }
        else return false;
    }

    public void shooting(boolean on) {
        if (on) {
            gatePos = 0.25;
            intakePower = 0.5;
            transferPower = 0.4;
        }
        else {
            gatePos = 0.62;
        }
    }

    public void shooterOn(boolean on) {
        shooterOn = on;
    }


    public void callUpdate(Pose pose) {
        intake.setPower(intakePower);
        transfer.setPower(transferPower);
        flowerMech.setPosition(flowerMCur);
        gate.setPosition(gatePos);

        if (shooterOn) {
            shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, 0, 0, F));
            shooter.setVelocity(shooterSpeed);
        }
        else {
            shooter.setVelocity(0.0);
        }

        curPose = pose;
    }
}
