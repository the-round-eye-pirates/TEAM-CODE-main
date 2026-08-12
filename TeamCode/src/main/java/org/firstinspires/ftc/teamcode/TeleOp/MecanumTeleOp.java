package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Mecanum TeleOp", group = "TeleOp")
public class MecanumTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor lf = hardwareMap.dcMotor.get("lf");
        DcMotor lr = hardwareMap.dcMotor.get("lr");
        DcMotor rf = hardwareMap.dcMotor.get("rf");
        DcMotor rr = hardwareMap.dcMotor.get("rr");

        // Reverse right side
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rr.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(
                    Math.abs(y) + Math.abs(x) + Math.abs(rx),
                    1
            );

            double lfPower = (y + x + rx) / denominator;
            double lrPower = (y - x + rx) / denominator;
            double rfPower = (y - x - rx) / denominator;
            double rrPower = (y + x - rx) / denominator;

            lf.setPower(lfPower);
            lr.setPower(lrPower);
            rf.setPower(rfPower);
            rr.setPower(rrPower);
        }
    }
}