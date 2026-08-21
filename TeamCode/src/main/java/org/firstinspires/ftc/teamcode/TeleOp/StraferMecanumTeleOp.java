package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

/**
 * Field-centric mecanum TeleOp for a goBILDA Strafer chassis.
 * Live-tunable constants via PANELS (Configurables).
 *
 * Motor config names expected in RC config: lf, rf, lr, rr
 * IMU config name expected in RC config:     imu
 *
 * Set LogoFacingDirection / UsbFacingDirection below to match how the
 * Control/Expansion Hub is physically mounted on the robot.
 */
@TeleOp(name = "Strafer Mecanum TeleOp (IMU, PANELS)", group = "TeleOp")
public class StraferMecanumTeleOp extends LinearOpMode {

    @Configurable
    public static class Tuning {
        public static double DRIVE_SPEED = 1.0;
        public static double SLOW_MODE_MULTIPLIER = 0.35;
        public static double ROTATION_SPEED = 0.8;
        public static double INPUT_DEADZONE = 0.05;
        public static boolean FIELD_CENTRIC = true;

        // Per-wheel trim to correct drift. Start at 1.0, tune down the
        // wheel(s) on the side the robot drifts toward.
        public static double TRIM_FL = 1.0;
        public static double TRIM_FR = 1.0;
        public static double TRIM_BL = 1.0;
        public static double TRIM_BR = 1.0;
    }

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private IMU imu;
    private TelemetryManager panelsTelemetry;

    private boolean slowMode = false;
    private boolean lastLeftBumper = false;
    private boolean lastOptionsButton = false;

    @Override
    public void runOpMode() {

        frontLeft  = hardwareMap.get(DcMotorEx.class, "lf");
        frontRight = hardwareMap.get(DcMotorEx.class, "rf");
        backLeft   = hardwareMap.get(DcMotorEx.class, "lr");
        backRight  = hardwareMap.get(DcMotorEx.class, "rr");

        // Reverse left side (standard goBILDA Strafer orientation)
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotorEx m : new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        panelsTelemetry.debug("Status", "Initialized. Waiting for start.");
        panelsTelemetry.update(telemetry);

        waitForStart();
        imu.resetYaw();

        while (opModeIsActive()) {

            // ---- Inputs ----
            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x;

            if (Math.abs(y) < Tuning.INPUT_DEADZONE) y = 0;
            if (Math.abs(x) < Tuning.INPUT_DEADZONE) x = 0;
            if (Math.abs(rx) < Tuning.INPUT_DEADZONE) rx = 0;

            // Slow mode toggle - left bumper, rising edge
            boolean leftBumper = gamepad1.left_bumper;
            if (leftBumper && !lastLeftBumper) {
                slowMode = !slowMode;
            }
            lastLeftBumper = leftBumper;

            // Reset field-centric heading - options button, rising edge
            boolean optionsButton = gamepad1.options;
            if (optionsButton && !lastOptionsButton) {
                imu.resetYaw();
            }
            lastOptionsButton = optionsButton;

            // ---- Field-centric rotation ----
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double heading = orientation.getYaw(AngleUnit.RADIANS);

            if (Tuning.FIELD_CENTRIC) {
                double cosA = Math.cos(-heading);
                double sinA = Math.sin(-heading);
                double rotX = x * cosA - y * sinA;
                double rotY = x * sinA + y * cosA;
                x = rotX;
                y = rotY;
            }

            rx *= Tuning.ROTATION_SPEED;

            // ---- Mecanum drive math ----
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            double frontLeftPower  = (y + x + rx) / denominator;
            double backLeftPower   = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower  = (y + x - rx) / denominator;

            double speedMultiplier = Tuning.DRIVE_SPEED * (slowMode ? Tuning.SLOW_MODE_MULTIPLIER : 1.0);

            frontLeft.setPower(frontLeftPower * speedMultiplier * Tuning.TRIM_FL);
            backLeft.setPower(backLeftPower * speedMultiplier * Tuning.TRIM_BL);
            frontRight.setPower(frontRightPower * speedMultiplier * Tuning.TRIM_FR);
            backRight.setPower(backRightPower * speedMultiplier * Tuning.TRIM_BR);

            // ---- Telemetry (PANELS) ----
            panelsTelemetry.debug("Slow Mode", slowMode);
            panelsTelemetry.debug("Field Centric", Tuning.FIELD_CENTRIC);
            panelsTelemetry.debug("Heading (deg)", orientation.getYaw(AngleUnit.DEGREES));
            panelsTelemetry.debug("FL Power", frontLeftPower * speedMultiplier * Tuning.TRIM_FL);
            panelsTelemetry.debug("FR Power", frontRightPower * speedMultiplier * Tuning.TRIM_FR);
            panelsTelemetry.debug("BL Power", backLeftPower * speedMultiplier * Tuning.TRIM_BL);
            panelsTelemetry.debug("BR Power", backRightPower * speedMultiplier * Tuning.TRIM_BR);
            panelsTelemetry.update(telemetry);
        }
    }
}