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
 * Heading comes from the Control/Expansion Hub IMU. Position for save/return
 * is dead-reckoned from the four drive motor encoders + IMU heading - there
 * is no dedicated odometry hardware, so this drifts with wheel slip. Good
 * enough for "get back roughly where I was", not competition-grade odometry.
 * Live-tunable constants via PANELS (Configurables).
 *
 * Motor config names expected in RC config: lf, rf, lr, rr
 * IMU config name expected in RC config:     imu
 *
 * SAVE / RETURN TO POSITION
 *   Y (rising edge) - save current estimated field position + heading
 *   A (held)         - drive back toward the saved position/heading;
 *                       releasing A hands control back to the sticks
 *   Panels toggle     - Tuning.RETURN_TO_SAVE_ENABLED disables the whole
 *                       feature; A does nothing when false
 *   OPTIONS           - re-zero heading AND position origin, clears any
 *                       saved position (it was relative to the old origin)
 *
 * WHEEL TEST MODE (for diagnosing strafe/swerve issues)
 *   DPAD_UP  - enter test mode
 *   dpad_up/right/down/left - spin FL/FR/BR/BL only, one at a time
 *   START     - exit test mode
 *
 * TODO before trusting save/return distances: set Tuning.TICKS_PER_MOTOR_REV,
 * Tuning.DRIVE_GEAR_REDUCTION, and Tuning.WHEEL_DIAMETER_MM to match your
 * actual motors/wheels. Wrong values scale every tracked distance wrong even
 * though driving itself still feels normal.
 */
@TeleOp(name = "Mecanum Field-Centric TeleOp v1", group = "TeleOp")
public class MecanumFieldCentricTeleOpV1 extends LinearOpMode {
    @Configurable
    public static class Tuning {
        public static double DRIVE_SPEED = 1.0;
        public static double SLOW_MODE_MULTIPLIER = 0.35;
        public static double ROTATION_SPEED = 0.8;
        public static double INPUT_DEADZONE = 0.05;
        public static boolean FIELD_CENTRIC = true;
        public static double TEST_MODE_POWER = 0.3;
        // Per-wheel trim to correct drift once wiring/wheel placement is confirmed correct.
        public static double TRIM_FL = 1.0;
        public static double TRIM_FR = 1.0;
        public static double TRIM_BL = 1.0;
        public static double TRIM_BR = 1.0;
        // Save / return to position
        public static boolean RETURN_TO_SAVE_ENABLED = true;
        public static double RETURN_TRANSLATION_KP = 0.006; // power per mm of position error
        public static double RETURN_HEADING_KP = 1.2;        // power per radian of heading error
        public static double RETURN_MAX_POWER = 0.6;
        public static double RETURN_POSITION_TOLERANCE_MM = 0.5;
        public static double RETURN_HEADING_TOLERANCE_DEG = 5.0;
        // Drive-encoder odometry calibration - MUST match your hardware.
        public static double TICKS_PER_MOTOR_REV = 537.7; // goBILDA 5203 312 RPM default
        public static double DRIVE_GEAR_REDUCTION = 1.0;   // external gearing beyond the motor, if any
        public static double WHEEL_DIAMETER_MM = 104;     // goBILDA mecanum wheel default
    }
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private IMU imu;
    private TelemetryManager panelsTelemetry;
    private boolean slowMode = false;
    private boolean testMode = false;
    private boolean lastLeftBumper = false;
    private boolean lastOptionsButton = false;
    private boolean lastDpadUp = false;
    private boolean lastStart = false;
    private boolean lastYButton = false;
    private boolean savedPositionValid = false;
    private double savedX_mm = 0, savedY_mm = 0, savedHeadingDeg = 0;
    // Dead-reckoned field position estimate, driven off encoder deltas.
    private double posX_mm = 0, posY_mm = 0;
    private double lastFLTicks = 0, lastFRTicks = 0, lastBLTicks = 0, lastBRTicks = 0;
    @Override
    public void runOpMode() {
        frontLeft  = hardwareMap.get
                // Left Front
                        (DcMotorEx.class, "lf");
        frontRight = hardwareMap.get
                //Right Front
                        (DcMotorEx.class, "rf");
        backLeft   = hardwareMap.get
                //left Rear
                        (DcMotorEx.class, "lr");
        backRight  = hardwareMap.get
                //Right Rear
                        (DcMotorEx.class, "rr");
        // Left side get reversed
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotorEx m : new DcMotorEx[]
                {frontLeft, frontRight, backLeft, backRight}) {
            m.setZeroPowerBehavior
                    (DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode
                    (DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        ));
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        panelsTelemetry.debug
                ("Status", "Initialized. Waiting for start.");
        panelsTelemetry.update(telemetry);
        waitForStart();
        // Moto r Position
        imu.resetYaw();
        lastFLTicks =
                frontLeft.getCurrentPosition();
        lastFRTicks =
                frontRight.getCurrentPosition();
        lastBLTicks =
                backLeft.getCurrentPosition();
        lastBRTicks =
                backRight.getCurrentPosition();
        while (opModeIsActive()) {
            // Test mode toggle (dpad_up rising edge, only when not already testing)
            boolean dpadUp = gamepad1.dpad_up;
            if (dpadUp && !lastDpadUp && !testMode) {
                testMode = true;
            }
            lastDpadUp = dpadUp;
            boolean startBtn = gamepad1.start;
            if (startBtn && !lastStart) {
                testMode = false;
            }
            lastStart = startBtn;
            if (testMode) {
                runWheelTestMode();
                continue;
            }
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double curHeadingRad = orientation.getYaw(AngleUnit.RADIANS);
            double curHeadingDeg = orientation.getYaw(AngleUnit.DEGREES);
            updateOdometry(curHeadingRad);
            // Re-zero heading + position origin - clears any saved position
            boolean optionsButton = gamepad1.options;
            if (optionsButton && !lastOptionsButton) {
                imu.resetYaw();
                posX_mm = 0;
                posY_mm = 0;
                savedPositionValid = false;
            }
            lastOptionsButton = optionsButton;
            // Save current position - Y, rising edge
            boolean yButton = gamepad1.y;
            if (yButton && !lastYButton) {
                savedX_mm = posX_mm;
                savedY_mm = posY_mm;
                savedHeadingDeg = curHeadingDeg;
                savedPositionValid = true;
            }
            lastYButton = yButton;
            boolean returning = Tuning.RETURN_TO_SAVE_ENABLED && gamepad1.a && savedPositionValid;
            if (returning) {
                driveReturnToSave(curHeadingRad, curHeadingDeg);
            } else {
                driveManual(curHeadingRad);
            }
            // Telemetry (PANELS)
            panelsTelemetry.debug("Robot is Ready");
            panelsTelemetry.debug("Mode", returning ? "RETURN TO SAVE" : "DRIVE");
            panelsTelemetry.debug("Slow Mode", slowMode);
            panelsTelemetry.debug("Field Centric", Tuning.FIELD_CENTRIC);
            panelsTelemetry.debug("Return Enabled", Tuning.RETURN_TO_SAVE_ENABLED);
            panelsTelemetry.debug("Heading (deg)", curHeadingDeg);
            panelsTelemetry.debug("Est Pos X (mm)", posX_mm);
            panelsTelemetry.debug("Est Pos Y (mm)", posY_mm);
            panelsTelemetry.debug("Saved Valid", savedPositionValid);
            if (savedPositionValid) {
                panelsTelemetry.debug("Saved X (mm)", savedX_mm);
                panelsTelemetry.debug("Saved Y (mm)", savedY_mm);
                panelsTelemetry.debug("Saved Heading (deg)", savedHeadingDeg);
                double dist = Math.hypot(savedX_mm - posX_mm, savedY_mm - posY_mm);
                panelsTelemetry.debug("Dist To Saved (mm)", dist);
            }
            panelsTelemetry.update(telemetry);
        }
    }
    /**
     * Dead-reckons field position from drive encoder deltas each loop.
     * Mecanum forward kinematics (robot-relative), rotated into field frame
     * using the current IMU heading. Accumulates error from wheel slip.
     */
    private void updateOdometry(double headingRad) {
        double flTicks = frontLeft.getCurrentPosition();
        double frTicks = frontRight.getCurrentPosition();
        double blTicks = backLeft.getCurrentPosition();
        double brTicks = backRight.getCurrentPosition();
        double dFL = flTicks - lastFLTicks;
        double dFR = frTicks - lastFRTicks;
        double dBL = blTicks - lastBLTicks;
        double dBR = brTicks - lastBRTicks;
        lastFLTicks = flTicks;
        lastFRTicks = frTicks;
        lastBLTicks = blTicks;
        lastBRTicks = brTicks;
        double ticksPerMM = (Tuning.TICKS_PER_MOTOR_REV * Tuning.DRIVE_GEAR_REDUCTION)
                / (Tuning.WHEEL_DIAMETER_MM * Math.PI);
        if (ticksPerMM <= 0) return;
        double dFL_mm = dFL / ticksPerMM;
        double dFR_mm = dFR / ticksPerMM;
        double dBL_mm = dBL / ticksPerMM;
        double dBR_mm = dBR / ticksPerMM;
        // Inverse of the drive mix used in applyDrivePowers().
        double axialDelta_mm   = (dFL_mm + dFR_mm + dBL_mm + dBR_mm) / 4.0;
        double lateralDelta_mm = (dFL_mm - dFR_mm - dBL_mm + dBR_mm) / 4.0;
        // Rotate robot-relative delta into the field frame (inverse of rotateFieldToRobot).
        double fieldDX = lateralDelta_mm * Math.cos(headingRad) - axialDelta_mm * Math.sin(headingRad);
        double fieldDY = lateralDelta_mm * Math.sin(headingRad) + axialDelta_mm * Math.cos(headingRad);
        posX_mm += fieldDX;
        posY_mm += fieldDY;
    }
    /** Normal stick-driven mecanum control. */
    private void driveManual(double curHeadingRad) {
        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x;
        double rx =  gamepad1.right_stick_x;
        if (Math.abs(y) < Tuning.INPUT_DEADZONE) y = 0;
        if (Math.abs(x) < Tuning.INPUT_DEADZONE) x = 0;
        if (Math.abs(rx) < Tuning.INPUT_DEADZONE) rx = 0;
        boolean leftBumper = gamepad1.left_bumper;
        if (leftBumper && !lastLeftBumper) {
            slowMode = !slowMode;
        }
        lastLeftBumper = leftBumper;
        rx *= Tuning.ROTATION_SPEED;
        if (Tuning.FIELD_CENTRIC) {
            double[] rotated = rotateFieldToRobot(x, y, curHeadingRad);
            x = rotated[0];
            y = rotated[1];
        }
        double speedMultiplier = Tuning.DRIVE_SPEED * (slowMode ? Tuning.SLOW_MODE_MULTIPLIER : 1.0);
        applyDrivePowers(y, x, rx, speedMultiplier);
    }
    /** Proportional drive back to the saved field position + heading. */
    private void driveReturnToSave(double curHeadingRad, double curHeadingDeg) {
        double dx = savedX_mm - posX_mm;
        double dy = savedY_mm - posY_mm;
        double distance = Math.hypot(dx, dy);
        double headingErrorDeg = normalizeAngleDeg(savedHeadingDeg - curHeadingDeg);
        boolean arrived = distance < Tuning.RETURN_POSITION_TOLERANCE_MM
                && Math.abs(headingErrorDeg) < Tuning.RETURN_HEADING_TOLERANCE_DEG;
        double fieldX, fieldY, rx;
        if (arrived) {
            fieldX = 0;
            fieldY = 0;
            rx = 0;
        } else {
            fieldX = clamp(dx * Tuning.RETURN_TRANSLATION_KP, -Tuning.RETURN_MAX_POWER, Tuning.RETURN_MAX_POWER);
            fieldY = clamp(dy * Tuning.RETURN_TRANSLATION_KP, -Tuning.RETURN_MAX_POWER, Tuning.RETURN_MAX_POWER);
            double headingErrorRad = Math.toRadians(headingErrorDeg);
            rx = clamp(headingErrorRad * Tuning.RETURN_HEADING_KP, -Tuning.RETURN_MAX_POWER, Tuning.RETURN_MAX_POWER);
        }
        // Return targets are field-relative, so always rotate into robot frame
        // regardless of the manual-drive FIELD_CENTRIC toggle.
        double[] rotated = rotateFieldToRobot(fieldX, fieldY, curHeadingRad);
        applyDrivePowers(rotated[1], rotated[0], rx, 1.0);
    }
    /** Rotates a field-relative (x, y) vector into the robot's frame using the given heading. */
    private double[] rotateFieldToRobot(double fx, double fy, double headingRad) {
        double cosA = Math.cos(-headingRad);
        double sinA = Math.sin(-headingRad);
        double rotX = fx * cosA - fy * sinA;
        double rotY = fx * sinA + fy * cosA;
        return new double[]{rotX, rotY};
    }
    /** Standard FTC mecanum formula, trims, and motor output. x/y/rx are already robot-relative. */
    private void applyDrivePowers(double y, double x, double rx, double speedMultiplier) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        double frontLeftPower  = (y + x + rx) / denominator;
        double backLeftPower   = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower  = (y + x - rx) / denominator;
        frontLeft.setPower(frontLeftPower * speedMultiplier * Tuning.TRIM_FL);
        backLeft.setPower(backLeftPower * speedMultiplier * Tuning.TRIM_BL);
        frontRight.setPower(frontRightPower * speedMultiplier * Tuning.TRIM_FR);
        backRight.setPower(backRightPower * speedMultiplier * Tuning.TRIM_BR);
    }
    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
    private double normalizeAngleDeg(double deg) {
        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }
    /**
     * Spins one wheel at a time so you can visually confirm every wheel
     * rotates the robot in the same rotational sense. Press START to exit.
     */
    private void runWheelTestMode() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        String active = "none";
        if (gamepad1.dpad_up) {
            frontLeft.setPower(Tuning.TEST_MODE_POWER);
            active = "FL";
        } else if (gamepad1.dpad_right) {
            frontRight.setPower(Tuning.TEST_MODE_POWER);
            active = "FR";
        } else if (gamepad1.dpad_down) {
            backRight.setPower(Tuning.TEST_MODE_POWER);
            active = "BR";
        } else if (gamepad1.dpad_left) {
            backLeft.setPower(Tuning.TEST_MODE_POWER);
            active = "BL";
        }
        panelsTelemetry.debug("Mode", "WHEEL TEST - press START to exit");
        panelsTelemetry.debug("Active Wheel", active);
        panelsTelemetry.debug("dpad_up=FL  dpad_right=FR  dpad_down=BR  dpad_left=BL", "");
        panelsTelemetry.update(telemetry);
    }
}