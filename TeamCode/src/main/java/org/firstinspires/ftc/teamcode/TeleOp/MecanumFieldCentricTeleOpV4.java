package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * V4 - stripped down, no Panels, single IMU. Just field-centric mecanum
 * drive + basic save/return-to-position. Everything is a plain constant at
 * the top - edit and redeploy to tune, no live dashboard.
 *
 * FIX vs earlier versions: driving was inverted (forward = backward).
 * The forward/back stick read has been un-negated below - see the comment
 * on that line. Strafing (x) was left untouched since you said it was
 * already correct. If forward/back is STILL backwards after this, see the
 * note right there for the 10-second alternate fix.
 *
 * Motor config names expected in RC config: lf, rf, lr, rr
 * IMU config name expected in RC config:     imu
 *
 * SAVE / RETURN TO POSITION
 *   Y (rising edge) - save current estimated field position + heading
 *   A (held)         - drive back toward the saved position/heading;
 *                       releasing A hands control back to the sticks
 *   OPTIONS           - re-zero heading AND position origin, clears any
 *                       saved position (it was relative to the old origin)
 *
 * WHEEL TEST MODE (diagnosing strafe/swerve issues)
 *   DPAD_UP  - enter test mode
 *   dpad_up/right/down/left - spin FL/FR/BR/BL only, one at a time
 *   START     - exit test mode
 *
 * Position tracking is dead-reckoned from drive encoders + IMU heading -
 * there's no dedicated odometry hardware, so it drifts with wheel slip.
 * Good enough to get back roughly where you were, not competition-grade.
 *
 * TODO: set TICKS_PER_MOTOR_REV, DRIVE_GEAR_REDUCTION, WHEEL_DIAMETER_MM
 * below to match your actual motors/wheels or save/return distances will
 * be wrong even though normal driving feels fine.
 */
@TeleOp(name = "Mecanum Field-Centric TeleOp v4", group = "TeleOp")
public class MecanumFieldCentricTeleOpV4 extends LinearOpMode {

    // ---- Constants - edit values here and redeploy to tune, no Panels ----
    private static final double DRIVE_SPEED = 1.0;
    private static final double SLOW_MODE_MULTIPLIER = 0.35;
    private static final double ROTATION_SPEED = 0.8;
    private static final double INPUT_DEADZONE = 0.05;
    private static final boolean FIELD_CENTRIC = true;
    private static final double TEST_MODE_POWER = 0.3;

    // Speeding UP is ramped (limits wheel slip / current spikes on quick starts).
    // Slowing DOWN / stopping is NOT ramped - full instant response, backed by
    // ZeroPowerBehavior.BRAKE below, so the robot stops as crisply as the
    // hardware allows the instant you let go of the stick.
    private static final double ACCEL_SLEW_PER_SEC = 6.0; // max power increase per wheel per second

    // Per-wheel trim, once wiring/wheel placement is confirmed correct.
    private static final double TRIM_FL = 1.0;
    private static final double TRIM_FR = 1.0;
    private static final double TRIM_BL = 1.0;
    private static final double TRIM_BR = 1.0;

    // Save / return to position
    private static final boolean RETURN_TO_SAVE_ENABLED = true;
    private static final double RETURN_TRANSLATION_KP = 0.006; // power per mm of position error
    private static final double RETURN_HEADING_KP = 1.2;        // power per radian of heading error
    private static final double RETURN_MAX_POWER = 0.6;
    private static final double RETURN_MIN_POWER = 0.12; // floor applied to any nonzero correction so it can beat static friction instead of stalling just short of the target
    private static final double RETURN_POSITION_TOLERANCE_MM = 0.5;
    private static final double RETURN_HEADING_TOLERANCE_DEG = 5.0;
    private static final double RETURN_TIMEOUT_SEC = 4.0; // give up and cut power if it hasn't arrived by then (e.g. physically blocked) instead of stalling the motors indefinitely

    // Drive-encoder odometry calibration - MUST match your hardware.
    private static final double TICKS_PER_MOTOR_REV = 537.7; // goBILDA 5203 312 RPM default
    private static final double DRIVE_GEAR_REDUCTION = 1.0;   // external gearing beyond the motor, if any
    private static final double WHEEL_DIAMETER_MM = 104;     // goBILDA mecanum wheel default

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

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

    // Per-wheel last commanded power, for accel-slew limiting in applyDrivePowers().
    private double lastFLPower = 0, lastFRPower = 0, lastBLPower = 0, lastBRPower = 0;
    private final ElapsedTime driveSlewTimer = new ElapsedTime();

    // Return-to-position timeout tracking.
    private boolean wasReturning = false;
    private final ElapsedTime returnTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        frontLeft  = hardwareMap.get(DcMotorEx.class, "lf"); // Left Front
        frontRight = hardwareMap.get(DcMotorEx.class, "rf"); // Right Front
        backLeft   = hardwareMap.get(DcMotorEx.class, "lr"); // Left Rear
        backRight  = hardwareMap.get(DcMotorEx.class, "rr"); // Right Rear

        // Left side gets reversed. If forward/back is STILL inverted after
        // this file's fix below, put the negative sign back on y in
        // driveManual() and instead swap these two lines to reverse
        // frontRight/backRight instead of frontLeft/backLeft - that means
        // your wiring is mirrored from the goBILDA default assumption.
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotorEx m : new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        ));

        telemetry.addData("Status", "Initialized. Waiting for start.");
        telemetry.update();

        waitForStart();

        imu.resetYaw();
        lastFLTicks = frontLeft.getCurrentPosition();
        lastFRTicks = frontRight.getCurrentPosition();
        lastBLTicks = backLeft.getCurrentPosition();
        lastBRTicks = backRight.getCurrentPosition();
        driveSlewTimer.reset();

        while (opModeIsActive()) {
            // Test mode toggle (dpad_up rising edge, only when not already testing)
            boolean dpadUp = gamepad1.dpad_up;
            if (dpadUp && !lastDpadUp && !testMode) {
                testMode = true;
                lastFLPower = 0; lastFRPower = 0; lastBLPower = 0; lastBRPower = 0;
            }
            lastDpadUp = dpadUp;

            boolean startBtn = gamepad1.start;
            if (startBtn && !lastStart) {
                testMode = false;
                driveSlewTimer.reset(); // avoid a huge dt (and thus no slew limiting) on the first frame back
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

            boolean returning = RETURN_TO_SAVE_ENABLED && gamepad1.a && savedPositionValid;
            if (returning && !wasReturning) {
                returnTimer.reset(); // just started returning - give it a fresh RETURN_TIMEOUT_SEC window
            }
            wasReturning = returning;
            if (returning) {
                driveReturnToSave(curHeadingRad, curHeadingDeg);
            } else {
                driveManual(curHeadingRad);
            }

            telemetry.addData("Mode", returning ? "RETURN TO SAVE" : "DRIVE");
            if (returning) {
                telemetry.addData("Return Timed Out", returnTimer.seconds() > RETURN_TIMEOUT_SEC);
            }
            telemetry.addData("Slow Mode", slowMode);
            telemetry.addData("Heading (deg)", curHeadingDeg);
            telemetry.addData("Est Pos X (mm)", posX_mm);
            telemetry.addData("Est Pos Y (mm)", posY_mm);
            telemetry.addData("Saved Valid", savedPositionValid);
            if (savedPositionValid) {
                telemetry.addData("Saved X (mm)", savedX_mm);
                telemetry.addData("Saved Y (mm)", savedY_mm);
                telemetry.addData("Saved Heading (deg)", savedHeadingDeg);
                double dist = Math.hypot(savedX_mm - posX_mm, savedY_mm - posY_mm);
                telemetry.addData("Dist To Saved (mm)", dist);
            }
            telemetry.update();
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

        double ticksPerMM = (TICKS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_MM * Math.PI);
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
        // FIX: was "-gamepad1.left_stick_y" - that negation was making
        // forward push drive the robot backward. Un-negated here. Strafe
        // (x) and rotation (rx) are untouched, as requested.
        double y  = gamepad1.left_stick_y;
        double x  = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        if (Math.abs(y) < INPUT_DEADZONE) y = 0;
        if (Math.abs(x) < INPUT_DEADZONE) x = 0;
        if (Math.abs(rx) < INPUT_DEADZONE) rx = 0;

        boolean leftBumper = gamepad1.left_bumper;
        if (leftBumper && !lastLeftBumper) {
            slowMode = !slowMode;
        }
        lastLeftBumper = leftBumper;

        rx *= ROTATION_SPEED;

        if (FIELD_CENTRIC) {
            double[] rotated = rotateFieldToRobot(x, y, curHeadingRad);
            x = rotated[0];
            y = rotated[1];
        }

        double speedMultiplier = DRIVE_SPEED * (slowMode ? SLOW_MODE_MULTIPLIER : 1.0);
        applyDrivePowers(y, x, rx, speedMultiplier);
    }

    /**
     * Proportional drive back to the saved field position + heading.
     * Two accuracy/safety additions over a plain P controller:
     *  - RETURN_MIN_POWER floors any nonzero correction so a small residual
     *    error still produces enough power to overcome static friction,
     *    instead of the robot stalling asymptotically just outside
     *    tolerance and never actually arriving.
     *  - RETURN_TIMEOUT_SEC force-arrives (cuts power) if it's been trying
     *    for too long - e.g. physically boxed in by another robot - so the
     *    drivetrain doesn't sit there stalling against an obstacle.
     */
    private void driveReturnToSave(double curHeadingRad, double curHeadingDeg) {
        double dx = savedX_mm - posX_mm;
        double dy = savedY_mm - posY_mm;
        double distance = Math.hypot(dx, dy);
        double headingErrorDeg = normalizeAngleDeg(savedHeadingDeg - curHeadingDeg);

        boolean withinTolerance = distance < RETURN_POSITION_TOLERANCE_MM
                && Math.abs(headingErrorDeg) < RETURN_HEADING_TOLERANCE_DEG;
        boolean timedOut = returnTimer.seconds() > RETURN_TIMEOUT_SEC;
        boolean arrived = withinTolerance || timedOut;

        double fieldX, fieldY, rx;
        if (arrived) {
            fieldX = 0;
            fieldY = 0;
            rx = 0;
        } else {
            fieldX = applyMinPower(clamp(dx * RETURN_TRANSLATION_KP, -RETURN_MAX_POWER, RETURN_MAX_POWER));
            fieldY = applyMinPower(clamp(dy * RETURN_TRANSLATION_KP, -RETURN_MAX_POWER, RETURN_MAX_POWER));
            double headingErrorRad = Math.toRadians(headingErrorDeg);
            rx = applyMinPower(clamp(headingErrorRad * RETURN_HEADING_KP, -RETURN_MAX_POWER, RETURN_MAX_POWER));
        }

        // Return targets are field-relative, so always rotate into robot frame
        // regardless of the manual-drive FIELD_CENTRIC toggle.
        double[] rotated = rotateFieldToRobot(fieldX, fieldY, curHeadingRad);
        applyDrivePowers(rotated[1], rotated[0], rx, 1.0);
    }

    /** Floors a nonzero correction to RETURN_MIN_POWER (preserving sign) so it can beat static friction. */
    private double applyMinPower(double power) {
        if (power == 0) return 0;
        if (Math.abs(power) < RETURN_MIN_POWER) {
            return Math.copySign(RETURN_MIN_POWER, power);
        }
        return power;
    }

    /** Rotates a field-relative (x, y) vector into the robot's frame using the given heading. */
    private double[] rotateFieldToRobot(double fx, double fy, double headingRad) {
        double cosA = Math.cos(-headingRad);
        double sinA = Math.sin(-headingRad);
        double rotX = fx * cosA - fy * sinA;
        double rotY = fx * sinA + fy * cosA;
        return new double[]{rotX, rotY};
    }

    /**
     * Standard FTC mecanum formula, trims, and motor output. x/y/rx are
     * already robot-relative.
     *
     * Braking behavior: speeding up is slew-limited (ACCEL_SLEW_PER_SEC) to
     * cut wheel slip and current spikes on quick starts. Slowing down,
     * stopping, or reversing direction is never slew-limited - those go
     * straight to the target power so the stop is as crisp as possible,
     * and ZeroPowerBehavior.BRAKE (set at init) makes the hub actively
     * resist motion once power is commanded to zero rather than coasting.
     */
    private void applyDrivePowers(double y, double x, double rx, double speedMultiplier) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        double frontLeftPower  = (y + x + rx) / denominator;
        double backLeftPower   = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower  = (y + x - rx) / denominator;

        double dt = driveSlewTimer.seconds();
        driveSlewTimer.reset();
        if (dt <= 0) dt = 1e-3;
        double maxIncrease = ACCEL_SLEW_PER_SEC * dt;

        double flTarget = frontLeftPower * speedMultiplier * TRIM_FL;
        double frTarget = frontRightPower * speedMultiplier * TRIM_FR;
        double blTarget = backLeftPower * speedMultiplier * TRIM_BL;
        double brTarget = backRightPower * speedMultiplier * TRIM_BR;

        lastFLPower = slewAccelOnly(lastFLPower, flTarget, maxIncrease);
        lastFRPower = slewAccelOnly(lastFRPower, frTarget, maxIncrease);
        lastBLPower = slewAccelOnly(lastBLPower, blTarget, maxIncrease);
        lastBRPower = slewAccelOnly(lastBRPower, brTarget, maxIncrease);

        frontLeft.setPower(lastFLPower);
        backLeft.setPower(lastBLPower);
        frontRight.setPower(lastFRPower);
        backRight.setPower(lastBRPower);
    }

    /**
     * Limits how fast power can INCREASE in magnitude; any decrease
     * (slowing down, stopping, or reversing direction) is passed through
     * immediately so braking is never artificially delayed.
     */
    private double slewAccelOnly(double last, double target, double maxIncrease) {
        boolean speedingUp = Math.abs(target) > Math.abs(last) && Math.signum(target) == Math.signum(last);
        if (!speedingUp) {
            return target; // decelerating, stopping, or changing direction - let it happen instantly
        }
        double delta = clamp(target - last, -maxIncrease, maxIncrease);
        return last + delta;
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
            frontLeft.setPower(TEST_MODE_POWER);
            active = "FL";
        } else if (gamepad1.dpad_right) {
            frontRight.setPower(TEST_MODE_POWER);
            active = "FR";
        } else if (gamepad1.dpad_down) {
            backRight.setPower(TEST_MODE_POWER);
            active = "BR";
        } else if (gamepad1.dpad_left) {
            backLeft.setPower(TEST_MODE_POWER);
            active = "BL";
        }

        telemetry.addData("Mode", "WHEEL TEST - press START to exit");
        telemetry.addData("Active Wheel", active);
        telemetry.addData("dpad_up=FL  dpad_right=FR  dpad_down=BR  dpad_left=BL", "");
        telemetry.update();
    }
}