package org.firstinspires.ftc.teamcode.arm;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.hardware.motors.NeveRest60Gearmotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

/*
 * Hardware class for a rotary arm (for linearly-actuated mechanisms, see Elevator).
 */
@Config
public class Arm {
    public static final MotorConfigurationType MOTOR_CONFIG =
            MotorConfigurationType.getMotorType(NeveRest60Gearmotor.class);
    private static final double TICKS_PER_REV = MOTOR_CONFIG.getTicksPerRev();

    public static double GEAR_RATIO = 6; // output/input

    // the operating range of the arm is [0, MAX_ANGLE]
    public static double MAX_ANGLE = 120f; // rad

    public static PIDCoefficients PID = new PIDCoefficients(0, 0, 0);

    public static double MAX_VEL = Math.PI / 8; // rad/s
    public static double MAX_ACCEL = Math.PI / 8; // rad/s^2
    public static double MAX_JERK = Math.PI / 8; // rad/s^3

    public static double kV = 0.01f;
    public static double kA = 0;
    public static double kStatic = 0;


    private DcMotorEx motor1,motor2;
    private PIDFController controller;
    private MotionProfile profile;
    private NanoClock clock = NanoClock.system();
    private double profileStartTime, desiredAngle = 0;
    private int offset;


    private static double encoderTicksToRadians(int ticks) {
        return 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public Arm(HardwareMap hardwareMap) {
        motor1 = hardwareMap.get(DcMotorEx.class, "arm_right");
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2 = hardwareMap.get(DcMotorEx.class, "arm_left");
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // if necessary, reverse the motor so CCW is positive
        motor2.setDirection(DcMotor.Direction.REVERSE);

        // motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // note: if the arm is affected by a non-negligible constant force along the direction
        // of motion (e.g., gravity, kinetic friction, or a combination thereof), it may be
        // beneficial to compensate for it here (assuming no velocity PID) like so:
        // e.g., controller = new PIDFController(PID, kV, kA, kStatic,
        //                                       angle -> kA * 9.81 * Math.cos(angle - VERT_ANGLE));
        controller = new PIDFController(PID, kV, kA, kStatic);
        offset = motor1.getCurrentPosition();
    }

    public boolean isBusy() {
        return profile != null && (clock.seconds() - profileStartTime) <= profile.duration();
    }

    public void setAngle(double angle) {
        angle = Math.min(Math.max(0, angle), MAX_ANGLE);

        double time = clock.seconds() - profileStartTime;
        MotionState start = isBusy() ? profile.get(time) : new MotionState(desiredAngle, 0, 0, 0);
        MotionState goal = new MotionState(angle, 0, 0, 0);
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                start, goal, MAX_VEL, MAX_ACCEL, MAX_JERK
        );
        profileStartTime = clock.seconds();

        this.desiredAngle = angle;
    }

    public double getCurrentAngle() {
        return encoderTicksToRadians(motor1.getCurrentPosition() - offset);
    }

    public void update() {
        double power;
        double currentAngle = getCurrentAngle();
        if (isBusy()) {
            // following a profile
            double time = clock.seconds() - profileStartTime;
            MotionState state = profile.get(time);
            controller.setTargetPosition(state.getX());
            power = controller.update(currentAngle, state.getV(), state.getA());
        } else {
            // just hold the position
            controller.setTargetPosition(desiredAngle);
            power = controller.update(currentAngle);
        }
        setPower(power);
    }

    public void setPower(double power)
    {
        motor1.setPower(power);
        motor2.setPower(power);
    }

}
