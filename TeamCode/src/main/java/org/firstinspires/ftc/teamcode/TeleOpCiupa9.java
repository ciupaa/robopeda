package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@TeleOp(name = "TeleOpCiupa9", group = "Robot")
//@Disabled
public class TeleOpCiupa9 extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor  fata_stanga   = null; //the left drivetrain motor
    public DcMotor  fata_dreapta  = null; //the right drivetrain motor
    public DcMotor  spate_stanga    = null;
    public DcMotor  spate_dreapta   = null;
    public DcMotor  motor_stanga         = null; //the arm motor
    public DcMotor  motor_glisiere        = null; //
    public CRServo  servoRotire           = null; //the active servoRotire servo
    public Servo    cleste            = null; //the cleste servo


    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 99.5 // This is the exact gear ratio of the gobilda 60rpm motor
                    * 10 // This is the external gear reduction
                    * 1/360.0; // we want ticks per degree, not per rotation


    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double cosSusBrat = 100 * ARM_TICKS_PER_DEGREE;
    final double cosJosBrat = 90 * ARM_TICKS_PER_DEGREE;
    final double SpecimenBrat = 90 * ARM_TICKS_PER_DEGREE;
    final double Hang = 110 * ARM_TICKS_PER_DEGREE;
    final double test = 10 * ARM_TICKS_PER_DEGREE;

    final double servoRetras = 0;
    final double servoTras = 1;
    final double cleste_score = 0.5;

    /* Variables to store the positions that the cleste should be set to when folding in, or folding out. */
    final double cleste_deschis   = 0.7;
    final double cleste_inchis  = 1;

    /* A number in degrees that the triggers can adjust the arm position by */


    /* Variables that are used to set the arm to a specific position */
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;


    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;


    double rotirePosition = (int)servoTras;


    final double LIFT_TICKS_PER_MM = 384.5 / 120.0; // Encoder ticks per mm for your specific motor and pulley setup

    final double LIFT_COLLAPSED = 0 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_LOW_BASKET = 0 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_HIGH_BASKET = 348 * LIFT_TICKS_PER_MM;
    double liftPosition = LIFT_COLLAPSED;

    final double LIFT_MAX_POSITION = (int) (348 * LIFT_TICKS_PER_MM); // Fully extended for 240mm slide


    double cycletime = 0;
    double looptime = 0;
    double oldtime = 0;

    double armLiftComp = 0;


    @Override
    public void runOpMode() {

        /* Define and Initialize Motors */
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("fata_stanga");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("spate_stanga");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("fata_dreapta");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("spate_dreapta");
        motor_glisiere = hardwareMap.dcMotor.get("motor_glisiere");
        motor_stanga   = hardwareMap.get(DcMotor.class, "motor_stanga"); //the arm motor


       /*
       we need to reverse the left side of the drivetrain so it doesn't turn when we ask all the
       drive motors to go forward.
        */

        fata_stanga.setDirection(DcMotor.Direction.REVERSE);
        spate_stanga.setDirection(DcMotor.Direction.REVERSE);

        /* Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to slow down
        much faster when it is coasting. This creates a much more controllable drivetrain. As the robot
        stops much quicker. */
        fata_stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fata_dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spate_stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spate_dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_glisiere.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior. BRAKE);

        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) motor_stanga).setCurrentAlert(5,CurrentUnit.AMPS);

        /* Before starting the motor_stanga. We'll make sure the TargetPosition is set to 0.
        Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
        If you do not have the encoder plugged into this motor, it will not run in this code. */
        motor_stanga.setTargetPosition(0);
        motor_stanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_stanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor_glisiere.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_glisiere.setTargetPosition(0);
        motor_glisiere.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_glisiere.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /* Define and initialize servos.*/
        servoRotire = hardwareMap.get(CRServo.class, "servoRotire");
        cleste  = hardwareMap.get(Servo.class, "cleste");

        /* Make sure that the servoRotire is off, and the cleste is folded in. */
      //  servoRotire.setPower(rotirePosition);


        cleste.setPosition(cleste_inchis);


        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();
        waitForStart();

        /* Run until the driver presses stop */
        while (opModeIsActive())

        {
            // Rotate the movement direction counter to the bot's rotation
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            /* Here we implement a set of if else statements to set our arm to different scoring positions.
            We check to see if a specific button is pressed, and then move the arm (and sometimes
            servoRotire and cleste) to match. For example, if we click the right bumper we want the robot
            to start collecting. So it moves the armPosition to the ARM_COLLECT position,
            it folds out the cleste to make sure it is in the correct orientation to servoRotire, and it
            turns the servoRotire on to the COLLECT mode.*/

            if(gamepad2.a) {
                cleste.setPosition(cleste_deschis);
            }
            else if (gamepad2.x) {
                cleste.setPosition(cleste_inchis);
            }

            if(gamepad2.y)
                servoRotire.setPower(servoTras);
            else if (gamepad2.b)
                servoRotire.setPower(servoRetras);

            if(gamepad2.dpad_left) {
                servoRotire.setPower(cleste_score);
                sleep(100);
                cleste.setPosition(cleste_deschis);

            }



            armPositionFudgeFactor = FUDGE_FACTOR * (gamepad2.right_trigger + (-gamepad2.left_trigger));




            if(gamepad2.dpad_up) {
                motor_stanga.setPower(cosSusBrat);
                motor_glisiere.setPower(LIFT_SCORING_IN_HIGH_BASKET);
            }
            if(gamepad2.dpad_down) {
                motor_glisiere.setPower(LIFT_SCORING_IN_LOW_BASKET);
                motor_stanga.setPower(cosJosBrat);
            }
            if(gamepad2.dpad_right) {
                motor_glisiere.setPower(LIFT_SCORING_IN_LOW_BASKET);
                motor_stanga.setPower(SpecimenBrat);
            }


// Apply lift compensation when arm is below 45 degrees
            if (armPosition < 45 * ARM_TICKS_PER_DEGREE) {
                armLiftComp = (0.25568 * liftPosition);
            } else {
                armLiftComp = 0;
            }

// Apply position limits
            if (armPosition > 250 * ARM_TICKS_PER_DEGREE) {
                armPosition = 250 * ARM_TICKS_PER_DEGREE; // Clamp to max position
            } else if (armPosition < ARM_COLLAPSED_INTO_ROBOT) {
                armPosition = ARM_COLLAPSED_INTO_ROBOT; // Clamp to min position
            }

// Set target position for the motor
            motor_stanga.setTargetPosition((int) (armPosition + armPositionFudgeFactor + armLiftComp));

// Set motor velocity (max speed)
            ((DcMotorEx) motor_stanga).setVelocity(3500); // Maximum velocity

// Set motor mode
            motor_stanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);

// Check for overcurrent condition and report via telemetry
            if (((DcMotorEx) motor_stanga).isOverCurrent()) {
                telemetry.addLine("BRAT EXCEEDED CURRENT LIMIT!");
            }


// Update lift position based on driver input
            if (gamepad2.right_bumper) {
                liftPosition += 2800 * cycletime; // Increased for faster movement
            } else if (gamepad2.left_bumper) {
                liftPosition -= 2800 * cycletime; // Increased for faster movement
            }

// Enforce limits
            // Enforce limits
            if (liftPosition > LIFT_MAX_POSITION) {
                liftPosition = LIFT_MAX_POSITION;
            }
            if (liftPosition < LIFT_COLLAPSED) {
                liftPosition = LIFT_COLLAPSED;
            }



// Set motor mode and target position
            motor_glisiere.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Ensure it's in the correct mode
            motor_glisiere.setTargetPosition((int) liftPosition);

// Set motor velocity (ticks per second)
            ((DcMotorEx) motor_glisiere).setVelocity(2800); // Adjust for desired speed


            servoRotire.setPower(gamepad2.left_stick_y);


            looptime = getRuntime();
            cycletime = looptime-oldtime;
            oldtime = looptime;


            /* send telemetry to the driver of the arm's current position and target position */
            telemetry.addData("arm Target Position: ", motor_stanga.getTargetPosition());
            telemetry.addData("arm Encoder: ", motor_stanga.getCurrentPosition());
            telemetry.addData("lift variable", liftPosition);
            telemetry.addData("Lift Target Position",motor_glisiere.getTargetPosition());
            telemetry.addData("lift current position", motor_glisiere.getCurrentPosition());
            telemetry.addData("motor_glisiere Current:",((DcMotorEx) motor_glisiere).getCurrent(CurrentUnit.AMPS));
            telemetry.update();



        }
    }
}
