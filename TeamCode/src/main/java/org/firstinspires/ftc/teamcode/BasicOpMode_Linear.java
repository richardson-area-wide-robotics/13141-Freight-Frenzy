/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Teleop", group="Robot code")
public class BasicOpMode_Linear extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor backleft = null;
    private DcMotor backright = null;
    private DcMotor frontright = null;
    private DcMotor frontleft = null;
    private DcMotor spinner = null;
    private DcMotor arm = null;
    private DcMotor intake = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        backleft  = hardwareMap.get(DcMotor.class, "BackLeft");
        backright = hardwareMap.get(DcMotor.class, "BackRight");
        frontleft = hardwareMap.get(DcMotor.class, "FrontLeft");
        frontright = hardwareMap.get(DcMotor.class, "FrontRight");
        arm = hardwareMap.get(DcMotor.class, "Arm");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        spinner = hardwareMap.get(DcMotor.class, "Spinner");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1.0);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        backright.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        
        boolean pressedRightTriggerIteration = false;
        boolean pressedLeftTriggerIteration = true;
        double intakePower = 0.0;
        int armPosition = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            // Setup a variable for each drive wheel to save power level for telemetry

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double y = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontleftpower = (y + x + rx) / denominator;
            double backleftpower = (y - x + rx) / denominator;
            double frontrightpower = (y - x - rx) / denominator;
            double backrightpower = (y + x - rx) / denominator;

            // Send calculated power to wheels
            backleft.setPower(backleftpower);
            backright.setPower(backrightpower);
            frontright.setPower(frontrightpower);
            frontleft.setPower(frontleftpower);

            // Arm
            if (gamepad1.left_bumper)
            {
                armPosition += 100;
            }
            else if (gamepad1.right_bumper)
            {
                armPosition -= 100;
            }
            else
                armPosition = arm.getCurrentPosition();
90

            arm.setTargetPosition(armPosition);

            
            boolean leftTriggerPressed = gamepad1.left_trigger > 0.01;
            
            if (leftTriggerPressed && !pressedLeftTriggerIteration) {
                // set intakePower based on existing value of intakePower
                // if intakePower = 1, intakePower = 0
                // if intakePower = 0, intakePower = 1
                if (intakePower == 1.0)
                {
                    intakePower = 0.0;
                }
                else
                {
                    intakePower = 1.0;
                }
            }
            pressedLeftTriggerIteration = leftTriggerPressed;
            
            
            boolean rightTriggerPressed = gamepad1.right_trigger > 0.01;
            
            if (rightTriggerPressed && !pressedRightTriggerIteration) {
                if (intakePower == -1.0)
                    {
                    intakePower = 0.0;
                }
                else
                {
                    intakePower = -1.0;
                }
            }
            pressedRightTriggerIteration = rightTriggerPressed;
            
            intake.setPower(intakePower);
                
            
            // Delivery mechanism
            if (gamepad1.dpad_left)
                spinner.setPower(1.0);
            else if (gamepad1.dpad_right)
                spinner.setPower(-1.0);
            else
                spinner.setPower(0.0);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", frontleftpower, frontrightpower);
            telemetry.update();
        }
    }
}
