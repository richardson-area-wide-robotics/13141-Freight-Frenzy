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

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.AnalogInput;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Autonomous", group="Robot Code")
//@Disabled
public class Autonomous_Mode1 extends LinearOpMode {

    @Autonomous(name="AutoRedBoth", group="Linear Opmode")  // @TeleOp(...) is the other common choice
// @Disabled
    public class AutoRedBoth extends LinearOpMode {

        // Declare Devices
        DcMotor frontleft = null;
        DcMotor frontright = null;
        DcMotor backleft = null;
        DcMotor backright = null;
     


        // drive motor position variables
        private int lfPos; private int rfPos; private int lrPos; private int rrPos;

        // operational constants
        private double fast = 0.5; // Limit motor power to this value for Andymark RUN_USING_ENCODER mode
        private double medium = 0.3; // medium speed
        private double slow = 0.1; // slow speed
        private double clicksPerInch = 87.5; // empirically measured
        private double clicksPerDeg = 21.94; // empirically measured
        private double lineThreshold = 0.7; // floor should be below this value, line above
        private double redThreshold = 1.9; // red should be below this value, blue above

        @Override
        public void runOpMode() {
            telemetry.setAutoClear(true);


            // Initialize the hardware variables.
            backleft  = hardwareMap.get(DcMotor.class, "BackLeft");        backright = hardwareMap.get(DcMotor.class, "BackRight");
            frontleft = hardwareMap.get(DcMotor.class, "FrontLeft");
            frontright = hardwareMap.get(DcMotor.class, "FrontRight");


            // The right motors need reversing
            frontright.setDirection(DcMotor.Direction.REVERSE);
            frontleft.setDirection(DcMotor.Direction.FORWARD);
            backright.setDirection(DcMotor.Direction.REVERSE);
            backleft.setDirection(DcMotor.Direction.FORWARD);

            // Set the drive motor run modes:
            frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Wait for the game to start (driver presses PLAY)
            waitForStart();

            // *****************Dead reckoning list*************
            // Distances in inches, angles in deg, speed 0.0 to 0.6
            moveForward(16, fast);
            turnClockwise(-45, fast);
            moveForward(33, fast);
            turnClockwise(-45, fast);
            moveForward(24, fast);
            moveToLine(24, medium);
            pushRedButton();
            moveForward(-6, fast);
            turnClockwise(-3, medium); // aiming tweak
            moveRight(36, fast);
            moveToLine(24, medium);
            pushRedButton();
            moveForward(-12, fast);
            turnClockwise(-135, fast);
            moveForward(66, fast);
        }

        private void moveForward(int howMuch, double speed) {
            // howMuch is in inches. A negative howMuch moves backward.

            // fetch motor positions
            lfPos = frontleft.getCurrentPosition();
            rfPos = frontright.getCurrentPosition();
            lrPos = backleft.getCurrentPosition();
            rrPos = backright.getCurrentPosition();

            // calculate new targets
            lfPos += howMuch * clicksPerInch;
            rfPos += howMuch * clicksPerInch;
            lrPos += howMuch * clicksPerInch;
            rrPos += howMuch * clicksPerInch;

            // move robot to new position
            frontleft.setTargetPosition(lfPos);
            frontright.setTargetPosition(rfPos);
            backright.setTargetPosition(lrPos);
            backright.setTargetPosition(rrPos);
            frontleft.setPower(speed);
            frontright.setPower(speed);
            backleft.setPower(speed);
            backright.setPower(speed);

            // wait for move to complete
            while (frontleft.isBusy() && frontright.isBusy() &&
                    backleft.isBusy() && backright.isBusy()) {

                // Display it for the driver.
                telemetry.addLine("Move Foward");
                telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lrPos, rrPos);
                telemetry.addData("Actual", "%7d :%7d", frontleft.getCurrentPosition(),
                        frontright.getCurrentPosition(),backleft.getCurrentPosition(),
                        backright.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            frontleft.setPower(0);
            frontright.setPower(0);
            backleft.setPower(0);
            backright.setPower(0);
        }

        private void moveRight(int howMuch, double speed) {
            // howMuch is in inches. A negative howMuch moves backward.

            // fetch motor positions
            lfPos = frontleft.getCurrentPosition();
            rfPos = frontright.getCurrentPosition();
            lrPos = backleft.getCurrentPosition();
            rrPos = backright.getCurrentPosition();

            // calculate new targets
            lfPos += howMuch * clicksPerInch;
            rfPos -= howMuch * clicksPerInch;
            lrPos -= howMuch * clicksPerInch;
            rrPos += howMuch * clicksPerInch;

            // move robot to new position
            frontleft.setTargetPosition(lfPos);
            frontright.setTargetPosition(rfPos);
            backleft.setTargetPosition(lrPos);
            backright.setTargetPosition(rrPos);
            frontleft.setPower(speed);
            frontright.setPower(speed);
            backleft.setPower(speed);
            backright.setPower(speed);

            // wait for move to complete
            while (frontleft.isBusy() && frontright.isBusy() &&
                    backleft.isBusy() && backright.isBusy()) {

                // Display it for the driver.
                telemetry.addLine("Strafe Right");
                telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lrPos, rrPos);
                telemetry.addData("Actual", "%7d :%7d",frontleft.getCurrentPosition(),
                        frontright.getCurrentPosition(),backleft.getCurrentPosition(),
                        backright.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            frontleft.setPower(0);
            frontright.setPower(0);
            backleft.setPower(0);
            backright.setPower(0);

        }

        private void turnClockwise(int whatAngle, double speed) {
            // whatAngle is in degrees. A negative whatAngle turns counterclockwise.

            // fetch motor positions
            lfPos = frontleft.getCurrentPosition();
            rfPos = frontright.getCurrentPosition();
            lrPos = backleft.getCurrentPosition();
            rrPos = backright.getCurrentPosition();

            // calculate new targets
            lfPos += whatAngle * clicksPerDeg;
            rfPos -= whatAngle * clicksPerDeg;
            lrPos += whatAngle * clicksPerDeg;
            rrPos -= whatAngle * clicksPerDeg;

            // move robot to new position
            frontleft.setTargetPosition(lfPos);
            frontright.setTargetPosition(rfPos);
            backleft.setTargetPosition(lrPos);
            backright.setTargetPosition(rrPos);
            frontleft.setPower(speed);
            frontright.setPower(speed);
            backleft.setPower(speed);
            backright.setPower(speed);

            // wait for move to complete
            while (frontleft.isBusy() && frontright.isBusy() &&
                    backleft.isBusy() && backright.isBusy()) {

                // Display it for the driver.
                telemetry.addLine("Turn Clockwise");
                telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lrPos, rrPos);
                telemetry.addData("Actual", "%7d :%7d", frontleft.getCurrentPosition(),
                        frontright.getCurrentPosition(), backleft.getCurrentPosition(),
                        backright.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            frontleft.setPower(0);
            frontright.setPower(0);
            backleft.setPower(0);
            backright.setPower(0);
        }
        private void moveToLine(int howMuch, double speed) {
            // howMuch is in inches. The robot will stop if the line is found before
            // this distance is reached. A negative howMuch moves left, positive moves right.

            // fetch motor positions
            lfPos = frontleft.getCurrentPosition();
            rfPos = frontright.getCurrentPosition();
            lrPos = backleft.getCurrentPosition();
            rrPos = backright.getCurrentPosition();

            // calculate new targets
            lfPos += howMuch * clicksPerInch;
            rfPos -= howMuch * clicksPerInch;
            lrPos -= howMuch * clicksPerInch;
            rrPos += howMuch * clicksPerInch;

            // move robot to new position
            fronteft.setTargetPosition(lfPos);
            frontright.setTargetPosition(rfPos);
            backleft.setTargetPosition(lrPos);
            backright.setTargetPosition(rrPos);
            frontleft.setPower(speed);
            frontright.setPower(speed);
            backleft.setPower(speed);
            backright.setPower(speed);

            // wait for move to complete
            while (frontleft.isBusy() && frontright.isBusy() &&
                    backleft.isBusy() && backright.isBusy()) {
                if (mrOds.getLightDetected() > lineThreshold) break;

                // Display it for the driver.
                telemetry.addLine("Move To Line");
                telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lrPos, rrPos);
                telemetry.addData("Actual", "%7d :%7d", frontleft.getCurrentPosition(),
                        frontright.getCurrentPosition(), backleft.getCurrentPosition(),
                        backright.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            frontleft.setPower(0);
            frontright.setPower(0);
            backleft.setPower(0);
            backright.setPower(0);

        }

        }

    }
