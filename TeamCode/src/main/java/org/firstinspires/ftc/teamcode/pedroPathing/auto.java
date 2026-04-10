package org.firstinspires.ftc.teamcode.pedroPathing;


import com.arcrobotics.ftclib.controller.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "red auto")
public class auto extends LinearOpMode {
    //Drivetrain
    DcMotorEx RightFront;
    DcMotorEx RightRear;
    DcMotorEx LeftRear;
    DcMotorEx LeftFront;
    //Spindexer
    Servo KickerServo;
    Servo SpindexServo1;
    Servo SpindexServo2;
    //IntakeSensor
    DigitalChannel BeamBreak;
    ColorSensor LeftSensor;
    ColorSensor BackSensor;
    ColorSensor RightSensor;
    //Intake
    DcMotorEx IntakeMotor;
    //Outtake
    DcMotorEx MasterShooterMotor;
    DcMotorEx SlaveShooterMotor;
    DcMotorEx TurretMotor;
    Servo HoodServo;

    Follower follower;
    AprilTagProcessor aprilTag;
    VisionPortal visionPortal;

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;
    public PathChain Path8;
    public PathChain Path9;
    public PathChain Path10;
    public PathChain Path11;
    public PathChain Path12;

    int shootindex = 2;
    int intakeindex = 0;
    double intakeConst = 0.8;
    double intakePower = 0;
    ElapsedTime cycleTimer = new ElapsedTime();
    //--------------------------------------------
    double targetX = 131.4223968565815;
    double targetY = 133.8821218074656;

    double startX = 125.09512770137523;
    double startY = 120.48707269155206;
    double startHeading = Math.toRadians(36);
    //--------------------------------------------

    int pathState = 0;
    ElapsedTime pathTimer = new ElapsedTime();
    public void runOpMode() {
        initialize();
        //shoot1, 0.5
        //shoot2, 0.28 right
        //shoot3, 0.08 right
        //shoot, 0.71 left
        //shoot, 0.93 left

        //intake, 0.61
        //intake, 0.82
        //intake, 0.39
        //intake 0.19

        HoodServo.setPosition(0.3);
        SpindexToShootPos(2);
        buildpaths();
        waitForStart();
        while (opModeIsActive()) {
            PoseStorage.save(follower.getPose());
            intakePower = 0.8;
            follower.update();

            if(pathState == 0){//start
                follower.followPath(Path1);//move to read pos
                pathState++;
                pathTimer.reset();
            }else if(pathState == 1){//wait until at read position
                if(detection()){
                    //visionPortal.close();
                    follower.followPath(Path2);//move to scoring pos
                    pathState++;
                    pathTimer.reset();
                }else if(!follower.isBusy() && pathTimer.seconds()>2){
                    //visionPortal.close();
                    motif = new int[] {1,1,2};
                    follower.followPath(Path2);//move to scoring pos
                    pathState++;
                    pathTimer.reset();
                }
            }else if(pathState == 2){//moving to scoring pos
                if(!follower.isBusy() && pathTimer.milliseconds()>1500){//at scoring position
                    if(ShootUnsorted()){//shoot
                        pathState++;
                        follower.followPath(Path3);//move to intake pos
                        pathTimer.reset();
                        IntakeState = 0;
                    }
                }
            }else if(pathState == 3){//moving to intake position
                Intake();
                if(!follower.isBusy() && pathTimer.milliseconds()>500){//at start of intake
                    follower.followPath(Path4,1,true);//move to end of intake
                    IntakeState = 0;
                    pathState++;
                    pathTimer.reset();
                }
            }else if(pathState == 4){//moving to end of intake position
                Intake();
                if(!follower.isBusy() && pathTimer.milliseconds()>500){//at end of intake position
                    follower.followPath(Path5);//move to gate pos
                    pathState = 222;
                    pathTimer.reset();
                }
            }else if(pathState == 222){
                Intake();
                if(!follower.isBusy()){
                    pathState = 5;
                    pathTimer.reset();
                }else if(pathTimer.milliseconds()>500){
                    pathState = 5;
                    pathTimer.reset();
                }
            }
            else if(pathState == 5){//moving to gate pos
                EndIntake();
                if(!follower.isBusy() && pathTimer.milliseconds()>1500){//at gate position
                    pathState++;
                    follower.followPath(Path6);//move to shoot pos
                    ShootState = 0;
                    pathTimer.reset();
                }
            }else if(pathState == 6){//moving to shoot pos
                if(!follower.isBusy() && pathTimer.milliseconds()>500){//at shoot pos
                    if(ShootSorted()){//when done shooting
                        follower.followPath(Path7);//move to 2nd intake pos
                        pathState++;
                        IntakeState = 0;
                        pathTimer.reset();
                    }
                }
            }else if(pathState == 7){//moving to 2nd intake pos
                Intake();
                if(!follower.isBusy() && pathTimer.milliseconds()>500){//at 2nd intake pos
                    follower.followPath(Path8,1,true);//move to end of 2nd intake
                    pathState++;
                    pathTimer.reset();
                }
            }else if(pathState == 8){//moving to 2nd intake
                Intake();
                if(!follower.isBusy() && pathTimer.milliseconds()>500){//at end of 2nd intake
                    follower.followPath(Path9);//move to shoot pos
                    pathState++;
                    pathTimer.reset();
                    SortShootState = 0;
                }
            }else if(pathState == 9){//moving to shooting pos
                EndIntake();
                if(!follower.isBusy() && pathTimer.milliseconds()>500){//at shoot pos
                    if(ShootSorted()){//when done shooting
                        follower.followPath(Path10);//move to 3rd intake pos
                        pathState++;
                        IntakeState = 0;
                        pathTimer.reset();
                    }
                }
            }else if(pathState == 10){//moving to 3rd intake
                Intake();
                if(!follower.isBusy() && pathTimer.milliseconds()>500){//at 3rd intake
                    follower.followPath(Path11,1,true);//go to end of 3rd intake pos
                    IntakeState = 0;
                    pathState++;
                    pathTimer.reset();
                }
            }else if(pathState == 11){//moving to end of 3rd intake pos
                Intake();
                if(!follower.isBusy() && pathTimer.milliseconds()>500){//at end of 3rd intake
                    follower.followPath(Path12,1,true);//move to shoot pos in a line
                    SortShootState = 0;
                    pathState++;
                    pathTimer.reset();
                }
            }else if(pathState == 12){//moving to shoot pos
                EndIntake();
                if(!follower.isBusy() && pathTimer.milliseconds()>500){//at score pos
                    if(ShootSorted()){
                        pathState++;
                        pathTimer.reset();
                        PoseStorage.save(follower.getPose());
                        break;
                    }
                }else if(pathTimer.milliseconds()<800){
                    Intake();
                }
            }

            IntakeMotor.setPower(intakePower);
            Kick_SM();
            shooter_tps = (int) getTargetTPS(getTurretDistance());
            Shooter(shooter_tps);
            aimTurretWithoutVel();

            telemetry.addData("motif: ", motif[0] + motif[1] + motif[2]);
            telemetry.addData("target angle: ", getAimAngle());
            telemetry.addData("Beam State: ", BeamBroken());
            telemetry.addData("Shooter Velocity: ", MasterShooterMotor.getVelocity());
            telemetry.addData("Turret Ticks: ", getTurretTicks());
            telemetry.addData("Shoot Index: ", shootindex);
            telemetry.addData("Loop Time: ", cycleTimer.milliseconds() + "ms");
            telemetry.update();
            cycleTimer.reset();
        }
    }
    public void buildpaths(){
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(125.095, 120.487),

                                new Pose(92.385, 112.672)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(110))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(92.385, 112.672),

                                new Pose(87.674, 96.625)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(-50))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(87.674, 96.625),

                                new Pose(99.383, 83.862)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-50), Math.toRadians(0))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(99.383, 83.862),

                                new Pose(126.442, 84.002)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(126.442, 84.002),
                                new Pose(109.992, 79.744),
                                new Pose(126.784, 72.758)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(126.784, 72.758),

                                new Pose(88.039, 95.564)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(-50))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(88.039, 95.564),
                                new Pose(87.598, 74.204),
                                new Pose(99.019, 59.127)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-50), Math.toRadians(0))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(99.019, 59.127),

                                new Pose(132.444, 58.057)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(132.444, 58.057),
                                new Pose(101.476, 64.209),
                                new Pose(87.561, 96.329)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-50))

                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(87.561, 96.329),
                                new Pose(83.656, 60.287),
                                new Pose(100.999, 35.362)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-50), Math.toRadians(0))

                .build();

        Path11 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(100.999, 35.362),

                                new Pose(129.354, 35.312)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path12 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(129.354, 35.312),

                                new Pose(91.055, 112.825)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-50))

                .build();
    }

    //INIT
    public void initialize(){
        follower = Constants.createFollower(hardwareMap);
        //follower.startTeleOpDrive();
        follower.setStartingPose(new Pose(startX ,startY, startHeading));
        //Drivetrain
        RightFront = hardwareMap.get(DcMotorEx.class, "fr");
        RightRear = hardwareMap.get(DcMotorEx.class, "br");
        LeftRear = hardwareMap.get(DcMotorEx.class, "bl");
        LeftFront = hardwareMap.get(DcMotorEx.class, "fl");
        RightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        RightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        RightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        RightRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        LeftRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        LeftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //Spindexer
        KickerServo = hardwareMap.get(Servo.class,"kicker");
        SpindexServo1 = hardwareMap.get(Servo.class,"spindex1");
        SpindexServo2 = hardwareMap.get(Servo.class,"spindex2");

        //Sensors
        BeamBreak = hardwareMap.get(DigitalChannel.class, "beambreak");
        BeamBreak.setMode(DigitalChannel.Mode.INPUT);
        LeftSensor = hardwareMap.get(ColorSensor.class, "csl");
        BackSensor = hardwareMap.get(ColorSensor.class, "csb");
        RightSensor = hardwareMap.get(ColorSensor.class, "csr");

        //Intake
        IntakeMotor = hardwareMap.get(DcMotorEx.class, "intake");

        //Outtake
        HoodServo = hardwareMap.get(Servo.class,"hood");
        MasterShooterMotor = hardwareMap.get(DcMotorEx.class, "mastershooter");
        SlaveShooterMotor = hardwareMap.get(DcMotorEx.class, "slaveshooter");
        MasterShooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        MasterShooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        SlaveShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        MasterShooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        SlaveShooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        MasterShooterMotor.setVelocityPIDFCoefficients(1000,0,0,17);//Flywheel Velocity PIDF

//        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
//        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);

        //Turret
        TurretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        TurretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        TurretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        TurretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        LeftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        LeftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    //DRIVE
    double forward = 1;
    double strafe = 1;
    double slowMulti = 1;
    boolean isRobotCentric = false;
    double rotation = 0;
    public void TeleDrive(){
        //RELOCALIZE
        if(gamepad1.dpadUpWasPressed()) follower.setPose(new Pose(startX ,startY, startHeading));

        //SLOW DRIVE
        if(gamepad1.left_bumper) slowMulti = 0.25;
        else slowMulti = 1;

        //DRIVE MODE TOGGLE
        if(gamepad1.leftStickButtonWasPressed()){//FIELD centric
            isRobotCentric = false;
            forward = 1;
            strafe = 1;
        }else if(gamepad1.rightStickButtonWasPressed()){//ROBOT centric
            isRobotCentric = true;
            forward = -1;
            strafe = -1;
        }

        //HEADING LOCK
        if(gamepad1.left_trigger>0.4){
            rotation = getHeadingLockRotation();
        }else{
            rotation = 0.75 * -gamepad1.right_stick_x * slowMulti;
        }

        //DRIVE MODE
        follower.setTeleOpDrive(
                gamepad1.left_stick_y * forward * slowMulti,
                gamepad1.left_stick_x * strafe * slowMulti,
                rotation,
                isRobotCentric
        );

        //LOCALIZATION
        double current_heading = follower.getPose().getHeading();
        double posX = follower.getPose().getX();
        double posY = follower.getPose().getY();
        double distance = Math.hypot(targetX - posX, targetY - posY);

        telemetry.addData("----------------------","");
        telemetry.addData("Heading: ", current_heading);
        telemetry.addData("X: ", posX);
        telemetry.addData("Y: ", posY);
        telemetry.addData("----------","");
        telemetry.addData("Distance: ", distance);
        telemetry.addData("Drive mode: " , isRobotCentric ? "ROBOT CENTRIC" : "FIELD CENTRIC");
        telemetry.addData("-----------------------","");
    }


    double HEADING_LOCK_ANGLE = 2.792;
    PIDFController HeadingPIDF = new PIDFController(0.8, 0, 0.05, 0);
    public double getHeadingLockRotation() {
        double currentHeading = follower.getPose().getHeading();
        double error = HEADING_LOCK_ANGLE - currentHeading;

        while (error > Math.PI) error -= 2 * Math.PI;
        while (error < -Math.PI) error += 2 * Math.PI;

        HeadingPIDF.setSetPoint(0);
        return HeadingPIDF.calculate(-error);
    }

    //VELOCITY COMPENSATION
    double COMPENSATION_CONSTANT = 100; //inches/sec
    public double[] getLeadCoords() {
        double[] turretCoords = getTurretCoords();
        double turretX = turretCoords[0];
        double turretY = turretCoords[1];

        double robotVelX = follower.getVelocity().getXComponent();
        double robotVelY = follower.getVelocity().getYComponent();

        //first estimate
        double distance = Math.hypot(targetX - turretX, targetY - turretY);
        double flightTime = distance / COMPENSATION_CONSTANT;

        //refine twice
        flightTime = refineToF(flightTime, robotVelX, robotVelY, turretX, turretY);
        flightTime = refineToF(flightTime, robotVelX, robotVelY, turretX, turretY);

        double leadX = targetX - (robotVelX * flightTime);
        double leadY = targetY - (robotVelY * flightTime);

        return new double[]{leadX, leadY};
    }
    public double refineToF(double flightTime, double robotVelX, double robotVelY, double turretX, double turretY){
        double leadX = targetX - (robotVelX * flightTime);
        double leadY = targetY - (robotVelY * flightTime);
        double refinedDistance = Math.hypot(leadX - turretX, leadY - turretY);

        return refinedDistance / COMPENSATION_CONSTANT;
    }
    public double getAimAngleCompensate() {
        double[] turretCoords = getTurretCoords();
        double[] lead = getLeadCoords();

        double dX = lead[0] - turretCoords[0];
        double dY = lead[1] - turretCoords[1];

        double angle = Math.toDegrees(Math.atan2(dY, dX));
        double robotHeading = Math.toDegrees(follower.getPose().getHeading());
        double relativeAngle = angle - robotHeading;

        relativeAngle = ((relativeAngle + 180) % 360 + 360) % 360 - 180;
        return relativeAngle;
    }
    public boolean aimTurretWithVel(){
        return MoveTurretToDegrees(getAimAngleCompensate());
    }
    public double getCompensatedShooterSpeed() {
        double[] turretCoords = getTurretCoords();
        double[] lead = getLeadCoords();

        double dX = lead[0] - turretCoords[0];
        double dY = lead[1] - turretCoords[1];
        double leadDistance = Math.hypot(dX, dY);

        return getTargetTPS(leadDistance);
    }

    //TURRET
    int TURRET_MIN = -6500;
    int TURRET_MAX = 6500;
    int TURRET_RIGHT = -4300;
    int TURRET_LEFT = 4300;
    double TURRET_OFFSET_X = -80 / 25.4;
    double TURRET_OFFSET_Y = 0;
    double TICKS_PER_DEGREE = 4300.0/90.0;//47.78
    double TURRET_MAX_POWER = 1;
    int TURRET_TOLERANCE = 30;
    double kF = 0.04;
    double kP = 0.00025;
    double kD = 0.000019;
    double kVF = 0.128;
    PIDFController TurretPIDF = new PIDFController(kP,0,kD,0);//need to tune

    public int getTurretTicks(){
        return LeftFront.getCurrentPosition();
    }
    public boolean MoveTurretToTick(int targetTicks){
        if(targetTicks <= TURRET_MIN || targetTicks >= TURRET_MAX){//out of bounds
            TurretMotor.setPower(0);
            return false;
        }
        TurretPIDF.setSetPoint(targetTicks);
        TurretPIDF.setTolerance(TURRET_TOLERANCE);
        int error = targetTicks - getTurretTicks();

        double output = TurretPIDF.calculate(getTurretTicks());

        output += follower.getAngularVelocity() * kVF;

        double ff = Math.abs(error) > TURRET_TOLERANCE ? Math.signum(error) * kF : 0;
        output+=ff;

        output = Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, output));//clamp to max power

        TurretMotor.setPower(output);
        telemetry.addData("output", output);
        telemetry.addData("error",getTurretTicks()-TurretPIDF.getSetPoint());
        return Math.abs(getTurretTicks()-TurretPIDF.getSetPoint()) <= TURRET_TOLERANCE;
    }
    public boolean MoveTurretToDegrees(double degrees){
        return MoveTurretToTick(degreesToTicks(degrees));
    }
    public int degreesToTicks(double degrees){
        return (int)(degrees * TICKS_PER_DEGREE);
    }
    public double ticksToDegrees(int ticks){
        return ticks / TICKS_PER_DEGREE;
    }
    public double[] getTurretCoords() {
        Pose pose = follower.getPose();

        double heading = pose.getHeading();
        double robotX = pose.getX();
        double robotY = pose.getY();

        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double turretX = robotX + (TURRET_OFFSET_X * cos - TURRET_OFFSET_Y * sin);
        double turretY = robotY + (TURRET_OFFSET_X * sin + TURRET_OFFSET_Y * cos);
        telemetry.addData("turret distance", Math.hypot(targetX - turretX, targetY - turretY));

        return new double[]{turretX, turretY};
    }
    public double getTurretDistance(){
        double[] turretCoords = getTurretCoords();
        double turretX = turretCoords[0];
        double turretY = turretCoords[1];
        return Math.hypot(targetX - turretX, targetY - turretY);
    }
    public double getAimAngle() {
        double[] turretCoords = getTurretCoords();
        double turretX = turretCoords[0];
        double turretY = turretCoords[1];

        double dX = targetX - turretX;
        double dY = targetY - turretY;

        double angle = Math.toDegrees(Math.atan2(dY, dX));

        double robotHeading = Math.toDegrees(follower.getPose().getHeading());
        double relativeAngle = angle - robotHeading;

        //normalize to [-180, 180]

        relativeAngle = -relativeAngle+180;
        relativeAngle = ((relativeAngle + 180) % 360 + 360) % 360 - 180;

        return relativeAngle;
    }
    public boolean aimTurretWithoutVel(){
        return MoveTurretToDegrees(getAimAngle());
    }
    public void holdTurretPos(){
        TurretMotor.setPower(0);
    }

    //SHOOTER
    int shooter_tps = 0;
    public boolean Shooter(int tps){
        if (tps == 0) {
            MasterShooterMotor.setPower(0);
            SlaveShooterMotor.setPower(0);
            return false;
        }
        double currentVelocity = MasterShooterMotor.getVelocity();
        double error = tps - currentVelocity;
        if (error > 200) {
            //below target
            MasterShooterMotor.setPower(1);
            SlaveShooterMotor.setPower(1);
            return false;
        }else if (error > -200) {
            //close to target, switch to pid
            MasterShooterMotor.setVelocity(tps);
            SlaveShooterMotor.setPower(MasterShooterMotor.getPower());
            return true;
        }else {
            //over target speed
            MasterShooterMotor.setVelocity(tps);
            SlaveShooterMotor.setPower(MasterShooterMotor.getPower());
            return false;
        }
    }
    public double getTargetTPS(double distance) {
        double distClose = 53;
        double tpsClose = 1000;
        double distFar = 142;
        double tpsFar = 1800;

        double m = (tpsFar - tpsClose) / (distFar - distClose);
        double b = tpsClose - (m * distClose);

        return Math.min(m * distance + b, 1500);
    }
    public double getTargetHood(double distance) {
        double distClose = 61.9;
        double posClose = 0.3;
        double distFar = 142;
        double posFar = 0.8;

        double m = (posFar - posClose) / (distFar - distClose);
        double b = posClose - (m * distClose);

        return Math.min(m * distance + b,0.8);
    }

    //SHOOTING MACRO
    int ShootState = 0;
    ElapsedTime ShootTimer = new ElapsedTime();
    int SPINDEX_SETTLE_TIME = 150;
    int num_shot = 0;
    public boolean ShootUnsorted(){
        if(ShootState == 0){
            shootindex = SpindexToShootPos(2);//go to first shoot position
            ShootTimer.reset();
            ShootState = 1;
        }else if(ShootState == 1){
            if(ShootTimer.milliseconds()>300){
                Kick();//kick
                num_shot = 1;
                ShootState = 2;
            }
        }else if(ShootState == 2){
            if(kickstate == 0){//kicker is down
                shootindex = SpindexToShootPos(shootindex+1);
                ShootTimer.reset();
                ShootState = 3;
            }
        }else if(ShootState == 3){
            if(ShootTimer.milliseconds()>SPINDEX_SETTLE_TIME){
                Kick();
                num_shot++;
                ShootTimer.reset();
                ShootState = 2;
                if(num_shot >= 3){
                    ShootState = 99;
                    return true;
                }
            }
        }
        return false;
    }

    int SortShootState = 0;
    ElapsedTime SortShootTimer = new ElapsedTime();
    int sort_num_shot = 0;
    int VARIABLE_SPINDEX_SETTLE_TIME = 140;
    int[] motif = {1,2,1};
    public boolean ShootSorted(){
        if(SortShootState == 0){
            shootindex = SpindexToShootPos(2);//go to first shoot position
            SortShootTimer.reset();
            sort_num_shot = 0;
            SortShootState++;
        }else if(SortShootState == 1){//init settle time
            if(SortShootTimer.milliseconds() >= 250){
                VARIABLE_SPINDEX_SETTLE_TIME = SpindexSpinToColor(motif[sort_num_shot]) * 180;
                SortShootTimer.reset();
                SortShootState++;
            }
        }else if(SortShootState == 2){
            if(SortShootTimer.milliseconds() >= VARIABLE_SPINDEX_SETTLE_TIME){
                Kick();
                SortShootState++;
            }
        }else if(SortShootState == 3){
            if(kickstate == 0){
                sort_num_shot++;
                if(sort_num_shot>=3){
                    SortShootState = 99;
                    return true;
                }else{
                    VARIABLE_SPINDEX_SETTLE_TIME = SpindexSpinToColor(motif[sort_num_shot]) * 180;
                    SortShootTimer.reset();
                    SortShootState = 2;
                }
            }
        }
        return false;
    }

    //INTAKE MACRO
    int IntakeState = 0;
    ElapsedTime IntakeTimer = new ElapsedTime();
    int SPINDEX_INTAKE_SETTLE_TIME = 100;
    public void Intake(){
        intakePower = 0.8;
        if(IntakeState == 0){
            intakeindex = SpindexToIntakePos(0);//go to first Intake position
            IntakeTimer.reset();
            IntakeState = 1;
        }else if(IntakeState == 1){
            if(IntakeTimer.milliseconds()>300){//wait for it to settle
                IntakeState = 2;
            }
        }else if(IntakeState == 2){
            if(BeamBroken() || gamepad1.a){//sense ball or manual advance
                intakeindex = SpindexToIntakePos(1);//go to next intake position
                IntakeTimer.reset();
                IntakeState = 3;
            }
        }else if(IntakeState == 3){
            if(IntakeTimer.milliseconds()>SPINDEX_INTAKE_SETTLE_TIME){//wait for it to settle
                IntakeState = 4;
            }
        }
    }
    public boolean EndIntake(){
        intakePower = 0.8;
        if(IntakeState == 4){
            intakeindex = SpindexToIntakePos(2);//go to last intake position
            IntakeTimer.reset();
            IntakeState = 5;
        }else if(IntakeState == 5){
            if(IntakeTimer.milliseconds()>800){//wait for it to settle
                IntakeState = 6;
            }
        }else if(IntakeState == 6){
            IntakeState = 0;
            return true;
            //Intake done
        }
        return false;
    }

    //SPINDEXER SHOOT POSITIONS
    double[] shootpos = {0.08, 0.28, 0.5, 0.71, 0.93,};
    public int SpindexToShootPos(int index){
        index = (index % 5 + 5) % 5;
        SpindexServo2.setPosition(shootpos[index]);
        SpindexServo1.setPosition(shootpos[index]);
        return index;
    }

    //SPINDEXER INTAKE POSITONS
    double[] intakepos = {0.61, 0.39, 0.19,};
    public int SpindexToIntakePos(int index){
        index = (index % 3 + 3) % 3;
        SpindexServo2.setPosition(intakepos[index]);
        SpindexServo1.setPosition(intakepos[index]);
        return index;
    }

    //SPINDEX SORTING
    public int SpindexSpinToColor(int color) {//1 = purple, 2 = green
        if (multiGetColor(BackSensor) == color) {
            return 0;
        } else if (multiGetColor(RightSensor) == color) {//decrement
            if(shootindex == 0){
                shootindex = SpindexToShootPos(2);
                return 2;
            }else{
                shootindex = SpindexToShootPos(shootindex - 1);
                return 1;//1
            }
        } else if (multiGetColor(LeftSensor) == color) {//increment
            if(shootindex == 4){
                shootindex = SpindexToShootPos(2);
                return 2;
            }else{
                shootindex = SpindexToShootPos(shootindex + 1);
                return 1;//1
            }
        }

        else if(getColor(RightSensor) == 0){//decrement
            if(shootindex == 0){
                shootindex = SpindexToShootPos(2);
                return 2;
            }else{
                shootindex = SpindexToShootPos(shootindex - 1);
                return 1;//1
            }
        }else if(getColor(LeftSensor) == 0){//decrement
            if(shootindex == 4){
                shootindex = SpindexToShootPos(2);
                return 2;
            }else{
                shootindex = SpindexToShootPos(shootindex + 1);
                return 1;//1
            }
        }
        return 0;
    }

    //SENSORS
    public boolean BeamBroken(){
        return !BeamBreak.getState();
    }
    public int getColor(ColorSensor colorsensor){
        double hue = readColor(colorsensor);
        if(colorsensor == LeftSensor){//rests at 160
            if(hue > 145 && hue < 155){//green at 150
                return 2;
            }else if(hue > 190 && hue < 210){//purple at 200
                return 1;
            }
        }else if(colorsensor == RightSensor){//rests at 151
            if(hue > 153 && hue < 165){//green at 156
                return 2;
            }else if(hue > 210 && hue < 230){//purple at 220
                return 1;
            }
        }else if(colorsensor == BackSensor){//rests at 142.5
            if(hue > 152 && hue < 165){//green at 159.5
                return 2;
            }else if(hue > 200 && hue < 220){//purple at 209.5
                return 1;
            }
        }


        return 0;
    }
    public double readColor(ColorSensor colorsensor) {
        float[] hsv = new float[3];
        android.graphics.Color.RGBToHSV(
                colorsensor.red(),
                colorsensor.green(),
                colorsensor.blue(),
                hsv
        );
        return hsv[0];
    }
    public int multiGetColor(ColorSensor colorsensor) {
        for (int i = 0; i < 10; i++) {
            int color = getColor(colorsensor);
            if(color!=0){
                return color;
            }
        }
        return 0;
    }

    //KICKER STATE MACHINE
    int kickstate = 0;
    double kicktime = 190;
    double kickcooldown = 30;
    ElapsedTime KickerTimer = new ElapsedTime();
    public void Kick_SM(){
        if(kickstate == 0){
            KickerServo.setPosition(0.32);//low position
        }else if(kickstate == 1){
            KickerServo.setPosition(0.6);//high position
            KickerTimer.reset();
            kickstate = 2;
        }else if(kickstate == 2){
            if(KickerTimer.milliseconds()>kicktime){//time needed to move up has passed
                KickerServo.setPosition(0.3);//low position
                KickerTimer.reset();
                kickstate = 3;
            }
        }else if(kickstate == 3) {
            if (KickerTimer.milliseconds() > kickcooldown) {//cooldown period has passed
                kickstate = 0;
            }
        }
    }
    public void Kick(){
        if(kickstate==0){
            kickstate = 1;
        }
    }

    //CAMERA
    public boolean detection(){
//        int tag = detectTag();
//        if(solveMotif(tag).length == 3){
//            motif = solveMotif(tag);
//            return true;
//        }
//        return false;
        return false;
    }
    public int detectTag() {
        int selectedTagId = -1;
        List<AprilTagDetection> detections = aprilTag.getDetections();

        for(AprilTagDetection detection : detections) {
            if(detection.id == 21 || detection.id == 22 || detection.id == 23) {
                selectedTagId = detection.id;
                telemetry.addData("Selected ID", selectedTagId);
            }
        }
        return selectedTagId;
    }
    public int[] solveMotif(int selectedTagId){
        //ID# 21 = gpp
        //22 = pgp
        //23 = ppg
        int[] thismotif = new int[] {0};
        if(selectedTagId == 21){
            thismotif = new int[] {2,1,1};//gpp
        }else if(selectedTagId == 22){
            thismotif = new int[] {1,2,1};//pgp
        }else if(selectedTagId == 23){
            thismotif = new int[] {1,1,2};//ppg
        }

        return thismotif;
    }
}
