package org.firstinspires.ftc.teamcode.pedroPathing;


import com.arcrobotics.ftclib.controller.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "blue tele")
public class servotest extends LinearOpMode {
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
    NormalizedColorSensor LeftSensor;
    NormalizedColorSensor BackSensor;
    NormalizedColorSensor RightSensor;
    //Intake
    DcMotorEx IntakeMotor;
    //Outtake
    DcMotorEx MasterShooterMotor;
    DcMotorEx SlaveShooterMotor;
    DcMotorEx TurretMotor;
    Servo HoodServo;

    Follower follower;

    int shootindex = 2;
    int intakeindex = 0;
    double intakeConst = 0.8;
    double intakePower = 0;
    int a = 0;
    //--------------------------------------------
//    double targetX = 12;
//    double targetY = 140;
    double targetX = 80;
    double targetY = 33.3;

    double startX = 105.3;
    double startY = 33.3;
    double startHeading = Math.PI;
    //--------------------------------------------

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
        waitForStart();
        while (opModeIsActive()) {
            follower.update();
            if(gamepad2.xWasPressed()) shootindex = SpindexToShootPos(shootindex-1);
            if(gamepad2.bWasPressed()) shootindex = SpindexToShootPos(shootindex+1);
            if(gamepad2.aWasPressed()) shootindex = SpindexToShootPos(2);
            if(gamepad2.yWasPressed()) Kick();

            if(gamepad1.xWasPressed()) intakeindex = SpindexToIntakePos(intakeindex-1);
            if(gamepad1.bWasPressed()) intakeindex = SpindexToIntakePos(intakeindex+1);


            if(gamepad2.dpad_up) ShootUnsorted();//shoot macro
            else ShootState = 0;

            if(gamepad1.right_trigger>0.3){//intake macro
                Intake();
                intakePower = intakeConst;
            }else if(IntakeState>=4){
                EndIntake();
                intakePower = intakeConst;
            }else if(gamepad1.right_bumper){
                intakePower = -intakeConst;
            }else{
                IntakeState = 0;
                intakePower = 0;
            }

            if(gamepad2.right_trigger > 0.2){//1000 for close
                shooter_tps = 1600;
                intakePower = intakeConst;
            }else if(gamepad2.right_bumper){
                shooter_tps = -500;
                intakePower = intakeConst;
            }else{
                shooter_tps = 0;
            }

            if(gamepad2.right_trigger>0.4 || gamepad1.left_trigger>0.4){
                boolean aimed = aimTurretWithoutVel();
                telemetry.addData("aimed", aimed);
            }else{
                holdTurretPos();
            }
            telemetry.addData("aim angle", getAimAngle());


            IntakeMotor.setPower(intakePower);
            TeleDrive();
            Kick_SM();
            Shooter(shooter_tps);

            telemetry.addData("beam", BeamBroken());
            telemetry.addData("vel: ", MasterShooterMotor.getVelocity());
            telemetry.addData("turretpos: ", getTurretTicks());
            telemetry.addData("shootindex", shootindex);
            telemetry.addData("turret degrees: ", ticksToDegrees(getTurretTicks()));
            telemetry.update();
        }
    }

    //INIT
    public void initialize(){
        follower = Constants.createFollower(hardwareMap);
        follower.startTeleOpDrive();
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
        LeftSensor = hardwareMap.get(NormalizedColorSensor.class, "csl");
        BackSensor = hardwareMap.get(NormalizedColorSensor.class, "csb");
        RightSensor = hardwareMap.get(NormalizedColorSensor.class, "csr");

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

        MasterShooterMotor.setVelocityPIDFCoefficients(500,0,0,15.5);//Flywheel Velocity PIDF

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

        //DRIVE MODE
        follower.setTeleOpDrive(
                gamepad1.left_stick_y * forward * slowMulti,
                gamepad1.left_stick_x * strafe * slowMulti,
                0.75*-gamepad1.right_stick_x * slowMulti,
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

    //VELOCITY COMPENSATION
    double BALL_SPEED = 1; //inches/sec
    public double[] getLeadCoords() {
        double[] turretCoords = getTurretCoords();
        double turretX = turretCoords[0];
        double turretY = turretCoords[1];

        double robotVelX = follower.getVelocity().getXComponent();
        double robotVelY = follower.getVelocity().getYComponent();

        //first estimate
        double distance = Math.hypot(targetX - turretX, targetY - turretY);
        double flightTime = distance / BALL_SPEED;

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

        return refinedDistance / BALL_SPEED;
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
    double kP = 0.0002;
    double kD = 0.000013;
    double kVF = 0.148;
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

        return new double[]{turretX, turretY};
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
        relativeAngle = ((relativeAngle + 180) % 360 + 360) % 360 - 180;

        return -relativeAngle;
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
        }else if (error > -100) {
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
        double distClose = 51;
        double tpsClose = 1150;
        double distFar = 132;
        double tpsFar = 1570;

        double m = (tpsFar - tpsClose) / (distFar - distClose);
        double b = tpsClose - (m * distClose);

        return m * distance + b;
    }
    public double getTargetHood(double distance) {
        double distClose = 71;
        double posClose = 0;
        double distFar = 145;
        double posFar = 1;

        double m = (posFar - posClose) / (distFar - distClose);
        double b = posClose - (m * distClose);

        return m * distance + b;
    }

    //SHOOTING MACRO
    int ShootState = 0;
    ElapsedTime ShootTimer = new ElapsedTime();
    int SPINDEX_SETTLE_TIME = 100;
    int num_shot = 0;
    public void ShootUnsorted(){
        if(ShootState == 0){
            shootindex = SpindexToShootPos(2);//go to first shoot position
            Kick();//kick
            ShootState = 1;
            num_shot = 1;
        }else if(ShootState == 1){
            if(kickstate == 0){//kicker is down
                shootindex = SpindexToShootPos(shootindex+1);
                ShootTimer.reset();
                ShootState = 2;
            }
        }else if(ShootState == 2){
            if(ShootTimer.milliseconds()>SPINDEX_SETTLE_TIME){
                Kick();
                num_shot++;
                ShootTimer.reset();
                ShootState = 1;
                if(num_shot >= 3){
                    ShootState = 99;
                }
            }
        }
    }

    //INTAKE MACRO
    int IntakeState = 0;
    ElapsedTime IntakeTimer = new ElapsedTime();
    int SPINDEX_INTAKE_SETTLE_TIME = 100;
    public void Intake(){
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
        if(IntakeState == 4){
            intakeindex = SpindexToIntakePos(2);//go to last intake position
            IntakeTimer.reset();
            IntakeState = 5;
        }else if(IntakeState == 5){
            if(IntakeTimer.milliseconds()>500){//wait for it to settle
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

    //BEAM BREAK, to be completed
    public boolean BeamBroken(){
        return !BeamBreak.getState();
    }

    //KICKER STATE MACHINE
    int kickstate = 0;
    double kicktime = 180;
    double kickcooldown = 30;
    ElapsedTime KickerTimer = new ElapsedTime();
    public void Kick_SM(){
        if(kickstate == 0){
            KickerServo.setPosition(0.32);//low position
        }else if(kickstate == 1){
            KickerServo.setPosition(0.5);//high position
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
}
