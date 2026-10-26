package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BioBuzz AUTO BLUE", group = "BioBuzz", preselectTeleOp = "BioBuzz TeleOp")
public class BioBuzzAutoBlue extends BioBuzzAutoBase {
    @Override
    protected HiveVision.Alliance alliance() {
        return HiveVision.Alliance.BLUE;
    }
}
