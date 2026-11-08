package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "OLD AUTO BLUE", group = "BioBuzz Old", preselectTeleOp = "BioBuzz TeleOp")
public class BioBuzzAutoBlue extends BioBuzzAutoBase {
    @Override
    protected HiveVision.Alliance alliance() {
        return HiveVision.Alliance.BLUE;
    }
}
