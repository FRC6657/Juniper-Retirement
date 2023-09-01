package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Constants {
    public static class PivotConstants {
        
        public static final double kThroughboreOffset = 0.7748;
        
        public static enum SETPOINTS {

        INTAKE(-15), 
        HYBRID(-10),
        ZERO(0),
        AUTO(15),
        CARRY(30), 
        MID(40),
        DOUBLE(55),
        START(54);

        public final int angle;

        private SETPOINTS(int angle) {
            this.angle = angle;
        }

        }

        public static enum WINCH_STATES {
            IDLE(0), 
            EXTENDING(0.5), 
            RETRACTING(-0.5);

            public final double value;

            private WINCH_STATES(double value) {
                this.value = value;
            }
        }

        public static enum RATCHET_STATES {
            ENGAGED(Value.kForward), DISENGAGED(Value.kReverse);

            public final Value value;

            private RATCHET_STATES(Value value) {
                this.value = value;
            }
        }

    }

    public static class INTAKE{
        public static enum INTAKE_STATES{
            STOPPED(Value.kForward, 0),
            IDLE(Value.kForward, -0.05),
            INTAKING(Value.kReverse, -0.4),
            OUTTAKING(Value.kForward, 0.2);

            public final Value value;
            public final double speed;
            
            private INTAKE_STATES(Value value, double speed){
                this.value = value;
                this.speed = speed;
            }

        }
    }

}
