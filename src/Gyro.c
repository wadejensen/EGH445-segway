const tSensors GyroSensor = (tSensors) S2;
#define GYRO_BIAS 596
#define SMOOTH_WINDOW 5
#define BIAS_WINDOW 10000

long Gyro_RawValue;
float compensated_Gyro;
float mov_RawValue;
float mov_bias=GYRO_BIAS;
float acc = 0.0;


task main()
{
	while(1)
	{
 Gyro_RawValue=SensorValue(GyroSensor); // Read the gyro sensor
		mov_RawValue = mov_RawValue+((float)Gyro_RawValue-mov_RawValue)/(SMOOTH_WINDOW+1);
		mov_bias = mov_bias+((float)Gyro_RawValue-mov_bias)/(BIAS_WINDOW+1);
		compensated_Gyro=round(mov_RawValue)-round(mov_bias);

		 nxtDisplayTextLine(2, " Gyro readouts ");
 //nxtDisplayTextLine(4, " Current: %04d ", Gyro_RawValue);
 nxtDisplayTextLine(5, " Smooth: %05.2f ", mov_RawValue);
 nxtDisplayTextLine(6, " Bias: %05.2f ", mov_bias);
 nxtDisplayTextLine(6, " Comp.: %04d ", compensated_Gyro);
 nxtDisplayTextLine(4, " Acc: %05.2f ", acc);

wait1Msec(200);
		acc = acc + compensated_Gyro*(200.0/1000.0);
	}

}
