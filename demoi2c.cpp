#include "MPU9250.h"



//MPU9250 myIMU(MPU9250_ADDRESS, i2cport, i2cClock);


int main()
{
	MPU9250 myIMU(1, 0x68);
	//myIMU.initMPU9250();
	uint8_t imu_address= myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
	printf("IMU Address: %d\n", imu_address);
	if(imu_address == 113)
	{
		printf("IMU Online...\n");

	}
 else
 {
   printf("Something is wrong...\n");
   exit(1);
 }
	// Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.SelfTest);
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
    myIMU.initMPU9250();
    printf("MPU9250 initialized for active data mode....\n");
    uint8_t d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);

    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();

    while(true)
    {
	    if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
	    {
        myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
		    // Now we'll calculate the accleration value into actual g's
  	    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes;  - myIMU.accelBias[0];
		    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes;  - myIMU.accelBias[1];
		    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes;  - myIMU.accelBias[2];

		    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
		    // Calculate the gyro value into actual degrees per second
		    // This depends on scale being set
		    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
		    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
		    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

		    //printf("Acc: X, Y, Z : %f, %f, %f \n", myIMU.ax, myIMU.ay, myIMU.az);
        //printf("Gyro Rate: X, Y, Z : %f, %f, %f\n ", myIMU.gx, myIMU.gy, myIMU.gz);
        //printf("Acc Rate: y : %f\n", myIMU.ay);
        //printf("Gyro Rate: y : %f\n ", myIMU.gy);
        //sleep(0.1);
        
        //calculating the pitch roll and yaw from accel and gyro data...
        float pitch= atan2(myIMU.ay, (sqrt((myIMU.ax*myIMU.ay) + (myIMU.az*myIMU.az))));
        float roll= atan2(-myIMU.ax, (sqrt((myIMU.ay*myIMU.ay) + (myIMU.az*myIMU.az))));
        
        float Yh = (myIMU.my * cos(roll)) - (myIMU.mz * sin(roll));
        float Xh = (myIMU.mx * cos(pitch))+(myIMU.my * sin(roll)*sin(pitch)) + (myIMU.mz * cos(roll) * sin(pitch));
        
        float yaw= atan2(Yh,Xh);
        
        roll = roll*57.3;
        pitch = pitch*57.3;
        yaw = yaw*57.3;
        
        printf("roll, pitch, Yaw: %f, %f, %f \n", roll, pitch, yaw);
        sleep(0.6);
        
        
              
	    } 
    }

}





