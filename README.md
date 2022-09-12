# MPU6050-SENSOR-DRIVER
PATH:                                          
MPU6050_V2/Drivers/MPU6050_V2.1

 **Driver User Advice**
 - You should define "Accel_Gyro" sturct variable for MPU6050

 Example->" Accel_Gyro MPU6050;

 - You should define "Temperature" sturct variable for MPU6050 

 Example->"Temperature temp;"

- *MPU6050_Init( &MPU6050 , FS_500 , AFS_4G)*  function takes 3 variables 

1. reference from Accel_Gyro MPU6050 ("Accel_Gyro" sturct variable)
2. Scale Range Gyroscope (FS_250, ->  250  degrees per second
	                      FS_500, ->  500  degrees per second
	                      FS_1000,->  1000 degrees per second
	                      FS_2000,->  2000 degrees per second)
						  
3. Scale Range Accelerometer (AFS_2G,-> 2g  Gravity
	                          AFS_4G,-> 4g  Gravity
	                          AFS_8G,-> 8g  Gravity
	                          AFS_16G-> 16g Gravity)# MPU6050-SENSOR-DRIVER
				  
4.  Start the MPU6050  *MPU6050_Start( &MPU6050 , &temp );*
		  
				  
				  
  ![CUBEMX](https://user-images.githubusercontent.com/93796314/188217409-ed73cd88-19e4-4c6a-be2c-a834d466056f.JPG)
				  

# IIR Filter Part 
![dd](https://user-images.githubusercontent.com/93796314/188218591-b90adb2a-ffef-42d4-9fba-99e12bc5c8d7.JPG)

# Complementary Filter Part
![gecisler](https://user-images.githubusercontent.com/93796314/188218734-66ac8579-5521-4127-b402-e8cb3d50ace6.jpg)

# Resources
Philip Salmony   -> https://www.youtube.com/playlist?list=PLXSyc11qLa1ZCn0JCnaaXOWN6Z46rK9jd

Coşkun Taşdemir -> https://www.youtube.com/watch?v=dDfwamkfz_c&t=885s

Brain Douglas   ->  https://www.youtube.com/user/ControlLectures

Joop Brokking   ->  https://www.youtube.com/watch?v=j-kE0AMEWy4&t=117s

[ourdev_665531S2JZG6.pdf](https://github.com/alihaanc/MPU6050-SENSOR-DRIVER/files/9480447/ourdev_665531S2JZG6.pdf)
			  
				  
# MPU6050-SENSOR-DRIVER
Linkedin Account:
https://www.linkedin.com/feed/update/urn:li:activity:6971895564720332800/
