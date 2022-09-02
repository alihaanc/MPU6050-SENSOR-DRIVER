# desktop-tutorial
Driver User Advice
-You should define "Accel_Gyro" sturct variable for MPU6050

example-> Accel_Gyro MPU6050;

-You should define "Temperature" sturct variable for MPU6050 

example->Temperature temp

- MPU6050_Init(&MPU6050, FS_500, AFS_4G)  function takes 3 variables 

1. refrence from Accel_Gyro MPU6050 ("Accel_Gyro" sturct variable)
2. Scale Range Gyroscope (FS_250, ->  250  degrees per second
	                      FS_500, ->  500  degrees per second
	                      FS_1000,->  1000 degrees per second
	                      FS_2000,->  2000 degrees per second)
						  
						  
3. Scale Range Accelerometer (AFS_2G,-> 2g  Gravity
	                          AFS_4G,-> 4g  Gravity
	                          AFS_8G,-> 8g  Gravity
	                          AFS_16G-> 16g Gravity)# MPU6050-SENSOR-DRIVER
				  
				  
				  ![sss](https://user-images.githubusercontent.com/93796314/188215021-7f1fc7e3-6914-4f06-aebf-6c8233fae66d.png)
                                 ![indir](https://user-images.githubusercontent.com/93796314/188215024-9e8397ca-488d-407c-9e58-baadee86fbab.png)

		
		  
				  
				  
				  
# MPU6050-SENSOR-DRIVER
