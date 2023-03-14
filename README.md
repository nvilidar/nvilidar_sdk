# NVILIDAR SDK DRIVER

## How to build NVILIDAR SDK samples

### 1.Get the SDK code
    1) Clone this project to your catkin's workspace src folder
    	$ git clone https://gitee.com/nvilidar/nvilidar_sdk.git       
		or
		$ git clone https://github.com/nvilidar/nvilidar_sdk.git

    2) download the sdk code from our webset,  http://www.nvistar.com/?jishuzhichi/xiazaizhongxin


### 2.build the SDK
	1) linux
		$ cd sdk
		$ cd ..
		$ mkdir build
		$ cd build
		$ cmake ../nvilidar_sdk
		$ make			
	2) windows
		$ cd sdk
		$ cd ..
		$ mkdir build
		$ cd build
		$ cmake ../nvilidar_sdk
		$ make	
		then you can open "Project.sln" to open the visual studio.
	you can also Open the "CMakeLists.txt" directly with VS2017 or later

### 3.Serialport configuration
    1) if you use the lidar device name,you must give the permissions to user.
        ---whoami
       get the user name.link ubuntu.
        ---sudo usermod -a -G dialout ubuntu
       ubuntu is the user name.
        ---sudo reboot   

## ROS Parameter Configuration
### 1. Lidar Support
    current ros support 2 types of Lidar,include ROC300 and VP350.ros can automatic lidar model recognition.

    if you want to use serialport,

    The ROC300 is a USB interface lidar, supporting various baud rates, bps is recommended.
    VP350 is a serial interface lidar, supporting only 512000bps.


### 2. Choice the SDKCommunication interface

    	(1). if you want use the serialport,you neet to change the code from "nvilidar_node.cpp",change the code like this:

```cpp
//choice use serialport or socket 
#if 1
	nvilidar::LidarProcess lidar(USE_SERIALPORT,"/dev/nvilidar",512000);
#else 
	nvilidar::LidarProcess lidar(USE_SOCKET, "192.168.1.200", 8100); 
#endif 
```
the 2 parameters represent the serialport name and serialport baudrate.

        (2). if you want to use the udp socket,you neet to change the code 
        from "nvilidar_node.cpp",change the code like this:

```cpp
//choice use serialport or socket 
#if 0
	nvilidar::LidarProcess lidar(USE_SERIALPORT,"/dev/nvilidar",512000);
#else 
	nvilidar::LidarProcess lidar(USE_SOCKET, "192.168.1.200", 8100); 
#endif 
```    
the 2 parameters represent IP address and port number. 

## Interface function definition
### 1. bool LidarProcess::LidarInitialialize()
    Initialize the lidar, including opening the serial/socket interface and synchronizing the radar with the SDK parameter information
	if initial fail return false.
### 2. bool LidarProcess::LidarTurnOn()
	Turn on lidar scanning so that it can output point cloud data.
### 3. bool LidarProcess::LidarSamplingProcess(LidarScan &scan, uint32_t timeout)
	Real-time radar data output interface.
	The LidarScan variables are described as follows:
 
|  value   | Element | define  |
|  :----:  | :----:  | :----:  |
|  stamp   | none    |  lidar stamps, unit ns|
|  config  | min_angle   | lidar angle min value, 0~2*PI,Unit Radian|
|          | max_angle   | lidar angle max value, 0~2*PI,Unit Radian|
|          | angle_increment   | angular interval between 2 points, 0~2PI|
|          | scan_time   | time interval of 2 turns of data|
|          | min_range   | Distance measurement min, unit m|
|          | max_range   | Distance measurement max, unit m|
|  points  | angle       | lidar angle,0~2PI|
|  | range       | lidar distance, unit m|
|  | intensity       | lidar intensity,it is aviliable when sensitive is true|
### 4. bool LidarProcess::LidarTurnOff()
	lidar turn off the scanning data 
### 5. void LidarProcess::LidarCloseHandle()
	lidar close serialport/socket 

## How to run NVILIDAR SDK samples
    $ cd samples

linux:

	$ ./nvilidar_test

windows:

	$ nvilidar_test.exe

You should see NVILIDAR's scan result in the console:

     _   ___      _______ _      _____ _____          _____ 
	| \ | \ \    / /_   _| |    |_   _|  __ \   /\   |  __ \
	|  \| |\ \  / /  | | | |      | | | |  | | /  \  | |__) |
	| . ` | \ \/ /   | | | |      | | | |  | |/ /\ \ |  _  / 
	| |\  |  \  /   _| |_| |____ _| |_| |__| / ____ \| | \ \
	|_| \_|   \/   |_____|______|_____|_____/_/    \_\_|  \ \


	lidar device info:
	lidar name:ROC300
	lidar soft version:V1.13
	lidar hard version:V2.0
	lidar serialnumber:2022033011145555

	lidar config info:
	lidar samplerate :10000
	lidar frequency :10.00
	lidar sesitive :no
	lidar tailling filter level :6
	lidar apd value :500
	[NVILidar]: [NVILIDAR INFO] Now NVILIDAR is scanning ......
	[NVILidar]: Scan received[1652084622502328378]: 1000 ranges is [10.043005]Hz
	[NVILidar]: Scan received[1652084622601900167]: 1001 ranges is [9.935510]Hz
	[NVILidar]: Scan received[1652084622702549251]: 1000 ranges is [10.030573]Hz
	[NVILidar]: Scan received[1652084622802244446]: 1001 ranges is [9.931978]Hz
	[NVILidar]: Scan received[1652084622902929326]: 1001 ranges is [10.021062]Hz
	[NVILidar]: Scan received[1652084623002719144]: 1000 ranges is [9.964178]Hz
	[NVILidar]: Scan received[1652084623103078654]: 1000 ranges is [10.025907]Hz
	[NVILidar]: Scan received[1652084623202820253]: 1001 ranges is [10.040942]Hz
	[NVILidar]: Scan received[1652084623302412498]: 1000 ranges is [9.945678]Hz
	[NVILidar]: Scan received[1652084623402958685]: 1000 ranges is [10.039329]Hz
	[NVILidar]: Scan received[1652084623502566938]: 999 ranges is [9.973519]Hz
	[NVILidar]: Scan received[1652084623602832448]: 1000 ranges is [10.035958]Hz
	[NVILidar]: Scan received[1652084623702474162]: 1000 ranges is [9.946627]Hz
	[NVILidar]: Scan received[1652084623803010753]: 1000 ranges is [10.040023]Hz
	[NVILidar]: Scan received[1652084623902612115]: 999 ranges is [10.049306]Hz
	[NVILidar]: Scan received[1652084624002121471]: 1000 ranges is [9.971862]Hz
	[NVILidar]: Scan received[1652084624102403645]: 1000 ranges is [10.041158]Hz


## NVILIDAR ROS Parameter
|  value   |  information  |
|  :----:    | :----:  |
| serialport_baud  | if use serialport,the lidar's serialport |
| serialport_name  | if use serialport,the lidar's port name |
| ip_addr  | if use udp socket,the lidar's ip addr,default:192.168.1.200 |
| lidar_udp_port  | if use udp socket,the lidar's udp port,default:8100 |
| config_tcp_port  | if use udp socket,config the net converter's para,default:8200 |
| frame_id  | it is useful in ros,lidar ros frame id |
| resolution_fixed  | Rotate one circle fixed number of points,it is 'true' in ros,default |
| auto_reconnect  | lidar auto connect,if it is disconnet in case |
| reversion  | lidar's point revert|
| inverted  | lidar's point invert|
| angle_max  | lidar angle max value,max:180.0°|
| angle_max  | lidar angle min value,min:-180.0°|
| range_max  | lidar's max measure distance,default:64.0 meters|
| range_min  | lidar's min measure distance,default:0.0 meters|
| aim_speed  | lidar's run speed,default:10.0 Hz|
| sampling_rate  | lidar's sampling rate,default:10.0 K points in 1 second|
| sensitive  | lidar's data with sensitive,default:false|
| tailing_level  | lidar's tailing level,The smaller the value, the stronger the filtering,default:6|
| angle_offset_change_flag  | angle offset enable to set,default:false|
| angle_offset  | angle offset,default:0.0|
| adp_change_flag  | change apd value,don't change it if nessesary,default:false|
| adp_value  | change apd value,if the 'apd_change_flag' is true,it is valid,default:500|
| ignore_array_string  | if you want to filter some point's you can change it,it is anti-clockwise for the lidar.eg. you can set the value "30,60,90,120",you can remove the 30°~60° and 90°~120° points in the view|
| filter_jump_enable | filter some jump point,default:true |
| filter_jump_value_min | filter some jump point,min value,Not recommended to modify |
| filter_jump_value_max | filter some jump point,max value,Not recommended to modify |

note: the 'serialport_baud','serialport_name','ip_addr','lidar_udp_port' will change in the 'LidarProcess' function para. it is not the finall values.