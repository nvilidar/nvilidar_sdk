NVILIDAR SDK PACKAGE V1.0.8
=====================================================================

How to build NVILIDAR SDK samples
=====================================================================
    $ git clone https://gitee.com/nvilidar/nvilidar_sdk.git       
	or
	$ git clone https://github.com/nvistar/nvistar-sdk.git

    $ cd sdk
    $ cd ..
    $ mkdir build
    $ cd build
    $ cmake ../nvilidar_sdk
    $ make			#linux
    $ vs open Project.sln	#windows
    
How to run NVILIDAR SDK samples
=====================================================================
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
	lidar name:VP300
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


NVILIDAR SDK Para List
=====================================================================
the SDK can support serialport & udp socket

the Para in the "src/nvilidar/nvilidar_process.cpp",you can change the "LidarDefaultUserConfig" function to change the default para.

|  value   |  information  |
|  :----:    | :----:  |
| serialport_baud  | if use serialport,the lidar's serialport |
| serialport_name  | if use serialport,the lidar's port name |
| ip_addr  | if use udp socket,the lidar's ip addr,default:192.168.1.200 |
| lidar_udp_port  | if use udp socket,the lidar's udp port,default:8100 |
| config_tcp_port  | if use udp socket,config the net converter's para,default:8200 |
| frame_id  | it is useful in ros,sdk don't use it |
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
| angle_offset  | angle offset,default:0.0|
| adp_change_flag  | change apd value,don't change it if nessesary,default:false|
| adp_value  | change apd value,if the 'apd_change_flag' is true,it is valid,default:500|
| single_channel  | it is default false,don't change it|
| ignore_array_string  | if you want to filter some point's you can change it,it is anti-clockwise for the lidar.eg. you can set the value "30,60,90,120",you can remove the 30°~60° and 90°~120° points in the view|
| filter_jump_enable| fix some points jump up and down,default:true|
| filter_jump_value_min| fix some points jump up and down,default:3|
| filter_jump_value_max| fix some points jump up and down,default:25|


note: the 'serialport_baud','serialport_name','ip_addr','lidar_udp_port' will change in the 'LidarProcess' function para. it is not the finall values.