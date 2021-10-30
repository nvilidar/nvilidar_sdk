NVILIDAR SDK PACKAGE V1.0.0
=====================================================================

How to build NVILIDAR SDK samples
=====================================================================
    $ git clone https://gitee.com/nvilidar/nvilidar_sdk.git       
	or
	$ git clone https://github.com/nvilidar/nvilidar_sdk.git

    $ cd sdk
    $ cd ..
    $ mkdir build
    $ cd build
    $ cmake ../sdk
    $ make					#linux
    $ vs open Project.sln	#windows
    
How to run NVILIDAR SDK samples
=====================================================================
    $ cd samples

linux:

	$ ./nvilidar_test

windows:

	$ nvilidar_test.exe

You should see NVILIDAR's scan result in the console:

	lidar device info:
	lidar name:VP300
	lidar soft version:V1.2
	lidar hard version:V1.0
	lidar serialnumber:00091511640128146414714880510

	lidar config info:
	lidar samplerate :20000
	lidar frequency :10.0
	lidar sesitive :no
	lidar trail filter level :6
	[NVILidar]: [NVILIDAR INFO] Now NVILIDAR is scanning ......
	[NVILidar]: Scan received[1635574574639575200]: 95 ranges is [212.765961]Hz
	[NVILidar]: Scan received[1635574574652212200]: 2614 ranges is [7.654038]Hz
	[NVILidar]: Scan received[1635574574791518000]: 2079 ranges is [9.624639]Hz
	[NVILidar]: Scan received[1635574574893106000]: 2021 ranges is [9.900990]Hz
	[NVILidar]: Scan received[1635574574996141000]: 1982 ranges is [10.095911]Hz
	[NVILidar]: Scan received[1635574575092594000]: 2005 ranges is [9.980040]Hz
	[NVILidar]: Scan received[1635574575194083000]: 2040 ranges is [9.808730]Hz
	[NVILidar]: Scan received[1635574575296030000]: 2037 ranges is [9.823183]Hz
	[NVILidar]: Scan received[1635574575399247000]: 2001 ranges is [10.000000]Hz
	[NVILidar]: Scan received[1635574575495266000]: 2001 ranges is [10.000000]Hz
	[NVILidar]: Scan received[1635574575598005000]: 2026 ranges is [9.876543]Hz
	[NVILidar]: Scan received[1635574575700107000]: 2030 ranges is [9.857072]Hz
	[NVILidar]: Scan received[1635574575802062000]: 2009 ranges is [9.960159]Hz
	[NVILidar]: Scan received[1635574575898707000]: 2003 ranges is [9.990010]Hz
	[NVILidar]: Scan received[1635574576000249000]: 2015 ranges is [9.930487]Hz
	[NVILidar]: Scan received[1635574576103022000]: 2014 ranges is [9.935420]Hz
	[NVILidar]: Scan received[1635574576199124000]: 2012 ranges is [9.945301]Hz
	[NVILidar]: Scan received[1635574576301782000]: 2010 ranges is [9.955201]Hz

