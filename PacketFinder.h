#ifndef INCLUDED_PacketFinder_h_
#define INCLUDED_PacketFinder_h_

#include <iostream>

struct CoreSensor {
		unsigned char header;					 		// 1
		unsigned char length;					 		// 1
		unsigned char timestamp[2];		 		// 2
		unsigned char bumper;					 		// 1
		unsigned char wheelDrop;			 		// 1
		unsigned char cliff;					 		// 1
		unsigned char leftEncoder[2];	 		// 2
		unsigned char rightEncoder[2]; 		// 2
		unsigned char leftPwm;				 		// 1
		unsigned char rightPwm;				 		// 1
		unsigned char buttons;				 		// 1
		unsigned char charger;				 		// 1
		unsigned char battery;				 		// 1
		unsigned char overCurrent;		 		// 1
	};

	struct DockInfraRed {
		unsigned char header;					 		// 1
		unsigned char length;					 		// 1
		unsigned char docking[3];			 		// 3
	};

	struct Inertia {
		unsigned char header;					 		// 1
		unsigned char length;					 		// 1
		unsigned char angle[2];				 		// 2
		unsigned char angleRate[2];		 		// 2
		unsigned char acc[3];					 		// 3
	};

	struct Cliff {
		unsigned char header;					 		// 1
		unsigned char length;					 		// 1
		unsigned char bottom0[2];					// 2
		unsigned char bottom1[2];					// 2
		unsigned char bottom2[2];					// 2
	};

	struct Current {
		unsigned char header;					 		// 1
		unsigned char length;					 		// 1
		unsigned char current[2];			 		// 2
	};
	
	struct ThreeAxisGyro {
		unsigned char header;					 		// 1
		unsigned char length;					 		// 1
		unsigned char frameId;				 		// 1
		unsigned char followedDataLength;	// 1
		unsigned char parameters[0];			// 12 or 18
	};

	struct GpInput {
		unsigned char header;					 		// 1
		unsigned char length;					 		// 1
		unsigned char digitalInput[2];		// 2
		unsigned char analogInput[14];
	};
	
	struct KobukiSensors {
		//CoreSensors
		unsigned short int timeStamp;
		unsigned char bumper;
		unsigned char wheelDrop;
		unsigned char cliff;
	 	short int leftEncoder;
		short int rightEncoder;
		unsigned char leftPwm;
		unsigned char rightPwm;
		unsigned char buttons;
		unsigned char charger;
		unsigned char battery;
		unsigned char overCurrent;
		
		//Dock Infra Red
		unsigned char docking[3];
		
		//Inertia
		short int angle;
		short int angleRate;
		unsigned char acc[3];
		
		//Cliff
		short int bottom[3];
		
		//Current
		unsigned char current[2];

		
		//Three Axis Gyro
		unsigned char frameId;
		unsigned char followedDataLength;
		short parameters[12];
				
		//GpInput
		short int digitalInput;
		short int analogInput[7];
	};
	
	
	enum State { 
	  lookingForHeader0, 
	  lookingForHeader1, 
	  waitingForPacket, 
	  gotPacket
	};
	
	enum Sensor { 
		coreSensor = 1,
		dockInfraRed = 3,
		inertia = 4,
		cliff = 5,
		current = 6,
		threeAxisGyro = 13,
		gpInput = 16,
	};
	
#endif
