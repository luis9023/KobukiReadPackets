#include "PacketFinder.h"
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <cmath>

using namespace std;

void updateLocation(unsigned short leftEncoder, unsigned short rightEncoder) {
	const double wheelRadius = 0.035;
	const double bias = 0.23;
	const double tickToRad = 0.002436916871363930187454f;
	
	static bool initLeft = false;
	static bool initRight = false;
	
	static unsigned short lastTickLeft = 0; 
	static unsigned short lastTickRight = 0;
	
	static double ds; 
	static double theta;
	double xPosition, yPosition;
	static double oldX;
	static double oldY;

	unsigned short currentTickLeft, currentTickRight;
	static double leftDiffTicks = 0.0f;
	static double rightDiffTicks = 0.0f;
	
	currentTickLeft = leftEncoder;
	if (!initLeft) {
		lastTickLeft = currentTickLeft;
		initLeft = true;
	}
	leftDiffTicks = (double)(short)((currentTickLeft - lastTickLeft) & 0xffff);
	lastTickLeft = currentTickLeft;
	
	currentTickRight = rightEncoder;
	if (!initRight) {
		lastTickRight = currentTickRight;
		initRight = true;
	}
	rightDiffTicks = (double)(short)((currentTickRight - lastTickRight) & 0xffff);
	lastTickRight = currentTickRight;
	
	ds = wheelRadius*((tickToRad*leftDiffTicks) + (tickToRad*rightDiffTicks))/2.0;
	theta += wheelRadius*((tickToRad*rightDiffTicks) - (tickToRad*leftDiffTicks))/bias;
	
	xPosition = oldX + ds * cos(theta);
	yPosition = oldY + ds * sin(theta);
	
	cout << "\nPose update: " << "x[" << xPosition*1000 << "] y[" << yPosition*1000 << "] heading[" << theta << 
"]" << endl;
	
	oldX = xPosition;
	oldY = yPosition; 	
}

int main() {

	FILE * file;
	file = fopen("/dev/kobuki","r");

	State currentState = lookingForHeader0;
	unsigned char data[1];
	unsigned int packetLength;
	unsigned char packet[256];
	KobukiSensors sensorData;
	unsigned int p=0;
	
	while (true) {
	
		while (currentState != gotPacket) {
		
			switch (currentState) {
				case lookingForHeader0:
					fread(data,1,1,file);
					if (data[0] == 0xaa)
						currentState = lookingForHeader1;
					break;
				case lookingForHeader1:
					fread(data,1,1,file);
					if (data[0] == 0x55)
						currentState = waitingForPacket;
					else if (data[0] == 0xaa)
						currentState = lookingForHeader1;
					else
						currentState = lookingForHeader0;
					break;
				case waitingForPacket:
					fread(data,1,1,file);
					packetLength = data[0];
					fread(packet,1,packetLength,file);
					currentState = gotPacket;
					//cout << "\n\n Got packet \n\n";
					break;
				case gotPacket:
					currentState = lookingForHeader0;
				default:
					break;
			}
		}
	
		/**cout << "\n\nThis is the packet: ";
		for (int i = 0; i < packetLength; i++) {
			printf("%x ",packet[i]);
		}
		cout << endl;*/
	
		// TODO compute checksum

		unsigned int index = 0;
		while (index < packetLength-1) {
			switch (packet[index]) {
				case coreSensor:
				{
					CoreSensor *data = (CoreSensor*) &packet[index];
					//short int le = *(short int *) &data->leftEncoder;
					unsigned short int time_stamp = (data->timestamp[1] << 8) | data->timestamp[0];
					short int left_encoder = (data->leftEncoder[1] << 8) | data->leftEncoder[0];
					short int right_encoder = (data->rightEncoder[1] << 8) | data->rightEncoder[0];
					//cout << "\nTimestamp: " << data->timestamp << " Left encoder: " << (unsigned int) data->leftEncoder[0] << " Right encoder " << (unsigned int) data->leftEncoder[1] << " " << left_encoder << endl;
				
					sensorData.timeStamp = time_stamp;
					sensorData.bumper = data->bumper;
					sensorData.wheelDrop = data->wheelDrop;
					sensorData.cliff = data->cliff;
					sensorData.leftEncoder = left_encoder;
					sensorData.rightEncoder = right_encoder;
					sensorData.leftPwm = data->leftPwm;
					sensorData.rightPwm = data->rightPwm;
					sensorData.buttons = data->buttons;
					sensorData.charger = data->charger;
					sensorData.battery = data->battery;
					sensorData.overCurrent = data->overCurrent;
		
					//cout << "\nCoreSensor: " << "Timestamp " << sensorData.timeStamp << " Bumper " << (void*)sensorData.bumper << " Wheel Drop " << (void*)sensorData.wheelDrop << " Cliff " << (void*)sensorData.cliff << " Left Encoder " << sensorData.leftEncoder << " Right Encoder " << sensorData.rightEncoder << " Left Pwm " << (void*)sensorData.leftPwm << " Right Pwm " << (void*)sensorData.rightPwm << " Buttons " << (void*)sensorData.buttons << " Charger " << (void*)sensorData.charger << " Battery " << (void*)sensorData.battery << " Over Current " << (void*)sensorData.overCurrent << endl;
					updateLocation(sensorData.leftEncoder, sensorData.rightEncoder);
					index += sizeof(CoreSensor);
					break;
				}
				case dockInfraRed:
				{
					DockInfraRed *data = (DockInfraRed*) &packet[index];
					//cout << "\nDock Infra Red: ";
					//printf("Header  %x Length  %x Docking  ", data->header, data->length);
					for (int i = 0; i < 3; i++) {
							//printf("%x ", data->docking[i]); 
							sensorData.docking[i] = data->docking[i];
					}
					//cout << endl;
					index += sizeof(DockInfraRed);
					break;
				}
				case inertia:
				{
					Inertia *data = (Inertia*) &packet[index];
					short int angle = (data->angle[1] << 8) | data->angle[0];
					short int angle_rate = (data->angleRate[1] << 8) | data->angleRate[0];
					sensorData.angle = angle;
					sensorData.angleRate = angle_rate;
					//cout << "\nInertia: ";
					//printf("Header  %x Length  %x ", data->header, data->length);
					//cout << " Angle  " << sensorData.angle << " AngleRate  " << sensorData.angleRate << " Acc  ";
					for (int i = 0; i < 3; i++) {
						sensorData.acc[i] = data->acc[i];
						//cout << (void*)sensorData.acc[i] << " ";
					}
					//cout << endl;
					index += sizeof(Inertia);
					break; 
				}
				case cliff:
				{
					Cliff *data = (Cliff*) &packet[index];
					short int cliffBottom0 = (data->bottom0[1] << 8) | data->bottom0[0];
					short int cliffBottom1 = (data->bottom1[1] << 8) | data->bottom1[0];
					short int cliffBottom2 = (data->bottom2[1] << 8) | data->bottom2[0];
				
					sensorData.bottom[0] = cliffBottom0;
					sensorData.bottom[1] = cliffBottom1;
					sensorData.bottom[2] = cliffBottom2;
				
					/**cout << "\nCliff: ";
					printf("Header %x Length %x Bottom ", data->header, data->length);
					for (int i = 0; i < 3; i++)
						cout << sensorData.bottom[i] << " ";
					cout << endl;**/
					index += sizeof(Cliff);
					break;
				}
				case current:
				{
					Current *data = (Current*) &packet[index];
					//cout << "\nCurrent: ";
					//printf("Header %x Length %x Current ", data->header, data->length);
					for (int i = 0; i < 2; i++) {
						sensorData.current[i] = data->current[i];
						//cout << (void*)sensorData.current[i] << " ";
					}
					//cout << endl;
					index += sizeof(Current);
					break;
				}
				case threeAxisGyro:
				{
					ThreeAxisGyro *data = (ThreeAxisGyro*) &packet[index];
					sensorData.frameId = data->frameId;
					sensorData.followedDataLength = data->followedDataLength;
					for (int i = 0; i < data->followedDataLength; i++) 
						sensorData.parameters[i] = (data->parameters[2*i+1] << 8) | data->parameters[2*i];
					/**cout << "\nThree Axis Gyro: ";
					printf("Header %d Length %d Frame ID %x Followed Data Lenght %d Parameters ", data->header, data->length, data->frameId, data->followedDataLength);
					for (int i = 0; i < data->followedDataLength; i++)
								cout << sensorData.parameters[i] << " ";
					cout << endl;**/
					index += sizeof(ThreeAxisGyro) + 2 * data->followedDataLength;
					break;
				}
				case gpInput:
				{
					GpInput *data = (GpInput*) &packet[index];
					sensorData.digitalInput =  (data->digitalInput[1] << 8) | data->digitalInput[0];
					for (int i = 0; i < (data->length -2)/2; i++)
						sensorData.analogInput[i] = (data->analogInput[2*i+1] << 8) | data->analogInput[2*i];
					/**cout << "\nGp Input: ";
					printf("Header %d Length %d Digital Input %d Analog Input ", data->header, data->length, sensorData.digitalInput);
					for (int i = 0; i < 7; i++)
						cout << sensorData.analogInput[i] << " ";
					cout << endl;**/ 
					index += sizeof(GpInput);
					break;
				}
				default:
					//cout << "Unknown packet found." << endl;
					index += packetLength;
					break;
			}
		}
		currentState = lookingForHeader0;
		usleep(10000);
	}
	
	fclose(file);
	return 0;
}
