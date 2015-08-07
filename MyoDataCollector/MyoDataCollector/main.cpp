// Created by: Ala Shaabana
// Copyright (C) 2013-2014 WiSeR lab, 
// Distributed under the Myo SDK license agreement. See LICENSE.txt for details.
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <algorithm>
#include <fstream>
#include <cmath>
#include <cstdio>
#include <chrono>
#include <sstream>
#include <unistd.h>
#include <termios.h>
#include <thread>

// The only file that needs to be included to use the Myo C++ SDK is myo.hpp.
#include <myo/myo.hpp>

bool spacebarPressed = false;
int num_channels = 8;


class BufferToggle {
private:
	struct termios t;
	
public:
	
	/*
	 * Disables buffered input
	 */
	void off(void){
		tcgetattr(STDIN_FILENO, &t);	// get the current terminal I/O structure
		t.c_lflag &= ~ICANON;			// Manipulate the flag bits to do what you want it to do
		tcsetattr(STDIN_FILENO, TCSANOW, &t);	// Apply new settings
	}
	
	/*
	 * Enables buffered input
	 */
	void on(void){
		tcgetattr(STDIN_FILENO, &t);		// get current terminal I/O structure
		t.c_lflag |= ICANON;				// Manipulate the flag bits to do what we want it to do
		tcsetattr(STDIN_FILENO, TCSANOW, &t);	// Apply new settings
	}
};

// Armband data structure
struct ArmBand {
	myo::Myo* myo;
	// These values are set by onArmSync() and onArmUnsync() above.
	bool onArm=false;
	myo::Arm whichArm = myo::armUnknown;
	
	// These values are set by onOrientationData() and onPose() above, as euler angles
	int roll_w=0, pitch_w=0, yaw_w=0;
	
	// Raw values
	float roll=0,pitch=0,yaw=0;
	
	// These values are set by onAccelerometerData()
	// Raw accelerometer values
	float xAccelRaw=0, yAccelRaw=0, zAccelRaw=0;
	
	// Values are set by onGyroscopeData()
	// Raw Gyroscope values
	float xGyroRaw=0, yGyroRaw=0, zGyroRaw=0;
	
	// Values are set by onEmgData()
	// Raw emg data
	int emg[8];
	
	// This is set by onUnlocked() and onLocked() above.
	bool isUnlocked=false;
	
	
	// Timestamp of last event
	std::chrono::milliseconds accel_timestamp, gyro_timestamp, emgTimestamp;

	myo::Pose currentPose;
};

struct EmgData{
	// Timestamp of event
	std::chrono::milliseconds timestamp;
	
	// Raw emg data in 8 channels
	int emg[8];
};

// Vector to hold all raw emg data
std::vector<EmgData> raw_data;

// Classes that inherit from myo::DeviceListener can be used to receive events from Myo devices. DeviceListener
// provides several virtual functions for handling different kinds of events. If you do not override an event, the
// default behavior is to do nothing.
// Roll - x, pitch - y, yaw - z
class DataCollector : public myo::DeviceListener {
public:
    float roll, pitch, yaw;
	bool spacebarPressed = false;
    // Data file values
    std::string accelDataFileNameRight = "right forearm/accelerometer_50 Hz.csv";
    std::string accelDataFileNameLeft = "left forearm/accelerometer_50 Hz.csv";
	std::string gyroDataFileNameRight = "right forearm/gyroscope_50 Hz.csv";
	std::string gyroDataFileNameLeft = "left forearm/gyroscope_50 Hz.csv";
	std::string emgDataFileNameRight = "right forearm/emg.csv";
	std::string emgDataFileNameLeft = "left forearm/emg.csv";
	
    std::ofstream accelDataFileRight, accelDataFileLeft, gyroDataFileRight, gyroDataFileLeft, emgDataFileRight, emgDataFileLeft;
    std::chrono::high_resolution_clock::time_point begin,end;

    
    // multiple myo container
    std::vector<ArmBand> knownMyos;
	
	DataCollector() { }
	
	DataCollector(std::vector<ArmBand>& passedKnownMyos) : knownMyos(passedKnownMyos){
        knownMyos = passedKnownMyos;
		
		// Accelerometer data files
		accelDataFileRight.open(DataCollector::accelDataFileNameRight, std::ios::out | std::ios::trunc);
        accelDataFileLeft.open(DataCollector::accelDataFileNameLeft, std::ios::out | std::ios::trunc);
		
		// Gyroscope data files
		gyroDataFileRight.open(DataCollector::gyroDataFileNameRight, std::ios::out | std::ios::trunc);
		gyroDataFileLeft.open(DataCollector::gyroDataFileNameLeft, std::ios::out | std::ios::trunc);
		
		// EMG data files
		emgDataFileRight.open(DataCollector::emgDataFileNameRight, std::ios::out | std::ios::trunc);
		emgDataFileLeft.open(DataCollector::emgDataFileNameLeft, std::ios::out | std::ios::trunc);
		
        begin = std::chrono::high_resolution_clock::now();
    }
	
	
    void onPair(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion){
        // Print out the MAC address of the armband we paired with.
        
        // The pointer address we get for a Myo is unique - in other words, it's safe to compare two Myo pointers to
        // see if they're referring to the same Myo.
        
        // Add the Myo pointer to our list of known Myo devices. This list is used to implement identifyMyo() below so
        // that we can give each Myo a nice short identifier.
		
		if(knownMyos.size() < 2){
			ArmBand myoBand;
			myoBand.myo = myo;
			myo->setStreamEmg(myo::Myo::streamEmgEnabled);
			knownMyos.push_back(myoBand);
			
			// Now that we've added it to our list, get our short ID for it and print it out.
			std::cout << "Paired with " << identifyMyo(myo) << "." << std::endl;
		} else {
			std::cerr << "ERROR: Two Myos are already connected!" << std::endl;
		}
    }
    
    // onUnpair() is called whenever the Myo is disconnected from Myo Connect by the user.
    void onUnpair(myo::Myo* myo, uint64_t timestamp)
    {
        // We've lost a Myo.
        // Let's clean up some leftover state.
        knownMyos[identifyMyo(myo)].roll_w = 0;
        knownMyos[identifyMyo(myo)].pitch_w = 0;
        knownMyos[identifyMyo(myo)].yaw_w = 0;
        knownMyos[identifyMyo(myo)].onArm = false;
        knownMyos[identifyMyo(myo)].isUnlocked = false;
		
        
    }
    
    // onOrientationData() is called whenever the Myo device provides its current orientation, which is represented
    // as a unit quaternion.
    void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat)
    {
        using std::atan2;
        using std::asin;
        using std::sqrt;
        using std::max;
        using std::min;
        
        // Calculate Euler angles (roll, pitch, and yaw) from the unit quaternion.
        roll = atan2(2.0f * (quat.w() * quat.x() + quat.y() * quat.z()),
                           1.0f - 2.0f * (quat.x() * quat.x() + quat.y() * quat.y()));
        pitch = asin(max(-1.0f, min(1.0f, 2.0f * (quat.w() * quat.y() - quat.z() * quat.x()))));
        yaw = atan2(2.0f * (quat.w() * quat.z() + quat.x() * quat.y()),
                          1.0f - 2.0f * (quat.y() * quat.y() + quat.z() * quat.z()));
		
		// For now, we will do nothing with these Euler angles as we do not need them, however we may use
		// them later.
    }
    
    // onPose() is called whenever the Myo detects that the person wearing it has changed their pose, for example,
    // making a fist, or not making a fist anymore.
    void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
    {
        knownMyos[identifyMyo(myo)].currentPose = pose;
        
        if (pose != myo::Pose::unknown && pose != myo::Pose::rest) {
            // Tell the Myo to stay unlocked until told otherwise. We do that here so you can hold the poses without the
            // Myo becoming locked.
            knownMyos[identifyMyo(myo)].myo->unlock(myo::Myo::unlockHold);
            
            // Notify the Myo that the pose has resulted in an action, in this case changing
            // the text on the screen. The Myo will vibrate.
            knownMyos[identifyMyo(myo)].myo->notifyUserAction();
        } else {
            // Tell the Myo to stay unlocked only for a short period. This allows the Myo to stay unlocked while poses
            // are being performed, but lock after inactivity.
            knownMyos[identifyMyo(myo)].myo->unlock(myo::Myo::unlockTimed);
        }
    }
    
    // onArmSync() is called whenever Myo has recognized a Sync Gesture after someone has put it on their
    // arm. This lets Myo know which arm it's on and which way it's facing.
    void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection)
    {
        knownMyos[identifyMyo(myo)].onArm = true;
        knownMyos[identifyMyo(myo)].whichArm = arm;
    }
    
    // onArmUnsync() is called whenever Myo has detected that it was moved from a stable position on a person's arm after
    // it recognized the arm. Typically this happens when someone takes Myo off of their arm, but it can also happen
    // when Myo is moved around on the arm.
    void onArmUnsync(myo::Myo* myo, uint64_t timestamp)
    {
        knownMyos[identifyMyo(myo)].onArm = false;
    }
    
    // onUnlock() is called whenever Myo has become unlocked, and will start delivering pose events.
    void onUnlock(myo::Myo* myo, uint64_t timestamp)
    {
        knownMyos[identifyMyo(myo)].isUnlocked = true;
    }
    
    // onLock() is called whenever Myo has become locked. No pose events will be sent until the Myo is unlocked again.
    void onLock(myo::Myo* myo, uint64_t timestamp)
    {
        knownMyos[identifyMyo(myo)].isUnlocked = false;
    }
    
    float magnitude(float x, float y, float z){
        return sqrt((x*x) + (y*y) + (z*z));
    }
    
    // This is a utility function implemented for this sample that maps a myo::Myo* to a unique ID starting at 1.
    // It does so by looking for the Myo pointer in knownMyos, which onPair() adds each Myo into as it is paired.
    size_t identifyMyo(myo::Myo* myo) {
        // Walk through the list of Myo devices that we've seen pairing events for.
        for (size_t i = 0; i < knownMyos.size(); ++i) {
            // If two Myo pointers compare equal, they refer to the same Myo device.
            if (knownMyos[i].myo == myo) {
                return i;
            }
        }
        
        return 0;
    }

    
    // There are other virtual functions in DeviceListener that we could override here, like onAccelerometerData() & onGyroscopeData()
	void onAccelerometerData(myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& accel){
		
		// Simply write the raw values to the armband data struct, we will apply filters with
		// the Python script that handles these values
		knownMyos[identifyMyo(myo)].xAccelRaw = accel[0];
		knownMyos[identifyMyo(myo)].yAccelRaw = accel[1];
		knownMyos[identifyMyo(myo)].zAccelRaw = accel[2];
		knownMyos[identifyMyo(myo)].accel_timestamp = std::chrono::duration_cast< std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch());
		printIMU(knownMyos[identifyMyo(myo)]);
	}
	
	void onGyroscopeData(myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& gyro){
		
		// Simply write the raw values to the armband data struct, we will apply filters with
		// the Python script that handles these values
		knownMyos[identifyMyo(myo)].xGyroRaw = gyro[0];
		knownMyos[identifyMyo(myo)].yGyroRaw = gyro[1];
		knownMyos[identifyMyo(myo)].zGyroRaw = gyro[2];
		knownMyos[identifyMyo(myo)].gyro_timestamp = std::chrono::duration_cast< std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch());
		printIMU(knownMyos[identifyMyo(myo)]);
	}
	
	void onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg){
		if(spacebarPressed){
			knownMyos[identifyMyo(myo)].emgTimestamp = std::chrono::duration_cast< std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch());
			for(size_t i = 0; i < 8; ++i){
				knownMyos[identifyMyo(myo)].emg[i] = static_cast<int>(emg[i]);
			}
			saveEMG(knownMyos[identifyMyo(myo)]);
		}
	}
	
	/*
	 * Function to save emg data of inputted myo armband 
	 * into master raw data vector
	 * for later feature extraction
	 */
	void saveEMG(ArmBand myoBand){
		EmgData data;
		data.timestamp = myoBand.emgTimestamp;
		for(int i = 0; i < num_channels; i++){
			data.emg[i] = myoBand.emg[i];
		}
		raw_data.push_back(data);
	}
	
	/*
	 * Function to print emg data of inputted myo armband 
	 * to file
	 */
	void printEMG(ArmBand myoBand){
		std::string emgOutput = std::to_string(myoBand.emgTimestamp.count());
		for(size_t i = 0; i < num_channels; ++i){
			emgOutput += "," + std::to_string(myoBand.emg[i]);
		}
		emgOutput += "\n";
		
		if(myoBand.whichArm == myo::armRight){
			emgDataFileRight << emgOutput;
		} else if(myoBand.whichArm == myo::armLeft){
			emgDataFileLeft << emgOutput;
		}

	}
	
    // We define this function to print the current values that were updated by the on...() functions above.
    // This function also prints the x, y, and z (roll, pitch, and yaw, respectively) to a file.
    void printIMU(ArmBand myoBand)
    {
        for(size_t i = 0; i < knownMyos.size(); ++i){
            // Find Myo corresponding to this arm
            myoBand = knownMyos[i];
			
            // Clear the current line
         //   std::cout << '\r';
	
			std::chrono::milliseconds ms = std::chrono::duration_cast< std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch());
			
			std::string timestamp = std::to_string(ms.count());
			
			std::string accelOutput = std::to_string(myoBand.accel_timestamp.count()) + "," + std::to_string(myoBand.xAccelRaw) + "," + std::to_string(myoBand.yAccelRaw) + "," + std::to_string(myoBand.zAccelRaw) +
				","	+ std::to_string((float)magnitude(myoBand.xAccelRaw, myoBand.yAccelRaw,	myoBand.zAccelRaw)) + "\n";
			
			std::string gyroOutput = std::to_string(myoBand.gyro_timestamp.count()) + "," + std::to_string(myoBand.xGyroRaw) + "," + std::to_string(myoBand.yGyroRaw) + "," + std::to_string(myoBand.zGyroRaw) +
				"," + std::to_string((float)magnitude(myoBand.xGyroRaw, myoBand.yGyroRaw, myoBand.zGyroRaw)) + "\n";
			
			
			if(myoBand.whichArm == myo::armRight){
				accelDataFileRight << accelOutput;
				gyroDataFileRight << gyroOutput;
			} else if(myoBand.whichArm == myo::armLeft){
				accelDataFileLeft << accelOutput;
				gyroDataFileLeft << gyroOutput;
			}
			
            // Print out the orientation. Orientation data is always available, even if no arm is currently recognized.
     /*       std::cout << '[' << std::string(myoBand.xAccelRaw, '*') << std::string(18 - myoBand.xAccelRaw, ' ') << ']'
            << '[' << std::string(myoBand.yAccelRaw, '*') << std::string(18 - myoBand.yAccelRaw, ' ') << ']'
            << '[' << std::string(myoBand.zAccelRaw, '*') << std::string(18 - myoBand.zAccelRaw, ' ') << ']';
        */
            
        /*    if (myoBand.onArm) {
                // Print out the lock state, the currently recognized pose, and which arm Myo is being worn on.
            
                // Pose::toString() provides the human-readable name of a pose. We can also output a Pose directly to an
                // output stream (e.g. std::cout << currentPose;). In this case we want to get the pose name's length so
                // that we can fill the rest of the field with spaces below, so we obtain it as a string using toString().
                std::string poseString = myoBand.currentPose.toString();
            
                std::cout << '[' << (myoBand.isUnlocked ? "unlocked" : "locked  ") << ']'
                << '[' << (myoBand.whichArm == myo::armLeft ? "L" : "R") << ']'
                << '[' << poseString << std::string(14 - poseString.size(), ' ') << ']';
            } else {
                // Print out a placeholder for the arm and pose when Myo doesn't currently know which arm it's on.
                std::cout << '[' << std::string(8, ' ') << ']' << "[?]" << '[' << std::string(14, ' ') << ']';
            }*/
          //  std::cout << identifyMyo(myoBand.myo);

     //       std::cout << std::flush;
        }
    }
};

/*
 * Function to extract the Mean Absolute Value
 * feature set (one value for each channel, making it 8 values)
 * input: empty array (size 8),
 * output: same array populated with MAV of each channel
 */

void extractMAV(double *MAVs){
	long L = raw_data[0].timestamp.count() - raw_data[raw_data.size()].timestamp.count();
	int channel_values[8] = {0};
	for(int i = 0; i < raw_data.size(); ++i){
		for(int j = 0; j < num_channels; ++j){
			channel_values[j] += std::abs(raw_data[i].emg[j]);
		}
	}
	for(int i = 0; i < num_channels; ++i){
		MAVs[i] = channel_values[i] / L;
	}
}

/*
 * Simple frequency measure representing the # of times that 
 * a waveform changes polarity. Due to the possibility of noise
 * induced zero crossings, a threshold must be defined. So,
 * a pair of consecutive samples constitutes a zero crossing only 
 * if their absolute difference exceeds both a noise threshold and 
 * their absolute sum (i.e. they have different polarity).
 * Threshold assumed to be *10* by default, but we can experiment
 * with different values later.
 */
void extractZC(double *ZCs){
	int T = 10;		// noise threshold
	for(int i = 0; i < raw_data.size()-1; ++i){
		for(int j = 0; j < num_channels; ++j){
			if((raw_data[i].emg[j] - raw_data[i+1].emg[j]) > std::max((raw_data[i].emg[j] - raw_data[i+1].emg[j]), T)){
				ZCs[j]++;
			}
		}
	}
}

/*
 * Function to extract the slope sign changes (SSCs) 
 * for each channel in EMG data. SSC is the number of times
 * that a waveform changes slope polarity. It may 
 * provide additional frequency information about the
 * signal. We will use the same Threshold as the ZC extraction
 * method above. (Not sure of this works yet, untested)
 */
void extractSSC(double *SSCs){
	int T = 10;
	for(int i = 1; i < raw_data.size()-1; ++i){
		for(int j = 0; j < num_channels; ++j){
			if((raw_data[i].emg[j] > std::max(raw_data[i-1].emg[j], raw_data[i+1].emg[j])
			   || raw_data[i].emg[j] < std::min(raw_data[i-1].emg[j], raw_data[i+1].emg[j]))
			   && std::max((std::abs(raw_data[i+1].emg[j] - raw_data[i].emg[j])),
						   (std::abs(raw_data[i].emg[j] - raw_data[i-1].emg[j]))) > T){
				   SSCs[j]++;
			}
		}
	}
}

/*
 * Function to extract the waveform length, which is the total 
 * length of the signal over the data window, which represents
 * a combined measure of aptitude, frequency and duration
 */

void extractWL(double *WLs){
	int channel_values[8] = {0};
	for(int i = 1; i < raw_data.size(); ++i){
		for(int j = 0; j < num_channels; ++j){
			WLs[j] += std::abs(raw_data[i].emg[j] - raw_data[i-1].emg[j]);
		}
	}
}

/*
 * Function to extract features from raw_data
 * and input them into file.
 * input is global variable raw_data
 * output is features written into file
 */
void extractFeatures(){
	double MAVs[8] = {0};
	double ZCs[8] = {0};
	double WLs[8] = {0};
	double SSCs[8] = {0};
	
	extractMAV(MAVs);
	extractZC(ZCs);
	extractWL(WLs);
	extractSSC(SSCs);
}



void toggleSpace(DataCollector* collector){
	while(1){
		if(std::getchar() == 32 || std::getchar() == ' '){
			collector->spacebarPressed = !collector->spacebarPressed;
			if(collector->spacebarPressed){
				std::cout << "Begin Measurement" << std::endl;
			} else {
				std::cout << "End Measurement" << std::endl;
				extractFeatures();
			}
		}
	}
}

int main(int argc, char** argv)
{
	BufferToggle bt;
	bt.off();
    // We catch any exceptions that might occur below -- see the catch statement for more details.
    try {
		// multiple myo container
		std::vector<ArmBand> knownMyos;
		
        // First, we create a Hub with our application identifier. Be sure not to use the com.example namespace when
        // publishing your application. The Hub provides access to one or more Myos.
        myo::Hub hub("com.example.hello-myo");
        
        std::cout << "Attempting to find a Myo..." << std::endl;
		
        // Next we construct an instance of our DeviceListener, so that we can register it with the Hub.
		DataCollector collector(knownMyos);
		
        // Hub::addListener() takes the address of any object whose class inherits from DeviceListener, and will cause
        // Hub::run() to send events to all registered device listeners.
        hub.addListener(&collector);
		
		std::thread t1(toggleSpace, &collector);
		
        // Finally we enter our main loop.
		
		// In each iteration of our main loop, we run the Myo event loop for a set number of milliseconds.
		// In this case, we wish to update our display 20 times a second, so we run for 1000/20 milliseconds.
		while(1){
		
			hub.run(20);
		}
		
           // After processing events, we call the print() member function we defined above to print out the values we've
		// obtained from any events that have occurred.
        //    collector.print();

		
        
        // If a standard exception occurred, we print out its message and exit.
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cerr << "Press enter to continue.";
        std::cin.ignore();
        return 1;
    }
}
