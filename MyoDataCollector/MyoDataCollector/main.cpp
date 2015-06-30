// Copyright (C) 2013-2014 WiSeR lab
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

// The only file that needs to be included to use the Myo C++ SDK is myo.hpp.
#include <myo/myo.hpp>

// Classes that inherit from myo::DeviceListener can be used to receive events from Myo devices. DeviceListener
// provides several virtual functions for handling different kinds of events. If you do not override an event, the
// default behavior is to do nothing.
// Roll - x, pitch - y, yaw - z


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
	
	// This is set by onUnlocked() and onLocked() above.
	bool isUnlocked=false;
	
	myo::Pose currentPose;
};

class DataCollector : public myo::DeviceListener {
public:
    float roll, pitch, yaw;
	
    // Data file values
    std::string accelDataFileNameRight = "accelerometerRight_50hz.csv";
    std::string accelDataFileNameLeft = "accelerometerLeft_50hz.csv";
	std::string gyroDataFileNameRight = "gyroscopeRight_50hz.csv";
	std::string gyroDataFileNameLeft = "gyroscopeLeft_50hz.csv";
	
    std::ofstream accelDataFileRight, accelDataFileLeft, gyroDataFileRight, gyroDataFileLeft;
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
	}
	
	void onGyroscopeData(myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& gyro){
		
		// Simply write the raw values to the armband data struct, we will apply filters with
		// the Python script that handles these values
		knownMyos[identifyMyo(myo)].xGyroRaw = gyro[0];
		knownMyos[identifyMyo(myo)].yGyroRaw = gyro[1];
		knownMyos[identifyMyo(myo)].zGyroRaw = gyro[2];
	}
	
    // We define this function to print the current values that were updated by the on...() functions above.
    // This function also prints the x, y, and z (roll, pitch, and yaw, respectively) to a file.
    void print()
    {
        for(size_t i = 0; i < knownMyos.size(); ++i){
            // Find Myo corresponding to this arm
            ArmBand myoBand = knownMyos[i];
        
            // Clear the current line
            std::cout << '\r';
	
			std::chrono::milliseconds ms = std::chrono::duration_cast< std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch());
			
			std::string timestamp = std::to_string(ms.count());
			
			std::string accelOutput = timestamp + "," + std::to_string(myoBand.xAccelRaw) + "," + std::to_string(myoBand.yAccelRaw) + "," + std::to_string(myoBand.zAccelRaw) +
				","	+ std::to_string((float)magnitude(myoBand.xAccelRaw, myoBand.yAccelRaw,	myoBand.zAccelRaw)) + "\n";
			
			std::string gyroOutput = timestamp + "," + std::to_string(myoBand.xGyroRaw) + "," + std::to_string(myoBand.yGyroRaw) + "," + std::to_string(myoBand.zGyroRaw) +
				"," + std::to_string((float)magnitude(myoBand.xGyroRaw, myoBand.yGyroRaw, myoBand.zGyroRaw)) + "\n";
			
			if(myoBand.whichArm == myo::armRight){
				accelDataFileRight << accelOutput;
				gyroDataFileRight << gyroOutput;
			} else if(myoBand.whichArm == myo::armLeft){
				accelDataFileLeft << accelOutput;
				gyroDataFileLeft << gyroOutput;
			}
			
            // Print out the orientation. Orientation data is always available, even if no arm is currently recognized.
            std::cout << '[' << std::string(myoBand.roll_w, '*') << std::string(18 - myoBand.roll_w, ' ') << ']'
            << '[' << std::string(myoBand.pitch_w, '*') << std::string(18 - myoBand.pitch_w, ' ') << ']'
            << '[' << std::string(myoBand.yaw_w, '*') << std::string(18 - myoBand.yaw_w, ' ') << ']';
        
            
            if (myoBand.onArm) {
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
            }
            std::cout << identifyMyo(myoBand.myo);

            std::cout << std::flush;
        }
    }
};

int main(int argc, char** argv)
{
    
    // We catch any exceptions that might occur below -- see the catch statement for more details.
    try {
		// multiple myo container
		std::vector<ArmBand> knownMyos;
		
        // First, we create a Hub with our application identifier. Be sure not to use the com.example namespace when
        // publishing your application. The Hub provides access to one or more Myos.
        myo::Hub hub("com.example.hello-myo");
        
        std::cout << "Attempting to find a Myo..." << std::endl;
        
        // Next, we attempt to find a Myo to use. If a Myo is already paired in Myo Connect, this will return that Myo
        // immediately.
        // waitForMyo() takes a timeout value in milliseconds. In this case we will try to find a Myo for 10 seconds, and
        // if that fails, the function will return a null pointer.
 //       myo::Myo* myo1 = hub.waitForMyo(10000);
        
        // If waitForMyo() returned a null pointer, we failed to find a Myo, so exit with an error message.
/*        if (!myo1) {
            throw std::runtime_error("Unable to find the first Myo!");
        } else {
			std::cout << "Found first Myo!" << std::endl;
            ArmBand armband;
            armband.myo = myo1;
            knownMyos.push_back(armband);
        }
		
        
        myo::Myo* myo2 = hub.waitForMyo(10000);
        if(!myo2){
            std::cout << "Proceeding without second Myo" << std::endl;
        } else {
			std::cout << "Found second Myo!" << std::endl;
			ArmBand armband;
			armband.myo = myo2;
			knownMyos.push_back(armband);
        }*/
		
        // Next we construct an instance of our DeviceListener, so that we can register it with the Hub.
		DataCollector collector(knownMyos);
		
        // Hub::addListener() takes the address of any object whose class inherits from DeviceListener, and will cause
        // Hub::run() to send events to all registered device listeners.
        hub.addListener(&collector);
        
        // Finally we enter our main loop.
        while (1) {
            // In each iteration of our main loop, we run the Myo event loop for a set number of milliseconds.
            // In this case, we wish to update our display 20 times a second, so we run for 1000/20 milliseconds.
            hub.run(20);
            // After processing events, we call the print() member function we defined above to print out the values we've
            // obtained from any events that have occurred.
            collector.print();
        }
        
        // If a standard exception occurred, we print out its message and exit.
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cerr << "Press enter to continue.";
        std::cin.ignore();
        return 1;
    }
}
