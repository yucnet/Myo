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
#include <complex>
#include <valarray>
#include <SFML/Window.hpp>
#include <SFML/Audio.hpp>
#include <SFML/Graphics.hpp>

// The only file that needs to be included to use the Myo C++ SDK is myo.hpp.
#include <myo/myo.hpp>

bool keyPressed = false;
int num_channels = 8;
int num_axis = 3;
int fft_data_sample_size = 32;
sf::Event event;

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
	myo::Arm whichArm;
	
	// Raw emg data in 8 channels
	int emg[8];
};

struct IMUData{
	// Timestamp of event
	std::chrono::milliseconds timestamp;
	myo::Arm whichArm;
	
	// Raw accel data
	float x=0, y=0, z=0;
};

// Vector to hold all raw emg data
std::vector<EmgData> raw_emg_data;
std::vector<IMUData> raw_imu_data;
// Classes that inherit from myo::DeviceListener can be used to receive events from Myo devices. DeviceListener
// provides several virtual functions for handling different kinds of events. If you do not override an event, the
// default behavior is to do nothing.
// Roll - x, pitch - y, yaw - z
class DataCollector : public myo::DeviceListener {
public:
	float roll, pitch, yaw;
	bool keyPressed = false;
	// Data file values
	std::string accelDataFileNameRight = "right forearm/accelerometer_50 Hz.csv";
	std::string accelDataFileNameLeft = "left forearm/accelerometer_50 Hz.csv";
	std::string gyroDataFileNameRight = "right forearm/gyroscope_50 Hz.csv";
	std::string gyroDataFileNameLeft = "left forearm/gyroscope_50 Hz.csv";
	std::string emgDataFileNameRight = "right forearm/emg.csv";
	std::string emgDataFileNameLeft = "left forearm/emg.csv";
	
	std::ofstream accelDataFileRight, accelDataFileLeft,
	gyroDataFileRight, gyroDataFileLeft,
	emgDataFileRight, emgDataFileLeft,
	emgFeatureFileLeft, emgFeatureFileRight;
	
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
		if(keyPressed){
			// Simply write the raw values to the armband data struct, we will apply filters with
			// the Python script that handles these values
			knownMyos[identifyMyo(myo)].xAccelRaw = accel[0];
			knownMyos[identifyMyo(myo)].yAccelRaw = accel[1];
			knownMyos[identifyMyo(myo)].zAccelRaw = accel[2];
			knownMyos[identifyMyo(myo)].accel_timestamp = std::chrono::duration_cast< std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch());
			saveIMU(knownMyos[identifyMyo(myo)]);
		}
	}
	
	void onGyroscopeData(myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& gyro){
		
		// Simply write the raw values to the armband data struct, we will apply filters with
		// the Python script that handles these values
		knownMyos[identifyMyo(myo)].xGyroRaw = gyro[0];
		knownMyos[identifyMyo(myo)].yGyroRaw = gyro[1];
		knownMyos[identifyMyo(myo)].zGyroRaw = gyro[2];
		knownMyos[identifyMyo(myo)].gyro_timestamp = std::chrono::duration_cast< std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch());
//		printIMU(knownMyos[identifyMyo(myo)]);
	}
	
	void onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg){
		if(keyPressed){
			knownMyos[identifyMyo(myo)].emgTimestamp = std::chrono::duration_cast< std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch());
			for(size_t i = 0; i < 8; ++i){
				knownMyos[identifyMyo(myo)].emg[i] = static_cast<int>(emg[i]);
			}
			saveEMG(knownMyos[identifyMyo(myo)]);
		}
	}
	
	/*
	 * Function to save IMU data of inputted myo armband
	 * into master raw IMU data vector (accelerometer only, for now)
	 * for later feature extraction
	 */
	void saveIMU(ArmBand myoBand) {
		IMUData data;
		data.timestamp = myoBand.accel_timestamp;
		data.whichArm = myoBand.whichArm;
		data.x = myoBand.xAccelRaw;
		data.y = myoBand.yAccelRaw;
		data.z = myoBand.zAccelRaw;
		raw_imu_data.push_back(data);
		
	}
	
	/*
	 * Function to save emg data of inputted myo armband
	 * into master raw data vector
	 * for later feature extraction
	 */
	void saveEMG(ArmBand myoBand){
		EmgData data;
		data.timestamp = myoBand.emgTimestamp;
		data.whichArm = myoBand.whichArm;
		for(int i = 0; i < num_channels; i++){
			data.emg[i] = myoBand.emg[i];
		}
		raw_emg_data.push_back(data);
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

/**
 * Functions for determining the FFT.
 *
 * NOTE: this is a quick solution with O(n) complexity.
 * Code assumes array size is power of 2.. which is reasonable
 * since this is something we stay persistent with throughout.
 * Future fix: make array size requirement more lenient.
 */

typedef std::complex<double> Complex;
typedef std::valarray<Complex> CArray;

// Cooley-Tukey FFT (in-place, breadth-first, decimation-in-frequency)
// Better optimized but less intuitive
void fft(CArray &x)
{
	// DFT
	size_t N = x.size(), k = N, n;
	double thetaT = 3.14159265358979323846264338328L / N;
	Complex phiT = Complex(cos(thetaT), sin(thetaT)), T;
	while (k > 1)
	{
		n = k;
		k >>= 1;
		phiT = phiT * phiT;
		T = 1.0L;
		for (int l = 0; l < k; ++l)
		{
			for (int a = l; a < N; a += n)
			{
				long b = a + k;
				Complex t = x[a] - x[b];
				x[a] += x[b];
				x[b] = t * T;
			}
			T *= phiT;
		}
	}
	// Decimate
	unsigned int m = (unsigned int)log2(N);
	for (int a = 0; a < N; ++a)
	{
		int b = a;
		// Reverse bits
		b = (((b & 0xaaaaaaaa) >> 1) | ((b & 0x55555555) << 1));
		b = (((b & 0xcccccccc) >> 2) | ((b & 0x33333333) << 2));
		b = (((b & 0xf0f0f0f0) >> 4) | ((b & 0x0f0f0f0f) << 4));
		b = (((b & 0xff00ff00) >> 8) | ((b & 0x00ff00ff) << 8));
		b = ((b >> 16) | (b << 16)) >> (32 - m);
		if (b > a)
		{
			Complex t = x[a];
			x[a] = x[b];
			x[b] = t;
		}
	}
	// Normalize
	Complex f = 1.0 / sqrt(N);
	for (int i = 0; i < N; i++)
	x[i] *= f;
}

// inverse fft (in-place)
void ifft(CArray& x)
{
	// conjugate the complex numbers
	x = x.apply(std::conj);
 
	// forward fft
	fft( x );
 
	// conjugate the complex numbers again
	x = x.apply(std::conj);
 
	// scale the numbers
	x /= x.size();
}

/*
 * Function to extract the Mean Absolute Value
 * feature set (one value for each channel, making it 8 values)
 * input: empty array (size 8),
 * output: same array populated with MAV of each channel
 */
void extractMAV(std::vector<double> &MAVs){
	int L = (int)raw_emg_data[raw_emg_data.size()-1].timestamp.count() - (int)raw_emg_data[0].timestamp.count();
	for(int i = 0; i < raw_emg_data.size(); ++i){
		for(int j = 0; j < num_channels; ++j){
			MAVs[j] += (float)((float)std::abs(raw_emg_data[i].emg[j]) / (float)L);
		}
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
void extractZC(std::vector<double> &ZCs){
	int T = 10;		// noise threshold
	for(int i = 0; i < raw_emg_data.size()-1; ++i){
		for(int j = 0; j < num_channels; ++j){
			if((raw_emg_data[i+1].emg[j] - raw_emg_data[i].emg[j]) > std::max((raw_emg_data[i].emg[j] - raw_emg_data[i+1].emg[j]), T)){
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
void extractSSC(std::vector<double> &SSCs){
	int T = 10;
	for(int i = 1; i < raw_emg_data.size()-1; ++i){
		for(int j = 0; j < num_channels; ++j){
			if((raw_emg_data[i].emg[j] > std::max(raw_emg_data[i-1].emg[j], raw_emg_data[i+1].emg[j])
				|| raw_emg_data[i].emg[j] < std::min(raw_emg_data[i-1].emg[j], raw_emg_data[i+1].emg[j]))
			   && std::max((std::abs(raw_emg_data[i+1].emg[j] - raw_emg_data[i].emg[j])),
						   (std::abs(raw_emg_data[i].emg[j] - raw_emg_data[i-1].emg[j]))) > T){
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

void extractWL(std::vector<double> &WLs){
	for(int i = 1; i < raw_emg_data.size(); ++i){
		for(int j = 0; j < num_channels; ++j){
			WLs[j] += std::abs(raw_emg_data[i].emg[j] - raw_emg_data[i-1].emg[j]);
		}
	}
}

void extractFFT(double **fft_data){
	for(int i = 0; i < num_channels; ++i){
		const size_t size = fft_data_sample_size;					// Limiting the size of data samples used for now.
		Complex *emg = new Complex[size];
		
		for(int j = 0; j < fft_data_sample_size ; ++j){
			emg[i] = (Complex)raw_emg_data[j].emg[i];
		}
		
		CArray data(emg, size);
		
		fft(data);
		
		for(int k = 0; k < data.size(); ++k){
			fft_data[i][k] = std::abs(data[k].real());
		}
		
		// we are no longer using emg, so we free up the space it used to take up
		//	delete [] emg;
	}
}
/*
 * Function to extract the average of acceleration for x,y,z axis, represented
 * inside input parameter axisAverage[0,1,2], respectively.
 */
void extractAverage(std::vector<double> &axisAverage){
	for(int i = 0; i < raw_imu_data.size(); ++i){
		axisAverage[0] += raw_imu_data[i].x;
		axisAverage[1] += raw_imu_data[i].y;
		axisAverage[2] += raw_imu_data[i].z;
	}
	axisAverage[0] /= raw_imu_data.size();
	axisAverage[1] /= raw_imu_data.size();
	axisAverage[2] /= raw_imu_data.size();
}

/*
 * Simple function to extract the standard deviation from a given vector.
 */
double deviation(std::vector<double> v, double ave)
{
	double E=0;
	// Quick Question - Can vector::size() return 0?
	double inverse = 1.0 / static_cast<double>(v.size());
	for(unsigned int i=0;i<v.size();i++)
	{
		E += pow(static_cast<double>(v[i]) - ave, 2);
	}
	return sqrt(inverse * E);
}

/*
 * Function to extract the standard deviation for each x,y,z axis, represented
 * inside input parameter stds[0,1,2], respectively.
 */
void extractStd(std::vector<double> &stds, std::vector<double> &axisAverage){
	std::vector<double> stdX, stdY, stdZ;
	for(int i = 0; i < raw_imu_data.size(); ++i){
		stdX.push_back(raw_imu_data[i].x);
		stdY.push_back(raw_imu_data[i].y);
		stdZ.push_back(raw_imu_data[i].z);
	}
	stds.push_back(deviation(stdX, axisAverage[0]));
	stds.push_back(deviation(stdX, axisAverage[1]));
	stds.push_back(deviation(stdX, axisAverage[2]));
}

/*
 * Function to extract average absolute difference between the value of each 
 * of the readings in the acceleration data, and the mean value over those values
 * this is done for each axis, represented inside input parameter AAD[0,1,2], respectively.
 */
void extractAAD(std::vector<double> &AAD, std::vector<double> &axisAverage){
	int sumX = 0, sumY = 0, sumZ = 0;
	int total = 0;
	// We get the sum of absolute differences between each raw value and the mean
	for(int i = 0; i < raw_imu_data.size(); ++i){
		sumX += std::abs(raw_imu_data[i].x - axisAverage[0]);
		sumY += std::abs(raw_imu_data[i].y - axisAverage[1]);
		sumZ += std::abs(raw_imu_data[i].z - axisAverage[2]);
		total++;
	}
	
	AAD.push_back(sumX / total);
	AAD.push_back(sumY / total);
	AAD.push_back(sumZ / total);
}

/*
 * Function to extract average resultant acceleration, which is the average of the
 * square roots of the sum of the values of each axis squared
 */
double extractARA(){
	int sum=0;
	double ara;
	for(int i = 0; i < raw_imu_data.size(); ++i){
		sum += sqrt(pow(raw_imu_data[i].x, 2) + pow(raw_imu_data[i].y, 2) + pow(raw_imu_data[i].z, 2));
	}
	ara = sum / raw_imu_data.size();
	return ara;
}

void writeToFeaturesFile(std::vector<std::vector<double>> featureArrays){
	std::string emgFeatureFileName = "emgFeatures.csv";
	std::ofstream emgFeatureFile;
	
	// Set up feature file
	emgFeatureFile.open(emgFeatureFileName, std::ios::out | std::ios::app);
	std::string output = "";
	
	for(int i = 0; i < featureArrays.size(); ++i){
		for(int j = 0; j < featureArrays[i].size(); ++j){
			output += std::to_string(featureArrays[i][j]);
			if(j == featureArrays[i].size()-1 && i == featureArrays.size()-1)
				output += "";
			else
				output += ",";
		}
	}
	output += "\n";
	emgFeatureFile << output;
}

/*
 * Function to extract features from raw_emg_data
 * and input them into file.
 * input is global variable raw_emg_data
 * output is features written into file
 */
void extractFeatures(){
	std::vector<std::vector<double>> featureArrays;
	std::vector<double> MAVs(num_channels, 0);
	std::vector<double> ZCs(num_channels, 0);
	std::vector<double> SSCs(num_channels, 0);
	std::vector<double> WLs(num_channels, 0);
	std::vector<double> axisAverage(num_axis, 0);
	std::vector<double> stds(num_axis, 0);
	std::vector<double> AADs(num_axis, 0);
	std::vector<double> ARAs(1,0);
	
	size_t size = fft_data_sample_size;
	
	double** fft_data = new double*[num_channels];
	
	for(int i = 0; i < num_channels; ++i){
		fft_data[i] = new double[size];
	}
	
	// extract time domain EMG features
	extractMAV(MAVs);
	extractWL(WLs);
	extractSSC(SSCs);
	extractZC(ZCs);
	
	// extract frequency domain EMG features
	extractFFT(fft_data);
	
	// extract Accelerometer features
	extractAverage(axisAverage);
	extractStd(stds, axisAverage);
	extractAAD(AADs, axisAverage);
	ARAs.push_back(extractARA());
	
	
	// Add all of the extracted feature arrays to a vector we can later pass to a file
	// First, the fft data
	for(int i = 0; i < num_channels; ++i){
		std::vector<double> data;
		for(int j = 0; j < size; ++j){
			data.push_back(fft_data[i][j]);
		}
		featureArrays.push_back(data);
	}
	delete [] fft_data;
	
	// Now, the rest
	featureArrays.push_back(MAVs);
	featureArrays.push_back(WLs);
	featureArrays.push_back(SSCs);
	featureArrays.push_back(ZCs);
	featureArrays.push_back(axisAverage);
	featureArrays.push_back(stds);
	featureArrays.push_back(AADs);
	featureArrays.push_back(ARAs);
	
	// Now we can write this to file
	writeToFeaturesFile(featureArrays);
}

void toggleSpace(DataCollector* collector){
	int count = 1;
	while(1){
		if(std::getchar() == 32 || std::getchar() == ' '){
			collector->keyPressed = !collector->keyPressed;
			if(collector->keyPressed){
				std::cout << "Begin Measurement" << std::endl;
			} else {
				std::cout << "End Measurement, recorded: " << count << std::endl;
				count++;
				std::cout << raw_emg_data.size() << " -- " << raw_imu_data.size() << std::endl;
				extractFeatures();
				raw_imu_data.clear();
				raw_emg_data.clear();
			}
		}
	}
}

int runHub(DataCollector *collector){
	
	// We catch any exceptions that might occur below -- see the catch statement for more details.
	try {
		
		// First, we create a Hub with our application identifier. Be sure not to use the com.example namespace when
		// publishing your application. The Hub provides access to one or more Myos.
		myo::Hub hub("com.example.hello-myo");
		
		std::cout << "Attempting to find a Myo..." << std::endl;
		
		// Hub::addListener() takes the address of any object whose class inherits from DeviceListener, and will cause
		// Hub::run() to send events to all registered device listeners.
		hub.addListener(collector);
		
	//	std::thread t1(toggleSpace, &collector);
		//	std::thread t2(setupWindow);
		// Finally we enter our main loop.
		
		// In each iteration of our main loop, we run the Myo event loop for a set number of milliseconds.
		// In this case, we wish to update our display 20 times a second, so we run for 1000/20 milliseconds.
		while(1){
			hub.run(20);
		}
		// If a standard exception occurred, we print out its message and exit.
	} catch (const std::exception& e) {
		std::cerr << "Error: " << e.what() << std::endl;
		std::cerr << "Press enter to continue.";
		std::cin.ignore();
		return 1;
	}

}

int main(int argc, char** argv)
{
	
	// multiple myo container
	std::vector<ArmBand> knownMyos;
	
	// Next we construct an instance of our DeviceListener, so that we can register it with the Hub.
	DataCollector collector(knownMyos);
	
	// Before doing anything, run MYO hub
	std::thread t1(runHub, &collector);
	
	// Create the main window
	sf::RenderWindow window(sf::VideoMode(800, 600), "SFML window");
	window.setKeyRepeatEnabled(false);
	
	// Create a graphical text to display
	sf::Font font;
	if (!font.loadFromFile("sansation.ttf")) {
		return EXIT_FAILURE;
	}
	sf::Text text("Press keys to begin", font, 50);
	text.setColor(sf::Color::White);
	
	//center text
	sf::FloatRect textRect = text.getLocalBounds();
	text.setOrigin(textRect.left + textRect.width/2.0f,
				   textRect.top  + textRect.height/2.0f);
	text.setPosition(sf::Vector2f(800/2.0f,600/2.0f));
	
	while (window.isOpen()) {
		sf::Event event;
		while (window.pollEvent(event)) {
			if (event.type == sf::Event::Closed) {
				window.close();
			}
			else if(event.type == sf::Event::KeyPressed) {
				if(event.key.code >= sf::Keyboard::A && event.key.code <= sf::Keyboard::Z) {
					text.setString("KeyPressed!");
					collector.keyPressed = !collector.keyPressed;
					// Place text in center of window
					sf::FloatRect textRect = text.getLocalBounds();
					text.setOrigin(textRect.left + textRect.width/2.0f,
								   textRect.top  + textRect.height/2.0f);
					text.setPosition(sf::Vector2f(800/2.0f,600/2.0f));
				}
			} else if(event.type == sf::Event::KeyReleased) {
				text.setString("KeyReleased!");
				collector.keyPressed = !collector.keyPressed;
				// Place text in center of window
				sf::FloatRect textRect = text.getLocalBounds();
				text.setOrigin(textRect.left + textRect.width/2.0f,
							textRect.top  + textRect.height/2.0f);
				text.setPosition(sf::Vector2f(800/2.0f,600/2.0f));
				extractFeatures();
				raw_imu_data.clear();
				raw_emg_data.clear();
			} else if(event.type == sf::Event::MouseMoved) {
				/* move cursor inside the window to observe how the counter
				 behaves while pressing the keys */
				
			}
		}
		
		// Clear screen
		window.clear();
		
		// Draw the string
		window.draw(text);
		
		// Update display
		window.display();
	}
	
	// Wait for hub to finish executing
	t1.join();

}
