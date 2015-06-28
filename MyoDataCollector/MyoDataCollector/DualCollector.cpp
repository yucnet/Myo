//
//  DualCollector.cpp
//  MyoDataCollector
//
//  Created by Ala Shaabana on 6/23/15.
//  Copyright (c) 2015 Ala Shaabana. All rights reserved.
//
#define _USE_MATH_DEFINES

// The only file that needs to be included to use the Myo C++ SDK is myo.hpp.
#include <myo/myo.hpp>
#include <fstream>

class DualCollector : public myo::DeviceListener {
public:
    
    struct ArmBand {
        myo::Myo* myo;
        myo::Arm whichArm;
        std::string dataFileName;
        std::ofstream dataFile;
    };
    
    int armBandCounter = 0;
    
    // Every time Myo Connect succesfully pairs with a Myo armband, this function will be called.
    //
    // We can rely on the following rules:
    //      - onPair() will only be called once for each Myo device
    //      - no other events will occur involving a given Myo device before onPair() is called with it
    //
    // If we need to do some kind of per-Myo preparation before handling events, we can safely do it in onPair()
    
    void onPair(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion){
        // Print out the MAC address of the armband we paired with.
        
        // The pointer address we get for a Myo is unique
        //      - it is safe to compare two Myo pointers to see if they're referring to the same Myo
        
        // Add the Myo pointer to our list of known Myo devices. This list is used to implement identifyMyo() below so that we can give each Myo a nice, short ID
        ArmBand newMyo;
        newMyo.myo = myo;
        knownMyos.push_back(newMyo);
        
        // Now that we've added it to our list, get our short ID for it and print it out
        std::cout << "Paired with " << identifyMyo(myo) << "." << std::endl;
    }
    
    void onPose(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion){
        std::cout << "Myo " << identifyMyo(myo) << " has connected" << std::endl;
    }
    
    void onConnect(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion){
        std::cout << "Myo " << identifyMyo(myo) << " has connected." << std::endl;
    }
    
    void onDisconnect(myo::Myo* myo, uint64_t timestamp){
        std::cout << "Myo " << identifyMyo(myo) << " has disconnected." << std::endl;
    }
    
    // onArmSync() is called whenever Myo has recognized a Sync Gesture after someone has put it on their
    // arm. This lets Myo know which arm it's on and which way it's facing.
    void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection)
    {
        knownMyos[identifyMyo(myo)].whichArm = arm;
        if(arm == myo::armLeft){
            knownMyos[identifyMyo(myo)].dataFileName = "MyoLeftArm.dat";
        } else {
            knownMyos[identifyMyo(myo)].dataFileName = "MyoRightArm.dat";
        }
    }
    
    
    // Utility function that maps a Myo to a unique ID starting at 1.
    // Does so by looking for the Myo pointer in knownMyos, which onPair() adds each Myo into as it is paired.
    size_t identifyMyo(myo::Myo* myo){
        // Walk through list of Myo devices that we've seen pairing events for
        for(size_t i = 0; i < knownMyos.size(); ++i){
            // If two Myo pointers compare equal, they refer to the same Myo device
            if(knownMyos[i].myo == myo){
                return i+1;
            }
        }
        return 0;
    }
    
    // We store each Myo pointer that we pair with in this list, so that we can keep track of the order we've seen
    // each Myo and give it a unique short ID (see onPair() and identifyMyo() above)
    std::vector<ArmBand> knownMyos;
};

int main(int argc, char** argv){
    try {
        myo::Hub hub("com.mcmaster.dual-myo-collector");
    
        // Instantiate the dual collector class defined above, and attach it as a listener to the Hub.
        DualCollector dc;
        hub.addListener(&dc);
    
        while(1){
            // In each iteration of our main loop, we run the Myo event loop for a set number of milliseconds.
            // In this case, we wish to update our display 20 times a second, so we run for 1000/20 milliseconds.
            hub.run(1000/20);
        }
    } catch (const std::exception& e){
        std::cerr << "Error: " << e.what() << std::endl;
        std::cerr << "Press enter to continue.";
        std::cin.ignore();
        return 1;
    }
    
}

