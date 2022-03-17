
//#include "ForceBeWithYou.h"
#include "Game_Audio.h"
#include "123.h"

Game_Audio_Class GameAudio(25,0); 

Game_Audio_Wav_Class pmWav(SoundData);                            
                                      

void setup() {
  Serial.begin(115200); 
  Serial.println("Begin...");
}

void loop() {   
	Serial.print("Sample Rate (Hz):");
	Serial.println(pmWav.getSampleRate());
	Serial.print("Duration (secs):");
	Serial.println(pmWav.getDuration());
	
    GameAudio.PlayWav(&pmWav, false, 1.0);
	
	// wait until done
	while(GameAudio.IsPlaying()){		
	}
	
}
