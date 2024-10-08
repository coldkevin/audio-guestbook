/**
 * Audio Guestbook, Copyright (c) 2022 Playful Technology
 * 
 * Tested using a Teensy 4.0 with Teensy Audio Shield, although should work 
 * with minor modifications on other similar hardware
 * 
 * When handset is lifted, a pre-recorded greeting message is played, followed by a tone.
 * Then, recording starts, and continues until the handset is replaced.
 * Playback button allows all messages currently saved on SD card through earpiece 
 * 
 * Files are saved on SD card as 44.1kHz, 16-bit, mono signed integer RAW audio format 
 * --> changed this to WAV recording, DD4WH 2022_07_31
 * --> added MTP support, which enables copying WAV files from the SD card via the USB connection, DD4WH 2022_08_01
 * 
 * 
 * Frank DD4WH, August 1st 2022 
 * for a DBP 611 telephone (closed contact when handheld is lifted) & with recording to WAV file
 * contact for switch button 0 is closed when handheld is lifted
 * 
 * GNU GPL v3.0 license
 * 
 */

#include <Bounce.h>
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <TimeLib.h>
#include <MTP_Teensy.h>
#include "play_sd_wav.h" // local copy with fixes
#include <elapsedMillis.h>

// DEFINES
#define SDCARD_CS_PIN    10
#define SDCARD_MOSI_PIN  7
#define SDCARD_SCK_PIN   14
#define HOOK_PIN 0
#define PLAYBACK_BUTTON_PIN 1
#define RECORDING_TIMEOUT 600000  // 10 minutes in milliseconds

// GLOBALS
AudioSynthWaveform waveform1;
AudioInputI2S i2s2;
AudioPlaySdWav playWav1;
AudioRecordQueue queue1;
AudioMixer4 mixer;
AudioOutputI2S i2s1;
AudioControlSGTL5000 sgtl5000_1;
AudioConnection patchCord1(waveform1, 0, mixer, 0);
AudioConnection patchCord3(playWav1, 0, mixer, 1);
AudioConnection patchCord4(mixer, 0, i2s1, 0);
AudioConnection patchCord6(mixer, 0, i2s1, 1);
AudioConnection patchCord5(i2s2, 0, queue1, 0);

char filename[15];
File frec;

Bounce buttonRecord = Bounce(HOOK_PIN, 40);  // Increased debounce time to original value
Bounce buttonPlay = Bounce(PLAYBACK_BUTTON_PIN, 40);

enum Mode {Initialising, Ready, Prompting, Recording, Playing};
Mode mode = Mode::Initialising;

float beep_volume = 0.04f;
uint32_t MTPcheckInterval;

elapsedMillis recordingTime; // Timer to limit recording duration

// WAV file parameters
unsigned long recByteSaved = 0L;

time_t getTeensy3Time() {
  return Teensy3Clock.get();
}

void dateTime(uint16_t* date, uint16_t* time, uint8_t* ms10) {
  *date = FS_DATE(year(), month(), day());
  *time = FS_TIME(hour(), minute(), second());
  *ms10 = second() & 1 ? 100 : 0;
}

void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 5000) {}
  Serial.println("Serial set up correctly");

  pinMode(HOOK_PIN, INPUT_PULLUP);
  pinMode(PLAYBACK_BUTTON_PIN, INPUT_PULLUP);

  AudioMemory(120);  // Increased memory to handle more data
  sgtl5000_1.enable();
  sgtl5000_1.inputSelect(AUDIO_INPUT_MIC);
  sgtl5000_1.volume(0.85); // Further reduce volume to avoid clipping
  sgtl5000_1.micGain(2);  // Lower mic gain to reduce static and warbling
  mixer.gain(0, 1.0f);
  mixer.gain(1, 1.0f);
  
  waveform1.begin(beep_volume, 440, WAVEFORM_SINE);
  wait(1000);
  waveform1.amplitude(0);
  delay(1000);

  SPI.setMOSI(SDCARD_MOSI_PIN);
  SPI.setSCK(SDCARD_SCK_PIN);
  if (!(SD.begin(SDCARD_CS_PIN))) {
    while (1) {
      Serial.println("Unable to access the SD card");
      delay(500);
    }
  } else {
    Serial.println("SD card correctly initialized");
  }

  MTP.begin();
  MTP.addFilesystem(SD, "Rolfo Audio guestbook");
  MTPcheckInterval = MTP.storage()->get_DeltaDeviceCheckTimeMS();

  setSyncProvider(getTeensy3Time);
  FsDateTime::setCallback(dateTime);

  mode = Mode::Ready; 
  print_mode();
}

void loop() {
  buttonRecord.update();
  buttonPlay.update();

  switch(mode){
    case Mode::Ready:
      if (buttonRecord.fallingEdge()) {
        Serial.println("Handset lifted");
        mode = Mode::Prompting;
        print_mode();
      } else if(buttonPlay.fallingEdge()) {
        playLastRecording();
      }
      break;

    case Mode::Prompting:
      wait(1000);
      playWav1.play("greeting.wav");
      while (!playWav1.isStopped()) {
        buttonRecord.update();
        buttonPlay.update();
        if(buttonRecord.risingEdge()) {
          playWav1.stop();
          mode = Mode::Ready;
          print_mode();
          return;
        }
        if(buttonPlay.fallingEdge()) {
          playWav1.stop();
          playLastRecording();
          return;
        }
      }
      Serial.println("Starting Recording");
      waveform1.begin(beep_volume, 440, WAVEFORM_SINE);
      wait(1250);
      waveform1.amplitude(0);
      startRecording();
      break;

    case Mode::Recording:
      if(buttonRecord.risingEdge()){
        Serial.println("Stopping Recording");
        stopRecording();
        end_Beep();
      } else {
        continueRecording();
      }
      break;

    case Mode::Playing: 
      if(playWav1.isStopped()) {
        Serial.println("Playback finished. Returning to Ready mode.");
        mode = Mode::Ready;
        print_mode();
      }
      break;

    case Mode::Initialising: 
      break;
  }

  MTP.loop();
}

void setMTPdeviceChecks(bool enable) {
  MTP.storage()->set_DeltaDeviceCheckTimeMS(enable ? MTPcheckInterval : 60000);
}

void startRecording() {
  setMTPdeviceChecks(false);
  recordingTime = 0; // Start the recording timer

  for (uint16_t i=0; i<9999; i++) {
    snprintf(filename, 11, " %05d.wav", i);
    if (!SD.exists(filename)) {
      break;
    }
  }
  frec = SD.open(filename, FILE_WRITE);
  Serial.println("Opened file!");

  if(frec) {
    Serial.print("Recording to ");
    Serial.println(filename);
    queue1.begin();
    mode = Mode::Recording;
    print_mode();
    recByteSaved = 0L;
  } else {
    Serial.println("Couldn't open file to record!");
  }
}

void continueRecording() {
  if (recordingTime > RECORDING_TIMEOUT) {  // Check for timeout (10 minutes)
    Serial.println("Recording timeout reached. Stopping recording.");
    stopRecording();
    end_Beep();
    return;
  }

  if (queue1.available() >= 32) {  // Increase the number of available blocks before writing
    byte buffer[32 * AUDIO_BLOCK_SAMPLES * sizeof(int16_t)];
    for (int i=0; i<32; i++) {
      memcpy(buffer + i * AUDIO_BLOCK_SAMPLES * sizeof(int16_t), queue1.readBuffer(), AUDIO_BLOCK_SAMPLES * sizeof(int16_t));
      queue1.freeBuffer();
    }
    frec.write(buffer, sizeof buffer);
    recByteSaved += sizeof buffer;
  }
}

void stopRecording() {
  queue1.end();
  while (queue1.available() > 0) {
    frec.write((byte*)queue1.readBuffer(), AUDIO_BLOCK_SAMPLES * sizeof(int16_t));
    queue1.freeBuffer();
    recByteSaved += AUDIO_BLOCK_SAMPLES * sizeof(int16_t);
  }
  writeOutHeader();
  frec.close();
  mode = Mode::Ready;
  print_mode();
  setMTPdeviceChecks(true);
}

void playLastRecording() {
  uint16_t idx = 0; 
  for (uint16_t i=0; i<9999; i++) {
    snprintf(filename, 11, " %05d.wav", i);
    if (!SD.exists(filename)) {
      idx = i - 1;
      break;
    }
  }
  snprintf(filename, 11, " %05d.wav", idx);
  Serial.println(filename);
  playWav1.play(filename);
  mode = Mode::Playing;
  print_mode();
}

void writeOutHeader() {
  unsigned long Subchunk2Size = recByteSaved - 42;
  unsigned long ChunkSize = Subchunk2Size + 34;

  frec.seek(0);  // Move back to the start of the file

  // Write "RIFF" chunk descriptor
  frec.write("RIFF");

  // Write chunk size
  frec.write(ChunkSize & 0xFF);
  frec.write((ChunkSize >> 8) & 0xFF);
  frec.write((ChunkSize >> 16) & 0xFF);
  frec.write((ChunkSize >> 24) & 0xFF);

  // Write file format
  frec.write("WAVE");

  // Write "fmt " sub-chunk
  frec.write("fmt ");

  // Subchunk1Size (16 for PCM)
  frec.write(16 & 0xFF);
  frec.write((16 >> 8) & 0xFF);
  frec.write((16 >> 16) & 0xFF);
  frec.write((16 >> 24) & 0xFF);

  // AudioFormat (1 for PCM)
  frec.write(1 & 0xFF);
  frec.write((1 >> 8) & 0xFF);

  // NumChannels (1 for mono)
  frec.write(1 & 0xFF);
  frec.write((1 >> 8) & 0xFF);

  // SampleRate (44100 Hz)
  frec.write(44100 & 0xFF);
  frec.write((44100 >> 8) & 0xFF);
  frec.write((44100 >> 16) & 0xFF);
  frec.write((44100 >> 24) & 0xFF);

  // ByteRate (SampleRate * NumChannels * BitsPerSample/8)
  unsigned long byteRate = 44100 * 1 * 16 / 8;
  frec.write(byteRate & 0xFF);
  frec.write((byteRate >> 8) & 0xFF);
  frec.write((byteRate >> 16) & 0xFF);
  frec.write((byteRate >> 24) & 0xFF);

  // BlockAlign (NumChannels * BitsPerSample/8)
  unsigned int blockAlign = 1 * 16 / 8;
  frec.write(blockAlign & 0xFF);
  frec.write((blockAlign >> 8) & 0xFF);

  // BitsPerSample (16 bits)
  frec.write(16 & 0xFF);
  frec.write((16 >> 8) & 0xFF);

  // Write "data" sub-chunk
  frec.write("data");

  // Subchunk2Size (NumSamples * NumChannels * BitsPerSample/8)
  frec.write(Subchunk2Size & 0xFF);
  frec.write((Subchunk2Size >> 8) & 0xFF);
  frec.write((Subchunk2Size >> 16) & 0xFF);
  frec.write((Subchunk2Size >> 24) & 0xFF);

  frec.close();  // Close the file after writing the header
  Serial.println("Header written");
}

void end_Beep() {
  for (int i = 0; i < 4; i++) {
    waveform1.frequency(523.25);
    waveform1.amplitude(beep_volume);
    wait(250);
    waveform1.amplitude(0);
    wait(250);
  }
}

void wait(unsigned int milliseconds) {
  elapsedMillis msec = 0;
  while (msec <= milliseconds) {
    buttonRecord.update();
    buttonPlay.update();
  }
}

void print_mode() {
  Serial.print("Mode switched to: ");
  if(mode == Mode::Ready) Serial.println(" Ready");
  else if(mode == Mode::Prompting) Serial.println(" Prompting");
  else if(mode == Mode::Recording) Serial.println(" Recording");
  else if(mode == Mode::Playing) Serial.println(" Playing");
  else if(mode == Mode::Initialising) Serial.println(" Initialising");
  else Serial.println(" Undefined");
}
  else if(mode == Mode::Initialising)  Serial.println(" Initialising");
  else Serial.println(" Undefined");
}
