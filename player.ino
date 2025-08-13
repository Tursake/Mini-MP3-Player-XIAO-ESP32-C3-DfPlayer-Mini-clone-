#include <Arduino.h>

#define RXD2 20
#define TXD2 21
#define BUTTON_PIN 5
#define BUSY_PIN 4
#define BUTTON_PIN_BITMASK 0x0000000008  // 2^3 = GPIO3
#define MP3_POWER_PIN D4

RTC_DATA_ATTR int bootCount = 0;

HardwareSerial dfSerial(1);
int totalTracks = 0;
bool playbackFinished = false;

void clearSerialBuffer() {
  while (dfSerial.available()) {
    dfSerial.read();
  }
}

void sendDFCommand(byte Command, int ParData) {
  byte commandData[10];
  int checkSum;

  commandData[0] = 0x7E;
  commandData[1] = 0xFF;
  commandData[2] = 0x06;
  commandData[3] = Command;
  commandData[4] = 0x01;
  commandData[5] = highByte(ParData);
  commandData[6] = lowByte(ParData);
  checkSum = -(commandData[1] + commandData[2] + commandData[3] +
               commandData[4] + commandData[5] + commandData[6]);
  commandData[7] = highByte(checkSum);
  commandData[8] = lowByte(checkSum);
  commandData[9] = 0xEF;

  for (int i = 0; i < 10; i++) {
    dfSerial.write(commandData[i]);
  }
}

void playTrack(int trackNum) {
  Serial.print("Playing track: ");
  Serial.println(trackNum);
  sendDFCommand(0x03, trackNum);
}

void setVolume(int vol) {
  Serial.print("Setting volume to: ");
  Serial.println(vol);
  sendDFCommand(0x06, vol);
}

void requestTotalTracks() {
  Serial.println("Requesting total tracks...");
  sendDFCommand(0x48, 0);
}

bool readDFResponse() {
  static byte buffer[10];
  static int index = 0;
  bool gotTotalTracks = false;
  bool processedPacket = false;

  while (dfSerial.available()) {
    byte b = dfSerial.read();

    if (index == 0 && b != 0x7E) continue;
    buffer[index++] = b;

    if (index == 10) {
      index = 0;
      processedPacket = true;

      if (buffer[3] == 0x3D) {
        Serial.println("✅ Playback finished.");
        playbackFinished = true;
      }

      if (buffer[3] == 0x48) {
        totalTracks = (buffer[5] << 8) | buffer[6];
        Serial.print("✅ Total tracks: ");
        Serial.println(totalTracks);
        gotTotalTracks = true;
      }
    }
  }
  return processedPacket;
}

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t reason = esp_sleep_get_wakeup_cause();
  switch (reason) {
    case ESP_SLEEP_WAKEUP_EXT0:
      Serial.println("Wakeup caused by EXT0");
      break;
    case ESP_SLEEP_WAKEUP_EXT1:
      Serial.println("Wakeup caused by EXT1");
      break;
    case ESP_SLEEP_WAKEUP_GPIO:
      Serial.println("Wakeup caused by GPIO");
      break;
    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("Wakeup caused by timer");
      break;
    default:
      Serial.printf("Wakeup not from deep sleep: %d\n", reason);
      break;
  }
}

void waitForPlaybackToFinish(unsigned long timeout_ms) {
  unsigned long start = millis();
  while (digitalRead(BUSY_PIN) == 0) {  // busy pin LOW = playing
    if (millis() - start > timeout_ms) {
      Serial.println("Timeout waiting for playback to finish.");
      break;
    }
    delay(50);
  }

  digitalWrite(MP3_POWER_PIN, LOW);
}

void setup() {
  Serial.begin(115200);
  delay(100);
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUSY_PIN, INPUT);

  pinMode(MP3_POWER_PIN, OUTPUT);
  digitalWrite(MP3_POWER_PIN, LOW);

  print_wakeup_reason();
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

  dfSerial.begin(9600, SERIAL_8N1, RXD2, TXD2);

  if (wakeup_reason != ESP_SLEEP_WAKEUP_GPIO) {
    Serial.println("Non-GPIO wakeup, waiting longer for DFPlayer startup...");
    digitalWrite(MP3_POWER_PIN, HIGH); // Power on MP3
    delay(3000);
  } else {
    Serial.println("GPIO wakeup, quick start.");
    digitalWrite(MP3_POWER_PIN, HIGH); // Power on MP3
    delay(500);
  }

  totalTracks = 0;
  requestTotalTracks();

  unsigned long start = millis();
  while (millis() - start < 3000) {
    if (readDFResponse() && totalTracks > 0) {
      break;
    }
    delay(10);
  }

  if (totalTracks == 0) {
    Serial.println("⚠ No tracks found after retries.");
  }

  setVolume(30);
  delay(200);               // Give DFPlayer time to process volume command
  clearSerialBuffer();      // Flush any old responses before playback

  if (totalTracks > 0) {
    int randTrack = random(1, totalTracks + 1);
    Serial.print("🎲 Playing random track: ");
    Serial.println(randTrack);
    playTrack(randTrack);

    delay(50);              // Give command time to take effect
    Serial.print("BUSY pin after playTrack(): ");
    Serial.println(digitalRead(BUSY_PIN));
  } else {
    Serial.println("⚠ No tracks available to play.");
  }

  waitForPlaybackToFinish(10000);
}

void loop() {
  readDFResponse();

  if (playbackFinished) {
    delay(100);
    Serial.println("💤 Going to deep sleep...");
    esp_deep_sleep_enable_gpio_wakeup(BUTTON_PIN_BITMASK, ESP_GPIO_WAKEUP_GPIO_LOW);
    esp_deep_sleep_start();
  }
}
