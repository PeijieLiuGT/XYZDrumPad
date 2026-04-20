/*
 * Velostat 3x3 Drum Pad - Interactive 5-Point Calibration
 * V3: Added Auto-Boundary Remapping to fix Centroid Shrinkage (0-127 output)
 * 04/19/2026 by Peijie Liu
 */

const int numRows = 3;
const int numCols = 3;

const int rowPins[numRows] = {D3, D2, D1};
const int colPins[numCols] = {A0, A1, A2};

int pressureValues[numRows][numCols];
float baselineValues[numRows][numCols];

const int noiseThreshold = 90;
const int settlingDelay = 60;

int pressThreshold = 120;       
int totalThreshold = 180;
int releaseThreshold = 80;

// Hold / debounce
const unsigned long holdTimeMs = 40;   // short hold to reduce flicker
unsigned long lastPressTime = 0;
bool wasPressed = false;

// ================== global calibration ==================
int velocityMax = 1200; 
int calibMinX = 10;
int calibMaxX = 116;
int calibMinY = 10;
int calibMaxY = 116;

// Calibration state tracking (0: Top-Left, 1: Top-Right, 2: Center, 3: Bottom-Left, 4: Bottom-Right)
bool zoneCalibrated[5] = {false, false, false, false, false};
int zoneMaxForce[5] = {0, 0, 0, 0, 0};
int zonePeakX[5] = {0, 0, 0, 0, 0}; // Records centroid X at peak force for each zone
int zonePeakY[5] = {0, 0, 0, 0, 0}; // Records centroid Y at peak force for each zone

const float coordAlpha = 0.35f;
const float velocityAlpha = 0.30f;
float smoothX = 63.0f, smoothY = 63.0f, smoothVelocity = 0.0f;

void setup() {
  Serial.begin(115200);
  
  for (int c = 0; c < numCols; c++) pinMode(colPins[c], INPUT);
  for (int r = 0; r < numRows; r++) pinMode(rowPins[r], INPUT);

  delay(1000);
  captureStaticBaseline();
  interactiveCalibration();
}

void loop() {
  scanMatrixRaw();

  int x = -1, y = -1, velocity = 0;
  bool pressed = findWeightedPressLocation(x, y, velocity);
  unsigned long now = millis();
/*
  if (pressed) {
    wasPressed = true;
    smoothX = coordAlpha * x + (1.0f - coordAlpha) * smoothX;
    smoothY = coordAlpha * y + (1.0f - coordAlpha) * smoothY;
    smoothVelocity = velocityAlpha * velocity + (1.0f - velocityAlpha) * smoothVelocity;

    Serial.print("X: "); Serial.print((int)(smoothX + 0.5f));
    Serial.print(" \t| Y: "); Serial.print((int)(smoothY + 0.5f));
    Serial.print(" \t| Vel: "); Serial.println((int)(smoothVelocity + 0.5f));
  } else {
    if (wasPressed) {
      Serial.println("--- Released ---");
      wasPressed = false;
      smoothVelocity = 0.0f;
    }
  }
  */
    if (pressed) {
    lastPressTime = now;
    wasPressed = true;

    // Smooth coordinates and velocity
    smoothX = coordAlpha * x + (1.0f - coordAlpha) * smoothX;
    smoothY = coordAlpha * y + (1.0f - coordAlpha) * smoothY;
    smoothVelocity = velocityAlpha * velocity + (1.0f - velocityAlpha) * smoothVelocity;

    Serial.print((int)(smoothX + 0.5f));
    Serial.print(",");
    Serial.print((int)(smoothY + 0.5f));
    Serial.print(",");
    Serial.println((int)(smoothVelocity + 0.5f));
  } else {
    // Short hold to prevent void flicker between noisy frames
    if (wasPressed && (now - lastPressTime < holdTimeMs)) {
      Serial.print((int)(smoothX + 0.5f));
      Serial.print(",");
      Serial.print((int)(smoothY + 0.5f));
      Serial.print(",");
      Serial.println((int)(smoothVelocity + 0.5f));
    } else {
      wasPressed = false;
      smoothVelocity = 0.0f;
      Serial.println("void");
    }
  }

  delay(15);
}

void captureStaticBaseline() {
  Serial.println("Capturing static baseline...");
  for (int r = 0; r < numRows; r++) {
    pinMode(rowPins[r], OUTPUT);
    digitalWrite(rowPins[r], HIGH);
    delayMicroseconds(settlingDelay);

    for (int c = 0; c < numCols; c++) {
      baselineValues[r][c] = analogRead(colPins[c]);
    }
    pinMode(rowPins[r], INPUT);
    digitalWrite(rowPins[r], LOW);
  }
  Serial.println("Baseline captured. Ignore preload.");
}

void scanMatrixRaw() {
  for (int r = 0; r < numRows; r++) {
    pinMode(rowPins[r], OUTPUT);
    digitalWrite(rowPins[r], HIGH);
    delayMicroseconds(settlingDelay);

    for (int c = 0; c < numCols; c++) {
      int raw = analogRead(colPins[c]);
      int diff = raw - (int)baselineValues[r][c];
      pressureValues[r][c] = (diff > 0) ? diff : 0; 
    }
    pinMode(rowPins[r], INPUT);
    digitalWrite(rowPins[r], LOW);
    delayMicroseconds(20);
  }
}

void interactiveCalibration() {
  Serial.println("\n=============================================");
  Serial.println("  Interactive 5-Point Calibration (V3)");
  Serial.println("  Added Auto-Boundary Remapping");
  Serial.println("=============================================");
  Serial.println("Please press the 4 corners and the center.");
  Serial.println("Use the MAXIMUM force you intend to play with.");
  Serial.println("Order does not matter. Release after each press.\n");

  int calibratedCount = 0;
  bool isPressing = false;
  int currentPeak = 0;
  int peakCoordX = -1;
  int peakCoordY = -1;

  while (calibratedCount < 5) {
    scanMatrixRaw();

    int maxCellVal = 0;
    for (int r = 0; r < numRows; r++) {
      for (int c = 0; c < numCols; c++) {
        if (pressureValues[r][c] > maxCellVal) {
          maxCellVal = pressureValues[r][c];
        }
      }
    }

    if (maxCellVal > noiseThreshold) { 
      isPressing = true;
      if (maxCellVal > currentPeak) {
        currentPeak = maxCellVal; 
        
        long wX = 0, wY = 0, tW = 0;
        for (int r = 0; r < numRows; r++) {
          for (int c = 0; c < numCols; c++) {
            int w = pressureValues[r][c];
            if (w < 0) w = 0;
            wX += (long)w * (c * 63); 
            wY += (long)w * (r * 63); 
            tW += w;
          }
        }
        if (tW > 0) {
          peakCoordX = (int)(wX / tW);
          peakCoordY = (int)(wY / tW);
        }
      }
    } else if (isPressing && maxCellVal < releaseThreshold) { 
      isPressing = false;

      int zoneIndex = -1;
      String zoneName = "";
      
      if (peakCoordX >= 40 && peakCoordX <= 86 && peakCoordY >= 40 && peakCoordY <= 86) {
        zoneIndex = 2; zoneName = "Center";
      } else {
        if (peakCoordY < 63) {
          if (peakCoordX < 63) { zoneIndex = 0; zoneName = "Top-Left"; }
          else                 { zoneIndex = 1; zoneName = "Top-Right"; }
        } else {
          if (peakCoordX < 63) { zoneIndex = 3; zoneName = "Bottom-Left"; }
          else                 { zoneIndex = 4; zoneName = "Bottom-Right"; }
        }
      }

      if (!zoneCalibrated[zoneIndex]) {
        zoneCalibrated[zoneIndex] = true;
        zoneMaxForce[zoneIndex] = currentPeak;
        zonePeakX[zoneIndex] = peakCoordX; // Record peak coordinates
        zonePeakY[zoneIndex] = peakCoordY;
        calibratedCount++;
        Serial.print("[SUCCESS] "); Serial.print(zoneName); 
        Serial.print(" Force:"); Serial.print(currentPeak);
        Serial.print(" RawX:"); Serial.print(peakCoordX);
        Serial.print(" RawY:"); Serial.println(peakCoordY);
        Serial.print("Progress: "); Serial.print(calibratedCount); Serial.println("/5\n");
      } else {
        if (currentPeak > zoneMaxForce[zoneIndex]) {
          zoneMaxForce[zoneIndex] = currentPeak;
          zonePeakX[zoneIndex] = peakCoordX; // Update peak coordinates
          zonePeakY[zoneIndex] = peakCoordY;
          Serial.print("[UPDATE] "); Serial.print(zoneName); 
          Serial.println(" Updated with harder press!");
        }
      }

      currentPeak = 0;
      peakCoordX = -1;
      peakCoordY = -1;
      delay(500); 
    }
    delay(15);
  }

  Serial.println("\n=============================================");
  Serial.println(" Calibration Complete! Calculating profiles...");
  
  // 1. Calculate global force ceiling
  long sumMax = 0;
  for (int i = 0; i < 5; i++) sumMax += zoneMaxForce[i];
  velocityMax = (int)((sumMax / 5) * 0.9f); 

  // 2. Core: Calculate coordinate stretch boundaries
  // Mean X of the two left corners used as Min X, mean X of the two right corners used as Max X
  calibMinX = (zonePeakX[0] + zonePeakX[3]) / 2;
  calibMaxX = (zonePeakX[1] + zonePeakX[4]) / 2;
  
  // Mean Y of the two top corners used as Min Y, mean Y of the two bottom corners used as Max Y
  calibMinY = (zonePeakY[0] + zonePeakY[1]) / 2;
  calibMaxY = (zonePeakY[3] + zonePeakY[4]) / 2;

  // Add a small edge deadzone to ensure the extreme edges easily reach 0 and 127
  calibMinX += 2; calibMaxX -= 2;
  calibMinY += 2; calibMaxY -= 2;

  Serial.print("--> Velocity MAX Limit: "); Serial.println(velocityMax);
  Serial.print("--> X Bound Remap: "); Serial.print(calibMinX); Serial.print(" to "); Serial.println(calibMaxX);
  Serial.print("--> Y Bound Remap: "); Serial.print(calibMinY); Serial.print(" to "); Serial.println(calibMaxY);
  Serial.println("System is ready. Start playing!");
  Serial.println("=============================================\n");
}

bool findWeightedPressLocation(int &x, int &y, int &velocity) {
  long weightedX = 0;
  long weightedY = 0;
  long totalWeight = 0;
  int maxCell = 0;

  for (int r = 0; r < numRows; r++) {
    for (int c = 0; c < numCols; c++) {
      int w = pressureValues[r][c];
      if (w > maxCell) maxCell = w;

      int px = (c * 126) / (numCols - 1);
      int py = (r * 126) / (numRows - 1);

      weightedX += (long)w * px;
      weightedY += (long)w * py;
      totalWeight += w;
    }
  }

  int activePressThreshold = wasPressed ? releaseThreshold : pressThreshold;
  int activeTotalThreshold = wasPressed ? (totalThreshold / 2) : totalThreshold;

  if (maxCell < activePressThreshold || totalWeight < activeTotalThreshold) {
    return false;
  }

  // Calculate the raw shrunken coordinates (e.g., between 30~90)
  int rawX = (int)(weightedX / totalWeight);
  int rawY = (int)(weightedY / totalWeight);

  // [CORE MODIFICATION] Map the raw coordinates to a perfect 0~127 range!
  x = map(rawX, calibMinX, calibMaxX, 0, 127);
  y = map(rawY, calibMinY, calibMaxY, 0, 127);

  // Strictly constrain to the 0-127 range to prevent out-of-bounds calculations
  x = constrain(x, 0, 127);
  y = constrain(y, 0, 127);

  velocity = map(constrain(maxCell, activePressThreshold, velocityMax),
                 activePressThreshold, velocityMax,
                 1, 127);
  velocity = constrain(velocity, 1, 127);

  return true;
}