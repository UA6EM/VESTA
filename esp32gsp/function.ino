void testFreq(float fGo, float fEnd, float fDelta) {
  float newFreq = fGo;
  float newEnd = fEnd;
  float newDelta = fDelta;
  if (newEnd <= 12000000) {     // для AD9833
    if (newDelta <= 1000000) {  // Шаг не больше мегагерца
      while (newFreq <= newEnd) {
        Ad9833.setFrequency(newFreq, 0);
        newFreq += newDelta;
        Serial.println(newFreq);
      }
    }
  }
}


// ************** Т Е С Т  Д И С П Л Е Я *****************/
void testDisplay() {
  drawTime = millis();

  for (int i = 0; i < 1000; i++) {
    yield();
    tft.drawNumber(i, 100, 80, 1);
  }

  drawTime = millis() - drawTime;
  tft.setTextColor(TFT_RED, TFT_BLACK);
  int xpos = 20;
  xpos += tft.drawFloat(drawTime / 2890.0, 3, xpos, 180, 4);
  tft.drawString(" ms per character", xpos, 180, 4);
  if (drawTime < 100) tft.drawString("Font 1 not loaded!", 50, 210, 4);
  yield();
  delay(4000);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  drawTime = millis();

  for (int i = 0; i < 1000; i++) {
    yield();
    tft.drawNumber(i, 100, 80, 2);
  }

  drawTime = millis() - drawTime;
  tft.setTextColor(TFT_RED, TFT_BLACK);
  xpos = 20;
  xpos += tft.drawFloat(drawTime / 2890.0, 3, xpos, 180, 4);
  tft.drawString(" ms per character", xpos, 180, 4);
  if (drawTime < 200) tft.drawString("Font 2 not loaded!", 50, 210, 4);
  delay(4000);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  drawTime = millis();

  for (int i = 0; i < 1000; i++) {
    yield();
    tft.drawNumber(i, 100, 80, 4);
  }

  drawTime = millis() - drawTime;
  tft.setTextColor(TFT_RED, TFT_BLACK);
  xpos = 20;
  xpos += tft.drawFloat(drawTime / 2890.0, 3, xpos, 180, 4);
  tft.drawString(" ms per character", xpos, 180, 4);
  if (drawTime < 200) tft.drawString("Font 4 not loaded!", 50, 210, 4);
  delay(4000);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  drawTime = millis();

  for (int i = 0; i < 1000; i++) {
    yield();
    tft.drawNumber(i, 100, 80, 6);
  }

  drawTime = millis() - drawTime;
  tft.setTextColor(TFT_RED, TFT_BLACK);
  xpos = 20;
  xpos += tft.drawFloat(drawTime / 2890.0, 3, xpos, 180, 4);
  tft.drawString(" ms per character", xpos, 180, 4);
  if (drawTime < 200) tft.drawString("Font 6 not loaded!", 50, 210, 4);
  delay(4000);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  drawTime = millis();

  for (int i = 0; i < 1000; i++) {
    yield();
    tft.drawNumber(i, 100, 80, 7);
  }

  drawTime = millis() - drawTime;
  tft.setTextColor(TFT_RED, TFT_BLACK);
  xpos = 20;
  xpos += tft.drawFloat(drawTime / 2890.0, 3, xpos, 180, 4);
  tft.drawString(" ms per character", xpos, 180, 4);
  if (drawTime < 200) tft.drawString("Font 7 not loaded!", 50, 210, 4);
  delay(4000);
  tft.fillScreen(TFT_YELLOW);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  drawTime = millis();

  for (int i = 0; i < 1000; i++) {
    yield();
    tft.drawNumber(i, 100, 80, 8);
  }

  drawTime = millis() - drawTime;
  tft.setTextColor(TFT_RED, TFT_BLACK);
  xpos = 20;
  xpos += tft.drawFloat(drawTime / 2890.0, 3, xpos, 180, 4);
  tft.drawString(" ms per character", xpos, 180, 4);
  if (drawTime < 200) tft.drawString("Font 8 not loaded!", 50, 210, 4);
  delay(4000);

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.drawString(" ESP32 GSP VESTA  ", 40, 110, 4);
}

// **************** ТЕСТ ПОТЕНЦИОМЕТРА *******************
void testMCP4151() {
#ifdef MCP4151MOD
  d_resis = 255;
#else
  d_resis = 127;
#endif

  Serial.println("START Test MCP4151");
  for (int i = 0; i < d_resis; i++) {
    Potentiometer.writeValue(i);
    delay(100);
    Serial.print("MCP4151 = ");
    Serial.println(i);
  }
  for (int j = d_resis; j >= 1; --j) {
    Potentiometer.writeValue(j);
    delay(100);
    Serial.print("MCP4151 = ");
    Serial.println(j);
  }
  Serial.println("STOP Test MCP4151");
}
