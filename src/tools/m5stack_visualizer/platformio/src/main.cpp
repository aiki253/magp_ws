#include <M5Core2.h>
#include <ArduinoJson.h>

// 表示モード
enum DisplayMode {
    MODE_MUX_ONLY = 0,      // デフォルト: /mux_input/status のみ大きく表示
    MODE_BAG_AND_MUX = 1,   // Bag Recorder上部 + mux下部
    MODE_BAG_ONLY = 2       // Bag Recorder状態のみ大きく表示
};

// Bag Recorder状態
struct BagRecorderStatus {
    String state = "IDLE";
    bool system_unlocked = false;
    String current_bag = "";
    int bag_counter = 0;
    String recording_start_time = "";
};

struct BagRecorderEvent {
    String event_type = "";
    String message = "";
    unsigned long timestamp = 0;
};

// Mux Input状態
struct MuxInputStatus {
    String mode = "Manual";  // Auto, Manual, Error
    unsigned long last_update = 0;
};

// グローバル変数
DisplayMode current_mode = MODE_MUX_ONLY;
BagRecorderStatus bag_status;
BagRecorderEvent last_event;
MuxInputStatus mux_status;
unsigned long last_receive_time = 0;
const unsigned long TIMEOUT_MS = 5000;  // 5秒タイムアウト

// 色定義
#define COLOR_RECORDING TFT_RED
#define COLOR_IDLE TFT_GREEN
#define COLOR_AUTO TFT_RED
#define COLOR_MANUAL TFT_BLUE
#define COLOR_ERROR TFT_YELLOW
#define COLOR_BG TFT_BLACK
#define COLOR_TEXT TFT_WHITE

// 関数宣言
void parseStatusMessage(String json_str);
void parseEventMessage(String json_str);
void parseMuxMessage(String json_str);
void updateDisplay();
void drawModeIndicator();
void drawBagRecorderLarge();
void drawBagAndMux();
void drawMuxOnlyLarge();
uint16_t getStateColor(String state);
uint16_t getMuxColor(String mode);
void handleButtonPress();

void setup() {
    M5.begin();
    Serial.begin(115200);
    
    M5.Lcd.fillScreen(COLOR_BG);
    M5.Lcd.setTextColor(COLOR_TEXT);
    M5.Lcd.setTextSize(2);
    
    // 起動メッセージ
    M5.Lcd.setCursor(50, 100);
    M5.Lcd.println("Bag Recorder");
    M5.Lcd.setCursor(70, 130);
    M5.Lcd.println("Visualizer");
    M5.Lcd.setCursor(40, 160);
    M5.Lcd.setTextSize(1);
    M5.Lcd.println("Waiting for data...");
    
    delay(2000);
    updateDisplay();
}

void loop() {
    M5.update();
    
    // シリアルデータの受信
    if (Serial.available()) {
        String line = Serial.readStringUntil('\\n');
        line.trim();
        
        if (line.startsWith("{")) {
            last_receive_time = millis();
            
            // メッセージタイプを判定
            if (line.indexOf("\\"state\\"") != -1) {
                // Bag Recorder Status
                parseStatusMessage(line);
            } else if (line.indexOf("\\"event_type\\"") != -1) {
                // Bag Recorder Event
                parseEventMessage(line);
            } else if (line.indexOf("\\"mode\\"") != -1) {
                // Mux Input Status
                parseMuxMessage(line);
            }
            
            updateDisplay();
        }
    }
    
    // ボタン処理
    handleButtonPress();
    
    // タイムアウトチェック
    if (millis() - last_receive_time > TIMEOUT_MS && last_receive_time > 0) {
        // 接続タイムアウト表示
        M5.Lcd.fillRect(0, 220, 320, 20, TFT_DARKGREY);
        M5.Lcd.setTextColor(TFT_WHITE);
        M5.Lcd.setTextSize(1);
        M5.Lcd.setCursor(80, 225);
        M5.Lcd.println("Connection Lost");
    }
    
    delay(50);
}

void parseStatusMessage(String json_str) {
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, json_str);
    
    if (!error) {
        bag_status.state = doc["state"].as<String>();
        bag_status.system_unlocked = doc["system_unlocked"];
        bag_status.current_bag = doc["current_bag"] | "";
        bag_status.bag_counter = doc["bag_counter"];
        bag_status.recording_start_time = doc["recording_start_time"] | "";
    }
}

void parseEventMessage(String json_str) {
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, json_str);
    
    if (!error) {
        last_event.event_type = doc["event_type"].as<String>();
        last_event.message = doc["message"].as<String>();
        last_event.timestamp = millis();
    }
}

void parseMuxMessage(String json_str) {
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, json_str);
    
    if (!error) {
        mux_status.mode = doc["mode"].as<String>();
        mux_status.last_update = millis();
    }
}

void handleButtonPress() {
    if (M5.BtnA.wasPressed()) {
        // モード切り替え
        current_mode = (DisplayMode)((current_mode + 1) % 3);
        updateDisplay();
    }
    
    if (M5.BtnB.wasPressed()) {
        // 画面リフレッシュ
        updateDisplay();
    }
    
    if (M5.BtnC.wasPressed()) {
        // 戻る
        current_mode = (DisplayMode)((current_mode - 1 + 3) % 3);
        updateDisplay();
    }
}

void updateDisplay() {
    M5.Lcd.fillScreen(COLOR_BG);
    
    switch (current_mode) {
        case MODE_MUX_ONLY:
            drawMuxOnlyLarge();
            break;
        case MODE_BAG_AND_MUX:
            drawBagAndMux();
            break;
        case MODE_BAG_ONLY:
            drawBagRecorderLarge();
            break;
    }
    
    drawModeIndicator();
}

void drawModeIndicator() {
    // 下部にモードインジケーター
    M5.Lcd.fillRect(0, 220, 320, 20, TFT_DARKGREY);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(TFT_WHITE);
    
    // ボタンラベル
    M5.Lcd.setCursor(10, 225);
    M5.Lcd.print("Next");
    M5.Lcd.setCursor(130, 225);
    M5.Lcd.print("Refresh");
    M5.Lcd.setCursor(260, 225);
    M5.Lcd.print("Prev");
}

void drawBagRecorderLarge() {
    // Bag Recorder状態を大きく表示
    M5.Lcd.setTextSize(3);
    uint16_t color = getStateColor(bag_status.state);
    M5.Lcd.setTextColor(color);
    
    // 状態表示
    int x = (320 - bag_status.state.length() * 18) / 2;
    M5.Lcd.setCursor(x, 60);
    M5.Lcd.println(bag_status.state);
    
    // システムロック状態
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(COLOR_TEXT);
    M5.Lcd.setCursor(40, 110);
    if (!bag_status.system_unlocked) {
        M5.Lcd.setTextColor(TFT_YELLOW);
        M5.Lcd.println("SYSTEM LOCKED");
    } else {
        M5.Lcd.setTextColor(TFT_GREEN);
        M5.Lcd.println("UNLOCKED");
    }
    
    // Bagカウンター
    if (bag_status.bag_counter > 0) {
        M5.Lcd.setTextColor(COLOR_TEXT);
        M5.Lcd.setTextSize(2);
        M5.Lcd.setCursor(80, 145);
        M5.Lcd.print("Bag #: ");
        M5.Lcd.println(bag_status.bag_counter);
    }
    
    // 最後のイベント
    if (last_event.event_type != "" && millis() - last_event.timestamp < 10000) {
        M5.Lcd.setTextSize(1);
        M5.Lcd.setTextColor(TFT_CYAN);
        M5.Lcd.setCursor(10, 185);
        M5.Lcd.println(last_event.event_type);
        M5.Lcd.setCursor(10, 200);
        M5.Lcd.println(last_event.message.substring(0, 35));
    }
}

void drawBagAndMux() {
    // 上部: Bag Recorder (小さめ)
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(TFT_CYAN);
    M5.Lcd.setCursor(70, 10);
    M5.Lcd.println("Bag Recorder");
    
    // Bag状態
    uint16_t bag_color = getStateColor(bag_status.state);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(bag_color);
    M5.Lcd.setCursor(100, 40);
    M5.Lcd.println(bag_status.state);
    
    // Bagカウンター
    if (bag_status.bag_counter > 0) {
        M5.Lcd.setTextSize(1);
        M5.Lcd.setTextColor(COLOR_TEXT);
        M5.Lcd.setCursor(110, 65);
        M5.Lcd.print("Bag #");
        M5.Lcd.println(bag_status.bag_counter);
    }
    
    // 区切り線
    M5.Lcd.drawLine(0, 90, 320, 90, TFT_DARKGREY);
    
    // 下部: Mux Input (大きめ)
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(TFT_CYAN);
    M5.Lcd.setCursor(90, 100);
    M5.Lcd.println("Mux Input");
    
    // Mux状態
    uint16_t mux_color = getMuxColor(mux_status.mode);
    M5.Lcd.setTextSize(3);
    M5.Lcd.setTextColor(mux_color);
    int mux_x = (320 - mux_status.mode.length() * 18) / 2;
    M5.Lcd.setCursor(mux_x, 140);
    M5.Lcd.println(mux_status.mode);
}

void drawMuxOnlyLarge() {
    // /mux_input/statusのみを大きく表示（デフォルト）
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(TFT_CYAN);
    M5.Lcd.setCursor(90, 30);
    M5.Lcd.println("Mux Input");
    
    // 状態を大きく太く表示
    uint16_t color = getMuxColor(mux_status.mode);
    M5.Lcd.setTextSize(4);
    M5.Lcd.setTextColor(color);
    
    int x = (320 - mux_status.mode.length() * 24) / 2;
    M5.Lcd.setCursor(x, 100);
    M5.Lcd.println(mux_status.mode);
    
    // タイムスタンプ
    if (mux_status.last_update > 0) {
        M5.Lcd.setTextSize(1);
        M5.Lcd.setTextColor(TFT_DARKGREY);
        M5.Lcd.setCursor(80, 180);
        M5.Lcd.print("Updated: ");
        M5.Lcd.print((millis() - mux_status.last_update) / 1000);
        M5.Lcd.println("s ago");
    }
}

uint16_t getStateColor(String state) {
    if (state == "RECORDING") {
        return COLOR_RECORDING;
    } else if (state == "IDLE") {
        return COLOR_IDLE;
    }
    return COLOR_TEXT;
}

uint16_t getMuxColor(String mode) {
    if (mode == "Auto") {
        return COLOR_AUTO;
    } else if (mode == "Manual") {
        return COLOR_MANUAL;
    } else if (mode == "Error") {
        return COLOR_ERROR;
    }
    return COLOR_TEXT;
}