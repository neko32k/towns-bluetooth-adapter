#include <bluefruit.h>
#include <Wire.h>
#include "BLEClientHidAdafruitEx.h"
#include "CircularQueue.h"

// --- 設定 ---
#define MAX_CONNECTION    4
#define ENABLE_DEBUG_LOG  0
#define MOUSE_SCALE_NUM   2
#define MOUSE_SCALE_DENOM 3
#define ANALOG_THRESHOLD  10000
#define TURBO_INTERVAL_MS 31      // 連射間隔(ms). 31ms*2=62ms周期 -> 約16連射/秒

// キーボード送信設定
#define ENABLE_BATCH_KEY_PROCESS  // 定義するとキューを一括処理、未定義なら1ループ1キー
#define TOWNS_KEY_DELAY_MS 2      // プレフィックスとコードの間の遅延(ms)

// アクティブポート検出
volatile unsigned long last_strobe_detect_ms[2] = {0, 0};
volatile int active_mouse_port = -1; // -1:なし, 0:ポート0, 1:ポート1

#if ENABLE_DEBUG_LOG
  #define DBG_Printf(...)    Serial.printf(__VA_ARGS__)
  #define DBG_Print(...)     Serial.print(__VA_ARGS__)
  #define DBG_Println(...)   Serial.println(__VA_ARGS__)
#else
  #define DBG_Printf(...)
  #define DBG_Print(...)
  #define DBG_Println(...)
#endif

// --- ピン定義 (マウス / ジョイパッド) ---
// ポート0 ストロボ: SDA (22番ピン)
// ポート1 ストロボ: D5  (5番ピン)
const int PIN_STROBE_P0 = 22;
const int PIN_STROBE_P1 = 5;

// 出力ピン (0-5: ポート0, 6-11: ポート1)
// マッピング: 0=上/D0, 1=下/D1, 2=左/D2, 3=右/D3, 4=Trig1/A, 5=Trig2/B
// 物理ピン:
// P0: 6, 9, 10, 11, 12, 13
// P1: A0, A1, A2, A3, A4, A5
const uint8_t OUT_PINS[] = {
  6,  9, 10, 11, 12, 13,  // ポート 0
  A0, A1, A2, A3, A4, A5  // ポート 1
};

// --- マウスの状態 ---
volatile int16_t mouse_accum_x[2] = {0, 0}; 
volatile int16_t mouse_accum_y[2] = {0, 0};
volatile int16_t latch_x[2] = {0, 0};
volatile int16_t latch_y[2] = {0, 0};
volatile uint8_t mouse_btns[2] = {0, 0};
volatile uint8_t strobe_seq[2] = {0, 0};
volatile unsigned long last_strobe_us[2] = {0, 0};
unsigned long last_strobe_ms_check[2] = {0, 0};
bool has_mouse[2] = {false, false};

// --- キーボードの状態 ---
const uint8_t FMR_PREFIX_MAKE  = 0xA0; // 0x80 | 0x20(JIS)
const uint8_t FMR_PREFIX_BREAK = 0xB0; // 0x90 | 0x20(JIS)
#define FMR_KEY_TABLE_SIZE       256
const unsigned long SERIAL_BAUD = 9600;

struct KeyEvent {
  uint8_t code;
  bool isPress;
};
Queue<KeyEvent, 64> key_queue;

// --- Xboxコントローラー用レポート構造体 ---
typedef struct __attribute__((packed)) {
  int16_t x; int16_t y; int16_t z; int16_t rz;
  uint16_t brake; uint16_t gas;
  uint8_t hat; uint8_t btn1; uint8_t btn2; uint8_t padding;
} xbox_report_t;

typedef struct {
  uint16_t conn_handle;
  BLEClientHidAdafruitEx hid;
  hid_keyboard_report_t last_kbd_report;
  hid_mouse_report_t    last_mse_report;
  xbox_report_t         last_xbox_report;
} prph_info_t;

prph_info_t prphs[MAX_CONNECTION];

// --- FMRキーコード変換テーブル ---
uint8_t usb2fmr[FMR_KEY_TABLE_SIZE]; 

// --- ログバッファ ---
#if ENABLE_DEBUG_LOG
#define LOG_BUF_SIZE 64
typedef struct {
  unsigned long time;
  uint8_t port;
  uint8_t seq;
  int16_t latch;
  uint8_t nibble;
  uint8_t raw_pins; // D0-D3の状態 (ビットマスク)
  uint8_t strobe_level; // ストロボピンの状態
} strobe_log_t;

volatile strobe_log_t strobe_logs[LOG_BUF_SIZE];
volatile uint8_t log_head = 0;
volatile uint8_t log_tail = 0;

void push_strobe_log(uint8_t p, uint8_t s, int16_t l, uint8_t n, uint8_t pins, uint8_t st_lvl) {
  uint8_t next = (log_head + 1) % LOG_BUF_SIZE;
  if (next != log_tail) {
    strobe_logs[log_head].time = millis();
    strobe_logs[log_head].port = p;
    strobe_logs[log_head].seq = s;
    strobe_logs[log_head].latch = l;
    strobe_logs[log_head].nibble = n;
    strobe_logs[log_head].raw_pins = pins;
    strobe_logs[log_head].strobe_level = st_lvl;
    log_head = next;
  }
}
#endif

// --- 前方宣言 ---
void processKeyboardReport(hid_keyboard_report_t* report, hid_keyboard_report_t* last_report);
void processMouseReport(hid_mouse_report_t* report, hid_mouse_report_t* last_report, int port_idx);
void processXboxReport(xbox_report_t* report, xbox_report_t* last_report, int port_idx);
void sendTownsKey(uint8_t code, bool isPress);
void processKeyQueue();
void handleStrobe(int port_idx);
int findConnHandle(uint16_t conn_handle);
void initFMRTable();
void setOut(uint8_t pin_idx_local, bool bit_val, int port_base);

// --- コールバック ---
void kbd_callback_0(hid_keyboard_report_t* report) { processKeyboardReport(report, &prphs[0].last_kbd_report); }
void kbd_callback_1(hid_keyboard_report_t* report) { processKeyboardReport(report, &prphs[1].last_kbd_report); }
void kbd_callback_2(hid_keyboard_report_t* report) { processKeyboardReport(report, &prphs[2].last_kbd_report); }
void kbd_callback_3(hid_keyboard_report_t* report) { processKeyboardReport(report, &prphs[3].last_kbd_report); }

void mse_callback_0(hid_mouse_report_t* report) { processMouseReport(report, &prphs[0].last_mse_report, 0); }
void mse_callback_1(hid_mouse_report_t* report) { processMouseReport(report, &prphs[1].last_mse_report, 1); }
void mse_callback_2(hid_mouse_report_t* report) { processMouseReport(report, &prphs[2].last_mse_report, 0); }
void mse_callback_3(hid_mouse_report_t* report) { processMouseReport(report, &prphs[3].last_mse_report, 1); }

void (*kbd_callbacks[MAX_CONNECTION])(hid_keyboard_report_t*) = { kbd_callback_0, kbd_callback_1, kbd_callback_2, kbd_callback_3 };
void (*mse_callbacks[MAX_CONNECTION])(hid_mouse_report_t*)    = { mse_callback_0, mse_callback_1, mse_callback_2, mse_callback_3 };

// Xbox コールバック
void xbox_input_callback_impl(uint16_t conn_handle, uint8_t* data, uint16_t len) {
  if (len == 16) {
    int id = findConnHandle(conn_handle);
    if (id >= 0) {
      // ゲームパッド 1->ポート0, ゲームパッド 2->ポート1 にマッピング (ロジック: id % 2)
      processXboxReport((xbox_report_t*)data, &prphs[id].last_xbox_report, id % 2);
    }
  }
}
void xbox_callback_0(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len) { xbox_input_callback_impl(prphs[0].conn_handle, data, len); }
void xbox_callback_1(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len) { xbox_input_callback_impl(prphs[1].conn_handle, data, len); }
void xbox_callback_2(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len) { xbox_input_callback_impl(prphs[2].conn_handle, data, len); }
void xbox_callback_3(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len) { xbox_input_callback_impl(prphs[3].conn_handle, data, len); }
void (*xbox_callbacks[MAX_CONNECTION])(BLEClientCharacteristic*, uint8_t*, uint16_t) = { xbox_callback_0, xbox_callback_1, xbox_callback_2, xbox_callback_3 };

// --- ISRラッパー ---
void isr_strobe_p0() { handleStrobe(0); }
void isr_strobe_p1() { handleStrobe(1); }

void setup() {
#if ENABLE_DEBUG_LOG
  Serial.begin(115200);
#endif
  DBG_Println("Towns BT Adapter: Integrated Firmware");

  initFMRTable();

  // GPIO 設定
  for (int i=0; i<12; i++) {
    pinMode(OUT_PINS[i], OUTPUT);
    digitalWrite(OUT_PINS[i], HIGH);
  }
  pinMode(PIN_STROBE_P0, INPUT_PULLUP);
  pinMode(PIN_STROBE_P1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_STROBE_P0), isr_strobe_p0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_STROBE_P1), isr_strobe_p1, CHANGE);

  // キーボード用UART (nRF52 featherではSerial1が標準)
  Serial1.begin(SERIAL_BAUD, SERIAL_8E1);

  // BLE設定
  Bluefruit.begin(0, MAX_CONNECTION);
  Bluefruit.setName("Towns BT Adapter");
  Bluefruit.setConnLedInterval(250);
  
  for (uint8_t i=0; i<MAX_CONNECTION; i++) {
    prphs[i].conn_handle = BLE_CONN_HANDLE_INVALID;
    prphs[i].hid.begin();
    memset(&prphs[i].last_kbd_report, 0, sizeof(hid_keyboard_report_t));
    memset(&prphs[i].last_mse_report, 0, sizeof(hid_mouse_report_t));
    memset(&prphs[i].last_xbox_report, 0, sizeof(xbox_report_t));
    prphs[i].last_xbox_report.hat = 8; // Hat center (released) initialization
    
    prphs[i].hid.setKeyboardReportCallback(kbd_callbacks[i]);
    prphs[i].hid.setMouseReportCallback(mse_callbacks[i]);
  }

  Bluefruit.Central.setConnectCallback(connect_callback);
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);
  Bluefruit.Security.setSecuredCallback(connection_secured_callback);
  
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80); 
  Bluefruit.Scanner.filterUuid(BLEUuid(0x1812)); 
  Bluefruit.Scanner.useActiveScan(true);
  Bluefruit.Scanner.start(0); 
}

void loop() {
  unsigned long now = millis();
  
  // キーキューの処理
  processKeyQueue();

  // --- ログ出力 ---
#if ENABLE_DEBUG_LOG
  while (log_head != log_tail) {
      strobe_log_t l;
      l.time     = strobe_logs[log_tail].time;
      l.port     = strobe_logs[log_tail].port;
      l.seq      = strobe_logs[log_tail].seq;
      l.latch    = strobe_logs[log_tail].latch;
      l.nibble   = strobe_logs[log_tail].nibble;
      l.raw_pins = strobe_logs[log_tail].raw_pins;
      l.strobe_level = strobe_logs[log_tail].strobe_level;
      
      log_tail = (log_tail + 1) % LOG_BUF_SIZE;
      
      DBG_Printf("[Port%d P:%d] Strobe:%d Latch:%d Nib:%X PinOut:%02X\n", 
                 l.port, l.seq, l.strobe_level, l.latch, l.nibble, l.raw_pins);
  }
#if ENABLE_DEBUG_LOG
  static unsigned long last_alive = 0;
  if (now - last_alive > 5000) {
     DBG_Println("[Sys] Status Report:");
     for(int p=0; p<2; p++) {
        unsigned long idle = now - last_strobe_ms_check[p];
        DBG_Printf("  Port%d: MouseMode=%d, StrobeIdle=%lums, GpdActive=%d\n", 
                   p, has_mouse[p], idle, (idle >= 200 && !has_mouse[p]));
     }
     last_alive = now;
  }
#endif
#endif

  // ゲームパッド出力の更新 (マウスモードでない場合)
  for(int p=0; p<2; p++) {
    // マウスモードが有効な場合でも、このポートが「アクティブなマウスポート」として確定していない場合は
    // ゲームパッドとして振る舞うことを許可する（これが「そうで無い方はゲームパッドにする」の実装）。
    // active_mouse_port は handleStrobe() で TOWNSからのストロボ信号を受けたときに更新される。
    if (has_mouse[p] && active_mouse_port == p) {
       continue;
    }

    // 最近ストロボがあった場合（レガシーチェック）、スキップ
    // FM-TOWNS本体が常にストロボ信号(ポート7)をポーリングしている可能性がある。
    // その場合、ここでブロックするとゲームパッド信号が出力されないため、コメントアウトする。
    // (マウスモードの排他制御は has_mouse[p] で十分に行われている)
    // if (now - last_strobe_ms_check[p] < 200) {
    //    continue; 
    // }
    
    // このポートのアクティブなゲームパッドを探す
    xbox_report_t* xb = NULL;
    
    // 【修正】1台のコントローラー(ID=0)だけでポート0/1の両方を操作可能にする(接続ミスやポート入替対策)
    // ID=0がつながっていて、このポートに対応する専用ID(p, p+2...)がつながっていない場合、ID=0を使う。
    bool specific_device_found = false;
    for(int id=p; id<MAX_CONNECTION; id+=2) { 
       if (prphs[id].conn_handle != BLE_CONN_HANDLE_INVALID && prphs[id].hid.gamepadPresent()) {
          xb = &prphs[id].last_xbox_report;
          specific_device_found = true;
          break; 
       }
    }
    // 専用デバイスが見つからず、かつID=0(Player1)が存在すれば、それをミラーリングで使用する
    if (!specific_device_found && prphs[0].conn_handle != BLE_CONN_HANDLE_INVALID && prphs[0].hid.gamepadPresent()) {
       xb = &prphs[0].last_xbox_report;
    }

    // --- 出力値の計算とログ ---
    bool out_vals[6]; // 0:U, 1:D, 2:L, 3:R, 4:A, 5:B (物理出力値)
    
    if (xb) {
       // ロジックマッピング : ハット または アナログ
       bool u = false, d = false, l = false, r = false;
       
       // ハットスイッチ (1-based mapping: 0=Center, 1=Up, 2=UR, ..., 8=UL)
       // ログ解析により、このコントローラーは「放すと0」を出力し、
       // 「下を押すと5」を出力することが判明したため、1-based (0=Center) として扱う。
       if (xb->hat > 0 && xb->hat <= 8) {
          // ビットマスク: 上=1, 下=2, 左=4, 右=8
          // Index: 0=Up, 1=UR, 2=R, 3=DR, 4=D, 5=DL, 6=L, 7=UL
          static const uint8_t HAT_BITS[] = { 1, 9, 8, 10, 2, 6, 4, 5 };
          uint8_t bits = HAT_BITS[xb->hat - 1]; // 1-basedなのを0-basedに補正
          if (bits & 1) u = true;
          if (bits & 2) d = true;
          if (bits & 4) l = true;
          if (bits & 8) r = true;
       }
       
       // アナログスティック (Unsigned 16bit 0-65535, Center 32768 と仮定して補正)
       // int16_tとして読み込まれているため、0x8000(-32768)付近がセンターとなる
       // キャストしてオフセットを引くことで 0中心の ±32767 に変換
       int32_t val_x = (uint16_t)xb->x - 32768;
       int32_t val_y = (uint16_t)xb->y - 32768;

       // y軸反転: 通常GamePadは上が0(Min) 下が65535(Max) -> 補正後 上が-32768, 下が+32767
       // Towns仕様や好みによるが、ここでは標準的な "上がマイナス" を前提にロジックを組む
       
       if (val_y >  ANALOG_THRESHOLD) d = true; else if (val_y < -ANALOG_THRESHOLD) u = true;
       if (val_x >  ANALOG_THRESHOLD) r = true; else if (val_x < -ANALOG_THRESHOLD) l = true;

       // ボタン (Xbox A=Btn1(0x01), B=Btn1(0x02), X=0x04, Y=0x08)
       // Towns: Trig1(A), Trig2(B)
       // ユーザー要望: X->連射A, Y->連射B
       // 連射サイクル: TURBO_INTERVAL_MS * 2
       bool turbo = (millis() / TURBO_INTERVAL_MS) % 2;
       
       bool rawA = (xb->btn1 & 0x01); // A
       bool rawB = (xb->btn1 & 0x02); // B
       // ログ解析による確定ビットマスク
       // X: Btn:0008 -> 0x08
       // Y: Btn:0010 -> 0x10
       bool rawX = (xb->btn1 & 0x08); 
       bool rawY = (xb->btn1 & 0x10); 

       // マッピング (btnA=実機A, btnB=実機B と確定)
       // X(0x08) -> 連射A
       // Y(0x10) -> 連射B
       bool btnA = rawA || (rawX && turbo);
       bool btnB = rawB || (rawY && turbo);

       // SELECT / RUN 対応 (FM-TOWNS仕様)
       // ユーザー指定:
       //   Btn:0400 (Select/Back) -> SELECT -> 上下同時 (u + d)
       //   Btn:0800 (Start/Menu)  -> RUN    -> 左右同時 (l + r)
       uint16_t raw_btns = xb->btn1 | (xb->btn2 << 8);
       
       if (raw_btns & 0x0400) { // Select/Back -> SELECT (上下)
           u = true; d = true; 
       }
       if (raw_btns & 0x0800) { // Start/Menu -> RUN (左右)
           l = true; r = true;
       }
       
       // 出力を決定 (Active Low: 押下=false(0), 解放=true(1))
       // 以前 "Val:00" (All Low) で「接続時に何か押された」＝LOWが押下判定であることを確認。
       // したがって、押下(true)の場合はfalse(LOW)を出力する。
       out_vals[0] = !u;
       out_vals[1] = !d;
       out_vals[2] = !l;
       out_vals[3] = !r;
       out_vals[4] = !btnA;
       out_vals[5] = !btnB;

    } else {
       // 全てリリース (Active Lowなので HIGH=true を出力)
       for(int i=0; i<6; i++) out_vals[i] = true;
    }

#if ENABLE_DEBUG_LOG
    static uint8_t last_out_bits[2] = {0xFF, 0xFF};
    uint8_t current_bits = 0;
    for(int i=0; i<6; i++) {
      if (out_vals[i]) current_bits |= (1 << i);
    }
    
    if (current_bits != last_out_bits[p]) {
      // 変化があった場合ログ出力
      // ビット順: 0(Up), 1(Dn), 2(Lf), 3(Rt), 4(A), 5(B) ※physical value
      DBG_Printf("[GPD Out Port%d] Val:%02X (U:%d D:%d L:%d R:%d A:%d B:%d)\n", 
                 p, current_bits, 
                 out_vals[0], out_vals[1], out_vals[2], out_vals[3], out_vals[4], out_vals[5]);
      last_out_bits[p] = current_bits;
    }
#endif

    // 実際の出力
    for(int i=0; i<6; i++) {
       setOut(i, out_vals[i], p);
    }
  }
}

// --- ロジック ---
void setOut(uint8_t pin_idx_local, bool bit_val, int port_base) {
  // pin_idx_local: 0-5
  // OUTPUT: LOW=Active(押下) ジョイパッド用, またはマウス用のビット値
  // バッファ(SN74AHCT125N)を使用しているため、Featherからは常にCMOSレベル(High/Low)を出力する。
  // 3.3V High -> Buffer -> 5V High
  // 0V Low    -> Buffer -> 0V Low
  int pin = OUT_PINS[port_base * 6 + pin_idx_local];
  digitalWrite(pin, bit_val ? HIGH : LOW);
}

void handleStrobe(int p) {
  unsigned long now = micros();
  int strobe_pin = (p == 0) ? PIN_STROBE_P0 : PIN_STROBE_P1;
  uint8_t st_lvl = digitalRead(strobe_pin);

  // マウス接続がない場合、ストロボ信号は無視する (ゲームパッド等への干渉を防ぐ)
  // 特に、タイムアウト処理でピンをHIGH(Release)に戻す処理が走ると、
  // ゲームパッドのボタン押しっぱなし(LOW)状態が解除されてしまうため。
  if (!has_mouse[p]) {
     return;
  }

  // アクティブなマウスポート検出を更新
  last_strobe_detect_ms[p] = millis();
  active_mouse_port = p;

  // タイムアウトリセット (時間 > 3ms)
  if ((now - last_strobe_us[p]) > 3000) {
      strobe_seq[p] = 0;
      // アイドル時はピンを解放(HIGH)して、スタックビットによるドリフトを防ぐ
      for(int i=0; i<6; i++) setOut(i, true, p); 
  }
  
  last_strobe_ms_check[p] = millis();

  // マウスが接続されていない場合は、ログ出力のみで戻る（ピンを駆動しない）
  // if (!has_mouse[p]) {
  //    return;
  // }

  int16_t val16 = 0;

  // シーケンス0でXをラッチ
  if (strobe_seq[p] == 0) {
      // 剰余を保持したスケーリングロジック
      int16_t move = (mouse_accum_x[p] * MOUSE_SCALE_NUM) / MOUSE_SCALE_DENOM;
      if (move != 0) {
          int16_t consumed = (move * MOUSE_SCALE_DENOM) / MOUSE_SCALE_NUM;
          mouse_accum_x[p] -= consumed;
          latch_x[p] = move;
      } else {
          latch_x[p] = 0;
      }
      
      // 8ビット範囲にクランプ
      if (latch_x[p] > 127) latch_x[p] = 127;
      if (latch_x[p] < -127) latch_x[p] = -127;
      
      val16 = latch_x[p];
  } 
  else if (strobe_seq[p] == 1) {
      val16 = latch_x[p];
  }
  // シーケンス2でYをラッチ
  else if (strobe_seq[p] == 2) {
      // 剰余を保持したスケーリングロジック
      int16_t move = (mouse_accum_y[p] * MOUSE_SCALE_NUM) / MOUSE_SCALE_DENOM;
      if (move != 0) {
          int16_t consumed = (move * MOUSE_SCALE_DENOM) / MOUSE_SCALE_NUM;
          mouse_accum_y[p] -= consumed;
          latch_y[p] = move;
      } else {
          latch_y[p] = 0;
      }

      // クランプ
      if (latch_y[p] > 127) latch_y[p] = 127;
      if (latch_y[p] < -127) latch_y[p] = -127;

      val16 = latch_y[p];
  } 
  else {
      val16 = latch_y[p];
  }

  int8_t val8 = (int8_t)val16;
  bool first = (strobe_seq[p] % 2 == 0);
  
  // ニブル出力 (上位 -> 下位)
  uint8_t nib = first ? ((val8 >> 4) & 0x0F) : (val8 & 0x0F);
  
  // D0-D3への出力 (標準ロジック)
  // Townsはこれらのピンでマウスデータを期待する
#if ENABLE_DEBUG_LOG
  uint8_t pins_val = 0;
#endif
  for (int i=0; i<4; i++) {
      bool bit = (nib >> i) & 1;
      // ロジック反転要求 (ユーザー戦略)
      // "反転ロジックでドリフトは止まった"。
      // 現在: !bit (Active Low)。
      // 以前ドリフトに効いたのは: bit (Active High)。
      // そのためここでは bit (Active High) を使用し、accum を -= で補正する。
      setOut(i, bit, p);
#if ENABLE_DEBUG_LOG
      // 物理レベルのログ記録
      if (bit) pins_val |= (1 << i); 
#endif
  }

  // リングバッファへのログ記録
#if ENABLE_DEBUG_LOG
  push_strobe_log(p, strobe_seq[p], val16, nib, pins_val, st_lvl);
#endif
  
  // ボタン出力 (Active Low)
  bool left = (mouse_btns[p] & 1);
  bool right = (mouse_btns[p] & 2);
  setOut(4, !left, p); 
  setOut(5, !right, p);
  
  strobe_seq[p] = (strobe_seq[p] + 1) & 0x03;
  last_strobe_us[p] = now;
}

// BLE 標準コールバック
void scan_callback(ble_gap_evt_adv_report_t* report) { Bluefruit.Central.connect(report); }

void connect_callback(uint16_t conn_handle) {
  int id = findConnHandle(BLE_CONN_HANDLE_INVALID);
  if (id < 0) { Bluefruit.disconnect(conn_handle); return; }
  
  prphs[id].conn_handle = conn_handle;
  // 新規接続時にレポート構造体をクリア（特にHatをニュートラルに）
  memset(&prphs[id].last_kbd_report, 0, sizeof(hid_keyboard_report_t));
  memset(&prphs[id].last_mse_report, 0, sizeof(hid_mouse_report_t));
  memset(&prphs[id].last_xbox_report, 0, sizeof(xbox_report_t));
  prphs[id].last_xbox_report.hat = 8;

  prphs[id].hid.discover(conn_handle);
  Bluefruit.Connection(conn_handle)->requestPairing();
  Bluefruit.Scanner.start(0);

  DBG_Printf("[Connect] ID: %d, Handle: %d\n", id, conn_handle);
}

void connection_secured_callback(uint16_t conn_handle) {
  int id = findConnHandle(conn_handle);
  if (id < 0) return;
  prph_info_t* peer = &prphs[id];

  DBG_Printf("[Secured] ID: %d\n", id);

#if ENABLE_DEBUG_LOG
  BLEConnection* conn = Bluefruit.Connection(conn_handle);
  char devName[32] = { 0 };
  if ( conn->getPeerName(devName, sizeof(devName)) ) {
    DBG_Printf(" - Name: %s\n", devName);
  }
  
  uint8_t hidInfo[4];
  if ( peer->hid.getHidInfo(hidInfo) ) {
    DBG_Printf(" - HID Ver: %d.%d, Country: %d, Flags: 0x%02X\n", hidInfo[0], hidInfo[1], hidInfo[2], hidInfo[3]);
  }

  DBG_Print(" - Protocol: ");
  if (peer->hid.gamepadPresent()) DBG_Print("Gamepad ");
  if (peer->hid.keyboardPresent()) DBG_Print("Keyboard ");
  if (peer->hid.mousePresent()) DBG_Print("Mouse ");
  DBG_Println();
#endif
  
  if (peer->hid.gamepadPresent() && !peer->hid.keyboardPresent()) {
     peer->hid.setBootMode(false);
     if (peer->hid.enableGamepad()) {
       peer->hid.getGamepadReportCharacteristic().setNotifyCallback(xbox_callbacks[id]);
       DBG_Println(" - Xbox Raw Mode: Enabled");
     }
  } else {
     peer->hid.setBootMode(true);
     if (peer->hid.keyboardPresent()) peer->hid.enableKeyboard();
     if (peer->hid.mousePresent()) {
       peer->hid.enableMouse();
       int p = id % 2;
       // マウスは物理的にどのポートに挿されるか不明であり(ケーブル検知不可)、
       // TOWNSからのストロボ信号(ポーリング)に基づいて応答する必要があるため、
       // どちらのポートからのストロボも受け付けるように両方のフラグを立てておく。
       // 実際にどちらに応答するかは handleStrobe() 内の active_mouse_port で動的に決まる。
       has_mouse[0] = true; 
       has_mouse[1] = true;
       // 接続時のドリフトを防ぐためにアキュムレータをリセット
       mouse_accum_x[p] = 0;
       mouse_accum_y[p] = 0;
       latch_x[p] = 0;
       latch_y[p] = 0;
       strobe_seq[p] = 0;
       mouse_btns[p] = 0; 
     }
     DBG_Println(" - Boot Mode: Enabled (KBD/MSE)");
  }
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  int id = findConnHandle(conn_handle);
  if (id >= 0) {
    if (prphs[id].hid.mousePresent()) has_mouse[id % 2] = false;
    prphs[id].conn_handle = BLE_CONN_HANDLE_INVALID;
  }
  DBG_Printf("[Disconnect] Handle: %d, Reason: 0x%02X\n", conn_handle, reason);
}

int findConnHandle(uint16_t conn_handle) {
  for(int id=0; id<MAX_CONNECTION; id++) if(conn_handle == prphs[id].conn_handle) return id;
  return -1;
}

void processMouseReport(hid_mouse_report_t* report, hid_mouse_report_t* last_report, int p) {
  // このポートがデータを受信すべきか判定。
  // 'p' が現在のアクティブなポートである場合、またはまだアクティブなポートがない場合にのみデータを送る。
  // コールバックは 'p' をハードコードしているが、
  // 代わりに、データを「アクティブなポート」にルーティングする。
  
  int target_port = active_mouse_port;
  if (target_port == -1) {
     // ストロボがまだ検出されていない場合はマッピングされたポートをデフォルトとする
     target_port = p % 2; 
  }

  // 強制反転ロジックテスト:
  // ユーザー報告 "ロジック反転でドリフトは止まったが動作が反転した"。
  // そのため、setOutでロジックを反転(Active High -> Active Low) し、かつ 移動アキュムレータも反転する。
  // これにより物理的には "反転ビット" を送るが、論理的には "反転値" となる。
  // ここでアキュムレータを反転(減算)してみる。
  
  mouse_accum_x[target_port] -= report->x; 
  mouse_accum_y[target_port] -= report->y;
  mouse_btns[target_port] = report->buttons; 
  
  if (report->x != 0 || report->y != 0 || report->buttons != last_report->buttons) {
     DBG_Printf("[MSE %d->%d] In(X:%d Y:%d) -> Accum(X:%d Y:%d)\n", p, target_port, report->x, report->y, mouse_accum_x[target_port], mouse_accum_y[target_port]);
  }

  memcpy(last_report, report, sizeof(hid_mouse_report_t));
}

void processXboxReport(xbox_report_t* report, xbox_report_t* last_report, int p) {
  if ( memcmp(last_report, report, sizeof(xbox_report_t)) ) {
     bool printed = false;
     DBG_Printf("[GPD %d] ", p); 
     
     if (report->hat != last_report->hat) {
       DBG_Printf("Hat:%d ", report->hat);
       printed = true;
     }

     if (report->x != last_report->x || report->y != last_report->y) {
       DBG_Printf("L(%d,%d) ", report->x, report->y);
       printed = true;
     }
     
     uint16_t btns = report->btn1 | (report->btn2 << 8);
     uint16_t last_btns = last_report->btn1 | (last_report->btn2 << 8);
     if (btns != last_btns) {
        DBG_Printf("Btn:%04X ", btns);
        printed = true;
     }

     if (report->brake != last_report->brake) {
        DBG_Printf("Brk:%d ", report->brake);
        printed = true;
     }
     if (report->gas != last_report->gas) {
        DBG_Printf("Gas:%d ", report->gas);
        printed = true;
     }

     if (printed) DBG_Println();
     memcpy(last_report, report, sizeof(xbox_report_t));
  }
}

void processKeyboardReport(hid_keyboard_report_t* report, hid_keyboard_report_t* last_report) {
   // 修飾キー
   uint8_t mod_diff = report->modifier ^ last_report->modifier;
   if (mod_diff) {
     DBG_Printf("[KBD] Mod: 0x%02X -> 0x%02X\n", last_report->modifier, report->modifier);
     if (mod_diff & KEYBOARD_MODIFIER_LEFTSHIFT) key_queue.enqueue({0x53, (bool)(report->modifier & KEYBOARD_MODIFIER_LEFTSHIFT)});
     if (mod_diff & KEYBOARD_MODIFIER_RIGHTSHIFT) key_queue.enqueue({0x53, (bool)(report->modifier & KEYBOARD_MODIFIER_RIGHTSHIFT)});
     if (mod_diff & KEYBOARD_MODIFIER_LEFTCTRL)  key_queue.enqueue({0x52, (bool)(report->modifier & KEYBOARD_MODIFIER_LEFTCTRL)});
     if (mod_diff & KEYBOARD_MODIFIER_LEFTALT)   key_queue.enqueue({0x5C, (bool)(report->modifier & KEYBOARD_MODIFIER_LEFTALT)});
   }

   // キー (新規押下)
   for(int i=0; i<6; i++) {
     bool newKey = true;
     if (report->keycode[i] == 0) continue;
     for(int j=0; j<6; j++) if (report->keycode[i] == last_report->keycode[j]) { newKey = false; break; }
     
     if (newKey) {
       uint8_t fmr = usb2fmr[report->keycode[i]];
       DBG_Printf("[KBD] Down: USB=0x%02X -> FMR=0x%02X\n", report->keycode[i], fmr);
       if (fmr) key_queue.enqueue({fmr, true});
     }
   }
   
   // キー (リリース)
   for(int i=0; i<6; i++) {
     bool released = true;
     if (last_report->keycode[i] == 0) continue;
     for(int j=0; j<6; j++) if (last_report->keycode[i] == report->keycode[j]) { released = false; break; }
     
     if (released) {
       uint8_t fmr = usb2fmr[last_report->keycode[i]];
       DBG_Printf("[KBD] Up  : USB=0x%02X -> FMR=0x%02X\n", last_report->keycode[i], fmr);
       if (fmr) key_queue.enqueue({fmr, false});
     }
   }

   memcpy(last_report, report, sizeof(hid_keyboard_report_t));
}

void processKeyQueue() {
#ifdef ENABLE_BATCH_KEY_PROCESS
  while(key_queue.size() > 0) {
    KeyEvent k = key_queue.front(); key_queue.dequeue();
    sendTownsKey(k.code, k.isPress);
  }
#else
  if (key_queue.size()==0) return;
  
  KeyEvent k = key_queue.front(); key_queue.dequeue();
  sendTownsKey(k.code, k.isPress);
#endif
}

void sendTownsKey(uint8_t code, bool isPress) {
  // プロトコル: プレフィックス + コード
  // reference bt_keycode.ino に準拠: Make=0x80, Break=0x90
  uint8_t prefix = isPress ? FMR_PREFIX_MAKE : FMR_PREFIX_BREAK;
  Serial1.write(prefix);
  // Townsキーボードコントローラーがプレフィックスを処理するのを確実にするために遅延を追加
  // 9600bps = 約1ms/byte。
  delay(TOWNS_KEY_DELAY_MS); 
  Serial1.write(code & 0x7F);

  // パケットをログ記録
  DBG_Printf("[UART] TX: %02X %02X (%s)\n", prefix, code, isPress ? "Make" : "Break");
}


void initFMRTable() {
  memset(usb2fmr, 0, FMR_KEY_TABLE_SIZE);
  
  // A-Z
  usb2fmr[0x04] = 0x1E; // A
  usb2fmr[0x05] = 0x2E; // B
  usb2fmr[0x06] = 0x2C; // C
  usb2fmr[0x07] = 0x20; // D
  usb2fmr[0x08] = 0x13; // E
  usb2fmr[0x09] = 0x21; // F
  usb2fmr[0x0A] = 0x22; // G
  usb2fmr[0x0B] = 0x23; // H
  usb2fmr[0x0C] = 0x18; // I
  usb2fmr[0x0D] = 0x24; // J
  usb2fmr[0x0E] = 0x25; // K
  usb2fmr[0x0F] = 0x26; // L
  usb2fmr[0x10] = 0x30; // M
  usb2fmr[0x11] = 0x2F; // N
  usb2fmr[0x12] = 0x19; // O
  usb2fmr[0x13] = 0x1A; // P
  usb2fmr[0x14] = 0x11; // Q
  usb2fmr[0x15] = 0x14; // R
  usb2fmr[0x16] = 0x1F; // S
  usb2fmr[0x17] = 0x15; // T
  usb2fmr[0x18] = 0x17; // U
  usb2fmr[0x19] = 0x2D; // V
  usb2fmr[0x1A] = 0x12; // W
  usb2fmr[0x1B] = 0x2B; // X
  usb2fmr[0x1C] = 0x16; // Y
  usb2fmr[0x1D] = 0x2A; // Z
  
  // 1-0
  usb2fmr[0x1E] = 0x02; // 1
  usb2fmr[0x1F] = 0x03; // 2
  usb2fmr[0x20] = 0x04; // 3
  usb2fmr[0x21] = 0x05; // 4
  usb2fmr[0x22] = 0x06; // 5
  usb2fmr[0x23] = 0x07; // 6
  usb2fmr[0x24] = 0x08; // 7
  usb2fmr[0x25] = 0x09; // 8
  usb2fmr[0x26] = 0x0A; // 9
  usb2fmr[0x27] = 0x0B; // 0
  
  // Enter, Esc, BS, Tab, Space
  usb2fmr[0x28] = 0x1D; // Enter
  usb2fmr[0x29] = 0x01; // ESC
  usb2fmr[0x2A] = 0x0F; // Backspace
  usb2fmr[0x2B] = 0x10; // Tab
  usb2fmr[0x2C] = 0x35; // Space
  
  // F1-F10 (USB 0x3A -> FMR 0x3A)
  for(int i=0; i<10; i++) usb2fmr[0x3A+i] = 0x3A + i; 

  // 記号 (部分的 - 日本語配列を想定)
  usb2fmr[0x2D] = 0x0C; // -
  usb2fmr[0x2E] = 0x0D; // ^
  usb2fmr[0x2F] = 0x1B; // @
  usb2fmr[0x30] = 0x1C; // [
  usb2fmr[0x33] = 0x27; // ;
  usb2fmr[0x34] = 0x28; // :
  usb2fmr[0x36] = 0x31; // ,
  usb2fmr[0x37] = 0x32; // .
  usb2fmr[0x38] = 0x33; // /
  
  usb2fmr[0x4F] = 0x4F; // Right
  usb2fmr[0x50] = 0x4B; // Left (0x4B or 4F?)
  // reference check: 
  // 0x6B /* 0x4F:← */
  // 0x74 /* 0x51:→ */
  // 0x75 /* 0x4D:↑ */
  // 0x72 /* 0x50:↓ */
  
  usb2fmr[0x4F] = 0x51; // Right (USB) -> FMR 0x51
  usb2fmr[0x50] = 0x4F; // Left (USB) -> FMR 0x4F
  usb2fmr[0x51] = 0x50; // Down (USB) -> FMR 0x50
  usb2fmr[0x52] = 0x4D; // Up (USB)   -> FMR 0x4D
}
