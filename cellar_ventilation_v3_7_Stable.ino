/*
 * ============================================================
 *  РОЗУМНА СИСТЕМА КЕРУВАННЯ ВИТЯЖКОЮ В ПОГРІБІ  ver3.7
 *
 *  Компоненти:
 *    - Arduino UNO R3
 *    - LCD Keypad Shield 16x2 (pins 8,9,4,5,6,7 | кнопки A0)
 *    - DHT22 — pin 2 (D2 Arduino, D4 DHT22 Shield)
 *    - BH1750 / GY-30 — I2C (A4=SDA, A5=SCL)
 *    - Реле HW-803 — pin 3  (D3 Arduino)
 *    - RTC DS1307 — I2C (опційно, працює і без нього)
 *
 *  ЛОГІКА РЕЛЕ HW-803:
 *    LOW  = реле УВІМКНЕНО (вентилятор працює)
 *    HIGH = реле ВИМКНЕНО (вентилятор стоїть)
 *
 *  СТАНИ ВЕНТИЛЯТОРА:
 *    W (Waiting) — вологість нижча за humWait, чекаємо підняття
 *    R (Resting) — вентилятор відпочиває після циклу роботи
 *    L/N/T       — заблоковано: світло / ніч / температура
 *    Fan:On      — вентилятор працює (цикл до runTimeMin хв)
 *    Fan:Off     — вентилятор вимкнений, нема активних станів
 *
 *  ЛОГІКА РОБОТИ:
 *    1. При старті — якщо вологість < humWait → стан W (Waiting)
 *    2. Як тільки вологість >= humWait → вентилятор вмикається
 *    3. Вентилятор працює поки: вологість > humOff І час < runTimeMin
 *    4. Вимикається коли вологість опустилась до humOff АБО час вийшов
 *    5. Після зупинки — пауза restTimeMin хв (стан R)
 *    6. Після паузи — знову W якщо вологість < humWait, або одразу цикл
 *
 *  ВАЖЛИВО: humOff < humWait (нижній поріг < поріг очікування)
 *    Приклад: humWait=50%, humOff=40%
 *    Вмикаємо при 50%, вимикаємо коли впаде до 40%
 * ============================================================
 */

#include <Wire.h>
#include <LiquidCrystal.h>
#include <DHT.h>
#include <BH1750.h>
#include <RTClib.h>

// ── Пінування ───────────────────────────────────────────────
#define DHTPIN      2       // Пін D2 підключення DHT22 до Arduino
#define DHTTYPE     DHT22
#define RELAY_PIN   3       // Пін D3 підключення Реле до Arduino
#define BTN_PIN     A0      // Аналоговий пін кнопок LCD Shield

// ── Об'єкти ─────────────────────────────────────────────────
DHT            dht(DHTPIN, DHTTYPE);    // Датчик температури і вологості
LiquidCrystal  lcd(8, 9, 4, 5, 6, 7);  // Дисплей LCD Keypad Shield
BH1750         lightMeter;              // Датчик освітлення GY-30
RTC_DS1307     rtc;                     // Модуль реального часу (опційно)

// ── Налаштування (змінюються через меню) ────────────────────
float humWait        = 50.0;   // Поріг очікування: нижче — стан W (Waiting), вище — вмикаємо вентилятор
float humOff         = 40.0;   // Нижній поріг: вимикати вентилятор коли вологість ОПУСТИЛАСЬ до цього значення
                               // ВАЖЛИВО: humOff < humWait, інакше вентилятор зупиниться одразу після вмикання
float minTemp        = 8.0;    // Мінімальна температура °C — нижче цього вентилятор не працює
int   lightLuxLimit  = 30;     // Поріг освітлення (lux) — вище цього вентилятор не працює
int   avgCount       = 30;     // Кількість останніх зразків для розрахунку середньої вологості
int   runTimeMin     = 15;     // Максимальний час роботи вентилятора за один цикл, хв
int   restTimeMin    = 30;     // Час паузи після циклу роботи (охолодження мотора), хв
int   startH         = 8;      // Початок дозволеного часу роботи (8 ранку)
int   endH           = 22;     // Кінець дозволеного часу роботи (22 вечора)

// ── Стан системи ────────────────────────────────────────────
int  hr = 12, mn = 0;          // Поточний час (з RTC або програмний)
bool rtcFound    = false;      // true якщо RTC знайдено і відповідає
bool autoMode    = true;       // true = автоматичний режим, false = ручний
bool fanIsOn     = false;      // true = вентилятор зараз працює (реле увімкнено)

// ── Прапори блокування (оновлюються в handleLogic) ──────────
bool blockLight  = false;   // L: освітлення перевищує lightLuxLimit
bool blockNight  = false;   // N: поточний час поза дозволеним діапазоном
bool blockTemp   = false;   // T: температура нижча за minTemp
bool isWaiting   = true;    // W: вологість нижча за humWait, чекаємо підняття
bool isResting   = false;   // R: пауза після нормального циклу роботи

// ── Таймери (millis) ────────────────────────────────────────
unsigned long lastTick       = 0;   // Для програмного годинника (без RTC)
unsigned long lastBtnTime    = 0;   // Час останнього натискання кнопки (антидребезг)
unsigned long lastSensorRead = 0;   // Час останнього зчитування датчиків
unsigned long fanStartMillis = 0;   // Час увімкнення вентилятора (для відліку runTimeMin)
unsigned long fanStopMillis  = 0;   // Час вимкнення після нормального циклу (для відліку restTimeMin)

// ── Захист реле від дрижання (debounce реле) ────────────────
// Перед будь-якою зміною стану реле нова умова має триматись
// стабільно RELAY_CONFIRM_MS мілісекунд поспіль.
// Захищає від: коротких спалахів світла, стрибків вологості,
// некоректних даних датчиків на початку роботи.
#define RELAY_CONFIRM_MS  3000UL

bool  relayPendingOn      = false;   // Очікує вмикання — умова є, але ще не підтверджена
bool  relayPendingOff     = false;   // Очікує вимикання — умова є, але ще не підтверджена
unsigned long relayPendingMillis = 0; // Час початку поточного очікування підтвердження

// ── Кешовані значення датчиків ───────────────────────────────
// Зчитуються раз на 2 секунди, не в кожному циклі loop()
float cachedTemp = 0.0;   // Остання виміряна температура °C
float cachedLux  = 0.0;   // Останній виміряний рівень освітлення lux

// ── Усереднення вологості ────────────────────────────────────
// Зберігаємо останні avgCount значень і рахуємо середнє.
// Це захищає від разових стрибків показань датчика.
float humHistory[50];     // Кільцевий буфер значень вологості
int   humIdx     = 0;     // Поточна позиція запису в буфері
bool  bufferFull = false; // true коли буфер заповнений хоча б раз
float avgHum     = 0.0;   // Поточне середнє значення вологості

// ── Меню ─────────────────────────────────────────────────────
/*
  Сторінки:
  0  — HOME       (Стартова сторінка. Загальні показники + стан)
  1  — Set Time   (Встановлення поточного часу год:хв)
  2  — Hum Wait % (Поріг очікування і вмикання: нижче — W, вище — запускаємо)
  3  — Hum OFF %  (Нижній поріг зупинки: вимикаємо коли вологість впала до цього)
  4  — Min Temp C (Мінімальна температура для роботи)
  5  — Light Lux  (Поріг освітлення для блокування)
  6  — Avg Samples(Кількість зразків для усереднення вологості)
  7  — Run Time   (Максимальний час роботи мотора, хв)
  8  — Rest Time  (Час паузи після циклу роботи, хв)
  9  — Start Hour (Нічний режим: година початку дня)
  10 — End Hour   (Нічний режим: година початку ночі)
  11 — Test Relay (Ручна перевірка реле, вимикає автоматику)
  12 — Diag DHT22 (Перевірка датчика температури і вологості)
  13 — Diag Light (Перевірка датчика освітлення)
  14 — Diag RTC   (Перевірка модуля реального часу)
*/
const int MENU_PAGES = 15;
int  menuPage  = 0;   // Поточна сторінка меню
int  editStep  = 0;   // 0 = перегляд, 1+ = режим редагування

// ── Блимання значення при редагуванні ───────────────────────
unsigned long blinkMillis = 0;   // Час останнього перемикання
bool showDigit = true;           // true = показувати значення, false = пробіли

// ════════════════════════════════════════════════════════════
//  SETUP — виконується один раз при увімкненні
// ════════════════════════════════════════════════════════════
void setup() {
  // Реле вимкнено з першої мілісекунди — до будь-якої ініціалізації
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);

  // Ініціалізація дисплея і стартовий екран
  lcd.begin(16, 2);
  lcd.print("Pogreb-OS ver3.7");
  lcd.setCursor(0, 1);
  lcd.print("Init            ");  // Очистити рядок перед прогрес-баром

  // Ініціалізація датчиків
  dht.begin();
  Wire.begin();
  lightMeter.begin();

  // RTC — опційно, без нього працює програмний годинник
  if (rtc.begin()) {
    rtcFound = true;
    if (rtc.isrunning()) {
      DateTime now = rtc.now();
      hr = now.hour();
      mn = now.minute();
    }
  }

  // Ініціалізація буфера усереднення вологості нулями
  for (int i = 0; i < 50; i++) humHistory[i] = 0;

  // Явне скидання стану — мотор ще не працював, пауза не потрібна
  isResting       = false;
  isWaiting       = true;   // За замовчуванням чекаємо поки вологість підніметься
  fanIsOn         = false;
  relayPendingOn  = false;
  relayPendingOff = false;

  // ── Прогрів датчиків під час стартового екрану ──────────
  // Протягом WARMUP_MS мілісекунд:
  //   - Читаємо датчики і наповнюємо буфер реальними даними
  //   - Показуємо прогрес-бар (рівномірно по часу, не по кроках)
  //   - handleLogic() НЕ викликається — реле мовчить повністю
  // Після прогріву loop() отримає стабільні дані і не смикне реле.
  #define WARMUP_MS 18000UL   // 18 секунд прогріву
  #define BAR_WIDTH 10        // Ширина прогрес-бару в символах

  lastTick = millis();
  unsigned long warmupStart = millis();

  while (millis() - warmupStart < WARMUP_MS) {

    // Читати датчики кожні 2 секунди для наповнення буфера
    if (millis() - lastSensorRead >= 2000) {
      lastSensorRead = millis();
      readSensors();
    }

    // Оновлювати прогрес-бар кожні 100 мс (плавно)
    unsigned long elapsed  = millis() - warmupStart;
    int filled = (int)((elapsed * BAR_WIDTH) / WARMUP_MS);  // 0..BAR_WIDTH
    if (filled > BAR_WIDTH) filled = BAR_WIDTH;

    // Відсоток завершення рівномірно по часу
    int pct = (int)((elapsed * 100UL) / WARMUP_MS);
    if (pct > 100) pct = 100;

    // Малюємо: "Init ########## 100%"
    // Позиція 5: починаємо блоки одразу після "Init "
    lcd.setCursor(5, 1);
    for (int i = 0; i < BAR_WIDTH; i++) {
      lcd.print(i < filled ? (char)0xFF : ' ');  // 0xFF = заповнений блок
    }
    lcd.print(" ");
    if (pct < 10)  lcd.print(" ");
    if (pct < 100) lcd.print(" ");
    lcd.print(pct);
    lcd.print("%");

    delay(100);
  }

  // Після прогріву визначаємо початковий стан Waiting
  // на основі реальних даних з датчика
  isWaiting = (avgHum < humWait);

  lcd.clear();
}

// ════════════════════════════════════════════════════════════
//  LOOP — головний цикл
// ════════════════════════════════════════════════════════════
void loop() {
  // 1. Оновити час (RTC або програмний)
  updateClock();

  // 2. Читати датчики кожні 2 секунди
  if (millis() - lastSensorRead >= 2000) {
    lastSensorRead = millis();
    readSensors();
  }

  // 3. Логіка керування вентилятором
  handleLogic();

  // 4. Обробка кнопок
  handleButtons();

  // 5. Автоповернення на HOME через 30 с бездіяльності
  if (millis() - lastBtnTime > 30000 && editStep == 0) {
    if (menuPage != 0) { menuPage = 0; lcd.clear(); }
  }

  // 6. Оновити дисплей
  updateDisplay();
}

// ════════════════════════════════════════════════════════════
//  ГОДИННИК
//  Якщо RTC є — беремо час з нього.
//  Якщо немає — рахуємо хвилини самі через millis().
// ════════════════════════════════════════════════════════════
void updateClock() {
  if (rtcFound && rtc.isrunning() && editStep == 0) {
    DateTime now = rtc.now();
    hr = now.hour();
    mn = now.minute();
  } else if (millis() - lastTick >= 60000UL) {
    lastTick += 60000UL;
    if (++mn >= 60) { mn = 0; if (++hr >= 24) hr = 0; }
  }
}

// ════════════════════════════════════════════════════════════
//  ЧИТАННЯ ДАТЧИКІВ
//  Температура і lux — кешуємо (не читаємо в кожному loop).
//  Вологість — додаємо до кільцевого буфера і рахуємо середнє.
// ════════════════════════════════════════════════════════════
void readSensors() {
  float t = dht.readTemperature();
  if (!isnan(t)) cachedTemp = t;

  cachedLux = lightMeter.readLightLevel();

  float h = dht.readHumidity();
  if (!isnan(h)) {
    humHistory[humIdx] = h;
    humIdx = (humIdx + 1) % avgCount;
    if (humIdx == 0) bufferFull = true;

    float sum  = 0;
    int   count = bufferFull ? avgCount : humIdx;
    if (count == 0) count = 1;
    for (int i = 0; i < count; i++) sum += humHistory[i];
    avgHum = sum / count;
  }
}

// ═══════════════════════════════════════════════════════════════════════════════════════════
//  ЛОГІКА КЕРУВАННЯ ВЕНТИЛЯТОРОМ
//
//  Стани (пріоритет зверху вниз):
//  1. !autoMode                           [M]→ (Manual) ручний режим, логіка не працює
//  2. blockLight/Night/LowTemperature [L/N/T]→ зовнішнє блокування, реле вимкнено
//  3. isResting                           [R]→ пауза після циклу, реле вимкнено
//  4. isWaiting                           [W]→ чекаємо поки вологість підніметься до humWait
//  5. fanIsOn                             [_]→ вентилятор працює, чекаємо умови зупинки
//  6. !fanIsOn                            [_]→ вентилятор вимкнений, чекаємо умови вмикання
//
//  Зміна стану реле відбувається тільки після RELAY_CONFIRM_MS мілісекунд стабільної
//  умови (захист від дрижання реле).
// ═══════════════════════════════════════════════════════════════════════════════════════════
void handleLogic() {
  if (!autoMode) return;

  // Оновити прапори зовнішнього блокування
  blockLight = (cachedLux  > lightLuxLimit);
  blockNight = (hr < startH || hr >= endH);
  blockTemp  = (cachedTemp < minTemp);
  bool forceOff = blockLight || blockNight || blockTemp;

  // Перевірити чи закінчилась пауза відпочинку
  if (isResting) {
    unsigned long restMs = (unsigned long)restTimeMin * 60000UL;
    if (millis() - fanStopMillis >= restMs) {
      isResting = false;
      // Після паузи — перевіряємо чи треба знову чекати (Waiting)
      isWaiting = (avgHum < humWait);
    }
  }

  // Оновити стан Waiting: якщо вологість піднялась до humWait — виходимо
  if (isWaiting && avgHum >= humWait) {
    isWaiting = false;
  }

  // ── Визначаємо бажаний стан реле ──────────────────────
  bool wantOn = false;

  if (!forceOff && !isResting && !isWaiting) {
    if (!fanIsOn) {
      // Вентилятор вимкнений — вмикаємо.
      // Сюди потрапляємо тільки якщо вийшли зі стану Waiting (avgHum >= humWait).
      // Тобто умова вмикання вже перевірена через isWaiting — просто вмикаємо.
      wantOn = true;
    } else {
      // Вентилятор працює — тримаємо поки не виконана умова зупинки:
      // 1. Відпрацював максимальний час runTimeMin
      // 2. Вологість ОПУСТИЛАСЬ до нижнього порогу humOff
      // ВАЖЛИВО: humOff (40%) < humWait (50%) — тому ця умова
      // не може спрацювати одразу після вмикання при нормальній вологості.
      unsigned long runMs   = (unsigned long)runTimeMin * 60000UL;
      bool maxRunReached     = (millis() - fanStartMillis >= runMs);
      bool humDroppedToOff   = (avgHum <= humOff);
      wantOn = !(maxRunReached || humDroppedToOff);
    }
  }
  // При forceOff, isResting або isWaiting — wantOn = false, реле вимкнено

  // ── Підтвердження зміни стану реле (debounce) ─────────
  if (wantOn && !fanIsOn) {
    // Хочемо УВІМКНУТИ
    if (!relayPendingOn) {
      relayPendingOn     = true;
      relayPendingOff    = false;
      relayPendingMillis = millis();
    } else if (millis() - relayPendingMillis >= RELAY_CONFIRM_MS) {
      relayPendingOn = false;
      startFan();
    }

  } else if (!wantOn && fanIsOn) {
    // Хочемо ВИМКНУТИ
    if (!relayPendingOff) {
      relayPendingOff    = true;
      relayPendingOn     = false;
      relayPendingMillis = millis();
    } else if (millis() - relayPendingMillis >= RELAY_CONFIRM_MS) {
      relayPendingOff = false;
      // forceOff = зовнішня причина → без паузи відпочинку
      // нормальна зупинка → з паузою відпочинку
      stopFan(!forceOff);
    }

  } else {
    // Бажаний стан співпадає з поточним — скидаємо очікування
    relayPendingOn  = false;
    relayPendingOff = false;
  }
}

// ════════════════════════════════════════════════════════════
//  УВІМКНУТИ ВЕНТИЛЯТОР
//  Фізично вмикає реле і запускає таймер роботи.
// ════════════════════════════════════════════════════════════
void startFan() {
  fanIsOn        = true;
  fanStartMillis = millis();
  isResting      = false;
  isWaiting      = false;
  digitalWrite(RELAY_PIN, LOW);   // LOW = реле увімкнено (HW-803)
}

// ════════════════════════════════════════════════════════════
//  ВИМКНУТИ ВЕНТИЛЯТОР
//  startRest=true  → нормальна зупинка, запускаємо таймер паузи
//  startRest=false → зовнішня причина (ніч/світло/мороз), без паузи
// ════════════════════════════════════════════════════════════
void stopFan(bool startRest) {
  fanIsOn = false;
  digitalWrite(RELAY_PIN, HIGH);  // HIGH = реле вимкнено (HW-803)
  if (startRest) {
    isResting     = true;
    fanStopMillis = millis();   // Запам'ятати час для відліку restTimeMin
  }
}

// ════════════════════════════════════════════════════════════
//  ОБРОБКА КНОПОК
//  Антидребезг: мінімум 220 мс між натисканнями.
//  RIGHT/LEFT — навігація між сторінками меню.
//  UP/DOWN    — зміна значення при редагуванні.
//  SELECT     — вхід/вихід з редагування, підтвердження.
// ════════════════════════════════════════════════════════════
void handleButtons() {
  if (millis() - lastBtnTime < 220) return;

  int x = analogRead(BTN_PIN);
  if (x > 1000) return;  // Жодна кнопка не натиснута

  lastBtnTime = millis();

  if (x < 50) {           // RIGHT → наступна сторінка
    if (editStep == 0) { menuPage = (menuPage + 1) % MENU_PAGES; lcd.clear(); }

  } else if (x < 150) {   // UP → збільшити значення
    if (editStep > 0) adjust(1);

  } else if (x < 350) {   // DOWN → зменшити значення
    if (editStep > 0) adjust(-1);

  } else if (x < 500) {   // LEFT → попередня сторінка
    if (editStep == 0) { menuPage = (menuPage > 0) ? menuPage - 1 : MENU_PAGES - 1; lcd.clear(); }

  } else if (x < 800) {   // SELECT
    handleSelect();
  }

  lcd.clear();
}

// ════════════════════════════════════════════════════════════
//  ОБРОБКА КНОПКИ SELECT
// ════════════════════════════════════════════════════════════
void handleSelect() {
  switch (menuPage) {
    case 0:   // HOME — перемикач авто/ручний режим
      autoMode = !autoMode;
      if (!autoMode && fanIsOn) stopFan(false);
      break;

    case 1:   // Set Time — цикл: перегляд → год → хв → зберегти
      editStep++;
      if (editStep > 2) {
        editStep = 0;
        if (rtcFound) rtc.adjust(DateTime(2026, 1, 1, hr, mn, 0));
        lastTick = millis();
      }
      break;

    case 12:  // Test Relay — вмикає ручний тест реле
      if (editStep == 0) {
        editStep = 1;
        autoMode = false;   // Вимкнути автоматику на час тесту
      } else {
        editStep = 0;
        digitalWrite(RELAY_PIN, HIGH);  // Гарантовано вимкнути реле
        fanIsOn  = false;
        autoMode = true;    // Повернути автоматичний режим
      }
      break;

    default:  // Всі інші сторінки — toggle редагування
      editStep = (editStep == 0) ? 1 : 0;
      break;
  }
}

// ════════════════════════════════════════════════════════════
//  ЗМІНА ЗНАЧЕНЬ В МЕНЮ (кнопки UP/DOWN при editStep > 0)
// ════════════════════════════════════════════════════════════
void adjust(int dir) {
  switch (menuPage) {
    case 1:
      if (editStep == 1) hr = (hr + dir + 24) % 24;
      else               mn = (mn + dir + 60) % 60;
      break;
    case 2:  humWait       = constrain(humWait + dir, 10, 95); break;
    case 3:  humOff        = constrain(humOff + dir, 20, 99); break;
    case 4:  minTemp      += dir * 0.5; minTemp = constrain(minTemp, -5, 25); break;
    case 5:  lightLuxLimit = constrain(lightLuxLimit + dir * 5, 5, 500); break;
    case 6:  avgCount      = constrain(avgCount + dir, 1, 50);
             humIdx = 0; bufferFull = false;  // Скинути буфер при зміні розміру
             break;
    case 7:  runTimeMin  = constrain(runTimeMin  + dir, 1, 120); break;
    case 8:  restTimeMin = constrain(restTimeMin + dir, 1, 180); break;
    case 9: startH      = constrain(startH + dir, 0, 23); break;
    case 10: endH        = constrain(endH   + dir, 0, 23); break;
    case 11: // Тест реле — UP/DOWN вмикає/вимикає вручну
      fanIsOn = !fanIsOn;
      digitalWrite(RELAY_PIN, fanIsOn ? LOW : HIGH);
      break;
  }
}

// ════════════════════════════════════════════════════════════
//  БЛИМАННЯ ЗНАЧЕННЯ ПРИ РЕДАГУВАННІ
// ════════════════════════════════════════════════════════════
void updateBlink() {
  if (millis() - blinkMillis > 400) {
    blinkMillis = millis();
    showDigit = !showDigit;
  }
}

// ════════════════════════════════════════════════════════════
//  ІНДИКАТОР СТАТУСУ |A|X| (позиції 11-15 першого рядка)
//  A = Auto mode, M = Manual mode
//  Символ X прокручується якщо активних станів більше одного:
//    L = Light (світло увімкнено, люди в погрібі)
//    N = Night (нічний режим)
//    T = Temperature (занадто холодно)
//    R = Resting (пауза після циклу роботи)
//    W = Waiting (чекаємо поки вологість підніметься)
// ════════════════════════════════════════════════════════════
void printStatusFlags() {
  lcd.setCursor(11, 0);
  lcd.print("|");
  lcd.print(autoMode ? "A" : "M");
  lcd.print("|");

  String flags = "";
  if (blockLight) flags += "L";
  if (blockNight) flags += "N";
  if (blockTemp)  flags += "T";
  if (isResting)  flags += "R";
  if (isWaiting)  flags += "W";

  if (flags.length() == 0) {
    lcd.print("_");   // Нема блокувань — все добре
  } else {
    int idx = (millis() / 1000) % flags.length();  // Прокрутка по секунді
    lcd.print(flags[idx]);
  }
  lcd.print("|");
}

// ════════════════════════════════════════════════════════════
//  РЯДОК 2 СТОРІНОК НАЛАШТУВАНЬ
//  Показує поточне значення (з блиманням при редагуванні)
//  і підказку [Edit] або [Ok?].
// ════════════════════════════════════════════════════════════
void printSettingRow2(float val, int decimals) {
  lcd.setCursor(0, 1);
  if (editStep > 0 && !showDigit) {
    lcd.print("      ");
  } else {
    lcd.print(val, decimals);
  }
  lcd.setCursor(10, 1);
  lcd.print(editStep > 0 ? "[Ok?] " : "[Edit]");
}

// ════════════════════════════════════════════════════════════
//  ОНОВЛЕННЯ ДИСПЛЕЯ
// ════════════════════════════════════════════════════════════
void updateDisplay() {
  updateBlink();

  switch (menuPage) {

    // ── HOME ────────────────────────────────────────────────
    case 0: {
      // Рядок 1: температура + вологість + статус
      lcd.setCursor(0, 0);
      if (cachedTemp >= 0 && cachedTemp < 10) lcd.print(" ");
      lcd.print(cachedTemp, 1);
      lcd.write(0xDF);
      lcd.print("C ");
      if (avgHum < 100) lcd.print(" ");
      if (avgHum < 10)  lcd.print(" ");
      lcd.print((int)avgHum);
      lcd.print("% ");
      printStatusFlags();

      // Рядок 2: час + стан вентилятора + залишок часу
      lcd.setCursor(0, 1);
      if (hr < 10) lcd.print("0"); lcd.print(hr);
      lcd.print(":");
      if (mn < 10) lcd.print("0"); lcd.print(mn);
      lcd.print(" F:");
      lcd.print(fanIsOn ? "On " : "Off");

      // Залишок часу: роботи (якщо увімкнений) або паузи (якщо відпочиває)
      unsigned long elapsed = 0, limit = 0;
      if (fanIsOn) {
        elapsed = millis() - fanStartMillis;
        limit   = (unsigned long)runTimeMin * 60000UL;
      } else if (isResting) {
        elapsed = millis() - fanStopMillis;
        limit   = (unsigned long)restTimeMin * 60000UL;
      }
      if (limit > 0 && limit > elapsed) {
        unsigned long rem = (limit - elapsed) / 1000;
        int remM = rem / 60, remS = rem % 60;
        if (remM > 0) { if (remM < 10) lcd.print(" "); lcd.print(remM); lcd.print("m"); }
        else lcd.print("   ");
        if (remS < 10) lcd.print("0");
        lcd.print(remS); lcd.print("s");
      } else {
        lcd.print("      ");
      }
      break;
    }

    // ── SET TIME ────────────────────────────────────────────
    case 1:
      lcd.setCursor(0, 0); lcd.print("Set Time        ");
      lcd.setCursor(0, 1);
      if (editStep == 1 && !showDigit) lcd.print("  "); else { if (hr<10) lcd.print("0"); lcd.print(hr); }
      lcd.print(":");
      if (editStep == 2 && !showDigit) lcd.print("  "); else { if (mn<10) lcd.print("0"); lcd.print(mn); }
      lcd.setCursor(10, 1); lcd.print(editStep > 0 ? "[Ok?] " : "[Edit]");
      break;

    // ── НАЛАШТУВАННЯ ────────────────────────────────────────
    case 2:
      lcd.setCursor(0, 0); lcd.print("Hum Wait %      ");
      printSettingRow2(humWait, 0); break;
    case 3:
      lcd.setCursor(0, 0); lcd.print("Hum OFF %       ");
      printSettingRow2(humOff, 0); break;
    case 4:
      lcd.setCursor(0, 0); lcd.print("Min Temp C      ");
      printSettingRow2(minTemp, 1); break;
    case 5:
      lcd.setCursor(0, 0); lcd.print("Light Lux lim   ");
      printSettingRow2(lightLuxLimit, 0); break;
    case 6:
      lcd.setCursor(0, 0); lcd.print("Avg Samples     ");
      printSettingRow2(avgCount, 0); break;
    case 7:
      lcd.setCursor(0, 0); lcd.print("Run Time min    ");
      printSettingRow2(runTimeMin, 0); break;
    case 8:
      lcd.setCursor(0, 0); lcd.print("Rest Time min   ");
      printSettingRow2(restTimeMin, 0); break;
    case 9:
      lcd.setCursor(0, 0); lcd.print("Start Hour      ");
      printSettingRow2(startH, 0); break;
    case 10:
      lcd.setCursor(0, 0); lcd.print("End Hour        ");
      printSettingRow2(endH, 0); break;

    // ── ТЕСТ РЕЛЕ ───────────────────────────────────────────
    case 11:
      lcd.setCursor(0, 0); lcd.print("Test Relay      ");
      lcd.setCursor(0, 1);
      if (editStep > 0 && !showDigit) lcd.print("        ");
      else lcd.print(fanIsOn ? "RUNNING " : "STOPPED ");
      lcd.setCursor(8, 1);
      lcd.print(editStep > 0 ? "UP/DN   " : "[SEL=Go]");
      break;

    // ── ДІАГНОСТИКА ─────────────────────────────────────────
    case 12:
      lcd.setCursor(0, 0); lcd.print("Diag DHT22      ");
      lcd.setCursor(0, 1);
      lcd.print("H:"); lcd.print(avgHum, 1);
      lcd.print(" T:"); lcd.print(cachedTemp, 1);
      lcd.print("   ");
      break;
    case 13:
      lcd.setCursor(0, 0); lcd.print("Diag Light      ");
      lcd.setCursor(0, 1);
      lcd.print("Lux: "); lcd.print((int)cachedLux);
      lcd.print("        ");
      break;
    case 14:
      lcd.setCursor(0, 0); lcd.print("Diag RTC        ");
      lcd.setCursor(0, 1);
      lcd.print(rtcFound ? "Status: OK      " : "NOT FOUND       ");
      break;
  }
}

// ════════════════════════════════════════════════════════════
//  CHANGELOG
// ════════════════════════════════════════════════════════════
/*
  ver 1.0 (база від Gemini)
    - Перша робоча версія на базі коду Gemini
    - Керування реле по вологості, температурі, освітленню
    - Меню налаштувань через кнопки LCD Shield
    - Усереднення вологості по буферу
    - Підтримка RTC DS1307
    - Програмний годинник якщо RTC відсутній
    - delay(250) в обробці кнопок (блокуючий)

  ver 2.0 (перше переписування Клодом)
    - ВИПРАВЛЕНО: humOn=40, humOff=70 (були переплутані місцями)
    - ВИПРАВЛЕНО: delay(250) замінено на millis() антидребезг
    - ВИПРАВЛЕНО: stopFan() більше не запускає паузу при зупинці
      через зовнішню причину (ніч/світло/мороз)
    - ВИПРАВЛЕНО: датчики кешуються, не читаються повторно в handleLogic()
    - ВИПРАВЛЕНО: тест реле більше не вимикає autoMode назавжди
    - ДОДАНО: сторінки Run Time, Rest Time, Start/End Hour в меню
    - ДОДАНО: індикатор R (Resting) в статусному рядку
    - ДОДАНО: constrain() на всіх параметрах меню
    - ДОДАНО: скидання буфера усереднення при зміні avgCount
    - ВИПРАВЛЕНО: конфлікт пін 8 (LCD) і реле → реле на пін 3

  ver 3.0
    - ДОДАНО: захист реле від дрижання (RELAY_CONFIRM_MS = 3000 мс)
      Будь-яка зміна стану реле тільки після 3с стабільної умови
    - ДОДАНО: прапори relayPendingOn / relayPendingOff
    - ДОДАНО: коментарі до всіх функцій і змінних

  ver 3.1
    - ДОДАНО: затримка при старті 15 секунд (стартовий екран)
      Реле не може спрацювати поки loop() не запущений
    - ДОДАНО: явне скидання isResting=false при старті
      (мотор ще не працював — пауза не потрібна)

  ver 3.4 (стабільна, зміни в коментарях)
    - ВИПРАВЛЕНО: коментарі в заголовку і меню
    - ЗМІНЕНО: час прогріву з 15 до 18 секунд
    - ДОДАНО: активне читання датчиків під час прогріву
      (loop не запущений, але буфер наповнюється)
    - ЗМІНЕНО: анімація прогріву — крапки замість статичного тексту

  ver 3.5 (невдала спроба виправлення)
    - СПРОБА виправити: вентилятор зупинявся через 2-3 секунди
      (humOff перевірявся одразу після вмикання)
    - СПРОБА виправити: % при старті показував рандомне значення
    - Не допомогло — проблема була глибша

  ver 3.6
    - ДОДАНО: стан W (Waiting) — очікування поки вологість підніметься
      до humWait перед початком циклів роботи
    - ДОДАНО: параметр humWait = 50% за замовчуванням
    - ДОДАНО: нова сторінка меню "Hum Wait %" (сторінка 2)
    - ВИПРАВЛЕНО: головний баг — humOff тепер перевіряється ТІЛЬКИ
      коли вентилятор вже працює, а не при спробі вмикання
      (усуває 2-3 секундний цикл одразу після старту)
    - ВИПРАВЛЕНО: прогрес-бар при старті — рівномірний по часу (0→100%)
      замість кроків по датчику. Вигляд: "Init [####      ] 67%"
    - ВИПРАВЛЕНО: після паузи (Resting) система знову перевіряє
      чи потрібен Waiting, а не одразу вмикає вентилятор
    - ВИПРАВЛЕНО: дублікат коментаря startFan/stopFan прибрано
    - ЗМІНЕНО: MENU_PAGES з 15 до 16 (додано Hum Wait)
    - ЗМІНЕНО: нумерація сторінок меню зсунута через новий пункт
    
  ver 3.7 (поточна)
    - Виправленна фундаментальна логічна помилка в значеннях за замовчуванням.
      Проблема: humOff = 70.0 означає "вимикати коли вологість опустилась
      до 70%". Але якщо реальна вологість, наприклад, 65% — то 
      умова avgHum <= humOff (65 <= 70) одразу істинна щойно вентилятор
      вмикається. Ось чому 3 секунди — це RELAY_CONFIRM_MS.
      Правильні значення:
      humWait = 50% — чекати поки підніметься до 50%
      humOff = 40% — вимикати коли опустилась до 40% (нижній поріг)
      humOn — взагалі зайвий з новою логікою Waiting, прибираємо.
    - Прибирана humOn, вентилятор вмикається при виході з Waiting

     ЗМІНЕНО в ручну:
    - ЗМІНЕНО: видалив humOn з сторінок і процессінгу, бо не проходила 
      компіляція через помилки, що не задекларовано, але в коді наявно.


*/
