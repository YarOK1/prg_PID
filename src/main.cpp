// Підключення основних бібліотек для роботи з ESP32
#include <Arduino.h>           // Основна бібліотека Arduino для роботи з платами, включає базові функції (pinMode, digitalWrite тощо)
#include <BluetoothSerial.h>   // Бібліотека для роботи з вбудованим Bluetooth ESP32
#include <PID_v1.h>            // Бібліотека PID-контролера від Brett Beauregard для регулювання швидкості двигунів
#include <freertos/FreeRTOS.h> // Бібліотека FreeRTOS для багатозадачності на ESP32
#include <freertos/task.h>     // Допоміжна бібліотека FreeRTOS для роботи з задачами

// Створення об’єкта для роботи з Bluetooth
BluetoothSerial SerialBT;      // Ініціалізація Bluetooth-з’єднання з назвою "ESP32_MotorSync" (встановлюється в setup)

// Налаштування PID-контролерів для регулювання швидкості та синхронізації двигунів
double Setpoint = 2.0;         // Цільова швидкість обертання коліс у обертах за секунду (об/с), початкове значення
double Input_L, Input_R;       // Поточна швидкість лівого (L) і правого (R) колеса в об/с, обчислюється з енкодерів
double Output_L, Output_R;     // Вихідні значення PID для лівого і правого двигунів (значення ШІМ від 0 до 255)
double Sync_Setpoint = 0;      // Цільова різниця між швидкостями коліс для синхронізації (0 означає однакову швидкість)
double Sync_Input;             // Поточна різниця між Input_L і Input_R, вхід для синхронізаційного PID
double Sync_Output;            // Вихід синхронізаційного PID для корекції швидкості двигунів
double Kp = 20.0, Ki = 5.0, Kd = 5.0;          // Коефіцієнти PID (пропорційний, інтегральний, диференціальний) для основних контролерів
double Sync_Kp = 3.0, Sync_Ki = 2.0, Sync_Kd = 2.0; // Коефіцієнти PID для синхронізації
PID PID_L(&Input_L, &Output_L, &Setpoint, Kp, Ki, Kd, DIRECT); // PID для лівого двигуна (вхід, вихід, ціль, коефіцієнти, режим)
PID PID_R(&Input_R, &Output_R, &Setpoint, Kp, Ki, Kd, DIRECT); // PID для правого двигуна
PID PID_Sync(&Sync_Input, &Sync_Output, &Sync_Setpoint, Sync_Kp, Sync_Ki, Sync_Kd, DIRECT); // PID для синхронізації

// Визначення пінів для ESP32 для підключення до драйвера TB6612FNG і енкодерів
#define PWMA 25    // ШІМ-сигнал для лівого двигуна (керування швидкістю)
#define AIN1 27    // Вхід 1 для лівого двигуна (логіка напрямку)
#define AIN2 26    // Вхід 2 для лівого двигуна (логіка напрямку)
#define PWMB 13    // ШІМ-сигнал для правого двигуна
#define BIN1 14    // Вхід 1 для правого двигуна
#define BIN2 12    // Вхід 2 для правого двигуна
#define STBY 33    // Пін для активації драйвера (HIGH = увімкнено)
#define ENCODER_L_pin 34  // Пін для цифрового сигналу лівого енкодера (без переривань)
#define ENCODER_R_pin 35  // Пін для цифрового сигналу правого енкодера

// Змінні для роботи з енкодерами та обчислення швидкості/відстані
int holeCount_L = 0, holeCount_R = 0;       // Лічильники отворів енкодерів для лівого і правого колеса
const int holesPerRotation = 20;            // Кількість отворів на один оберт колеса (залежить від енкодера FC-03)
float WheelD = 0.065;                       // Діаметр колеса в метрах (6.5 см)
float WheelLength = WheelD * 3.1415;        // Довжина окружності колеса (D * π)
float holeDistance = WheelLength / holesPerRotation; // Відстань між отворами в метрах
float smoothed_Input_L = 0, smoothed_Input_R = 0;    // Згладжені значення швидкості для лівого і правого колеса
const float alpha = 0.2;                    // Коефіцієнт експоненційного згладжування (0–1, менший = плавніше)
float TotalDistance_L = 0, TotalDistance_R = 0;      // Загальна відстань, пройдена лівим і правим колесом (м)

// Налаштування апаратного таймера для періодичного зчитування енкодерів
hw_timer_t *timer = NULL;                   // Вказівник на об’єкт таймера (nullptr до ініціалізації)
volatile bool timerFlag = false;            // Прапорець, який сигналізує про спрацювання таймера (volatile для коректної роботи в ISR)

// Команди для керування через Bluetooth
#define FORWARD 'F'    // Збільшити цільову швидкість (Setpoint += 0.1)
#define BACKWARD 'B'   // Зменшити цільову швидкість (Setpoint -= 0.1)
#define START 'A'      // Запустити автоналаштування PID
#define PAUSE 'P'      // Зупинити двигуни (Setpoint = 0)

// Змінні для автоналаштування PID
const int SAMPLE_SIZE = 50;                 // Кількість зразків для аналізу помилок під час автоналаштування
double error_L[SAMPLE_SIZE], error_R[SAMPLE_SIZE], sync_error[SAMPLE_SIZE]; // Масиви для зберігання помилок
int sample_count = 0;                       // Лічильник зібраних зразків
bool tuning = false;                        // Прапорець режиму автоналаштування

// Прототип функції автоналаштування PID (оголошення перед використанням)
void adjustPIDCoefficients();

// Функція обробки таймера (виконується в перериванні)
void IRAM_ATTR onTimer() {  // IRAM_ATTR розміщує функцію в швидкій пам’яті ESP32 для ISR
  timerFlag = true;         // Встановлює прапорець, щоб основний код знав про спрацювання таймера
}

// Функція зчитування енкодерів без переривань (використовує таймер)
void readEncoders() {
  static int lastState_L = HIGH, lastState_R = HIGH; // Статичні змінні для збереження попереднього стану енкодерів
  int currentState_L = digitalRead(ENCODER_L_pin);   // Поточний стан лівого енкодера (HIGH або LOW)
  int currentState_R = digitalRead(ENCODER_R_pin);   // Поточний стан правого енкодера

  // Перевірка переходу з HIGH на LOW (спадний фронт) для лівого енкодера
  if (currentState_L == LOW && lastState_L == HIGH) {
    holeCount_L++;              // Збільшуємо лічильник отворів
    TotalDistance_L += holeDistance; // Додаємо відстань за один отвір до загальної
  }
  // Те саме для правого енкодера
  if (currentState_R == LOW && lastState_R == HIGH) {
    holeCount_R++;
    TotalDistance_R += holeDistance;
  }

  lastState_L = currentState_L; // Оновлюємо попередній стан
  lastState_R = currentState_R;
}

// Завдання для ядра 0: Управління двигунами та обчислення PID
void motorControlTask(void *pvParameters) {
  unsigned long lastTime = millis(); // Час останнього оновлення для періодичності

  while (1) { // Безкінечний цикл задачі
    if (timerFlag) { // Якщо таймер спрацював
      readEncoders(); // Зчитуємо енкодери
      timerFlag = false; // Скидаємо прапорець
    }

    unsigned long currentTime = millis(); // Поточний час
    if (currentTime - lastTime >= 500) { // Оновлення кожні 500 мс
      // Обчислення обертів за секунду на основі кількості отворів
      float rotationsPerSecond_L = (float)holeCount_L / holesPerRotation;
      float rotationsPerSecond_R = (float)holeCount_R / holesPerRotation;

      // Експоненційне згладжування швидкості для зменшення шумів
      smoothed_Input_L = alpha * rotationsPerSecond_L + (1 - alpha) * smoothed_Input_L;
      smoothed_Input_R = alpha * rotationsPerSecond_R + (1 - alpha) * smoothed_Input_R;

      Input_L = smoothed_Input_L; // Оновлюємо вхідні дані для PID лівого двигуна
      Input_R = smoothed_Input_R; // Оновлюємо вхідні дані для PID правого двигуна
      Sync_Input = Input_L - Input_R; // Різниця швидкостей для синхронізації

      PID_L.Compute(); // Розрахунок PID для лівого двигуна
      PID_R.Compute(); // Розрахунок PID для правого двигуна
      PID_Sync.Compute(); // Розрахунок PID для синхронізації

      // Корекція ШІМ-сигналів із урахуванням синхронізації
      double Final_Output_L = Output_L - Sync_Output; // Зменшуємо лівий, якщо Sync_Output > 0
      double Final_Output_R = Output_R + Sync_Output; // Збільшуємо правий, якщо Sync_Output > 0

      // Обмежуємо значення ШІМ у діапазоні 0-255
      Final_Output_L = constrain(Final_Output_L, 0, 255);
      Final_Output_R = constrain(Final_Output_R, 0, 255);

      // Керування напрямком і швидкістю лівого двигуна
      digitalWrite(AIN1, HIGH); // Напрямок вперед
      digitalWrite(AIN2, LOW);
      analogWrite(PWMA, (int)Final_Output_L); // Встановлюємо ШІМ

      // Керування напрямком і швидкістю правого двигуна
      digitalWrite(BIN1, HIGH); // Напрямок вперед
      digitalWrite(BIN2, LOW);
      analogWrite(PWMB, (int)Final_Output_R);

      // Збір даних для автоналаштування
      if (tuning && sample_count < SAMPLE_SIZE) {
        error_L[sample_count] = Setpoint - Input_L; // Помилка для лівого двигуна
        error_R[sample_count] = Setpoint - Input_R; // Помилка для правого двигуна
        sync_error[sample_count] = Sync_Input;      // Помилка синхронізації
        sample_count++; // Збільшуємо лічильник зразків
      }

      // Якщо зібрано достатньо даних, запускаємо автоналаштування
      if (tuning && sample_count == SAMPLE_SIZE) {
        adjustPIDCoefficients(); // Налаштування коефіцієнтів PID
        tuning = false; // Вихід із режиму автоналаштування
      }

      holeCount_L = 0; // Скидаємо лічильники отворів
      holeCount_R = 0;
      lastTime = currentTime; // Оновлюємо час останнього циклу
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // Затримка 10 мс для зменшення навантаження на ядро
  }
}

// Завдання для ядра 1: Обробка Bluetooth і виведення статистики
void bluetoothTask(void *pvParameters) {
  while (1) { // Безкінечний цикл задачі
    if (SerialBT.available()) { // Якщо є дані з Bluetooth
      char received = SerialBT.read(); // Зчитуємо символ
      switch (received) { // Обробка команд
        case FORWARD: Setpoint += 0.1; break; // Збільшуємо цільову швидкість на 0.1 об/с
        case BACKWARD: Setpoint -= 0.1; break; // Зменшуємо цільову швидкість на 0.1 об/с
        case PAUSE: Setpoint = 0; break;       // Зупиняємо двигуни
        case START: tuning = true; sample_count = 0; break; // Запускаємо автоналаштування
      }
      Setpoint = constrain(Setpoint, 0, 4); // Обмежуємо Setpoint у межах 0-4 об/с
      Serial.print("Setpoint: "); Serial.println(Setpoint); // Виводимо нове значення Setpoint
    }

    // Виведення статистики в Serial Monitor
    Serial.printf("Setpoint: %.2f | L: %.2f -> %.0f | R: %.2f -> %.0f | Sync: %.2f -> %.0f | Dist L: %.2f | Dist R: %.2f\n",
                  Setpoint,        // Цільова швидкість
                  Input_L, Output_L, // Поточна швидкість і ШІМ для лівого двигуна
                  Input_R, Output_R, // Поточна швидкість і ШІМ для правого двигуна
                  Sync_Input, Sync_Output, // Різниця швидкостей і корекція
                  TotalDistance_L, TotalDistance_R); // Загальна відстань для обох коліс

    vTaskDelay(500 / portTICK_PERIOD_MS); // Затримка 500 мс для періодичного виведення
  }
}

// Функція автоналаштування коефіцієнтів PID
void adjustPIDCoefficients() {
  double avg_error_L = 0, avg_error_R = 0, avg_sync_error = 0; // Середні помилки
  double max_error_L = 0, max_error_R = 0, max_sync_error = 0; // Максимальні помилки

  // Обчислення середніх і максимальних помилок на основі зібраних зразків
  for (int i = 0; i < SAMPLE_SIZE; i++) {
    avg_error_L += error_L[i];        // Сума помилок для лівого двигуна
    avg_error_R += error_R[i];        // Сума помилок для правого двигуна
    avg_sync_error += sync_error[i];  // Сума помилок синхронізації
    max_error_L = max(max_error_L, abs(error_L[i])); // Максимальна абсолютна помилка лівого
    max_error_R = max(max_error_R, abs(error_R[i])); // Максимальна абсолютна помилка правого
    max_sync_error = max(max_sync_error, abs(sync_error[i])); // Максимальна помилка синхронізації
  }
  avg_error_L /= SAMPLE_SIZE; // Середня помилка для лівого двигуна
  avg_error_R /= SAMPLE_SIZE; // Середня помилка для правого двигуна
  avg_sync_error /= SAMPLE_SIZE; // Середня помилка синхронізації

  // Коригування основних PID-коефіцієнтів
  if (avg_error_L > 0.1 || avg_error_R > 0.1) Ki += 1.0; // Збільшуємо Ki при великій сталій помилці
  else if (avg_error_L < -0.1 || avg_error_R < -0.1) Ki -= 1.0; // Зменшуємо Ki при від’ємній помилці
  if (max_error_L > 0.5 || max_error_R > 0.5) Kd += 1.0; // Збільшуємо Kd при великих коливаннях
  if (max_error_L < 0.2 && max_error_R < 0.2) Kp += 5.0; // Збільшуємо Kp при повільній реакції

  // Коригування коефіцієнтів синхронізації
  if (abs(avg_sync_error) > 0.05) Sync_Ki += 1.0; // Збільшуємо Sync_Ki при сталій різниці
  if (max_sync_error > 0.3) { // Якщо корекція занадто сильна
    Sync_Kp -= 1.0; // Зменшуємо пропорційну складову
    Sync_Kd += 0.5; // Збільшуємо диференціальну для стабільності
  }

  // Обмеження коефіцієнтів у розумних межах
  Kp = constrain(Kp, 5.0, 50.0);        // Пропорційний коефіцієнт
  Ki = constrain(Ki, 0.0, 20.0);        // Інтегральний коефіцієнт
  Kd = constrain(Kd, 0.0, 10.0);        // Диференціальний коефіцієнт
  Sync_Kp = constrain(Sync_Kp, 1.0, 20.0); // Синхронізація: пропорційний
  Sync_Ki = constrain(Sync_Ki, 0.0, 10.0); // Синхронізація: інтегральний
  Sync_Kd = constrain(Sync_Kd, 0.0, 5.0);  // Синхронізація: диференціальний

  // Оновлення коефіцієнтів у PID-контролерах
  PID_L.SetTunings(Kp, Ki, Kd);         // Оновлення для лівого двигуна
  PID_R.SetTunings(Kp, Ki, Kd);         // Оновлення для правого двигуна
  PID_Sync.SetTunings(Sync_Kp, Sync_Ki, Sync_Kd); // Оновлення для синхронізації

  // Виведення нових коефіцієнтів для відлагодження
  Serial.printf("New PID: Kp=%.1f, Ki=%.1f, Kd=%.1f\n", Kp, Ki, Kd);
  Serial.printf("New Sync PID: Kp=%.1f, Ki=%.1f, Kd=%.1f\n", Sync_Kp, Sync_Ki, Sync_Kd);
}

void setup() {
  Serial.begin(115200); // Ініціалізація Serial-порту зі швидкістю 115200 бод для моніторингу
  SerialBT.begin("ESP32_MotorSync"); // Ініціалізація Bluetooth із заданою назвою пристрою

  // Налаштування пінів як виходів для драйвера двигунів
  pinMode(PWMA, OUTPUT);  // ШІМ для лівого двигуна
  pinMode(AIN1, OUTPUT);  // Вхід 1 лівого двигуна
  pinMode(AIN2, OUTPUT);  // Вхід 2 лівого двигуна
  pinMode(PWMB, OUTPUT);  // ШІМ для правого двигуна
  pinMode(BIN1, OUTPUT);  // Вхід 1 правого двигуна
  pinMode(BIN2, OUTPUT);  // Вхід 2 правого двигуна
  pinMode(STBY, OUTPUT);  // Пін активації драйвера
  digitalWrite(STBY, HIGH); // Увімкнення драйвера TB6612FNG

  // Налаштування пінів енкодерів як входів із внутрішнім підтягуванням
  pinMode(ENCODER_L_pin, INPUT_PULLUP); // Лівий енкодер
  pinMode(ENCODER_R_pin, INPUT_PULLUP); // Правий енкодер

  // Налаштування PID-контролерів
  PID_L.SetMode(AUTOMATIC);      // Автоматичний режим роботи для лівого PID
  PID_R.SetMode(AUTOMATIC);      // Автоматичний режим для правого PID
  PID_Sync.SetMode(AUTOMATIC);   // Автоматичний режим для синхронізації
  PID_L.SetOutputLimits(0, 255); // Обмеження ШІМ-виходу для лівого двигуна
  PID_R.SetOutputLimits(0, 255); // Обмеження ШІМ-виходу для правого двигуна
  PID_Sync.SetOutputLimits(-10, 10); // Обмеження корекції синхронізації
  PID_L.SetSampleTime(100);      // Період обчислення PID (100 мс)
  PID_R.SetSampleTime(100);
  PID_Sync.SetSampleTime(100);

  // Налаштування апаратного таймера для зчитування енкодерів
  timer = timerBegin(0, 80, true);  // Ініціалізація таймера 0, дільник 80 (1 мкс), повторення = true
  timerAttachInterrupt(timer, &onTimer, true); // Прив’язка функції onTimer до таймера
  timerAlarmWrite(timer, 100, true); // Встановлення періоду 100 мкс (10 кГц), авто-перезавантаження
  timerAlarmEnable(timer);           // Увімкнення таймера

  // Створення завдань FreeRTOS із прив’язкою до ядер
  xTaskCreatePinnedToCore(
    motorControlTask,  // Функція задачі
    "MotorTask",       // Ім’я задачі
    4096,              // Розмір стеку (байт)
    NULL,              // Параметри (не використовуються)
    1,                 // Пріоритет (1 = низький)
    NULL,              // Дескриптор задачі (не потрібен)
    0);                // Ядро 0
  xTaskCreatePinnedToCore(
    bluetoothTask,     // Функція задачі
    "BluetoothTask",   // Ім’я задачі
    4096,              // Розмір стеку
    NULL,              // Параметри
    1,                 // Пріоритет
    NULL,              // Дескриптор
    1);                // Ядро 1
}

void loop() {
  // Функція loop() порожня, оскільки вся логіка реалізована в задачах FreeRTOS
}