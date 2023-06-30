#pragma once
#include <Arduino.h>
#include "Hamming.h"

#include <LinkedList.h>
#include <EEManager.h>
#include <MemoryFree.h>
// #include <mString.h>
#include "config.h"
#include <SoftwareSerial.h>

#pragma pack(push, 1)
struct DeviceData
{
  byte id, remoteness, informer_id, informer_remoteness, error_count;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct Transaction
{
  byte Rem_id = 0, Addr_id = 0, Trans_id = 0, type = 0;
  char cmd[MAX_CMD_SIZE] = "";
  unsigned long millis = 0;
};
#pragma pack(pop)

class SerialDevices
{
protected:
  int k = 0;
  const char *PRIVATE_KEY = PRIVATE_KEY_CONF;
  const size_t PRIVATE_KEY_LENGTH = strlen(PRIVATE_KEY);

  LinkedList<DeviceData> *DevicesList;
  // SoftwareSerial serial = SoftwareSerial(RX_PIN, TX_PIN);
  byte _id = 0, _remoteness = 0;
  char _name[16] = "\0";

  EEManager *E_id;
  EEManager *E_remoteness;
  EEManager *E_name;

  Hamming<HAMMING_SIZE> *HammingVar;

  Transaction command;

  unsigned long t1=0,t2=0,t3=0;

public:
  SoftwareSerial *SSerial;

  /// @brief Конструктор по умолчанию
  SerialDevices();
  ~SerialDevices();

  /// @brief Ид этого девайса
  const byte &id = _id;

  /// @brief Удаленность этого девайса от сервака
  const byte &remoteness = _remoteness;

  /// @brief Имя этого девайса
  const char *name = _name;

  /// @brief Сменить id и rem девайса
  /// @param id
  /// @param remoteness
  void setDevice(byte &id, byte &remoteness);

  /// @brief Сменить имя девайса
  /// @param name
  void setDevice(char name[16]);

  /// @brief Добавить новое устройство
  /// @param device
  /// @return Успешность операции
  bool ListAdd(DeviceData &device);

  /// @brief Добавить новое устройство
  /// @param id
  /// @param remoteness
  /// @param informer_id
  /// @param informer_remoteness
  /// @param error_count
  /// @return Успешность операции
  bool ListAdd(byte &id, byte &remoteness, byte &informer_id, byte &informer_remoteness, byte &error_count);

  ///@brief Поиск ид листа по ид девайса
  ///@param device_id ид девайса
  ///@return Ид листа
  byte ListSearch(const byte &device_id);

  ///@brief Получить запись о девайсе в сети
  ///@param list_id ид записи в списке
  ///@return DeviceData|null
  DeviceData ListGet(const byte &list_id);

  /// @brief Количество записей в списке
  byte ListSize();

  /// @brief Основная функция-обработчик событий
  void tick();

  /// @brief Приём сообщения
  bool serial_reciever(Transaction &cmd);

  /// @brief Отправка сообщения
  /// @param cmd Структура транзакция
  void serial_sender(Transaction &cmd);

  /// @brief Xor key и милисекунды особым образом
  /// @param key 
  /// @param millis 
  void tmpKey(char *key, const unsigned long &millis);

  /// @brief Xor строку с ключом и милисекундами
  /// @param str строка для изменения
  /// @param millis миллисекунды для шифратора
  void xorStr(char *str, const unsigned long &millis);

  ///@brief Выводит в Serial.println сообщение
  ///@param out сообщение для вывода
  template <typename T>
  void dd(const T &out);
};

// extern SerialDevices Device;


// .......................................................................................................................
SerialDevices::SerialDevices()
{
  DevicesList = new LinkedList<DeviceData>();
  E_id = new EEManager(_id);
  E_remoteness = new EEManager(_remoteness);
  E_name = new EEManager(_name);
  HammingVar = new Hamming<HAMMING_SIZE>;
  SSerial = new SoftwareSerial(RX_PIN, TX_PIN);

  // Serial.begin(SERIAL_SPEED);
  SSerial->begin(SSERIAL_SPEED);
  SSerial->setTimeout(SERIAL_TIMEOUT);

  pinMode(LED_BUILTIN, OUTPUT);

  E_id->begin(0, 'i');
  E_remoteness->begin(E_id->nextAddr(), 'r');
  E_name->begin(E_remoteness->nextAddr(), 'n');
}

SerialDevices::~SerialDevices()
{
  if (DevicesList)
  {
    DevicesList->~LinkedList();
    // delete DevicesList;
    DevicesList = nullptr;
  }
  if (E_id)
  {
    delete E_id;
    E_id = nullptr;
  }
  if (E_remoteness)
  {
    delete E_remoteness;
    E_remoteness = nullptr;
  }
  if (E_name)
  {
    delete E_name;
    E_name = nullptr;
  }
  if (HammingVar)
  {
    HammingVar->~Hamming();
    delete HammingVar;
    HammingVar = nullptr;
  }
  if (SSerial)
  {
    SSerial->~SoftwareSerial();
    // delete SSerial;
    SSerial = nullptr;
  }
}

void SerialDevices::setDevice(byte &id, byte &remoteness)
{

  if (_id != id)
  {
    _id = id;
    E_id->updateNow();
  }
  if (_remoteness != remoteness)
  {
    _remoteness = remoteness;
    E_remoteness->updateNow();
  }
}

void SerialDevices::setDevice(char name[16])
{
  if (strcmp(_name, name))
  {
    strcpy(_name, name);
    E_name->updateNow();
  }
}

bool SerialDevices::ListAdd(DeviceData &device)
{
  if (freeMemory() > MIN_RAM)
  {
    return DevicesList->add(device);
  }
  return false;
}

bool SerialDevices::ListAdd(byte &id, byte &remoteness, byte &informer_id, byte &informer_remoteness, byte &error_count)
{
  DeviceData tmp;
  tmp.id = id;
  tmp.remoteness = remoteness;
  tmp.informer_id = informer_id;
  tmp.informer_remoteness = informer_remoteness;
  tmp.error_count = error_count;
  return ListAdd(tmp);
}

byte SerialDevices::ListSearch(const byte &device_id)
{

  for (byte i = 0; i < DevicesList->size(); ++i)
  {
    DeviceData tmp = DevicesList->get(i);
    if (tmp.id == device_id)
      return i;
  }
  return 0;
}

DeviceData SerialDevices::ListGet(const byte &list_id)
{
  return DevicesList->get(list_id);
}

byte SerialDevices::ListSize()
{
  return DevicesList->size();
}

void SerialDevices::tick()
{
  unsigned long tt1 = millis();
  if (tt1 - t1 > 1000 || tt1 < t1)
  {
    t1 = tt1;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  unsigned long tt2 = millis();
  if (tt2 - t2 > 1000 || tt2 < t2)
  {
    t2 = tt2;
    // Serial.print("PR_KEY_LEN: ");
    // Serial.println(PRIVATE_KEY_LENGTH);
  }

  unsigned long tt3 = millis();
  if (tt3 - t3 > 500 || tt3 < t3)
  // if (1)
  {
    // Serial.println("----------------------");
    k++;
    t3 = tt3;
    Transaction tmp;
    tmp.Addr_id = 1;
    tmp.millis = tt3;
    tmp.Rem_id = 2;
    tmp.Trans_id = 4;
    tmp.type = 5;
    strcpy(tmp.cmd, "testStr");
    serial_sender(tmp);
    // Serial.println(k);
    // Serial.println("----------------------");
  }
  if (SSerial->available())
  {
    Serial.println(freeMemory());
    // delay(10);
    Transaction trans;
    if (serial_reciever(trans))
    {
      // Serial.println(trans.Addr_id);
      // Serial.println(trans.Rem_id);
      // Serial.println(trans.Trans_id);
      Serial.println(trans.cmd);
      Serial.println(freeMemory());
      // Serial.println(trans.millis);
      // Serial.println(trans.type);
    }
    while (SSerial->available())
    {
      SSerial->read();
    }
  }
}

void SerialDevices::tmpKey(char *key, const unsigned long &milis)
{
  uint8_t millis_sault[4];
  for (uint8_t i = 0; i < 4; i++)
  {
    millis_sault[i] = (milis >> (i * 8)) & 0xFF;
    if (!millis_sault[i])
    {
      for (uint8_t j = 0; j < i; j++)
      {
        millis_sault[i] += millis_sault[j];
      }

      if (!millis_sault[i])
      {
        ++millis_sault[i];
      }
    }
  }
  for (uint8_t i = 0; i < PRIVATE_KEY_LENGTH; i++)
  {
    key[i] = PRIVATE_KEY[i] ^ millis_sault[i & 3];
  }
  key[PRIVATE_KEY_LENGTH + 1] = 0;
}

void SerialDevices::xorStr(char *str, const unsigned long &milis)
{
  char key[PRIVATE_KEY_LENGTH + 1];
  // strcpy(key,PRIVATE_KEY);

  tmpKey(key, milis);

  for (byte i = 0; i < (MAX_CMD_SIZE - 1); i++)
  {
    str[i] ^= key[i % (PRIVATE_KEY_LENGTH)];
  }
}

void SerialDevices::serial_sender(Transaction &transa)
{
  xorStr(transa.cmd, transa.millis);
  // Hamming<HAMMING_SIZE> Hambuf;
  // Serial.println(transa.cmd);
  HammingVar->pack(transa);
  SSerial->write(HammingVar->buffer, HammingVar->length());
  SSerial->flush();
  // Hambuf.stop(); //Никогда не делай эту херню!!!!!!!
}

bool SerialDevices::serial_reciever(Transaction &cmd)
{
  // Hamming<HAMMING_SIZE> Hambuf;
  HammingVar->pack(cmd);
  size_t h_length = HammingVar->length();
  Serial.println(h_length);
  uint8_t buffer[h_length];

  if (SSerial->readBytes((uint8_t *)&buffer, h_length))
  {
    while (SSerial->available())
    {
      SSerial->read();
    }
    HammingVar->unpack(buffer, h_length);
    if (!HammingVar->status())
    {
      cmd = *(Transaction *)HammingVar->buffer;
      xorStr(cmd.cmd, cmd.millis);
      return 1;
    }
  }

  return 0;
}

template <typename T>
void SerialDevices::dd(const T &out)
{
  // Serial.println(out);
}

// имя_класса имя_объекта = имя_класса();
// SerialDevices Device = SerialDevices();