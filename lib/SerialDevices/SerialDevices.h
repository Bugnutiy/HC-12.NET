#pragma once
#include <Arduino.h>
#include <Hamming.h>

#include <LinkedList.h>
#include <EEManager.h>
#include <MemoryFree.h>
// #include <mString.h>
#include "config.h"
#include <SoftwareSerial.h>
#include <Dbg.h>

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
  byte cmd[MAX_CMD_SIZE];
  uint32_t millis = 0;
};
#pragma pack(pop)

class SerialDevices
{
protected:
  int k = 0;
  const char *PRIVATE_KEY = PRIVATE_KEY_CONF;
  const size_t PRIVATE_KEY_LENGTH = strlen(PRIVATE_KEY);

  LinkedList<DeviceData> *DevicesList;
  LinkedList<Transaction> *SendList;
  byte _id = 0, _remoteness = 0;
  char _name[16] = "\0";

  EEManager *E_id;
  EEManager *E_remoteness;
  EEManager *E_name;

  Hamming<HAMMING_SIZE> *HammingVar;

  Transaction command;

public:
  SoftwareSerial *SSerial;

  /// @brief Конструктор по умолчанию
  SerialDevices();
  ~SerialDevices();

  SerialDevices(uint8_t id, char name[16] = '\0', uint8_t remoteness = 0);

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
  void tmpKey(char *key, const uint32_t &millis);

  /// @brief Xor строку с ключом и милисекундами
  /// @param str строка для изменения
  /// @param millis миллисекунды для шифратора
  void encrypt(char *str, const uint32_t &millis);

  ///@brief Xor byte массив с ключом и милис
  ///@param str byte-массив (будет изменен)
  /// @param millis миллисекунды для шифратора
  void encrypt(byte *str, const uint32_t &millis);
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
  SendList = new LinkedList<Transaction>();

  // Serial.begin(SERIAL_SPEED);
  SSerial->begin(SSERIAL_SPEED);
  SSerial->setTimeout(SERIAL_TIMEOUT);

  pinMode(LED_BUILTIN, OUTPUT);

  byte state = E_id->begin(0, 'i');
  E_remoteness->begin(E_id->nextAddr(), 'r');
  E_name->begin(E_remoteness->nextAddr(), 'n');
  if ((!state) || (_id == 0))
  {
    uint16_t timeout = millis();
    uint16_t sender = timeout;
    while (_id == 0)
    {
      ddd("Мы в цикле while");
      command.Addr_id = 0;
      command.Rem_id = 0;
      command.Trans_id = 0;
      command.type = 1;
      command.millis = millis();
      command.cmd[0] = 0;

      if ((uint16_t)millis() - sender >= 500)
      {
        sender = millis();
        serial_sender(command);
      }

      if (SSerial->available())
      {
        serial_reciever(command);
      }

      if ((uint16_t)millis() - timeout >= 10000)
      {
        _id = 1;
        _remoteness = 0;
        _name[0] = 0;
      }
    }
    ddd("Мы вышли из while");

    E_id->updateNow();
    E_remoteness->updateNow();
    E_name->updateNow();
  }
  else
  {
  }
}
SerialDevices::SerialDevices(uint8_t id, char name[16], uint8_t remoteness)
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

  byte state = E_id->begin(0, 'i');
  E_remoteness->begin(E_id->nextAddr(), 'r');
  E_name->begin(E_remoteness->nextAddr(), 'n');
  if ((!state) || (_id == 0))
  {
    _id = id;
    strcpy(_name, name);
    _remoteness = remoteness;

    E_id->updateNow();
    E_name->updateNow();
    E_remoteness->updateNow();
  }
  else
  {
  }
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
}

void SerialDevices::tmpKey(char *key, const uint32_t &milis)
{
  uint8_t millis_sault[4];
  for (uint8_t i = 0; i < 4; i++)
  {
    millis_sault[i] = (milis >> (i * 8)) & 0xFF; // раскладываем millis по байтам
    if (!millis_sault[i])                        // если байт равен 0, то этот байт будет суммой всех байтов
    {
      for (uint8_t j = 0; j < i; j++)
      {
        millis_sault[i] += millis_sault[j];
      }

      if (!millis_sault[i]) // если даже предыдущее не помогло, то тупо вкинем единицу в этот байт.
      {
        ++millis_sault[i];
      }
    }
  }
  for (uint8_t i = 0; i < PRIVATE_KEY_LENGTH; i++)
  {
    key[i] = PRIVATE_KEY[i] ^ millis_sault[i & 3]; // засаливаем приватный ключ
  }
  key[PRIVATE_KEY_LENGTH + 1] = 0;
}

void SerialDevices::encrypt(char *str, const uint32_t &milis)
{
  char key[PRIVATE_KEY_LENGTH + 1];
  // strcpy(key,PRIVATE_KEY);

  tmpKey(key, milis);

  for (byte i = 0; i < (MAX_CMD_SIZE - 1); i++)
  {
    str[i] ^= key[i % (PRIVATE_KEY_LENGTH)];
  }
}

void SerialDevices::encrypt(byte *str, const uint32_t &milis)
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
  encrypt(transa.cmd, transa.millis);
  // Hamming<HAMMING_SIZE> Hambuf;
  // Serial.println(transa.cmd);
  HammingVar->pack(transa);
  SSerial->write(HammingVar->buffer, HammingVar->length());
  SSerial->flush();
  // Hambuf.stop(); //Никогда не делай эту херню!!!!!!!
}

bool SerialDevices::serial_reciever(Transaction &package)
{
  // Hamming<HAMMING_SIZE> Hambuf;
  HammingVar->pack(package);
  size_t h_length = HammingVar->length();
  // Serial.println(h_length);
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
      package = *(Transaction *)HammingVar->buffer;
      encrypt(package.cmd, package.millis);
      return 1;
    }
  }

  return 0;
}

// имя_класса имя_объекта = имя_класса();
// SerialDevices Device = SerialDevices();