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
