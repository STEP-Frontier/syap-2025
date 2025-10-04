#include <Arduino.h>
#include <Wire.h>

constexpr int SDA_PIN = 4;
constexpr int SCL_PIN = 5;

constexpr uint8_t SENSOR_ADDR = 0x68; // ICM-20948 の AD0=Low 時 I2C アドレス
constexpr uint8_t REG_BANK_SEL = 0x7F;
static const uint8_t UB0_WHO_AM_I = 0x00;
constexpr uint8_t REG_ACCEL_CONFIG = 0x14;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x2D; // USER BANK 0
constexpr uint8_t REG_ACCEL_XOUT_L = 0x2E; // USER BANK 0
constexpr uint8_t REG_ACCEL_YOUT_H = 0x2F; // USER BANK 0
constexpr uint8_t REG_ACCEL_YOUT_L = 0x30; // USER BANK 0
constexpr uint8_t REG_ACCEL_ZOUT_H = 0x31; // USER BANK 0
constexpr uint8_t REG_ACCEL_ZOUT_L = 0x32; // USER BANK 0

void read_register(uint8_t reg, uint8_t *data, uint8_t len)
{
  Wire.beginTransmission(SENSOR_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(SENSOR_ADDR, len);

  for (uint8_t i = 0; i < len; i++)
  {
    if (Wire.available())
    {
      data[i] = Wire.read();
    }
  }
}

void write_register(uint8_t reg, uint8_t data)
{
  Wire.beginTransmission(SENSOR_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

void set_bank(uint8_t bank)
{
  write_register(REG_BANK_SEL, (bank & 0x03) << 4);
}

bool init_sensor()
{
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  set_bank(0);
  uint8_t who_am_i_val;
  read_register(UB0_WHO_AM_I, &who_am_i_val, 1);
  if (who_am_i_val != 0xEA)
  {
    return false;
  }
  return true;
}

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
    ; // シリアルポートの接続を待つ
  }

  Serial.println("Hello World! from practice-2");

  while (!init_sensor())
  {
    Serial.println("Sensor init failed, retrying...");
    delay(1000);
  }

  // set accel scale factor
  set_bank(2);
  uint8_t accel_config_reg;
  read_register(REG_ACCEL_CONFIG, &accel_config_reg, 1);
  uint8_t new_accel_config_reg = (accel_config_reg & ~0x06);
  write_register(REG_ACCEL_CONFIG, new_accel_config_reg);
}

void loop()
{
  /*
  int16_t ax = 0;
  int16_t ay = 0;
  int16_t az = 0;
  if (readAcceleration(ax, ay, az))
  {
    const float scale = 16384.0f; // +/-2g フルスケール換算
    Serial.print("Accel[g] X:");
    Serial.print(ax / scale, 3);
    Serial.print(" Y:");
    Serial.print(ay / scale, 3);
    Serial.print(" Z:");
    Serial.println(az / scale, 3);
  }
  else
  {
    Serial.println("I2C read error");
  }
  delay(500);*/

  set_bank(0);
  byte accel_x_h;
  byte accel_x_l;
  read_register(REG_ACCEL_XOUT_H, &accel_x_h, 1);
  read_register(REG_ACCEL_XOUT_L, &accel_x_l, 1);

  int16_t raw_accel_x = (int16_t)((accel_x_h << 8) | accel_x_l);
  double accel_x = raw_accel_x / 16384.0;

  byte accel_y_h;
  byte accel_y_l;
  read_register(REG_ACCEL_YOUT_H, &accel_y_h, 1);
  read_register(REG_ACCEL_YOUT_L, &accel_y_l, 1);

  int16_t raw_accel_y = (int16_t)((accel_y_h << 8) | accel_y_l);
  double accel_y = raw_accel_y / 16384.0;

  byte accel_z_h;
  byte accel_z_l;
  read_register(REG_ACCEL_ZOUT_H, &accel_z_h, 1);
  read_register(REG_ACCEL_ZOUT_L, &accel_z_l, 1);

  int16_t raw_accel_z = (int16_t)((accel_z_h << 8) | accel_z_l);
  double accel_z = raw_accel_z / 16384.0;

  // printf形式でx, y, zの値を同時に表示
  Serial.printf("Accel (X, Y, Z): %.3f, %.3f, %.3f\n", accel_x, accel_y, accel_z);

  delay(500);
}
