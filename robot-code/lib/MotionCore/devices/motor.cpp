#include "motor.h"

static BLDCDriver3PWM left_motor_driver = BLDCDriver3PWM(32, 33, 25, 22);     // 前 3 个参数是 UH, VH, WH 引脚，最后一个是使能 EN
static BLDCDriver3PWM right_motor_driver = BLDCDriver3PWM(26, 27, 14, 12);
static MagneticSensorI2C left_encoder = MagneticSensorI2C(AS5600_I2C);
static MagneticSensorI2C right_encoder = MagneticSensorI2C(AS5600_I2C);

BLDCMotor left_motor = BLDCMotor(7, 12.27166f, 100.0f);      // 极对数，线电阻，KV 值
BLDCMotor right_motor = BLDCMotor(7, 12.27166f, 100.0f);     // 极对数，线电阻，KV 值

/**
 * @brief 初始化电机
 */
void motor_init(void)
{
    i2c_bus_init(&i2c1);
    i2c_bus_init(&i2c2);

    // 初始化编码器
    left_encoder.init(i2c1.get_TwoWire_handle(&i2c1));
    right_encoder.init(i2c2.get_TwoWire_handle(&i2c2));

    // 绑定传感器
    left_motor.linkSensor(&left_encoder);
    right_motor.linkSensor(&right_encoder);

    // 绑定驱动器
    left_motor.linkDriver(&left_motor_driver);
    right_motor.linkDriver(&right_motor_driver);

    // PWM 调制方式
    left_motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    right_motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

    // 驱动器配置
    left_motor_driver.voltage_power_supply = 8.0f;
    right_motor_driver.voltage_power_supply = 8.0f;
    left_motor_driver.init();
    right_motor_driver.init();

    // 电机配置
    left_motor.voltage_sensor_align = 6.0f;
    right_motor.voltage_sensor_align = 6.0f;
    left_motor.controller = MotionControlType::torque;      // 运动控制器类型为扭矩模式
    right_motor.controller = MotionControlType::torque;
    left_motor.torque_controller = TorqueControlType::voltage;      // 扭矩控制器类型为电压模式
    right_motor.torque_controller = TorqueControlType::voltage;

    // 初始化电机
    left_motor.init();
    left_motor.initFOC();
    right_motor.init();
    right_motor.initFOC();
}
