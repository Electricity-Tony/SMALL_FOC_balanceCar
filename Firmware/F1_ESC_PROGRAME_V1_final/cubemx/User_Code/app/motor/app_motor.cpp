

#include "app_motor.hpp"
#include "tim.h"
#include "flash/bsp_flash.h"
#include "Key_cpp/bsp_key.hpp"
#include "Led_cpp/bsp_led.hpp"

uint32_t motor_canid = 0x301; // 电机canid

/* 参数存储flash位置 */
#define parameters_address 62

/* 校准的标志位 防止在中断中调用 */
uint8_t can_CAL_ZeroElectricalAngle_flag = 0;

/* ******************************** flash 读取写入关联部分 ********************************* */
/**
 * @brief  获取flash中的pid参数到电机中
 * @details
 * @param motor foc电机类
 * @retval
 */
void app_motor_flash_pidget(foc *motor)
{
	/* 读取 电流环 */
	bsp_flash_read_float(parameters_address, (4 * 0), &motor->_PID_IQ->P, 1);
	bsp_flash_read_float(parameters_address, (4 * 1), &motor->_PID_IQ->I, 1);
	bsp_flash_read_float(parameters_address, (4 * 2), &motor->_PID_IQ->D, 1);
	bsp_flash_read_float(parameters_address, (4 * 3), &motor->_PID_IQ->IMax, 1);
	bsp_flash_read_float(parameters_address, (4 * 4), &motor->_PID_IQ->PIDMax, 1);

	/* IQ与ID赋同样的值 */
	motor->_PID_ID->P = motor->_PID_IQ->P;
	motor->_PID_ID->I = motor->_PID_IQ->I;
	motor->_PID_ID->D = motor->_PID_IQ->D;
	motor->_PID_ID->IMax = motor->_PID_IQ->IMax;
	motor->_PID_ID->PIDMax = motor->_PID_IQ->PIDMax;

	/* 读取 速度环 */
	bsp_flash_read_float(parameters_address, (4 * 5), &motor->_PID_IN->P, 1);
	bsp_flash_read_float(parameters_address, (4 * 6), &motor->_PID_IN->I, 1);
	bsp_flash_read_float(parameters_address, (4 * 7), &motor->_PID_IN->D, 1);
	bsp_flash_read_float(parameters_address, (4 * 8), &motor->_PID_IN->IMax, 1);
	bsp_flash_read_float(parameters_address, (4 * 9), &motor->_PID_IN->PIDMax, 1);
	/* 读取 位置环 */
	bsp_flash_read_float(parameters_address, (4 * 10), &motor->_PID_OUT->P, 1);
	bsp_flash_read_float(parameters_address, (4 * 11), &motor->_PID_OUT->I, 1);
	bsp_flash_read_float(parameters_address, (4 * 12), &motor->_PID_OUT->D, 1);
	bsp_flash_read_float(parameters_address, (4 * 13), &motor->_PID_OUT->IMax, 1);
	bsp_flash_read_float(parameters_address, (4 * 14), &motor->_PID_OUT->PIDMax, 1);

	/* 电机硬件参数 */
	// uint32_t dir_temp;
	bsp_flash_read(parameters_address, (4 * 15), (uint32_t *)&motor->dir, 1); // 电角度方向
	// motor->dir = (DIR_STATE)dir_temp;
	bsp_flash_read(parameters_address, (4 * 16), (uint32_t *)&motor->pole_pairs, 1);	  // 极对数
	bsp_flash_read_float(parameters_address, (4 * 17), &motor->zero_electrical_angle, 1); // 零电角度值

	/* canid */
	bsp_flash_read(parameters_address, 128, &motor_canid, 1);
	/* 更新 led 显示 */
	BORAD_LED.twinkle_Config(300, (motor_canid % 0x300));
}

/**
 * @brief  将当前的pid参数写入flash中
 * @details
 * @param motor foc电机类
 * @retval
 */
void app_motor_flash_pidsave(foc *motor)
{
	/* 擦写当前扇区 */
	bsp_flash_Erase(parameters_address, 1);
	/* 写入 电流环 */
	bsp_flash_Write_float(parameters_address, (4 * 0), &motor->_PID_IQ->P, 1);
	bsp_flash_Write_float(parameters_address, (4 * 1), &motor->_PID_IQ->I, 1);
	bsp_flash_Write_float(parameters_address, (4 * 2), &motor->_PID_IQ->D, 1);
	bsp_flash_Write_float(parameters_address, (4 * 3), &motor->_PID_IQ->IMax, 1);
	bsp_flash_Write_float(parameters_address, (4 * 4), &motor->_PID_IQ->PIDMax, 1);

	/* 写入 速度环 */
	bsp_flash_Write_float(parameters_address, (4 * 5), &motor->_PID_IN->P, 1);
	bsp_flash_Write_float(parameters_address, (4 * 6), &motor->_PID_IN->I, 1);
	bsp_flash_Write_float(parameters_address, (4 * 7), &motor->_PID_IN->D, 1);
	bsp_flash_Write_float(parameters_address, (4 * 8), &motor->_PID_IN->IMax, 1);
	bsp_flash_Write_float(parameters_address, (4 * 9), &motor->_PID_IN->PIDMax, 1);

	/* 写入 位置环 */
	bsp_flash_Write_float(parameters_address, (4 * 10), &motor->_PID_OUT->P, 1);
	bsp_flash_Write_float(parameters_address, (4 * 11), &motor->_PID_OUT->I, 1);
	bsp_flash_Write_float(parameters_address, (4 * 12), &motor->_PID_OUT->D, 1);
	bsp_flash_Write_float(parameters_address, (4 * 13), &motor->_PID_OUT->IMax, 1);
	bsp_flash_Write_float(parameters_address, (4 * 14), &motor->_PID_OUT->PIDMax, 1);

	/* 电机硬件参数 */
	// uint32_t dir_temp = (uint32_t)motor->dir;
	bsp_flash_Write(parameters_address, (4 * 15), (uint32_t *)&motor->dir, 1); // 电角度方向
	// uint32_t pole_temp = (uint32_t)motor->pole_pairs;
	bsp_flash_Write(parameters_address, (4 * 16), (uint32_t *)&motor->pole_pairs, 1);	   // 极对数
	bsp_flash_Write_float(parameters_address, (4 * 17), &motor->zero_electrical_angle, 1); // 零电角度值

	/* 同时把 candid 写入 */
	bsp_flash_Write(parameters_address, 128, &motor_canid, 1);
}

/* ************************* 按键切换 CANid 部分 ***************************************** */
/* 同样需要调用flash部分 */

/**
 * @brief  将当前的canid参数写入flash中
 * @details
 * @param motor foc电机类
 * @retval
 */
void app_motor_key_canidset(void)
{
	Button_State tempState = USER_Button.update();
	if (tempState == Button_HOLD)
	{
		BORAD_LED.twinkle_Config(20, 10);
	}
	if (USER_Button.OUT_State == Button_HOLD)
	{
		/* 此时表显为 长按 can_id 增加 */
		motor_canid++;
		if (motor_canid == 0x30A) // 挂载了10个
		{
			motor_canid = 0x301; // 重新回到0x301
		}
		/* 这里要调用 flash 每次上电能保存 */
		// /* 与pid也公用一个扇区 */
		// app_motor_flash_pidget(&motor_1612);
		/* 这里包含一次擦除，可以进行canid写入，这里都包含了 */
		app_motor_flash_pidsave(&motor_1612);
		// /* 从第128开始写入 */
		// bsp_flash_Write(parameters_address, 128, &motor_canid, 1);

		/* 更新 led 显示 */
		BORAD_LED.twinkle_Config(300, (motor_canid % 0x300));
		USER_Button.clean();
	}
}

/* ************************* CAN 命令执行函数 ***************************************** */
/**
 * @brief  发送标准函数
 * @details 发送一帧can协议帧
 * @param
 * @retval
 */
HAL_StatusTypeDef app_motor_can_Sendmessage(CAN_HandleTypeDef *hcan, int16_t StdId, uint8_t *Can_Send_Data)
{
	uint32_t MailBox;
	CAN_TxHeaderTypeDef bsp_can_Tx;
	HAL_StatusTypeDef HAL_RESULT;

	// 将传入的数据转换为标准CAN帧数据
	uint8_t Data[8];
	Data[0] = (uint8_t)(*(Can_Send_Data + 0));
	Data[1] = (uint8_t)(*(Can_Send_Data + 1));
	Data[2] = (uint8_t)(*(Can_Send_Data + 2));
	Data[3] = (uint8_t)(*(Can_Send_Data + 3));
	Data[4] = (uint8_t)(*(Can_Send_Data + 4));
	Data[5] = (uint8_t)(*(Can_Send_Data + 5));
	Data[6] = (uint8_t)(*(Can_Send_Data + 6));
	Data[7] = (uint8_t)(*(Can_Send_Data + 7));

	// 设置CAN帧配置
	bsp_can_Tx.StdId = StdId;
	bsp_can_Tx.RTR = CAN_RTR_DATA;
	bsp_can_Tx.IDE = CAN_ID_STD;
	bsp_can_Tx.DLC = 8;
	HAL_RESULT = HAL_CAN_AddTxMessage(hcan, &bsp_can_Tx, Data, &MailBox);
#ifndef BSP_CAN_USE_FREERTOS
	while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 3)
		; // 等待发送完成，如果是使用FreeRTOS则可以不需要这句,因为任务调度本身是需要延时的
#endif

	return HAL_RESULT;
}

/**
 * @brief  电机校准
 * @details 收到校准命令后进行相关参数的工作，得到电调度方向，极对数，零点角度值
 * @param
 * @retval
 */
void app_motor_can_CAL(foc *motor)
{
	/* 设置默认电角度方向 FORWARD */
	motor->dir = FORWARD;
	/* 设置默认极对数数量 1 */
	motor->pole_pairs = 7;
	/* 设置零点角度为0 */
	motor->set_ZeroElectricalAngle(0);

	/* 相关参数 */
	float sum_data;				  // 校准时间内的转动角度
	float start_mechanical_angle; // 开始时的机械角度
	float end_mechanical_angle;	  // 结束时的机械角度
	float true_speed = 0;		  // 校准时的实际速度
	uint32_t SysTick_start = 0;	  // 中间用的定时器时刻
	int32_t time = 0;			  // 中间用的定时器时间

	float Detection_time = 5000; // 检测时间 ms

	/* 关闭定时器所有功能 */
	HAL_TIM_Base_Stop_IT(&htim1);
	/* 开环目标 60rpm 运行 */
	motor->set_speed(60, openloop);
	/* 上电启动 */
	HAL_GPIO_WritePin(M_EN_GPIO_Port, M_EN_Pin, GPIO_PIN_SET);
	/* 获取启动时的机械角度 */
	start_mechanical_angle = motor->_encoder->get_date();
	/* 运行一个校准时间 */
	SysTick_start = MICROS_ms();
	while (1)
	{
		time = MICROS_ms() - SysTick_start;
		if (time < 0)
		{
			SysTick_start = MICROS_ms();
		}
		else
		{
			if (time > Detection_time)
			{
				break;
			}
		}
		// motor->run();
		// HAL_Delay(100);
	}
	/* 获取结束时机械角度 */
	end_mechanical_angle = motor->_encoder->get_date();
	/* 关闭电机运行 */
	motor->set_speed(0, openloop);
	HAL_GPIO_WritePin(M_EN_GPIO_Port, M_EN_Pin, GPIO_PIN_RESET);
	/* 获取校准过程中的运行角度 */
	sum_data = end_mechanical_angle - start_mechanical_angle;
	/* 转化为速度 */
	true_speed = ABS(sum_data) / (Detection_time / 1000);
	/* 计算速度比例获取极对数 */
	motor->pole_pairs = (uint8_t)((60 * 7 / Rad2Rot(true_speed)) + 0.5);
	/* 验证旋转方向 */
	if (sum_data < 0)
	{
		motor->dir = REVERSE;
	}

	/* 校准初始电角度 */
	HAL_GPIO_WritePin(M_EN_GPIO_Port, M_EN_Pin, GPIO_PIN_SET);
	motor_1612.init_ZeroElectricalAngle(500);
	HAL_GPIO_WritePin(M_EN_GPIO_Port, M_EN_Pin, GPIO_PIN_RESET);

	HAL_TIM_Base_Start_IT(&htim1);

	/* 保存至 flash  */
	app_motor_flash_pidsave(&motor_1612);

	/* 关闭电流 */
	motor_1612.set_current(0);
}

/**
 * @brief  电机校准 仅零点角度值
 * @details 收到校准命令后进行零点角度值
 * @param
 * @retval
 */
void app_motor_can_CAL_ZeroElectricalAngle(foc *motor)
{
	HAL_TIM_Base_Stop_IT(&htim1);
	HAL_TIM_Base_Stop_IT(&htim2);

	motor_1612.init();

	/* 校准初始电角度 */
	HAL_GPIO_WritePin(M_EN_GPIO_Port, M_EN_Pin, GPIO_PIN_SET);
	motor_1612.init_ZeroElectricalAngle(3000);
	HAL_GPIO_WritePin(M_EN_GPIO_Port, M_EN_Pin, GPIO_PIN_RESET);

	/* 保存至 flash  */
	// app_motor_flash_pidsave(&motor_1612);

	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim2);

	/* 关闭电流 */
	motor_1612.set_current(0);
}

/**
 * @brief  采样数据回发
 * @details 回发当前电机状态
 * @param
 * @retval
 */
/* | 编码器帧首 | 功能字 | 转子角度 | 转子转速 | 电流     | 补零 |  和检验 |
| ---------- | ------ | -------- | -------- | -------- |  ------- | ------ |
| 0x30x      | 0xF1   | int16_t | int16_t | int16_t | 0     | 0x0E   |
| 标准2字节  | 1字节  | 2字节    | 2字节    | 2字节    | 0     | 1字节  |

| 数据内容 | 单位                                |
| -------- | ----------------------------------- |
| 转子角度 | 0.1 ° 度 （-3276.8 ~ 3276.8）       |
| 转子转速 | 0.1 rpm 转每分 （-3276.8 ~ 3276.8） |
| 电流大小 | 0.001 A 安培 （-32.768 ~ -32.768）          | */
void app_motor_can_Basic_postback(foc *motor)
{
	uint8_t Basic_postback_data[8];
	/* 功能字 */
	Basic_postback_data[0] = SAMPLE_DATA;
	/* 数据 */
	float send_angle = Rad2Angle(motor->shaft_angle);
	// float send_angle = motor->electrical_angle;
	/* 转子角度 */
	Basic_postback_data[1] = (uint8_t)(((int16_t)(send_angle * 10)) >> 8);
	Basic_postback_data[2] = (uint8_t)((int16_t)(send_angle * 10)) & 0xFF;
	/* 转子转速 */
	Basic_postback_data[3] = (uint8_t)(((int16_t)(motor->speed * 10)) >> 8);
	Basic_postback_data[4] = (uint8_t)((int16_t)(motor->speed * 10)) & 0xFF;
	// Basic_postback_data[3] = (uint8_t)(((int16_t)(motor->U_q * 10)) >> 8);
	// Basic_postback_data[4] = (uint8_t)((int16_t)(motor->U_q * 10)) & 0xFF;
	/* 电流大小 */
	// Basic_postback_data[5] = (uint8_t)(((int16_t)(motor->I_d * 1000)) >> 8);
	// Basic_postback_data[6] = (uint8_t)((int16_t)(motor->I_d * 1000)) & 0xFF;
	Basic_postback_data[5] = (uint8_t)(((int16_t)(motor->I_q * 1000)) >> 8);
	Basic_postback_data[6] = (uint8_t)((int16_t)(motor->I_q * 1000)) & 0xFF;
	/* 校验位 */
	Basic_postback_data[7] = 0xFF - Basic_postback_data[0];

	/* 发送 */
	app_motor_can_Sendmessage(&hcan, motor_canid, Basic_postback_data);
}

/**
 * @brief  pid 数据回发
 * @details 回发当前 pid 的设定值
 * @param
 * @retval
 */
/* * 接收到 pid 查询命令 0x13 才发送以下内容
* 功能字：0xE1
* 发送 当前 pid 参数

| 编码器帧首 | 功能字 | 三环声明 | pid声明 | 数据  | 补零 |  和检验 |
| ---------- | ------ | -------- | -------- | -------- |  ------- | ------ |
| 0x30x      | 0xE1  | uint8_t  | uint8_t | float | 0     | 0xE   |
| 标准2字节  | 1字节  | 1字节   | 1字节   | 4字节   | 0     | 1字节  |

| 三环   | 声明字节 |
| ------ | -------- |
| 电流环 | 0x01     |
| 速度环 | 0x02     |
| 位置环 | 0x03     |

| pid三参数 | 声明字节 |
| --------- | -------- |
| P         | 0x01     |
| I         | 0x02     |
| D         | 0x03     |*/
void app_motor_can_PostBackpid(foc *motor)
{
	uint8_t Basic_postback_data[8];
	/* 功能字 */
	Basic_postback_data[0] = PID_DATA;
	/* 校验位 */
	Basic_postback_data[7] = 0xFF - Basic_postback_data[0];
	/* 数据 */
	uint8_t i;
	inline_Struct RX_pid_data[15];

	/* pid 数值转移 */
	RX_pid_data[0].f = motor->_PID_IQ->P;
	RX_pid_data[1].f = motor->_PID_IQ->I;
	RX_pid_data[2].f = motor->_PID_IQ->D;
	RX_pid_data[3].f = motor->_PID_IQ->IMax;
	RX_pid_data[4].f = motor->_PID_IQ->PIDMax;
	RX_pid_data[5].f = motor->_PID_IN->P;
	RX_pid_data[6].f = motor->_PID_IN->I;
	RX_pid_data[7].f = motor->_PID_IN->D;
	RX_pid_data[8].f = motor->_PID_IN->IMax;
	RX_pid_data[9].f = motor->_PID_IN->PIDMax;
	RX_pid_data[10].f = motor->_PID_OUT->P;
	RX_pid_data[11].f = motor->_PID_OUT->I;
	RX_pid_data[12].f = motor->_PID_OUT->D;
	RX_pid_data[13].f = motor->_PID_OUT->IMax;
	RX_pid_data[14].f = motor->_PID_OUT->PIDMax;

	for (i = 0; i < 15; i++)
	{
		uint8_t PID_RING = i / 5;	   // 选择 pid 的三个环
		uint8_t PID_PARAMETER = i % 5; // 选择 pid 的三个参数 p,i,d

		/* 三环确认 */
		if (PID_RING == 0)
		{
			/* 电流环 */
			Basic_postback_data[1] = CURRENT;
		}
		else if (PID_RING == 1)
		{
			/* 电流环 */
			Basic_postback_data[1] = SPEED;
		}
		else if (PID_RING == 2)
		{
			/* 电流环 */
			Basic_postback_data[1] = POSITION;
		}

		/* pid 参数确认 */
		if (PID_PARAMETER == 0)
		{
			/* 参数 p */
			Basic_postback_data[2] = P;
		}
		else if (PID_PARAMETER == 1)
		{
			/* 参数 I */
			Basic_postback_data[2] = I;
		}
		else if (PID_PARAMETER == 2)
		{
			/* 参数 D */
			Basic_postback_data[2] = D;
		}
		else if (PID_PARAMETER == 3)
		{
			/* 参数 D */
			Basic_postback_data[2] = IMAX;
		}
		else if (PID_PARAMETER == 4)
		{
			/* 参数 D */
			Basic_postback_data[2] = PIDMAX;
		}
		/* 输入数据 */
		Basic_postback_data[3] = RX_pid_data[i].c[0];
		Basic_postback_data[4] = RX_pid_data[i].c[1];
		Basic_postback_data[5] = RX_pid_data[i].c[2];
		Basic_postback_data[6] = RX_pid_data[i].c[3];

		/* 发送 */
		app_motor_can_Sendmessage(&hcan, motor_canid, Basic_postback_data);

		// HAL_Delay(1);
	}
}

/**
 * @brief  pid 数据设置
 * @details 介绍到需要设置的pid值并设置
 * @param
 * @retval
 */
/* * 功能字 0x12
* 可以设置三环的pid参数，设置会自动写入 flash 中
* 三环参数均为 float 类型 通过数据的第一个功能字判断

| 帧首      | 功能字 | 三环声明 | pid声明 | 数据  | 补零 | 和检验 |
| --------- | ------ | -------- | ------- | ----- | ---- | ------ |
| 0x30x     | 0x12   | uint8_t  | uint8_t | float | 0    | 0xEE   |
| 标准2字节 | 1字节  | 1字节    | 1字节   | 4字节 | 0    | 1字节  |

| 三环   | 声明字节 |
| ------ | -------- |
| 电流环 | 0x01     |
| 速度环 | 0x02     |
| 位置环 | 0x03     |

| pid三参数 | 声明字节 |
| --------- | -------- |
| P         | 0x01     |
| I         | 0x02     |
| D         | 0x03     |
| IMAX      | 0x04     |
| PIDMAX      | 0x05     |*/
void app_motor_can_SETpid(foc *motor, uint8_t *CAN_RX_data)
{
	/* pid 位置 */
	uint8_t PID_RING = *(CAN_RX_data + 1);		// 选择 pid 的三个环
	uint8_t PID_PARAMETER = *(CAN_RX_data + 2); // 选择 pid 的三个参数 p,i,d
	/* 数据 */
	inline_Struct RX_pid_data;
	/* 数据转移 */
	RX_pid_data.c[0] = *(CAN_RX_data + 3);
	RX_pid_data.c[1] = *(CAN_RX_data + 4);
	RX_pid_data.c[2] = *(CAN_RX_data + 5);
	RX_pid_data.c[3] = *(CAN_RX_data + 6);

	/* 设置参数 */
	if (PID_RING == CURRENT)
	{
		/* 电流环 */
		if (PID_PARAMETER == P)
		{
			/* 参数 P */
			motor->_PID_IQ->P = RX_pid_data.f;
		}
		else if (PID_PARAMETER == I)
		{
			/* 参数 I */
			motor->_PID_IQ->I = RX_pid_data.f;
		}
		else if (PID_PARAMETER == D)
		{
			/* 参数 D */
			motor->_PID_IQ->D = RX_pid_data.f;
		}
		else if (PID_PARAMETER == IMAX)
		{
			/* 参数 D */
			motor->_PID_IQ->IMax = RX_pid_data.f;
		}
		else if (PID_PARAMETER == PIDMAX)
		{
			/* 参数 D */
			motor->_PID_IQ->PIDMax = RX_pid_data.f;
		}

		/* IQ与ID赋同样的值 */
		motor->_PID_ID->P = motor->_PID_IQ->P;
		motor->_PID_ID->I = motor->_PID_IQ->I;
		motor->_PID_ID->D = motor->_PID_IQ->D;
		motor->_PID_ID->IMax = motor->_PID_IQ->IMax;
		motor->_PID_ID->PIDMax = motor->_PID_IQ->PIDMax;
	}
	else if (PID_RING == SPEED)
	{
		/* 速度环 */
		if (PID_PARAMETER == P)
		{
			/* 参数 P */
			motor->_PID_IN->P = RX_pid_data.f;
		}
		else if (PID_PARAMETER == I)
		{
			/* 参数 I */
			motor->_PID_IN->I = RX_pid_data.f;
		}
		else if (PID_PARAMETER == D)
		{
			/* 参数 D */
			motor->_PID_IN->D = RX_pid_data.f;
		}
		else if (PID_PARAMETER == IMAX)
		{
			/* 参数 D */
			motor->_PID_IN->IMax = RX_pid_data.f;
		}
		else if (PID_PARAMETER == PIDMAX)
		{
			/* 参数 D */
			motor->_PID_IN->PIDMax = RX_pid_data.f;
		}
	}
	else if (PID_RING == POSITION)
	{
		/* 位置环 */
		if (PID_PARAMETER == P)
		{
			/* 参数 P */
			motor->_PID_OUT->P = RX_pid_data.f;
		}
		else if (PID_PARAMETER == I)
		{
			/* 参数 I */
			motor->_PID_OUT->I = RX_pid_data.f;
		}
		else if (PID_PARAMETER == D)
		{
			/* 参数 D */
			motor->_PID_OUT->D = RX_pid_data.f;
		}
		else if (PID_PARAMETER == IMAX)
		{
			/* 参数 D */
			motor->_PID_OUT->IMax = RX_pid_data.f;
		}
		else if (PID_PARAMETER == PIDMAX)
		{
			/* 参数 D */
			motor->_PID_OUT->PIDMax = RX_pid_data.f;
		}
	}
}

/**
 * @brief  pid 数据保存
 * @details 介绍到需要设置的pid值并设置
 * @param
 * @retval
 */
/* * 功能字 0x14
* 命令电调保存当前pid参数至flash

| 帧首      | 功能字 | 补零 | 和检验 |
| --------- | ------ | ---- | ------ |
| 0x30x     | 0x14   | 0    | 0xED   |
| 标准2字节 | 1字节  | 0    | 1字节  |*/
void app_motor_can_SAVEpid(foc *motor)
{
	app_motor_flash_pidsave(motor);
}

/**
 * @brief  电机下电进入安全模式
 * @details 断开电机供电
 * @param
 * @retval
 */
/* * 功能字 0x15
* 电机使能，断开电源

| 帧首      | 功能字 | 补零  | 和检验 |
| --------- | ------ | ----- | ------ |
| 0x30x     | 0x15   | 0xF0  | 0xED   |
| 标准2字节 | 1字节  | 6字节 | 1字节  |*/
void app_motor_can_SAFEmode(foc *motor)
{
	motor->disable();
}

/**
 * @brief  电机上电进入正常运行模式
 * @details 启动电机供电
 * @param
 * @retval
 */
/* * 功能字 0x16
* 使电机进入正常模式，上电

| 帧首      | 功能字 | 补零  | 和检验 |
| --------- | ------ | ----- | ------ |
| 0x30x     | 0x16   | 0xF0  | 0xED   |
| 标准2字节 | 1字节  | 6字节 | 1字节  |*/
void app_motor_can_RUNmode(foc *motor)
{
	motor->enable();
}

/**
 * @brief  电机设置电角度方向和极对数
 * @details
 * @param
 * @retval
 */
/* * 功能字 0x17
* 设置编码器方向，极对数参数

| 帧首      | 功能字 | 参数选择功能字 | 设置值 | 补零  | 和检验 |
| --------- | ------ | -------------- | ------ | ----- | ------ |
| 0x30x     | 0x17   | uint8_t        | int8_t | 0     | 0xE9   |
| 标准2字节 | 1字节  | 1字节          | 1字节  | 4字节 | 1字节  |

| 控制模式   | 字节声明 |
| ---------- | -------- |
| 编码器方向 | 0x01     |
| 极对数     | 0x02     |*/
void app_motor_can_SETMotor(foc *motor, uint8_t *CAN_RX_data)
{
	if (*(CAN_RX_data + 1) == 0x01)
	{
		/* 设置电角度方向 */
		DIR_STATE dir_temp;
		if ((int16_t) * (CAN_RX_data + 2) >= 1)
		{
			dir_temp = FORWARD;
		}
		else
		{
			dir_temp = REVERSE;
		}
		motor->dir = dir_temp;
	}
	else if (*(CAN_RX_data + 1) == 0x02)
	{
		/* 设置极对数 */
		motor->pole_pairs = *(CAN_RX_data + 2);
	}
}

/**
 * @brief  标准控制解析函数
 * @details 解析需要运行的模式并运行
 * @param
 * @retval
 */
/* * 功能字：0x21
* 可控制电机进行 电流闭环，速度闭环 或 位置闭环

| 帧首      | 功能字 | 控制方式功能字 | 目标参数 | 补零  | 和检验 |
| --------- | ------ | -------------- | -------- | ----- | ------ |
| 0x30x     | 0x21   | uint8_t        | float    | 0     | 0xDF   |
| 标准2字节 | 1字节  | 1字节          | 4字节    | 1字节 | 1字节  |

| 控制模式 | 字节声明 |
| -------- | -------- |
| 电流闭环 | 0x01     |
| 速度闭环 | 0x02     |
| 位置闭环 | 0x03     |*/
void app_motor_can_Ctrl(foc *motor, uint8_t *CAN_RX_data)
{
	/* pid 位置 */
	uint8_t Ctrl_Mode = *(CAN_RX_data + 1); // 识别控制模式
	/* 数据 */
	inline_Struct RX_ctrl_data;
	/* 数据转移 */
	RX_ctrl_data.c[0] = *(CAN_RX_data + 2);
	RX_ctrl_data.c[1] = *(CAN_RX_data + 3);
	RX_ctrl_data.c[2] = *(CAN_RX_data + 4);
	RX_ctrl_data.c[3] = *(CAN_RX_data + 5);

	/* 设置参数 */
	if (Ctrl_Mode == CURRENT)
	{
		/* 设置目标电流电流 */
		motor->set_current(RX_ctrl_data.f);
	}
	else if (Ctrl_Mode == SPEED)
	{
		/* 设置目标速度 */
		motor->set_speed(RX_ctrl_data.f);
	}
	else if (Ctrl_Mode == POSITION)
	{
		/* 设置目标位置 */
		motor->set_angle(RX_ctrl_data.f);
	}
}

/**
 * @brief  标准控制解析函数
 * @details 解析需要运行的模式并运行
 * @param
 * @retval
 */
/* * 接收到 pid 查询命令 0x13 才发送以下内容
* 功能字：0xE2
* 发送 当前 电机参数 电角度方向、极对数、零电角度

| 编码器帧首 | 功能字 | 电角度方向 | 极对数 | 零电角度 | 补零 |  和检验 |
| ---------- | ------ | -------- | -------- | -------- |  ------- | ------ |
| 0x30x      | 0xE1  | int8_t  | uint8_t | float | 0     | 0xE   |
| 标准2字节  | 1字节  | 1字节   | 1字节   | 4字节   | 0     | 1字节  |*/
void app_motor_can_ShowMotor(foc *motor)
{
	uint8_t Basic_postback_data[8];
	/* 功能字 */
	Basic_postback_data[0] = MOTOR_DATA;
	/* 校验位 */
	Basic_postback_data[7] = 0xFF - Basic_postback_data[0];
	/* 数据 */
	inline_Struct data_temp;
	data_temp.f = motor->zero_electrical_angle;

	Basic_postback_data[1] = motor->dir;
	Basic_postback_data[2] = motor->pole_pairs;
	Basic_postback_data[3] = data_temp.c[0];
	Basic_postback_data[4] = data_temp.c[1];
	Basic_postback_data[5] = data_temp.c[2];
	Basic_postback_data[6] = data_temp.c[3];

	/* 发送 */
	app_motor_can_Sendmessage(&hcan, motor_canid, Basic_postback_data);
}

/* ************************* 命令跳转函数 **************************************************** */

/**
 * @brief  解析接受的一帧数据，判断是否跳转校准函数
 * @details
 * @param
 * @retval 0:有触发运行 1：无触发
 */
uint8_t app_motor_can_toCAL(uint8_t *CAN_RX_data)
{
	uint8_t RX_function = *(CAN_RX_data + 0);
	uint8_t i, CAL_flag = 0;
	if (RX_function == CAL)
	{
		for (i = 0; i < 6; i++)
		{
			if (*(CAN_RX_data + 1 + i) == 0xF0)
			{
				CAL_flag++;
			}
		}
		if (CAL_flag == 6)
		{
			/* 这里有大坑 delay 在can中断中使用了，卡死 */
			// app_motor_can_CAL(&motor_1612);
		}
		else if (*(CAN_RX_data + 1) == 0x01)
		{
			/* 这里有大坑 delay 在can中断中使用了，卡死 */
			can_CAL_ZeroElectricalAngle_flag = 1;
			// app_motor_can_CAL_ZeroElectricalAngle(&motor_1612);
		}
		return 0;
	}
	else
	{
		return 1;
	}
}

/**
 * @brief  解析接受的一帧数据，判断执行PID查询
 * @details
 * @param
 * @retval 0:有触发运行 1：无触发
 */
uint8_t app_motor_can_toPostBackpid(uint8_t *CAN_RX_data)
{
	uint8_t RX_function = *(CAN_RX_data + 0);
	if (RX_function == SHOW_PID)
	{
		app_motor_can_PostBackpid(&motor_1612);
		app_motor_can_ShowMotor(&motor_1612);
		return 0;
	}
	else
	{
		return 1;
	}
}

/**
 * @brief  解析接受的一帧数据，判断执行PID参数设定
 * @details
 * @param
 * @retval 0:有触发运行 1：无触发
 */
uint8_t app_motor_can_toSETpid(uint8_t *CAN_RX_data)
{
	uint8_t RX_function = *(CAN_RX_data + 0);
	if (RX_function == SET_PID)
	{
		app_motor_can_SETpid(&motor_1612, CAN_RX_data);
		return 0;
	}
	else
	{
		return 1;
	}
}

/**
 * @brief  解析接受的一帧数据，判断执行PID参数保存
 * @details
 * @param
 * @retval 0:有触发运行 1：无触发
 */
uint8_t app_motor_can_toSAVEpid(uint8_t *CAN_RX_data)
{
	uint8_t RX_function = *(CAN_RX_data + 0);
	if (RX_function == SAVE_PID)
	{
		app_motor_can_SAVEpid(&motor_1612);
		return 0;
	}
	else
	{
		return 1;
	}
}

/**
 * @brief  解析接受的一帧数据，判断执行电机下电命令
 * @details
 * @param
 * @retval 0:有触发运行 1：无触发
 */
uint8_t app_motor_can_toSAFEmode(uint8_t *CAN_RX_data)
{
	uint8_t RX_function = *(CAN_RX_data + 0);
	if (RX_function == SAFE_MODE)
	{
		app_motor_can_SAFEmode(&motor_1612);
		return 0;
	}
	else
	{
		return 1;
	}
}

/**
 * @brief  解析接受的一帧数据，判断执行电机上电命令
 * @details
 * @param
 * @retval 0:有触发运行 1：无触发
 */
uint8_t app_motor_can_toRUNmode(uint8_t *CAN_RX_data)
{
	uint8_t RX_function = *(CAN_RX_data + 0);
	if (RX_function == RUN_MODE)
	{
		app_motor_can_RUNmode(&motor_1612);
		return 0;
	}
	else
	{
		return 1;
	}
}

/**
 * @brief  解析接受的一帧数据，判断执行电机编码器方向和极对数设置
 * @details
 * @param
 * @retval 0:有触发运行 1：无触发
 */
uint8_t app_motor_can_toSETMotor(uint8_t *CAN_RX_data)
{
	uint8_t RX_function = *(CAN_RX_data + 0);
	if (RX_function == SET_MOTOR)
	{
		app_motor_can_SETMotor(&motor_1612, CAN_RX_data);
		return 0;
	}
	else
	{
		return 1;
	}
}

/**
 * @brief  解析接受的一帧数据，判断执行标准控制命令
 * @details
 * @param
 * @retval 0:有触发运行 1：无触发
 */
uint8_t app_motor_can_toCtrl(uint8_t *CAN_RX_data)
{
	uint8_t RX_function = *(CAN_RX_data + 0);
	if (RX_function == CONTROL)
	{
		app_motor_can_Ctrl(&motor_1612, CAN_RX_data);
		return 0;
	}
	else
	{
		return 1;
	}
}

/* **************************** 命令解析函数 ******************************************************** */

/**
 * @brief  命令数据解析
 * @details 解析接受的命令函数
 * @param
 * @retval
 */
void app_motor_can_parse(uint8_t *CAN_RX_data)
{
	/* 提取功能字 */
	uint8_t RX_function = *(CAN_RX_data + 0);
	/* 提取尾帧 */
	uint8_t RX_Tail = *(CAN_RX_data + 7);

	if ((RX_function + RX_Tail) == 0xFF)
	{
		/* 校验成功，进行处理 */
		app_motor_can_toCAL(CAN_RX_data);
		app_motor_can_toPostBackpid(CAN_RX_data);
		app_motor_can_toSETpid(CAN_RX_data);
		app_motor_can_toSAVEpid(CAN_RX_data);
		app_motor_can_toCtrl(CAN_RX_data);
		app_motor_can_toSAFEmode(CAN_RX_data);
		app_motor_can_toRUNmode(CAN_RX_data);
		app_motor_can_toSETMotor(CAN_RX_data);
	}
	else
	{
		/* 校验不成功，可能需要返回报错 */
	}
}

/* **************************** can 回调函数 ************************************ */

/**
 * @brief  CAN接收中断
 * @details  重新定义接收中断，会自动在CAN中断中调用，不需要手动添加,使用的时候自行在此函数中替换解析函数
 * @param  NULL
 * @retval  NULL
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	static CAN_RxHeaderTypeDef bsp_can_Rx;
	uint8_t CAN_RxData[8];

	if (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) != 0) // 判断中断产生
	{
		HAL_CAN_GetRxMessage(hcan, 0, &bsp_can_Rx, CAN_RxData); // 获取CAN报文

// 使用电机库须返回相对应的电机值
#ifdef USE_bsp_motor
		motor::CANUpdate(hcan, &bsp_can_Rx, (uint8_t *)CAN_RxData);
#endif // USE_bsp_motor

// CAN1收到数据的解析函数
#ifdef BSP_CAN_USE_CAN1
		if (hcan == &hcan1)
		{
		}
#endif

// CAN2收到数据的解析函数
#ifdef BSP_CAN_USE_CAN2
		if (hcan == &hcan2)
		{
		}
#endif

// 使用F1时CAN收到数据的解析函数
#ifdef BSP_CAN_USE_CAN
		if (bsp_can_Rx.StdId == motor_canid)
		{
			app_motor_can_parse(CAN_RxData);
		}

#endif
	}
}
