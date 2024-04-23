

#include "app_txmotor.hpp"
#include "tim.h"
#include "usart_cpp/bsp_usart.hpp"

can_motor motor1(&hcan, 0x301);
can_motor motor2(&hcan, 0x302);
/**
 * @brief  发送标准函数
 * @details 发送一帧can协议帧
 * @param
 * @retval
 */
HAL_StatusTypeDef app_txmotor_can_Sendmessage(CAN_HandleTypeDef *hcan, int16_t StdId, uint8_t *Can_Send_Data)
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

////******************************************* can_motor 电机类*************************************************************************////
/**
 * @brief  can_motor 电机类
 * @param [in]   can_id_			can 的 id 号
 *
 */
WEAK can_motor::can_motor(CAN_HandleTypeDef *motor_hcan, int16_t can_id)
{
	this->motor_hcan = motor_hcan;
	this->can_id = can_id;
}

/* ************************* 命令执行函数 ***************************************** */
/**
 * @brief  电机校准
 * @details 发送电机校准命名
 * @param
 * @retval
 */
/* * 功能字：0x11

| 编码器帧首 | 功能字 | 数据     | 和检验 |
| ---------- | ------ |  -------  | ------ |
| 0x30x      | 0x10   | 0xF0 | 0xEF   |
| 标准2字节  | 1字节  |  6字节  |1字节|

* 发送该命令后，将自动校准电角度方向，初始零电角度值，电流采样偏差值，自动写明pid方向
* 注意此时电机空载
* 相关参数自动写入 flash 中 */
void can_motor::command_CAL(void)
{
	uint8_t Data[8];
	/* g构造协议帧 */
	Data[0] = CAL;
	Data[1] = 0x01;
	Data[2] = 0x00;
	Data[3] = 0x00;
	Data[4] = 0x00;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0xFF - Data[0];

	/* 发送 */
	app_txmotor_can_Sendmessage(this->motor_hcan, this->can_id, Data);
}

/* ************************* 命令执行函数 ***************************************** */
/**
 * @brief  电机校准
 * @details 发送电机校准命名
 * @param
 * @retval
 */
/* * 功能字：0x11

| 编码器帧首 | 功能字 | 数据     | 和检验 |
| ---------- | ------ |  -------  | ------ |
| 0x30x      | 0x10   | 0x01 | 0xEF   |
| 标准2字节  | 1字节  |  1字节  |1字节|

* 发送该命令后，将自动校准电角度方向，初始零电角度值，电流采样偏差值，自动写明pid方向
* 注意此时电机空载
* 相关参数自动写入 flash 中 */
void can_motor::command_CAL_zeroElectricalAngle(void)
{
	uint8_t Data[8];
	/* g构造协议帧 */
	Data[0] = CAL;
	Data[1] = 0x10;
	Data[2] = 0x00;
	Data[3] = 0x00;
	Data[4] = 0x00;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0xFF - Data[0];

	/* 发送 */
	app_txmotor_can_Sendmessage(this->motor_hcan, this->can_id, Data);
}

/**
 * @brief  电机pid参数设置
 * @details 设置pid参数
 * @param PID_RING pid环
 * @param PID_PARAMETER p i d 选择三个参数
 * @param value 设置的值
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
| PIDMAX    | 0x05     | */
void can_motor::command_SETpid(PID_RING PID_RING, PID_PARAMETER PID_PARAMETER, float value)
{
	uint8_t Data[8];
	/* 构造协议帧 */
	Data[0] = SET_PID;
	Data[7] = 0xFF - Data[0];

	/* 三环声明 */
	Data[1] = PID_RING;
	/* 参数声明 */
	Data[2] = PID_PARAMETER;
	/* 赋值 */
	inline_Struct value_set;
	value_set.f = value;
	Data[3] = value_set.c[0];
	Data[4] = value_set.c[1];
	Data[5] = value_set.c[2];
	Data[6] = value_set.c[3];
	/* 发送 */
	app_txmotor_can_Sendmessage(this->motor_hcan, this->can_id, Data);
}

/**
 * @brief  电机pid查询
 * @details
 * @param
 * @retval
 */
/* * 功能字 0x14
* 命令电调发送当前所有的 pid 参数

| 帧首      | 功能字 |  补零 | 和检验 |
| --------- | ------ |  ---- | ------ |
| 0x30x     | 0x13   |  0    | 0xED   |
| 标准2字节 | 1字节  |  0    | 1字节  | */
void can_motor::command_Savepid(void)
{
	uint8_t Data[8];
	/* 构造协议帧 */
	Data[0] = SAVE_PID;
	Data[7] = 0xFF - Data[0];

	/* 发送 */
	app_txmotor_can_Sendmessage(this->motor_hcan, this->can_id, Data);
}

/**
 * @brief  电机pid保存
 * @details
 * @param
 * @retval
 */
/* * 功能字 0x13
* 命令电调发送当前所有的 pid 参数
| 帧首      | 功能字 |  补零 | 和检验 |
| --------- | ------ |  ---- | ------ |
| 0x30x     | 0x13   |  0    | 0xED   |
| 标准2字节 | 1字节  |  0    | 1字节  | */
void can_motor::command_PostBackpid(void)
{
	uint8_t Data[8];
	/* 构造协议帧 */
	Data[0] = SHOW_PID;
	Data[7] = 0xFF - Data[0];

	/* 发送 */
	app_txmotor_can_Sendmessage(this->motor_hcan, this->can_id, Data);
}

/**
 * @brief  电机安全模式
 * @details 下电
 * @param
 * @retval
 */
/* * 功能字 0x15
* 电机使能，断开电源

| 帧首      | 功能字 | 补零  | 和检验 |
| --------- | ------ | ----- | ------ |
| 0x30x     | 0x15   | 0xF0  | 0xED   |
| 标准2字节 | 1字节  | 6字节 | 1字节  | */
void can_motor::command_SafeMode(void)
{
	uint8_t Data[8];
	/* 构造协议帧 */
	Data[0] = SAFE_MODE;
	Data[7] = 0xFF - Data[0];
	uint8_t i;
	for (i = 1; i < 7; i++)
	{
		Data[i] = 0xF0;
	}

	/* 发送 */
	app_txmotor_can_Sendmessage(this->motor_hcan, this->can_id, Data);
}

/**
 * @brief  电机正常模式
 * @details 上电
 * @param
 * @retval
 */
/* * 功能字 0x16
* 使电机进入正常模式，上电

| 帧首      | 功能字 | 补零  | 和检验 |
| --------- | ------ | ----- | ------ |
| 0x30x     | 0x16   | 0xF0  | 0xED   |
| 标准2字节 | 1字节  | 6字节 | 1字节  | */
void can_motor::command_RunMode(void)
{
	uint8_t Data[8];
	/* 构造协议帧 */
	Data[0] = RUN_MODE;
	Data[7] = 0xFF - Data[0];
	uint8_t i;
	for (i = 1; i < 7; i++)
	{
		Data[i] = 0xF0;
	}

	/* 发送 */
	app_txmotor_can_Sendmessage(this->motor_hcan, this->can_id, Data);
}

/**
 * @brief  电机设置电角度方向和极对数
 * @details
 * @param parameter_id 选择参数
 * @param data 设置数据
 * @retval
 */
/* * 功能字 0x17
* 设置编码器方向，极对数参数

| 帧首      | 功能字 | 参数选择功能字 | 设置值 | 补零  | 和检验 |
| --------- | ------ | -------------- | ------ | ----- | ------ |
| 0x30x     | 0x17   | uint8_t        | int8_t | 0     | 0xE9   |
| 标准2字节 | 1字节  | 1字节          | 1字节  | 4字节 | 1字节  |

| 参数选择功能字   | 字节声明 |
| ---------- | -------- |
| 编码器方向 | 0x01     |
| 极对数     | 0x02     |
|            |          | */
void can_motor::command_SetMotor(uint8_t parameter_id, int8_t data)
{
	uint8_t Data[8];
	/* 构造协议帧 */
	Data[0] = SET_MOTOR;
	Data[7] = 0xFF - Data[0];

	Data[1] = parameter_id;
	Data[2] = data;

	/* 发送 */
	app_txmotor_can_Sendmessage(this->motor_hcan, this->can_id, Data);
}

/* ****************** 标准控制指令 ********************************** */

/**
 * @brief  控制命令
 * @details 命令电机运行
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
| 位置闭环 | 0x03     | */
void can_motor::ctrl(run_mode mode, float value)
{
	uint8_t Data[8];
	/* 构造协议帧 */
	Data[0] = CONTROL;
	Data[7] = 0xFF - Data[0];
	/* 模式设置 */
	Data[1] = mode;
	/* 赋值 */
	inline_Struct value_set;
	value_set.f = value;
	Data[2] = value_set.c[0];
	Data[3] = value_set.c[1];
	Data[4] = value_set.c[2];
	Data[5] = value_set.c[3];
	/* 发送 */
	app_txmotor_can_Sendmessage(this->motor_hcan, this->can_id, Data);
}

/* ************************ 接受数据 ******************************************** */
/**
 * @brief  读取 回发 的 pid 数据
 * @details 获取pid 数据
 * @param
 * @retval
 */
/* * 发送到 pid 查询命令 0x13 才接受以下内容
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
| D         | 0x03     |
| IMAX      | 0x04     |
| PIDMAX    | 0x05     |*/
uint8_t can_motor::extract_pid(uint8_t *CAN_RX_data)
{
	uint8_t RX_function = *(CAN_RX_data + 0);
	if (RX_function == PID_DATA)
	{
		/* 这里放运行函数 */
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
				this->PID_CURRENT.P = RX_pid_data.f;
			}
			else if (PID_PARAMETER == I)
			{
				/* 参数 I */
				this->PID_CURRENT.I = RX_pid_data.f;
			}
			else if (PID_PARAMETER == D)
			{
				/* 参数 D */
				this->PID_CURRENT.D = RX_pid_data.f;
			}
			else if (PID_PARAMETER == IMAX)
			{
				/* 参数 D */
				this->PID_CURRENT.IMax = RX_pid_data.f;
			}
			else if (PID_PARAMETER == PIDMAX)
			{
				/* 参数 D */
				this->PID_CURRENT.PIDMax = RX_pid_data.f;
			}
		}
		else if (PID_RING == SPEED)
		{
			/* 速度环 */
			if (PID_PARAMETER == P)
			{
				/* 参数 P */
				this->PID_IN.P = RX_pid_data.f;
			}
			else if (PID_PARAMETER == I)
			{
				/* 参数 I */
				this->PID_IN.I = RX_pid_data.f;
			}
			else if (PID_PARAMETER == D)
			{
				/* 参数 D */
				this->PID_IN.D = RX_pid_data.f;
			}
			else if (PID_PARAMETER == IMAX)
			{
				/* 参数 D */
				this->PID_IN.IMax = RX_pid_data.f;
			}
			else if (PID_PARAMETER == PIDMAX)
			{
				/* 参数 D */
				this->PID_IN.PIDMax = RX_pid_data.f;
			}
		}
		else if (PID_RING == POSITION)
		{
			/* 位置环 */
			if (PID_PARAMETER == P)
			{
				/* 参数 P */
				this->PID_OUT.P = RX_pid_data.f;
			}
			else if (PID_PARAMETER == I)
			{
				/* 参数 I */
				this->PID_OUT.I = RX_pid_data.f;
			}
			else if (PID_PARAMETER == D)
			{
				/* 参数 D */
				this->PID_OUT.D = RX_pid_data.f;
			}
			else if (PID_PARAMETER == IMAX)
			{
				/* 参数 D */
				this->PID_OUT.IMax = RX_pid_data.f;
			}
			else if (PID_PARAMETER == PIDMAX)
			{
				/* 参数 D */
				this->PID_OUT.PIDMax = RX_pid_data.f;
			}
		}

		return 0;
	}
	else
	{
		return 1;
	}
}

/**
 * @brief  读取基本的电机参数数据
 * @details	电角度方向、极对数、零电角度
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
uint8_t can_motor::extract_MmotorData(uint8_t *CAN_RX_data)
{
	uint8_t RX_function = *(CAN_RX_data + 0);
	if (RX_function == MOTOR_DATA)
	{
		/* 数据 */
		inline_Struct RX_motor_data;

		this->dir = (int8_t) * (CAN_RX_data + 1);
		this->pole_pairs = (uint8_t) * (CAN_RX_data + 2);

		/* 数据转移 */
		RX_motor_data.c[0] = *(CAN_RX_data + 3);
		RX_motor_data.c[1] = *(CAN_RX_data + 4);
		RX_motor_data.c[2] = *(CAN_RX_data + 5);
		RX_motor_data.c[3] = *(CAN_RX_data + 6);

		this->zero_electrical_angle = RX_motor_data.f;

		return 0;
	}
	else
	{
		return 1;
	}
}

/**
 * @brief  读取基本的传感器数据
 * @details
 * @param
 * @retval
 */
/* * 功能字：0xF1
* 回发传感器采样数据

| 编码器帧首 | 功能字 | 转子角度 | 转子转速 | 电流     | 补零 |  和检验 |
| ---------- | ------ | -------- | -------- | -------- |  ------- | ------ |
| 0x30x      | 0xF1   | int16_t | int16_t | int16_t | 0     | 0x0E   |
| 标准2字节  | 1字节  | 2字节    | 2字节    | 2字节    | 0     | 1字节  |

| 数据内容 | 单位                                |
| -------- | ----------------------------------- |
| 转子角度 | 0.1 ° 度 （-3276.8 ~ 3276.8）       |
| 转子转速 | 0.1 rpm 转每分 （-3276.8 ~ 3276.8） |
| 电流大小 | 0.01 A 安培 （-327.68 ~ -327.68）          |*/
uint8_t can_motor::extract_SampleData(uint8_t *CAN_RX_data)
{
	uint8_t RX_function = *(CAN_RX_data + 0);
	if (RX_function == SAMPLE_DATA)
	{
		/* 这里放运行函数 */
		int16_t temp_int16;
		int32_t temp_int32;
		/* 数据转移 */
		temp_int16 = (((int16_t) * (CAN_RX_data + 1) << 8) | *(CAN_RX_data + 2));
		temp_int32 = (int32_t)temp_int16;
		this->true_angle = (float)temp_int32 / 10;
		temp_int16 = (((int16_t) * (CAN_RX_data + 3) << 8) | *(CAN_RX_data + 4));
		temp_int32 = (int32_t)temp_int16;
		this->true_speed = (float)temp_int32 / 10;
		temp_int16 = (((int16_t) * (CAN_RX_data + 5)) << 8) | ((int16_t) * (CAN_RX_data + 6));
		temp_int32 = (int32_t)temp_int16;
		this->true_current = (float)temp_int32 / 1000;
		// this->true_current = (float)((((int32_t) * (CAN_RX_data + 5)) << 8) | ((int32_t)*(CAN_RX_data + 6))) / 100;

		return 0;
	}
	else
	{
		return 1;
	}
}

/* *********************** 接收校验函数 *************************** */
/**
 * @brief  命令数据解析
 * @details 解析接受的命令函数
 * @param
 * @retval
 */
uint8_t can_motor::parse(CAN_RxHeaderTypeDef bsp_can_Rx, uint8_t *CAN_RX_data)
{
	/* 判断 canid  */
	if (bsp_can_Rx.StdId == this->can_id)
	{ /* 提取功能字 */
		uint8_t RX_function = *(CAN_RX_data + 0);
		/* 提取尾帧 */
		uint8_t RX_Tail = *(CAN_RX_data + 7);

		if ((RX_function + RX_Tail) == 0xFF)
		{
			/* 校验成功，进行处理 */
			this->extract_pid(CAN_RX_data);
			this->extract_SampleData(CAN_RX_data);
			this->extract_MmotorData(CAN_RX_data);

			return 0;
		}
		else
		{
			/* 校验不成功，可能需要返回报错 */
		}
	}
	return 1;
}

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
			motor_1612.parse(CAN_RxData);
		}
#endif

// 使用F1时CAN收到数据的解析函数
#ifdef BSP_CAN_USE_CAN

		motor1.parse(bsp_can_Rx, CAN_RxData);
		motor2.parse(bsp_can_Rx, CAN_RxData);

		// static uint16_t timeflag = 0;
		// timeflag++;
		// if (timeflag == 1000)
		// {
		// 	printf("time_ms = %d\r\n", HAL_GetTick());
		// 	timeflag = 0;
		// }
#endif
	}
}
