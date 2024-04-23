/**
 * @file app_usartcmd.cpp
 * @brief usartcmd 应用级支持包
 * @author Tony_Wang
 * @version 1.0
 * @date 2023-8-29
 * @copyright
 * @par 日志:
 *   V1.0 针对串口接受控制命令解析
 *
 */

/* 包含头文件 ----------------------------------------------------------------*/
#include "app_usartcmd.hpp"
#include "txmotor/app_txmotor.hpp"

/* 私有类型定义 --------------------------------------------------------------*/

/* 对外的目标 前后左右 速度 */
float car_target_speed = 0;
float car_target_ratate = 0;
////******************************************* UsartCMD 编码器类*************************************************************************////
/**
 * @brief  UsartCMD 编码器类
 * @param [in]   counter_period	完整一圈的计数值
 * @param [in]	get_count_link	获取计数值连接函数
 */
WEAK UsartCMD::UsartCMD(UART_HandleTypeDef *huart, char eol, bool echo)
{
	com_port = huart;
	this->eol = eol;
	this->echo = echo;
}
UsartCMD::UsartCMD(char eol, bool echo)
{
	this->eol = eol;
	this->echo = echo;
}

/**
 * @brief  add 添加命令函数
 * @details 加入回调函数到 UsartCMD
 * @param [in]   id	命令字符
 * @param [in]	onCommand 回调函数名
 * @param [in]	label 对应的字符串名称
 * @retval
 */
void UsartCMD::add(char id, CommandCallback onCommand, char *label)
{
	call_list[call_count] = onCommand;
	call_ids[call_count] = id;
	call_label[call_count] = label;
	call_count++;
}

/**
 * @brief  run 运行函数
 * @details 串口有声明通道进行下一步
 * @param [in]
 * @retval
 */
void UsartCMD::run()
{
	if (com_port == nullptr)
		return;
	run(com_port, eol);
}

/**
 * @brief  run 运行函数
 * @details 接串口fifo中转存，进入后续解析
 * @param [in] 串口通道
 * @param [in] 尾帧
 * @retval
 */
void UsartCMD::run(UART_HandleTypeDef *huart, char eol)
{
	UART_HandleTypeDef *tmp = com_port; // save the serial instance
	char eol_tmp = this->eol;
	this->eol = eol;
	com_port = huart;

	// a string to hold incoming data
	while (RxDebug_fifo.check_state() != FIFO_EMPTY)
	{
		// get the new byte:
		uint8_t ch;
		RxDebug_fifo.pop(&ch);
		received_chars[rec_cnt++] = (char)ch;
		// end of user input
		if (echo)
			printf("%c", (char)ch);
		if (isSentinel(ch))
		{
			// execute the user command
			run(received_chars);

			// reset the command buffer
			received_chars[0] = 0;
			rec_cnt = 0;
		}
		if (rec_cnt >= MAX_COMMAND_LENGTH)
		{ // prevent buffer overrun if message is too long
			received_chars[0] = 0;
			rec_cnt = 0;
		}
	}

	com_port = tmp; // reset the instance to the internal value
	this->eol = eol_tmp;
}

/**
 * @brief  run 运行函数
 * @details 接受内容解析
 * @param [in] 转存后的数据地址
 * @retval
 */
void UsartCMD::run(char *user_input)
{
	// execute the user command
	char id = user_input[0];
	switch (id)
	{
	case CMD_SCAN:
		for (int i = 0; i < call_count; i++)
		{
			printf("%c", call_ids[i]);
			printf(":");
			if (call_label[i])
				printf("%s\r\n", call_label[i]);
			else
				printf("\r\n");
		}
		break;
	case CMD_VERBOSE:
		/* 控制回发的变量，对于 stm32 没什么用 */
		// if (!isSentinel(user_input[1]))
		// 	verbose = (VerboseMode)atoi(&user_input[1]);
		// printVerbose(F("Verb:"));
		// switch (verbose)
		// {
		// case VerboseMode::nothing:
		// 	println(F("off!"));
		// 	break;
		// case VerboseMode::on_request:
		// case VerboseMode::user_friendly:
		// 	println(F("on!"));
		// 	break;
		// }
		break;
	case CMD_DECIMAL:
		/* 控制数据采样的小数位数，这部分功能使用 serialstudio 了，对于 stm32 没什么用 */
		// if (!isSentinel(user_input[1]))
		// 	decimal_places = atoi(&user_input[1]);
		// printVerbose(F("Decimal:"));
		// println(decimal_places);
		// break;
	default:
		for (int i = 0; i < call_count; i++)
		{
			if (id == call_ids[i])
			{
				call_list[i](&user_input[1]);
				break;
			}
		}
		break;
	}
}

/**
 * @brief  scalar 浮点型转换
 * @details 解析提取字符串数据为浮点
 * @param [in] value：放置的浮点数据地址
 * @param [in] user_cmd：接受转存后的数据地址
 * @retval
 */
void UsartCMD::scalar(float *value, char *user_cmd)
{
	bool GET = isSentinel(user_cmd[0]);
	if (!GET)
		*value = atof(user_cmd);
	//   println(*value);
	printf("%.*s\n", rec_cnt, received_chars);
}

/**
 * @brief  isSentinel 尾帧判断
 * @details 对是否检测到尾帧进行判断
 * @param [in] ch：需要检测的字符
 * @retval
 */
bool UsartCMD::isSentinel(char ch)
{
	if (ch == eol)
		return true;
	else if (ch == '\r')
	{
		//   printVerbose(F("Warn: \\r detected! \n"));
		printf("Warn: \\r detected! \n");
	}
	return false;
}

// void Commander::print(const int number){
//   if( !com_port || verbose == VerboseMode::nothing ) return;
//   com_port->print(number);
// }
// void Commander::print(const float number){
//   if(!com_port || verbose == VerboseMode::nothing ) return;
//   com_port->print((float)number,(int)decimal_places);
// }
// void Commander::print(const char* message){
//   if(!com_port || verbose == VerboseMode::nothing ) return;
//   com_port->print(message);
// }
// void Commander::print(const __FlashStringHelper *message){
//   if(!com_port || verbose == VerboseMode::nothing ) return;
//   com_port->print(message);
// }
// void Commander::print(const char message){
//   if(!com_port || verbose == VerboseMode::nothing ) return;
//   com_port->print(message);
// }

// void Commander::println(const int number){
//   if(!com_port || verbose == VerboseMode::nothing ) return;
//   com_port->println(number);
// }
// void Commander::println(const float number){
//   if(!com_port || verbose == VerboseMode::nothing ) return;
//   com_port->println((float)number, (int)decimal_places);
// }
// void Commander::println(const char* message){
//   if(!com_port || verbose == VerboseMode::nothing ) return;
//   com_port->println(message);
// }
// void Commander::println(const __FlashStringHelper *message){
//   if(!com_port || verbose == VerboseMode::nothing ) return;
//   com_port->println(message);
// }
// void Commander::println(const char message){
//   if(!com_port || verbose == VerboseMode::nothing ) return;
//   com_port->println(message);
// }

// void Commander::printVerbose(const char* message){
//   if(verbose == VerboseMode::user_friendly) print(message);
// }
// void Commander::printVerbose(const __FlashStringHelper *message){
//   if(verbose == VerboseMode::user_friendly) print(message);
// }
// void Commander::printError(){
//  println(F("err"));
// }

/* ***************************以下为声明 UsartCMD 变量****************************** */
UsartCMD command_usart1 = UsartCMD(fusart_Debug);

/* ***************************以下为自写回调功能函数****************************** */
/* callback code start */
/**
 * @brief  打印接受到的数据
 * @details
 * @param ID：A
 * @param cmd：数据地址
 * @retval
 */
void doDebugValue(char *cmd)
{
	static float debugvalue;
	command_usart1.scalar(&debugvalue, cmd);
	printf("%f\n", debugvalue);
}

/**
 * @brief  pid 设置函数
 * @details
 * @param ID1：K
 * @param ID2：num 电机号
 * @param ID3：(?:查看pid参数）(&:保存pid参数至flash）（I:修改内环pid参数）（O：修改外环pid参数）（C：修改电流环pid参数）
 * @param ID4：P/I/D 分别修改三个参数
 * @param cmd：数据地址
 * @retval
 */
void app_usartcmd_pidSet(char *cmd)
{
	/* 选择电机号 */
	can_motor motor_temp;
	if (*cmd == '1')
	{
		motor_temp = motor1;
	}
	else if (*cmd == '2')
	{
		motor_temp = motor2;
	}

	cmd = cmd + 1;
	float debugvalue;
	if (*cmd == '?')
	{
		motor_temp.command_PostBackpid();
		HAL_Delay(50);
		printf("\r\n in_p  = %f , in_i  = %f , in_d  = %f , in_IMax  = %f , in_PIDMax  = %f \r\n",
			   motor_temp.PID_IN.P,
			   motor_temp.PID_IN.I,
			   motor_temp.PID_IN.D,
			   motor_temp.PID_IN.IMax,
			   motor_temp.PID_IN.PIDMax);
		HAL_Delay(5);
		printf(" out_p = %f , out_i = %f , out_d = %f , out_IMax  = %f , out_PIDMax  = %f \r\n",
			   motor_temp.PID_OUT.P,
			   motor_temp.PID_OUT.I,
			   motor_temp.PID_OUT.D,
			   motor_temp.PID_OUT.IMax,
			   motor_temp.PID_OUT.PIDMax);
		HAL_Delay(5);
		printf(" cur_p = %f , cur_i = %f , cur_d = %f , cur_IMax  = %f , cur_PIDMax  = %f \r\n",
			   motor_temp.PID_CURRENT.P,
			   motor_temp.PID_CURRENT.I,
			   motor_temp.PID_CURRENT.D,
			   motor_temp.PID_CURRENT.IMax,
			   motor_temp.PID_CURRENT.PIDMax);
		HAL_Delay(5);
		printf(" dir = %d , pole_pairs = %d , zero_electrical_angle = %f  \r\n",
			   motor_temp.dir,
			   motor_temp.pole_pairs,
			   motor_temp.zero_electrical_angle);
		HAL_Delay(5);
	}
	else if (*cmd == '&')
	{
		motor_temp.command_Savepid();
		printf("pid saved \r\n");
	}
	else if (*cmd == 'I')
	{
		command_usart1.scalar(&debugvalue, (cmd + 2));
		switch (*(cmd + 1))
		{
		case 'P':
			motor_temp.command_SETpid(SPEED, P, debugvalue);
			break;
		case 'I':
			motor_temp.command_SETpid(SPEED, I, debugvalue);
			break;
		case 'D':
			motor_temp.command_SETpid(SPEED, D, debugvalue);
			break;
		case '[': // IMax 设置
			motor_temp.command_SETpid(SPEED, IMAX, debugvalue);
			break;
		case '{': // PIDMax 设置
			motor_temp.command_SETpid(SPEED, PIDMAX, debugvalue);
			break;
		default:
			break;
		}
	}
	else if (*cmd == 'O')
	{
		command_usart1.scalar(&debugvalue, (cmd + 2));
		switch (*(cmd + 1))
		{
		case 'P':
			motor_temp.command_SETpid(POSITION, P, debugvalue);
			break;
		case 'I':
			motor_temp.command_SETpid(POSITION, I, debugvalue);
			break;
		case 'D':
			motor_temp.command_SETpid(POSITION, D, debugvalue);
			break;
		case '[': // IMax 设置
			motor_temp.command_SETpid(POSITION, IMAX, debugvalue);
			break;
		case '{': // PIDMax 设置
			motor_temp.command_SETpid(POSITION, PIDMAX, debugvalue);
			break;
		default:
			break;
		}
	}
	else if (*cmd == 'C')
	{
		command_usart1.scalar(&debugvalue, (cmd + 2));
		switch (*(cmd + 1))
		{
		case 'P':
			motor_temp.command_SETpid(CURRENT, P, debugvalue);
			break;
		case 'I':
			motor_temp.command_SETpid(CURRENT, I, debugvalue);
			break;
		case 'D':
			motor_temp.command_SETpid(CURRENT, D, debugvalue);
			break;
		case '[': // IMax 设置
			motor_temp.command_SETpid(CURRENT, IMAX, debugvalue);
			break;
		case '{': // PIDMax 设置
			motor_temp.command_SETpid(CURRENT, PIDMAX, debugvalue);
			break;
		default:
			break;
		}
	}
}

/**
 * @brief  设置目标
 * @details
 * @param ID1：T
 * @param ID2：num 电机号
 * @param ID3：(S:设置速度)(A:设置角度)(?:显示当前运行状态)
 * @param cmd：数据地址
 * @retval
 */
void app_usartcmd_targetSet(char *cmd)
{
	float debugvalue;
	if (*cmd == '?')
	{
		// switch (motor1.run_mode)
		// {
		// case openloop:
		// 	printf("\r\nopenloop , speed = %f\r\n", motor1.target_speed);
		// 	break;
		// case speedMode:
		// 	printf("\r\nspeedMode , speed = %f\r\n", motor1.target_speed);
		// 	break;
		// case angleMode:
		// 	printf("\r\nangleMode , angle = %f\r\n", motor1.target_angle);
		// 	break;
		// case currentMode:
		// 	printf("\r\ncurrentMode , current = %f\r\n", motor1.target_current);
		// 	break;
		// default:
		// 	break;
		// }
	}
	else
	{
		can_motor motor_temp;
		if (*cmd == '1')
		{
			motor_temp = motor1;
		}
		else if (*cmd == '2')
		{
			motor_temp = motor2;
		}

		command_usart1.scalar(&debugvalue, (cmd + 2));
		if (*(cmd + 1) == 'S')
		{
			motor_temp.ctrl(speedMode, debugvalue);
			printf("\r\n Speed set = %f\r\n", debugvalue);
		}
		else if (*(cmd + 1) == 'A')
		{
			motor_temp.ctrl(angleMode, debugvalue);
		}
		else if (*(cmd + 1) == 'C')
		{

			motor_temp.ctrl(currentMode, debugvalue);
		}
	}
}

/**
 * @brief  设置模式
 * @details
 * @param ID1：M
 * @param ID2：(E:使能)(D:失能)(?:校准)
 * @param cmd：数据地址
 * @retval
 */
void app_usartcmd_modeSet(char *cmd)
{
	float debugvalue;

	command_usart1.scalar(&debugvalue, (cmd + 1));
	if (*cmd == 'E')
	{
		motor1.command_RunMode();
		motor2.command_RunMode();
		printf("\r\n *** Motor run ***\r\n");
	}
	else if (*cmd == 'D')
	{
		motor1.command_SafeMode();
		motor2.command_SafeMode();
		printf("\r\n *** Motor stop ***\r\n");
	}
	else if (*cmd == '?')
	{
		motor1.command_CAL_zeroElectricalAngle();
		motor2.command_CAL_zeroElectricalAngle();
		printf("\r\n *** Motor cal ***\r\n");
	}
	else if (*cmd == '1')
	{
		if (*(cmd + 1) == 'd')
		{
			command_usart1.scalar(&debugvalue, (cmd + 2));
			motor1.command_SetMotor(0x01, (int8_t)(debugvalue + 0.5));
			printf("\r\n *** Motor1 dir set ***\r\n");
		}
		else if (*(cmd + 1) == 'p')
		{
			command_usart1.scalar(&debugvalue, (cmd + 2));
			motor1.command_SetMotor(0x02, (int8_t)(debugvalue + 0.5));
			printf("\r\n *** Motor1 pole_pairs set ***\r\n");
		}
	}
	else if (*cmd == '2')
	{
		if (*(cmd + 1) == 'd')
		{
			command_usart1.scalar(&debugvalue, (cmd + 2));
			motor2.command_SetMotor(0x01, (int8_t)(debugvalue + 0.5));
			printf("\r\n *** Motor2 dir set ***\r\n");
		}
		else if (*(cmd + 1) == 'p')
		{
			command_usart1.scalar(&debugvalue, (cmd + 2));
			motor2.command_SetMotor(0x02, (int8_t)(debugvalue + 0.5));
			printf("\r\n *** Motor2 pole_pairs set ***\r\n");
		}
	}
}

/**
 * @brief  小车前进
 * @details
 * @param ID1：G
 * @param cmd：数据地址
 * @retval
 */
void app_usartcmd_cargo(char *cmd)
{
	float debugvalue;

	command_usart1.scalar(&debugvalue, cmd);

	car_target_speed = debugvalue;
}

/**
 * @brief  小车旋转
 * @details
 * @param ID1：R
 * @param cmd：数据地址
 * @retval
 */
void app_usartcmd_carrotate(char *cmd)
{
	float debugvalue;

	command_usart1.scalar(&debugvalue, cmd);

	car_target_ratate = debugvalue;
}

/**
 * @brief  UsartCMD初始化
 * @details 加入回调函数到 UsartCMD
 * @param ID：A
 * @param function：回调函数名
 * @param label：回调函数标签
 * @retval
 */
void command_usart1_init(void)
{
	command_usart1.add('A', doDebugValue, (char *)"doDebugValue");
	command_usart1.add('K', app_usartcmd_pidSet, (char *)"app_usartcmd_pidSet");
	command_usart1.add('T', app_usartcmd_targetSet, (char *)"app_usartcmd_targetSet");
	command_usart1.add('M', app_usartcmd_modeSet, (char *)"app_usartcmd_modeSet");
	/* ESP32控制前后函数 */
	command_usart1.add('G', app_usartcmd_cargo, (char *)"app_usartcmd_cargo");
	/* ESP32控制旋转函数 */
	command_usart1.add('R', app_usartcmd_carrotate, (char *)"app_usartcmd_carrotate");
}

/* callback code end */
