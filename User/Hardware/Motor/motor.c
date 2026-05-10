/**
 * @file motor.c
 * @author Wang Zihao
 * @brief �����������뷴��
 * @version 0.1
 * @date 2025-9-12
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "motor.h"
#include "math.h"
#include "string.h"

#define abs(a) a > 0 ? a : -a

#if (USE_DJIMotor == 1)
// ������ݶ���
DJI_motor_data_s DJIMotor_data[QUANTITY_OF_CAN][QUANTITY_OF_DJIMOTOR];

/**
 * @brief �󽮵����ʼ������ʱֻ������ʼ������
 *
 * @param motor_type �������
 * @param motor_id ���canͨ����ID
 */
void DJIMotor_init(Motor_Type_e motor_type, DJIcan_id motor_id)
{
    uint8_t cantype = motor_id / 11; // ��õ������can·
    uint8_t canid = motor_id % 11;   // �õ����IDֵ��

    DJIMotor_data[cantype][canid].Motor_type = motor_type; // ��ʼ����Ӧ���
}

/**
 * @brief ���ô󽮵�����
 *
 * @param zero_angle ���Ƕ�
 * @param motor_id ���ID
 */
void DJIMotor_setzero(double zero_angle, DJIcan_id motor_id)
{
    uint8_t cantype = motor_id / 11; // ��õ������can·
    uint8_t canid = motor_id % 11;   // �õ����IDֵ��

    DJIMotor_data[cantype][canid].angle_zero = zero_angle; // ��ʼ����Ӧ���
}

/**
 * @brief ���ô󽮵������
 *
 * @param val ����ֵ
 * @param motor_id ���canͨ����ID
 */
void DJIMotor_set(int16_t val, DJIcan_id motor_id)
{
    DJIMotor_data[motor_id / 11][motor_id % 11].set = val; // ���õ���
}

/**
 * @brief ��ȡ�󽮵������
 *
 * @param motor_id ���canͨ����ID
 * @return DJI_motor_data_s ������ݽṹ�塣
 */
DJI_motor_data_s DJIMotor_get_data(DJIcan_id motor_id) // ��ȡ��������
{
    return DJIMotor_data[motor_id / 11][motor_id % 11];
}

/**
 * @brief �󽮵��CAN���ݽ����Լ�����
 *
 * @param ptr �������
 * @param data can����
 */
void DJIMotor_get_process_motor_data(DJI_motor_data_s *ptr, uint8_t data[])
{
    // get raw data
    (ptr)->last_ecd = (ptr)->ecd;
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);
    (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);
    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);
    (ptr)->temperate = (data)[6];

    // process the data
    // count cnt
    if ((ptr)->last_ecd > 7000 && (ptr)->ecd < 1000)
        (ptr)->ecd_cnt += ((ECD_MAX - (ptr)->last_ecd) + (ptr)->ecd);
    else if ((ptr)->last_ecd < 1000 && (ptr)->ecd > 7000)
        (ptr)->ecd_cnt -= ((ECD_MAX - (ptr)->ecd) + (ptr)->last_ecd);
    else
        (ptr)->ecd_cnt += ((ptr)->ecd - (ptr)->last_ecd);
    // process data
    (ptr)->angle_cnt = (ptr)->ecd_cnt * ECD_TO_ANGEL_DJI;
    // ���ݲ�ͬ������в�ͬ����
    if ((ptr)->Motor_type == DJI_GM6020)
    {
        // �������ת��
        (ptr)->round_speed = (ptr)->speed_rpm;

        // ������ԽǶ� -180~180 �������ȶ�ʧ �ܽǶȹ���ʱ
        float angle = (ptr)->angle_cnt - (ptr)->angle_zero;
        uint32_t mul = abs((int)angle) / 180;
        if (angle > 180.0f)
        {
            if (mul % 2 == 1) // ����-180��
                angle -= (mul + 1) * 180;
            else // ����180��
                angle -= mul * 180;
        }
        if (angle < -180.0f)
        {
            if (mul % 2 == 1) // ����180��
                angle += (mul + 1) * 180;
            else // ����-180��
                angle += mul * 180;
        }
        (ptr)->angle = angle;
    }
    else if ((ptr)->Motor_type == DJI_M3508)
    {
        (ptr)->round_speed = (ptr)->speed_rpm / M3508_P;
    }
    else if ((ptr)->Motor_type == DJI_M2006)
    {
        (ptr)->round_speed = (ptr)->speed_rpm / M2006_P;
    }
}

/**
 * @brief �󽮵��can���ݴ���
 *
 * @param hfdcan CANͨ��
 * @param id can��ʶ��
 * @param data can����
 */
void DJIMotor_decode_candata(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data)
{
    if (id < CAN_ID1)
        return;
    if (id - CAN_ID1 <= 11) // ��ֹ�������
    {
        if (hfdcan == &hfdcan1)
        {
            DJIMotor_get_process_motor_data(&DJIMotor_data[0][id - CAN_ID1], data);
        }
        else if (hfdcan == &hfdcan2)
        {
            DJIMotor_get_process_motor_data(&DJIMotor_data[1][id - CAN_ID1], data);
        }
        else if (hfdcan == &hfdcan3)
        {
            DJIMotor_get_process_motor_data(&DJIMotor_data[2][id - CAN_ID1], data);
        }
    }
}

/**
 * @brief �󽮵������ֵ���ͣ������freertos�ﶨ�ڷ���
 *
 */
void DJIMotor_send_current(DJIcan_send_id_e CAN_Send_ID, DJIcan_send_id_e CAN_Type)
{
    uint8_t can_send_data[8];
    uint8_t canid;
    uint16_t identifier;
    FDCAN_HandleTypeDef *hcan ;

    if (hfdcan1.ErrorCode)
        HAL_FDCAN_ErrorCallback(&hfdcan1);
    if (hfdcan2.ErrorCode)
        HAL_FDCAN_ErrorCallback(&hfdcan2);
    if (hfdcan3.ErrorCode)
        HAL_FDCAN_ErrorCallback(&hfdcan3);

    // �жϵ�������Լ����ID
    switch (CAN_Send_ID)
    {
        case CAN_20063508_1_4_ID:
        {
            identifier = CAN_20063508_1_4_send_ID;
            canid = 0;
            break;
        }
        case CAN_20063508_5_8_ID:
        {
            identifier = CAN_20063508_5_8_send_ID;
            canid = 4;
             break;
        }
        case CAN_6020_1_4_ID:
        {
            identifier = CAN_6020_1_4_send_ID;
            canid = 4;
             break;
        }
        case CAN_6020_5_7_ID:
        {
            identifier = CAN_6020_5_7_send_ID;
            canid = 8;
             break;
        }
    }
    
    // �ж�CAN·
    switch (CAN_Type)
    {
        case DJI_CAN_1: hcan = &hfdcan1 ; break;
        case DJI_CAN_2: hcan = &hfdcan2 ; break;
        case DJI_CAN_3: hcan = &hfdcan3 ; break;
    }   
    
    // һ·can���θ�ֵ
    for (int i = 0; i < 8; i += 2)
    {
        can_send_data[i] = (DJIMotor_data[CAN_Type][canid].set >> 8);
        can_send_data[i+1] = DJIMotor_data[CAN_Type][canid].set;
        canid++;    
    }
   
    // ����CAN����
    fdcanx_send_data(hcan, identifier, can_send_data, 8);

}

#endif // USE_DJIMotor


#if (USE_DMMotor == 1)

// ���������ݶ���
DM_motor_data_s DM_Motor_data[QUANTITY_OF_CAN][QUANTITY_OF_DMMOTOR];

/**
 * @brief ʹ�ܴ�����
 * @param motor_id ���ID
 */
void DMMotor_enable(DMcan_id motor_id)
{
    if (motor_id >= DM_MOTOR_NUM)
        return;

    uint8_t cantype = motor_id / 6; // ��õ������can·
    uint8_t canid = motor_id % 6;   // �õ����IDֵ��

    switch (cantype)
    {
    case 0:
        dm_motor_enable(&hfdcan1, &DM_Motor_data[cantype][canid].motor_data);
        break;
    case 1:
        dm_motor_enable(&hfdcan2, &DM_Motor_data[cantype][canid].motor_data);
        break;
    case 2:
        dm_motor_enable(&hfdcan3, &DM_Motor_data[cantype][canid].motor_data);
        break;
    default:
        break;
    }
}

/**
 * @brief ʧ�ܴ�����
 * @param motor_id ���ID
 */
void DMMotor_disable(uint8_t motor_id)
{
    if (motor_id >= DM_MOTOR_NUM)
        return;
    uint8_t cantype = motor_id / 6; // ��õ������can·
    uint8_t canid = motor_id % 6;   // �õ����IDֵ��
    dm_motor_disable(&hfdcan1, &DM_Motor_data[cantype][canid].motor_data);
}

/**
 * @brief ���ô���������ģʽ
 * @param motor_id ���ID
 * @param mode ����ģʽ
 */
void DMMotor_set_mode(DMcan_id motor_id, DM_mode_e mode)
{
    if (motor_id >= DM_MOTOR_NUM)
        return;

    uint8_t cantype = motor_id / 6; // ��õ������can·
    uint8_t canid = motor_id % 6;   // �õ����IDֵ��

    DM_Motor_data[cantype][canid].motor_data.ctrl.mode = mode;
}

/**
 * @brief ��������ʼ��
 * @param motor_id ���ID (0-5)
 */
void DMMotor_init(Motor_Type_e motor_type,DMcan_id motor_id)
{
    if (motor_id >= DM_MOTOR_NUM)
        return;

    uint8_t cantype = motor_id / 6; // ��õ������can·
    uint8_t canid = motor_id % 6;   // �õ����IDֵ��

    memset(&DM_Motor_data[cantype][canid], 0, sizeof(DM_motor_data_s));

    DM_Motor_data[cantype][canid].motor_data.id = canid + 1;
    DM_Motor_data[cantype][canid].motor_data.mst_id = canid + 1;
    DM_Motor_data[cantype][canid].motor_data.tmp.read_flag = 1;
    DM_Motor_data[cantype][canid].motor_data.tmp.PMAX = P_MAX;
    DM_Motor_data[cantype][canid].motor_data.tmp.VMAX = V_MAX;
    DM_Motor_data[cantype][canid].motor_data.tmp.TMAX = T_MAX;
    DM_Motor_data[cantype][canid].motor_data.ctrl.mode = mit_mode; // Ĭ��MITģʽ

    // ʹ�ܵ��
    DMMotor_enable(motor_id);
}

/**
 * @brief ���ô��������Ʋ���
 * @param motor_id ���ID
 * @param pos λ���趨ֵ (rad)
 * @param vel �ٶ��趨ֵ (rad/s)
 * @param tor Ť���趨ֵ (N*M)
 * @param kp λ�ñ�������(N/r)
 * @param kd λ��΢������(N*s/r)
 */
void DMMotor_set(DMcan_id motor_id, float pos, float vel, float tor, float kp, float kd)
{
    if (motor_id >= DM_MOTOR_NUM)
        return;

    uint8_t cantype = motor_id / 6; // ��õ������can·
    uint8_t canid = motor_id % 6;   // �õ����IDֵ��

    DM_motor_t *m = &DM_Motor_data[cantype][canid].motor_data;
    m->ctrl.pos_set = pos;
    m->ctrl.vel_set = vel;
    m->ctrl.tor_set = tor;
    m->ctrl.kp_set = kp;
    m->ctrl.kd_set = kd;
}

/**
 * @brief ���ô��������
 * @param zero_angle ���Ƕ�
 * @param motor_id ���ID
 */
void DMMotor_setzero(double zero_angle, DMcan_id motor_id)
{
    uint8_t cantype = motor_id / 6; // ��õ������can·
    uint8_t canid = motor_id % 6;   // �õ����IDֵ��

    DM_Motor_data[cantype][canid].motor_data.angle_zero = zero_angle; // ��ʼ����Ӧ���
}

/**
 * @brief ������CAN���ݴ���
 * @param hfdcan CANͨ��
 * @param id can��ʶ��
 * @param data can����
 */
void DMMotor_decode_candata(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data)
{
      if (id > 0x06)
        return;
    uint8_t cantype; // ��õ������can·
    uint8_t canid;   // �õ����IDֵ��
    id--;
    if (hfdcan == &hfdcan1)
    { // ����������ID
        cantype = 0;
    }
    else if (hfdcan == &hfdcan2)
    {
        cantype = 1;
    }
    else if (hfdcan == &hfdcan3)
    {
        cantype = 2;
    }
    canid = id;
    // �������д������ķ�������
    dm_motor_fbdata(&DM_Motor_data[cantype][canid].motor_data, data);
}

/**
 * @brief ���������������
 */
int DMMotor_send_ctrl(DMcan_id motor_id)
{
    if (motor_id >= DM_MOTOR_NUM)
         return 0;

    uint8_t cantype = motor_id / 6; // ��õ������can·
    uint8_t canid = motor_id % 6;   // �õ����IDֵ��
    FDCAN_HandleTypeDef *hcan = get_can_handle(cantype);

    if (DM_Motor_data[cantype][canid].motor_data.ctrl.mode != 0)
    { // ��������ʹ��
        dm_motor_ctrl_send(hcan, &DM_Motor_data[cantype][canid].motor_data);
        return 1;
    }
    else
        return 0;
}

/**
 * @brief ��ȡ����������
 *
 * @param motor_id ���canͨ����ID
 * @return DM_motor_data_s ������ݽṹ�塣
 */
DM_motor_data_s DMMotor_get_data(DMcan_id motor_id) // ��ȡ��������
{
    return DM_Motor_data[motor_id / 6][motor_id % 6];
}

#endif // USE_DMMotor

#if (USE_LZMotor == 1)
// �����������
LZ_Motor_t LZ_Motors[QUANTITY_OF_CAN][QUANTITY_OF_LZMOTOR];

/**
 * @brief ��ʼ��������
 */
void LZMotor_init(LZ_Motor_ID_t motor_id) {
    if (motor_id >= LZ_MOTOR_NUM) return;

    uint8_t cantype = motor_id / 6; // ��õ������can·
    uint8_t canid = motor_id % 6;   // �õ����IDֵ��

    memset(&LZ_Motors[cantype][canid], 0, sizeof(LZ_Motor_t));
    
    // ����Ĭ��ID
    LZ_Motors[cantype][canid].id = canid + 1;
    LZ_Motors[cantype][canid].master_id = DEFAULT_MASTER_ID;
    
    // ����Ĭ��ģʽ
    LZ_Motors[cantype][canid].mode = LZ_MODE_MIT;

    LZMotor_enable(motor_id);
}

/**
 * @brief ʹ��������
 */
void LZMotor_enable(LZ_Motor_ID_t motor_id) {
    if (motor_id >= LZ_MOTOR_NUM) return;
    
    uint8_t cantype = motor_id / 6; // ��õ������can·
    uint8_t canid = motor_id % 6;   // �õ����IDֵ��
    
    lz_enable_motor(cantype, LZ_Motors[cantype][canid].id);
    LZ_Motors[cantype][canid].mode = LZ_MODE_MIT; // Ĭ��ʹ�ܺ����MITģʽ
}

/**
 * @brief ʧ��������
 */
void LZMotor_disable(LZ_Motor_ID_t motor_id) {
    if (motor_id >= LZ_MOTOR_NUM) return;
    
    uint8_t cantype = motor_id / 6; // ��õ������can·
    uint8_t canid = motor_id % 6;   // �õ����IDֵ��
    
    lz_disable_motor(cantype, LZ_Motors[cantype][canid].id);
    LZ_Motors[cantype][canid].mode = LZ_MODE_DISABLE;
}

/**
 * @brief ��������������ģʽ
 */
void LZMotor_set_mode(LZ_Motor_ID_t motor_id, LZ_Mode_t mode) {
    if (motor_id >= LZ_MOTOR_NUM) return;
    
    uint8_t cantype = motor_id / 6; // ��õ������can·
    uint8_t canid = motor_id % 6;   // �õ����IDֵ��
    
    uint8_t mode_val;
    switch (mode) {
        case LZ_MODE_MIT: mode_val = 0; break;
        case LZ_MODE_POSITION: mode_val = 1; break;
        case LZ_MODE_VELOCITY: mode_val = 2; break;
        default: return;
    }
    
    lz_set_mode(cantype, LZ_Motors[cantype][canid].id, mode_val);
    LZ_Motors[cantype][canid].mode = mode;
}

/**
 * @brief �������������Ʋ���
 */
void LZMotor_set_params(LZ_Motor_ID_t motor_id, float pos, float vel, float tor, float kp, float kd, float current_limit) {
    if (motor_id >= LZ_MOTOR_NUM) return;
    
    uint8_t cantype = motor_id / 6; // ��õ������can·
    uint8_t canid = motor_id % 6;   // �õ����IDֵ��
    
    LZ_Motors[cantype][canid].pos_set = pos;
    LZ_Motors[cantype][canid].vel_set = vel;
    LZ_Motors[cantype][canid].tor_set = tor;
    LZ_Motors[cantype][canid].kp_set = kp;
    LZ_Motors[cantype][canid].kd_set = kd;
    LZ_Motors[cantype][canid].current_limit = current_limit;
}

/**
 * @brief ������������������
 */
void LZMotor_send_command(LZ_Motor_ID_t motor_id) {
    if (motor_id >= LZ_MOTOR_NUM) return;
    
    uint8_t cantype = motor_id / 6; // ��õ������can·
    uint8_t canid = motor_id % 6;   // �õ����IDֵ��
    
    LZ_Motor_t *motor = &LZ_Motors[cantype][canid];
    
    switch (motor->mode) {
        case LZ_MODE_MIT:
            lz_send_mit_params(cantype, motor->id, motor->pos_set, motor->vel_set, 
                              motor->kp_set, motor->kd_set, motor->tor_set);
            break;
            
        case LZ_MODE_POSITION:
            lz_set_position(cantype, motor->id, motor->pos_set, motor->vel_set);
            break;
            
        case LZ_MODE_VELOCITY:
            lz_set_velocity(cantype, motor->id, motor->vel_set, motor->current_limit);
            break;
            
        default:
            // ����ģʽ�����Ϳ�������
            break;
    }
}

/**
 * @brief ��ȡ�������ָ��
 */
LZ_Motor_t* LZMotor_get(LZ_Motor_ID_t motor_id) {
    if (motor_id >= LZ_MOTOR_NUM) return NULL;
    
    uint8_t cantype = motor_id / 6; // ��õ������can·
    uint8_t canid = motor_id % 6;   // �õ����IDֵ��
    
    return &LZ_Motors[cantype][canid];
}

/**
 * @brief ������CAN���ݴ���
 */
void LZMotor_decode_candata(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data) {
    // ����˵���飬MITЭ�鷴��֡��ID��ʽΪ (���ID << 8) | ����ID
    uint8_t motor_id = (id >> 8) & 0xFF;
    uint8_t master_id = id & 0xFF;
    
    // ȷ��CAN����
    uint8_t can_bus = 0;
    if (hfdcan == &hfdcan2) can_bus = 1;
    else if (hfdcan == &hfdcan3) can_bus = 2;
    
    // ���Ҷ�Ӧ�ĵ��
    for (int i = 0; i < MOTORS_PER_CAN; i++) {
        if (LZ_Motors[can_bus][i].id == motor_id && LZ_Motors[can_bus][i].master_id == master_id) {
            // �����������ݣ�����˵�����е�ͨ������2��ʽ��
            // ������Ҫ����ʵ�ʷ������ݸ�ʽ���н���
            // ʾ�����룬ʵ��Ӧ����˵�������
            LZ_Motors[can_bus][i].state.angle = uint_to_float_LZ((data[0] << 8) | data[1], P_MIN, P_MAX, 16);
            LZ_Motors[can_bus][i].state.velocity = uint_to_float_LZ((data[2] << 4) | (data[3] >> 4), V_MIN, V_MAX, 12);
            LZ_Motors[can_bus][i].state.torque = uint_to_float_LZ(((data[3] & 0x0F) << 8) | data[4], T_MIN, T_MAX, 12);
            LZ_Motors[can_bus][i].state.temperature = data[5] * 0.1f; // �����¶�������data[5]
            break;
        }
    }
}


#endif // USE_LZMotor