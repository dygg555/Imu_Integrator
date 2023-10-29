#include "serial.h"
#include "wit_c_sdk.h"
#include "REG.h"
#include "Imu_Integrator.h"
#include <stdint.h>
#include "spdlog/spdlog.h"
#include <chrono>

using namespace std;

#define ACC_UPDATE 0x01
#define GYRO_UPDATE 0x02
#define ANGLE_UPDATE 0x04
#define MAG_UPDATE 0x08
#define READ_UPDATE 0x80

static int fd, s_iCurBaud = 9600;
static volatile char s_cDataUpdate = 0;

const int c_uiBaud[] = {2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};

static void AutoScanSensor(const char *dev);
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);

int main()
{

	spdlog::warn("Welocme to IMU");

	std::string uart_port = "/dev/ttyS3";
	if ((fd = serial_open(uart_port.data(), 115200) < 0))
	{
		spdlog::error("open uart port fail");
		return 0;
	}
	else
		spdlog::info("open port success");

	float fAcc[3], fGyro[3], fAngle[3];
	int i, ret;
	unsigned char cBuff[1];

	WitInit(WIT_PROTOCOL_NORMAL, 0x50);
	WitRegisterCallBack(SensorDataUpdata);

	AutoScanSensor(uart_port.data()); // 自动扫描设备波特率

	bool imu_frame_update = false;
	int imu_frame_skip = 0;

	ImuIntegrator *imu_integrator = new ImuIntegrator();

	while (1)
	{
		
		while (serial_read_data(fd, cBuff, 1))
		{
			WitSerialDataIn(cBuff[0]);
		}
		//   Delayms(1);

		if (s_cDataUpdate)
		{

			// for (i = 0; i < 3; i++)
			// {
			// 	fAcc[i] = sReg[AX + i] / 32768.0f * 16.0f;
			// 	fGyro[i] = sReg[GX + i] / 32768.0f * 2000.0f;
			// 	fAngle[i] = sReg[Roll + i] / 32768.0f * 180.0f;
			// }

			if (s_cDataUpdate & ACC_UPDATE)
			{
				// spdlog::info("ACC_UPDATE");
				//    printf("acc:%.3f %.3f %.3f\r\n", fAcc[0], fAcc[1], fAcc[2]);
				s_cDataUpdate &= ~ACC_UPDATE;
			}
			if (s_cDataUpdate & GYRO_UPDATE)
			{
				// spdlog::info("GYRO_UPDATE");
				imu_frame_update = true;
				//    printf("gyro:%.3f %.3f %.3f\r\n", fGyro[0], fGyro[1], fGyro[2]);
				s_cDataUpdate &= ~GYRO_UPDATE;
			}
			//   if(s_cDataUpdate & ANGLE_UPDATE)
			//     {
			// 	//    printf("angle:%.3f %.3f %.3f\r\n", fAngle[0], fAngle[1], fAngle[2]);
			// 	   s_cDataUpdate &= ~ANGLE_UPDATE;
			//     }
			//   if(s_cDataUpdate & MAG_UPDATE)
			//     {
			// 	//    printf("mag:%d %d %d\r\n", sReg[HX], sReg[HY], sReg[HZ]);
			// 	   s_cDataUpdate &= ~MAG_UPDATE;
			//     }
		}

		if(imu_frame_update)
		{
			if(imu_frame_skip < 10)
			{
				imu_frame_skip++;
				imu_frame_update = false;
			}
		}

		if(imu_frame_update)
		{
			imu_frame_update = false;
			for (i = 0; i < 3; i++)
			{
				fAcc[i] = sReg[AX + i] / 32768.0f * 16.0f;
				fGyro[i] = sReg[GX + i] / 32768.0f * 2000.0f;
				// fAngle[i] = sReg[Roll + i] / 32768.0f * 180.0f;
			}
			ImuMsg curmsg;
			curmsg.time = std::chrono::steady_clock::now();
			// auto duration = curmsg.time.time_since_epoch();
			// double seconds = std::chrono::duration_cast<std::chrono::duration<double>>(duration).count();
			// spdlog::info("curmsg.time:{}",seconds);
			curmsg.linear_acceleration << fAcc[0] , fAcc[1] , fAcc[2];
			curmsg.angular_velocity << fGyro[0] , fGyro[1] , fGyro[2];
			imu_integrator->ImuCallback(curmsg);
			Eigen::Vector3d posenow = imu_integrator->getCurrPose();
			spdlog::info("curmsgpose : {:.2f},{:.2f},{:.2f}",posenow.x(),posenow.y(),posenow.z());
		}
	}

	serial_close(fd);
	return 0;
}

static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{
	int i;
	for (i = 0; i < uiRegNum; i++)
	{
		switch (uiReg)
		{
			//            case AX:
			//            case AY:
		case AZ:
			s_cDataUpdate |= ACC_UPDATE;
			break;
			//            case GX:
			//            case GY:
		case GZ:
			s_cDataUpdate |= GYRO_UPDATE;
			break;
			//            case HX:
			//            case HY:
		case HZ:
			s_cDataUpdate |= MAG_UPDATE;
			break;
			//            case Roll:
			//            case Pitch:
		case Yaw:
			s_cDataUpdate |= ANGLE_UPDATE;
			break;
		default:
			s_cDataUpdate |= READ_UPDATE;
			break;
		}
		uiReg++;
	}
}

static void Delayms(uint16_t ucMs)
{
	usleep(ucMs * 1000);
}

static void AutoScanSensor(const char *dev)
{
	int i, iRetry;
	unsigned char cBuff[1];

	for (i = 1; i < 10; i++)
	{
		serial_close(fd);
		s_iCurBaud = c_uiBaud[i];
		fd = serial_open(dev, c_uiBaud[i]);

		iRetry = 2;
		do
		{
			s_cDataUpdate = 0;
			WitReadReg(AX, 3);
			Delayms(200);
			while (serial_read_data(fd, cBuff, 1))
			{
				WitSerialDataIn(cBuff[0]);
			}
			if (s_cDataUpdate != 0)
			{
				printf("%d baud find sensor\r\n\r\n", c_uiBaud[i]);
				return;
			}
			iRetry--;
		} while (iRetry);
	}
	printf("can not find sensor\r\n");
	printf("please check your connection\r\n");
}
