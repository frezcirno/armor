//
//created by 刘雍熙 on 2020-02-01
//

#ifndef COMMUNICATE_HPP
#define COMMUNICATE_HPP

#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>
#include <cmath>

#include <serial/serial.h>

#include "crc_table.hpp"
#include "usbio.hpp"

#include "base.hpp"

namespace armor{
    
    //与电控的通信，需要与电控协商确定发的数据（共三个标识符）
    //1:击打状态
    typedef enum{
        SEND_STATUS_AUTO_PLACEHOLDER = 0x00, // 占位符
        SEND_STATUS_AUTO_AIM_FORMER = 0x30,  // 打上一帧
        SEND_STATUS_AUTO_AIM = 0x31,         // 去瞄准
        SEND_STATUS_AUTO_SHOOT = 0x32,       // 去打
        SEND_STATUS_AUTO_NOT_FOUND = 0x33,   // 没找到
        SEND_STATUS_WM_AIM = 0x34            // 使用大风车
    } emSendStatusA;
    //2:大小符标识符
    typedef enum{
        SEND_STATUS_WM_PLACEHOLDER = 0x00,   // 占位符
        SEND_STATUS_WM_FIND = 0x41,          // 大风车瞄准
        SEND_STATUS_WM_NO = 0x42             // 大风车没识别
    } emSendStatusB;
    //3：工作模式（自瞄/风车）
    typedef enum{
        RM_AUTO_ATTACK = 0x11,
        RM_WINDMILL_SMALL_CLOCK = 0x21,
        RM_WINDMILL_SMALL_ANTIC = 0x22,
        RM_WINDMILL_LARGE_CLOCK = 0x23,
        RM_WINDMILL_LARGE_ANTIC=0x24
    } emWorkMode;

    /**
     * 通讯基类
     */
    class Communicator{
        protected:
//为整个包减少空字符使用pragma
#pragma pack(1)
        //头帧 时间戳 yaw pitch 标识符 校验 尾帧
        struct __FrameSt
        {
            uint8_t head = 0xf1;
            uint16_t timeStamp = 0;
            float yaw = 0.0;
            float pitch = 0.0;
            uint8_t extra[2] = {0, 0}; // additional imformation
            uint8_t crc8check = 0;
            uint8_t end = 0xf2;
        } m_frame;
#pragma pack()

        //crc校验用
        static uint8_t m_calcCRC8(const uint8_t *buff, size_t len) {
            uint8_t ucIndex, ucCRC8 = (uint8_t) CRC8_INIT;
            while (len--) {
                ucIndex = ucCRC8 ^ (*buff++);
                ucCRC8 = CRC8_Table[ucIndex];
            }
            return (ucCRC8);
        }

        const size_t m_frameSize = sizeof(__FrameSt);

        //包校验
        bool m_checkFrame(const uint8_t *buff) {
            if (buff[0] == m_frame.head && buff[m_frameSize - 1] == m_frame.end) {
                return buff[m_frameSize - 2] == m_calcCRC8(buff, m_frameSize - 2);
            }
            return false;
        }

        //多线程用
        std::mutex m_mutex;

        std::atomic_bool m_isEnableReceiveGlobalAngle;
        std::atomic_int m_WorkMode;   // 当前工作模式: 摸鱼, 自瞄, 大风车
        std::deque<float> m_gYaws;
        std::deque<float> m_gPitches;

        std::thread m_receiveThread;  // 接收线程
        std::atomic_bool m_letStop;   // 暂停接收线程

        std::atomic_bool m_isDisable; // 是否使能

        std::atomic<int64> m_lastTick;
        std::atomic<int64> m_currentInterval;

        public:
        //构造函数
        explicit Communicator(): m_isEnableReceiveGlobalAngle(false), m_WorkMode(RM_AUTO_ATTACK),
        m_letStop(false), m_isDisable(false), m_lastTick(cv::getTickCount()),
        m_currentInterval(-1110)
        {
            m_gYaws.resize(100);
            m_gPitches.resize(100);
        }

        /**
         * 禁用, debug 用
         * @param disable
         */
        void disable(bool disable) {
            m_isDisable.exchange(disable);
        }

        /**
         * 发送给电控
         * @param rYaw
         * @param rPitch
         * @param extra0
         * @param extra1
         */
        virtual void send(float rYaw, float rPitch, emSendStatusA extra0, emSendStatusB extra1) {};

        /**
         * 开始接收线程
         */
        virtual void startReceiveService() {};

        /**
         * 获得当前工作模式: 摸鱼, 自瞄, 大风车
         * @return
         */
        emWorkMode getWorkMode() {
            switch (m_WorkMode.load()) {
                default:
                case RM_AUTO_ATTACK:
                    return RM_AUTO_ATTACK;
                case RM_WINDMILL_SMALL_CLOCK:
                    return RM_WINDMILL_SMALL_CLOCK;
                case RM_WINDMILL_SMALL_ANTIC:
                    return RM_WINDMILL_SMALL_ANTIC;
                case RM_WINDMILL_LARGE_CLOCK:
                    return RM_WINDMILL_LARGE_CLOCK;
                case RM_WINDMILL_LARGE_ANTIC:
                    return RM_WINDMILL_LARGE_ANTIC;
            }
        };

        /**
         * 获得当前 wait_and_get 函数获得的图像时间间隔
         * @return 采集时间间隔us
         */
        int64 getCurrentInterval() {
            return m_currentInterval.load();
        }

        /**
         * 获得最新的云台全局欧拉角
         * @param gYaw
         * @param gPitch
         * @param delay 取前 delay 个欧拉角
         */
        void getGlobalAngle(float *gYaw, float *gPitch, uint8_t delay = 0) {
            if (m_isEnableReceiveGlobalAngle) {
                std::lock_guard<std::mutex> lockGuard(m_mutex);
                *gYaw = m_gYaws[delay];
                *gPitch = m_gPitches[delay];
            } else {
                *gYaw = 0;
                *gPitch = 0;
            }
        }

        void enableReceiveGlobalAngle(bool enable = true) {
            m_isEnableReceiveGlobalAngle.exchange(enable);
        }

        /**
         * 结束所有
         */
        void letStop() { m_letStop.exchange(true); }

        /**
         * 等待线程结束
         */
        void join() {
            if (m_receiveThread.joinable()) m_receiveThread.join();
        };

        ~Communicator() { join(); }

    };

    /**
     * 使用串口通信的子类
     */
    class CommunicatorSerial : public Communicator {
    private:
        serial::Serial m_ser;

    public:
        explicit CommunicatorSerial() = default;

        /**
         * 打开设备, 带后台线程
         * @param portName
         * @param baudrate
         */
        void open(const cv::String &portName, uint32_t baudrate = 115200) {
            if (m_isDisable.load()) return;
            m_ser.setPort(portName);
            m_ser.setBaudrate(baudrate);

            /* 守护线程 */
            int openSerialCounter = 0;
            while (!m_ser.isOpen() && !m_letStop.load()) {
                PRINT_WARN("[serial] try open %d\n", openSerialCounter++);
                try {
                    m_ser.open();
                } catch (serial::IOException &e) {
                    PRINT_ERROR("[serial] error: %s\n", e.what());
                }
                /* 转移时间片 */
                armor::thread_sleep_ms(100);
                if (openSerialCounter > 10) break;
            }
            if (m_ser.isOpen()) PRINT_INFO("[serial] open\n");
            else exit(-666);
        }

        void send(float rYaw, float rPitch, emSendStatusA extra0, emSendStatusB extra1) override {
            if (m_isDisable.load()) return;
            if (!m_ser.isOpen()) return;
            /* 刷新结构体 */
            if (m_frame.timeStamp > 0xfffe) m_frame.timeStamp = 0;
            m_frame.timeStamp++;
            m_frame.yaw = rYaw;
            m_frame.pitch = rPitch;
            m_frame.extra[0] = extra0;
            m_frame.extra[1] = extra1;

            /* 间隔3ms以内不发送 */
            if (1000.0 * (cv::getTickCount() - m_lastTick.load()) / cv::getTickFrequency() > 3) {
                m_currentInterval.exchange(
                        1000000.0 * (cv::getTickCount() - m_lastTick.load()) / cv::getTickFrequency());
                m_lastTick.exchange(cv::getTickCount());

                /* 组织字节 */
                uint8_t msg[m_frameSize];
                memcpy(msg, &m_frame, m_frameSize);
                msg[m_frameSize - 2] = m_calcCRC8(msg, m_frameSize - 2);

                /* 发送 */
                m_ser.flushOutput();
                m_ser.write(msg, m_frameSize);
            }
        };

        void startReceiveService() override {
            if (m_isDisable.load()) return;
            DEBUG("startReceiveService")
            m_receiveThread = std::thread([&]() {
                while (!m_ser.isOpen() && !m_letStop.load()) { armor::thread_sleep_us(200); }
                std::vector<uint8_t> buffer;
                while (!m_letStop.load()) {
                    size_t size_temp = m_ser.available();
                    if (size_temp > 0) {
                        /* 读值 */
                        std::vector<uint8_t> buffer_tmp;
                        m_ser.read(buffer_tmp, size_temp);
                        buffer.insert(buffer.end(), buffer_tmp.begin(), buffer_tmp.end());
                        /* 校验 */
                        while (buffer.size() >= m_frameSize) {
                            size_t i = 0;
                            for (i = 0; i < buffer.size() - m_frameSize + 1; ++i) {
                                if (m_checkFrame(buffer.data())) {
                                    /* 更新值 begin */
                                    struct __FrameSt frame;
                                    memcpy(&frame, buffer.data(), m_frameSize);

                                    /* 更新成员变量 */
                                    m_WorkMode.exchange(frame.extra[0]);

                                    if (m_isEnableReceiveGlobalAngle) {
                                        std::lock_guard<std::mutex> lock(m_mutex);
                                        m_gYaws.emplace_front(frame.yaw / M_PI * 180);  // 转化成角度
                                        m_gPitches.emplace_front(frame.pitch / M_PI * 180);
                                        if (m_gYaws.size() > 10) m_gYaws.pop_back();
                                        if (m_gPitches.size() > 10) m_gPitches.pop_back();
                                        printf("receive (yaw, pitch) = (%3.2f, %3.2f)\n", frame.yaw, frame.pitch);
                                    }
                                    /* 更新值 end */
                                }
                            }
                            buffer.erase(buffer.begin(), buffer.begin() + i + m_frameSize - 1);
                        }
                    }
                    /* 转移时间片 */
                    thread_sleep_us(5);
                }
            });
        }
    };


    /**
     * USB 通讯子类
     */

    class CommunicatorUSB : public Communicator {
        private:
        superpower::usbio::spUSB *m_usb;

        public:
        explicit CommunicatorUSB(): m_usb(nullptr){}

        /**
         * 打开设备, 带后台线程
         * @param vid 0x0477 16进制
         * @param pid 0x5620 16进制
         */
        void open(int vid, int pid) {
            m_usb = new superpower::usbio::spUSB(vid, pid);
        }

        void send(float rYaw, float rPitch, emSendStatusA extra0, emSendStatusB extra1) override {
            /* 刷新结构体 */
            if (m_frame.timeStamp > 0xfffe) m_frame.timeStamp = 0;
            m_frame.timeStamp++;
            m_frame.yaw = rYaw;
            m_frame.pitch = rPitch;
            m_frame.extra[0] = extra0;
            m_frame.extra[1] = extra1;

            /* 组织字节 */
            uint8_t msg[m_frameSize];
            memcpy(msg, &m_frame, m_frameSize);
            msg[m_frameSize - 2] = m_calcCRC8(msg, m_frameSize - 2);

            /* 发送 */
            int len;
            m_usb->write(msg, m_frameSize, len);
            m_currentInterval.exchange(1000000.0 * (cv::getTickCount() - m_lastTick.load()) / cv::getTickFrequency());
            m_lastTick.exchange(cv::getTickCount());
        }

        void startReceiveService() override {
            m_receiveThread = std::thread([&]() {
                uint8_t buffer[1024] = {0};
                while (!m_letStop.load()) {
                    size_t size_temp = m_usb->available();
                    if (size_temp > 0) {
                        uint8_t buf[1024];
                        uint8_t size = m_usb->read(buf, size_temp);

                        memcpy(buffer, buf, size);

                        printf("%d\n", size);
                        while (size >= m_frameSize) {
                            int i = 0;
                            for (i = 0; i < size - m_frameSize + 1; ++i) {
                                if (m_checkFrame(&buffer[i])) {
                                    /* 更新值 begin */
                                    struct __FrameSt frame;
                                    memcpy(&frame, &buffer[i], m_frameSize);

                                    /* 更新成员变量 */
                                    m_WorkMode.exchange(frame.extra[0]);

                                    if (m_isEnableReceiveGlobalAngle) {
                                        std::lock_guard<std::mutex> lock(m_mutex);
                                        m_gYaws.emplace_front(frame.yaw);
                                        m_gPitches.emplace_front(frame.pitch);
                                        printf("receive (yaw, pitch) = (%3.2f, %3.2f)\n", frame.yaw, frame.pitch);
                                    }
                                    /* 更新值 end */
                                }
                            }
                            size = size - i - m_frameSize;
                            // 0, {1, 2, 3}, 4, 5, 6, 7
                            // size = 8, i = 1, m_frameSize = 3
                            // size = 8 - 1 - 3 = 4
                            memcpy(buffer, &buf[i + m_frameSize], size);
                        }
                        /* 转移时间片 */
                        armor::thread_sleep_us(5);
                    }
                }
            });
        }
    };
}

#endif //COMMUNICATE_HPP