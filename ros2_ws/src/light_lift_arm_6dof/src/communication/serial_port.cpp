#include "communication/serial_port.h"

namespace serialport
{

    static const uint16_t CRC16_TABLE[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202};

    SerialPortWrapper::SerialPortWrapper() : running(false)
    {

        receivedmotorData.resize(FRAME_DATA_LENGTH);
        try
        {
            // 打开串口
            serialPort.Open(PORT_NAME);
            serialPort.SetBaudRate(BAUD_RATE);
            serialPort.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
            serialPort.SetParity(Parity::PARITY_NONE);
            serialPort.SetStopBits(StopBits::STOP_BITS_1);
            serialPort.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);

            std::cout << "串口已打开：" << PORT_NAME << " 波特率：" << static_cast<int>(BAUD_RATE) << std::endl;
        }
        catch (const OpenFailed &)
        {
            std::cerr << "无法打开串口：" << PORT_NAME << std::endl;
            throw;
        }
    }

    SerialPortWrapper::~SerialPortWrapper()
    {
        if (serialPort.IsOpen())
        {
            serialPort.Close();
        }
    }

    uint16_t SerialPortWrapper::calculateCRC16(const uint8_t *data, uint16_t length)
    {
        uint16_t crc = 0xFFFF;
        for (uint16_t i = 0; i < length; ++i)
        {
            uint8_t index = (crc >> 8) ^ data[i];
            crc = (crc << 8) ^ CRC16_TABLE[index];
        }
        return crc;
    }

    void SerialPortWrapper::sendData(const DataBuffer &buffer)
    {
        if (buffer.size() != DATA_SIZE)
        { // 这里假设你发送的是 64 字节数据
            std::cerr << "发送数据大小错误，必须为 64 字节。" << std::endl;
            return;
        }

        try
        {
            // 调用 Write() 函数，传递 DataBuffer 类型的参数
            serialPort.Write(buffer);

            // std::cout << "已发送数据：" << buffer.size() << " 字节" << std::endl;
            // for (const auto &byte : buffer)
            // {
            //     std::cout << std::hex << static_cast<int>(byte) << " ";
            // }
        }
        catch (const std::exception &e)
        {
            std::cerr << "发生错误: " << e.what() << std::endl;
        }
    }

    /**
     * 接收64字节数据
     */
    std::vector<uint8_t> SerialPortWrapper::receiveData()
    {
        //  If msTimeout is zero, then the method will block until all requested bytes are received.
        std::vector<uint8_t> buffer(DATA_SIZE, 0);
        try
        {
            size_t bytesRead = 0;
            while (bytesRead < DATA_SIZE)
            {
                DataBuffer byte;
                serialPort.Read(byte, 1); // 每次读取1字节，超时10ms
                // buffer[bytesRead++] = static_cast<uint8_t>(byte);
                // 假设每次读取1字节，将其存储到 buffer 中
                if (!byte.empty())
                {
                    buffer[bytesRead++] = byte[0]; // 获取 byte 中的第一个字节
                }
                // std::cout << "已接收数据：" << bytesRead << " 字节" << std::endl;
            }
            // std::cout << "已接收数据：" << buffer[0] << " " << buffer[1] << " " << buffer[2] << " " << buffer[3] << " " << std::endl;
            // std::cout << "已接收数据：";
            // for (const auto &byte : buffer)
            // {
            //     std::cout << std::hex << static_cast<int>(byte) << " ";
            // }
            // std::cout << std::endl;
        }
        catch (const ReadTimeout &)
        {
            std::cerr << "接收数据超时。" << std::endl;
        }
        return buffer;
    }

    void SerialPortWrapper::packFrame(const std::vector<uint8_t> &data, std::vector<uint8_t> &frame)
    {
        if (data.size() != FRAME_DATA_LENGTH)
        {
            throw std::invalid_argument("Data length does not match FRAME_DATA_LENGTH.");
        }

        frame.resize(FRAME_SIZE);
        frame[0] = FRAME_HEAD;
        frame[1] = FRAME_DATA_LENGTH;

        for (size_t i = 0; i < FRAME_DATA_LENGTH; ++i)
        {
            frame[2 + i] = data[i];
        }

        uint16_t crc = calculateCRC16(data.data(), FRAME_DATA_LENGTH);
        frame[2 + FRAME_DATA_LENGTH] = crc & 0xFF;
        frame[2 + FRAME_DATA_LENGTH + 1] = (crc >> 8) & 0xFF;

        frame[FRAME_SIZE - 1] = FRAME_TAIL;
    }

    bool SerialPortWrapper::parseFrame(const std::vector<uint8_t> &frame, std::vector<uint8_t> &data)
    {
        if (frame.size() != FRAME_SIZE || frame[0] != FRAME_HEAD || frame[FRAME_SIZE - 1] != FRAME_TAIL)
        {
            return false;
        }

        uint16_t receivedCRC = frame[2 + FRAME_DATA_LENGTH] | (frame[2 + FRAME_DATA_LENGTH + 1] << 8);
        uint16_t calculatedCRC = calculateCRC16(frame.data() + 2, FRAME_DATA_LENGTH);

        if (receivedCRC != calculatedCRC)
        {
            return false;
        }

        data.assign(frame.begin() + 2, frame.begin() + 2 + FRAME_DATA_LENGTH);
        return true;
    }

    // 接收线程的主循环
    void SerialPortWrapper::receiveLoop()
    {
        while (running)
        {
            auto data = receiveData(); // 从串口接收数据
            {
                //用于锁住 dataMutex，确保在锁的作用域内，只有一个线程能访问共享数据。
                std::lock_guard<std::mutex> lock(dataMutex);
                receivedData = data; // 更新接收到的数据，receivedData 是共享资源
                // std::cout << std::hex << data[0] << " "<< data[1]<< " "<< data[2]<< " "<< data[3]<<std::endl;
                parseFrame(receivedData, receivedmotorData);
            }

            // 控制循环频率，小于 1ms
            std::this_thread::sleep_for(std::chrono::microseconds(500));
        }
    }

    // 启动接收线程
    void SerialPortWrapper::startReceiving()
    {
        running = true;
        receiveThread = std::thread(&SerialPortWrapper::receiveLoop, this);
    }

    // 停止接收线程
    void SerialPortWrapper::stopReceiving()
    {
        running = false;
        if (receiveThread.joinable())
        {
            receiveThread.join(); // 等待线程退出
        }
    }

    // 获取接收的数据
    std::vector<uint8_t> SerialPortWrapper::getReceivedData()
    {
        std::lock_guard<std::mutex> lock(dataMutex);

        return receivedmotorData;
    }

} // namespace libserial_example
