#include <osvr/PluginKit/PluginKit.h>
#include <osvr/PluginKit/AnalogInterfaceC.h>
#include <osvr/PluginKit/TrackerInterfaceC.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

// Generated JSON header file
#include "com_motionreality_OSVR_NetPlugin_json.h"

#include <iostream>

#include <WinSock2.h>
#include <ws2tcpip.h>
#include <Windows.h>

#include <hidapi.h>
#include <thread>
#include <atomic>
#include <mutex>

#pragma optimize("",off)
#pragma comment(lib,"Ws2_32.lib")

// Anonymous namespace to avoid symbol collision
namespace {

    class OSVR_NetPlugin
    {
    public:
        OSVR_NetPlugin(OSVR_PluginRegContext ctx)
            : m_lastStatus(EnStatus_CLOSED)
            , m_timeoutCount(0)
            , m_socket(INVALID_SOCKET)
            , m_pHidDevice(nullptr)
            , m_updateCounter(0)
        {
            OpenSocket();            

            if (hid_init())
            {
                throw std::runtime_error("Failed to initialize HID library");
            }

            /// Create the initialization options
            OSVR_DeviceInitOptions opts = osvrDeviceCreateInitOptions(ctx);
            osvrDeviceTrackerConfigure(opts, &m_tracker);
            osvrDeviceAnalogConfigure(opts, &m_analog, 2);

            /// Create the device token with the options
            m_devToken.initSync(ctx, "OSVR_NetPlugin", opts);

            /// Send JSON descriptor
            m_devToken.sendJsonDescriptor(com_motionreality_OSVR_NetPlugin_json);

            /// Register update callback
            m_devToken.registerUpdateCallback(this);
            std::cerr << "ctor" << std::endl;

            m_recvQueue.reserve(100);
            m_processQueue.reserve(100);

            m_shutdown = false;
            m_thread = std::move(std::thread(&OSVR_NetPlugin::ThreadEntry, this));
        }

        ~OSVR_NetPlugin()
        {
            std::cerr << "dtor" << std::endl;
            m_shutdown = true;
            CloseSocket();
            if (m_thread.joinable())
                m_thread.join();
            CloseDevice();            
        }

        OSVR_ReturnCode update()
        {
            {
                std::lock_guard<std::mutex> lockGuard(m_dataMutex);
                m_processQueue.swap(m_recvQueue);
            }

            for ( auto const & msgBuf : m_processQueue )
            {
                uint16_t const * const pShorts = reinterpret_cast<uint16_t const *>(&msgBuf.data[2]);

                unsigned int const ver = msgBuf.data[0] & 0x0F;

                double fValues[7];
                double const SCALE = 1.f / (1 << 14);
                for (size_t i = 0; i < 7; ++i)
                {
                    fValues[i] = pShorts[i] * SCALE;
                }

                OSVR_OrientationState q;
                osvrQuatSetX(&q, fValues[0]);
                osvrQuatSetY(&q, fValues[1]);
                osvrQuatSetZ(&q, fValues[2]);
                osvrQuatSetW(&q, fValues[3]);
                osvrDeviceTrackerSendOrientationTimestamped(m_devToken, m_tracker, &q, 0, &msgBuf.timestamp);

                if (ver >= 2)
                {
                    auto const dt = 1. / 400.;
                    auto const rotVec = Eigen::Map<Eigen::Vector3d>(&fValues[4]);
                    auto const magnitude = rotVec.norm(); // radians per second
                    auto const rotAxis = rotVec / magnitude;
                    auto const deltaAngle = magnitude * dt; // radians per dt
                    auto const qDelta = Eigen::Quaterniond(Eigen::AngleAxisd(deltaAngle, rotAxis));
                    auto const qBase = Eigen::Map<Eigen::Quaterniond>(&fValues[0]);
                    auto const qIncRot = (qBase * qDelta * qBase.inverse()).normalized();

                    OSVR_AngularVelocityState dq;
                    dq.dt = dt;
                    osvrQuatSetX(&dq.incrementalRotation, qIncRot.x());
                    osvrQuatSetY(&dq.incrementalRotation, qIncRot.y());
                    osvrQuatSetZ(&dq.incrementalRotation, qIncRot.z());
                    osvrQuatSetW(&dq.incrementalRotation, qIncRot.w());
                    osvrDeviceTrackerSendAngularVelocityTimestamped(m_devToken, m_tracker, &dq, 0, &msgBuf.timestamp);
                }

                enum {
                    VideoStatus_UNKNOWN = 0,
                    VideoStatus_NO_VIDEO_INPUT = 1,
                    VideoStatus_PORTRAIT_VIDEO_INPUT = 2,
                    VideoStatus_LANDSCAPE_VIDEO_INPUT = 3,
                };
                int videoStatus = VideoStatus_UNKNOWN;
                if (ver >= 3)
                {
                    // v3+: We've got status info in the upper nibble of the first byte.
                    bool gotVideo = (msgBuf.data[0] & 0x10) != 0;    // got video?
                    bool gotPortrait = (msgBuf.data[0] & 0x20) != 0; // portrait mode?
                    if (!gotVideo) {
                        videoStatus = VideoStatus_NO_VIDEO_INPUT;
                    }
                    else {
                        if (gotPortrait) {
                            videoStatus = VideoStatus_PORTRAIT_VIDEO_INPUT;
                        }
                        else {
                            videoStatus = VideoStatus_LANDSCAPE_VIDEO_INPUT;
                        }
                    }
                }

                /// Report the value of channel 0
                osvrDeviceAnalogSetValueTimestamped(m_devToken, m_analog, ver, 0, &msgBuf.timestamp);
                osvrDeviceAnalogSetValueTimestamped(m_devToken, m_analog, videoStatus, 1, &msgBuf.timestamp);
            }
            return OSVR_RETURN_SUCCESS;
        }

    private:
        osvr::pluginkit::DeviceToken m_devToken;
        OSVR_TrackerDeviceInterface m_tracker;
        OSVR_AnalogDeviceInterface m_analog;

        std::atomic<bool> m_shutdown;
        std::thread m_thread;
        std::mutex m_dataMutex;
        struct MsgBuffer
        {
            OSVR_TimeValue timestamp;
            uint8_t data[16];
        };
        std::vector<MsgBuffer> m_recvQueue;
        std::vector<MsgBuffer> m_processQueue;

        enum EnStatus
        {
            EnStatus_CLOSED,
            EnStatus_FAILED,
            EnStatus_OPENED,
        } m_lastStatus;
        size_t m_timeoutCount;
        size_t m_updateCounter;

        SOCKET m_socket;
        hid_device * m_pHidDevice;        
        SOCKADDR_IN m_remoteAddr;

        unsigned short m_sLastValues[4];

        void ThreadEntry()
        {
            while (!m_shutdown)
            {
                AcceptIncomingConnections();

                if(!m_pHidDevice)
                {
                    if(OpenDevice())
                    {
                        m_lastStatus = EnStatus_OPENED;
                    }
                    else
                    {
                        if (EnStatus_FAILED != m_lastStatus)
                        {
                            m_lastStatus = EnStatus_FAILED;
                            std::cerr << "<NetPlugin> Failed to open device" << std::endl;
                        }
                        // Sleep for 1 second and try again
                        for (size_t i = 0; i < 100 && !m_shutdown; ++i)
                        {
                            std::this_thread::sleep_for(std::chrono::milliseconds(10));
                        }
                        continue;
                    }
                }

                ++m_updateCounter;

                ProcessMessages();
            }
            CloseDevice();
        }

        void OpenSocket()
        {
            std::cerr << "Opening socket" << std::endl;

            WSADATA wsaData;
            if (::WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
            {
                std::cerr << "Failed to initialize WinSock." << std::endl;
                std::cerr << "GetLastError() = " << ::GetLastError() << std::endl;
                throw std::runtime_error("Failed to initialize WinSock.");
            }

            m_socket = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            if (m_socket == INVALID_SOCKET)
            {
                std::cerr << "Failed to create socket" << std::endl;
                std::cerr << "GetLastError() = " << ::GetLastError() << std::endl;
                ::WSACleanup();
                throw std::runtime_error("Failed to create socket");
            }

            {
                int val = 1;
                if (setsockopt(m_socket, SOL_SOCKET, SO_REUSEADDR, (char*)&val, sizeof(val)) != 0)
                {
                    std::cerr << "setsockopt(SO_REUSEADDR) failed" << std::endl;
                    std::cerr << "GetLastError() = " << ::GetLastError() << std::endl;
                    CloseSocket();
                    throw std::runtime_error("setsockopt(SO_REUSEADDR) failed");
                }
            }

            short const port = 6002;
            {
                SOCKADDR_IN addr = { 0 };
                addr.sin_family = AF_INET;
                addr.sin_port = htons(port);
                addr.sin_addr.S_un.S_addr = htonl(INADDR_LOOPBACK);

                if (::bind(m_socket, (SOCKADDR*)&addr, sizeof(addr)) != 0)
                {
                    std::cerr << "Failed to bind socket to port " << port << std::endl;
                    std::cerr << "GetLastError() = " << ::GetLastError() << std::endl;
                    CloseSocket();
                    throw std::runtime_error("Failed to bind socket to port");
                }
            }

            {
                // If iMode!=0, non-blocking mode is enabled.
                u_long iMode = 1;
                ioctlsocket(m_socket, FIONBIO, &iMode);
            }

            memset(&m_remoteAddr, 0, sizeof(m_remoteAddr));
            memset(&m_sLastValues[0], 0, sizeof(m_sLastValues));

            std::cerr << "Finished opening socket successfully" << std::endl;
        }

        void CloseSocket()
        {
            if (m_socket != INVALID_SOCKET)
            {
                ::closesocket(m_socket);
                m_socket = INVALID_SOCKET;
                ::WSACleanup();
            }            
        }

        bool OpenDevice()
        {
            if (m_pHidDevice)
                return true;

            // Open the device using the VID, PID,
            // and optionally the Serial number.
            struct VenDev
            {
                unsigned short vendor;
                unsigned short device;
            } static const s_devices[] = {
                { 0x03EB, 0x2421 },
                { 0x1532, 0x0b00 },
            };
            hid_device * handle = 0;

            for (size_t i = 0; i < sizeof(s_devices) / sizeof(VenDev); ++i)
            {
                auto const & id = s_devices[i];
                m_pHidDevice = hid_open(id.vendor, id.device, NULL);
                if (m_pHidDevice)
                {
                    char buffer[128];
                    sprintf(buffer, "<NetPlugin> Opened OSVR HDK IMU device (VID%04X/PID%04X)", id.vendor, id.device);
                    std::cerr << buffer << std::endl;
                    break;
                }
            }

            if (!m_pHidDevice)
            {
                return false;
            }

            // Dump all existing data to ensure that we are timestamping fresh data
            int ret = 0;
            unsigned char buf[256];
            while ((ret = hid_read_timeout(m_pHidDevice, buf, sizeof(buf), 0)) > 0)
            {
                /*No op*/
            }

            if (ret < 0) // Error while purging data
            {
                std::cerr << "Error reading HID device: " << ret << std::endl;
                CloseDevice();
                return false;
            }

            m_timeoutCount = 0;
            return true;
        }

        void CloseDevice()
        {
            if (m_pHidDevice)
            {
                hid_close(m_pHidDevice);
                m_pHidDevice = nullptr;
            }
        }

        void AcceptIncomingConnections()
        {
            // Don't check the socket every update. It's just wasteful
            if (m_lastStatus == EnStatus_OPENED)
            {
                if ((m_updateCounter % 100) == 0 || m_timeoutCount > 0)
                    return;
            }

            char buffer[128];
            SOCKADDR_IN fromAddr = { 0 };
            int const recvFlags = 0;
            while (true)
            {
                int fromAddrLen = sizeof(fromAddr);
                int const ret = ::recvfrom(m_socket, buffer, sizeof(buffer), recvFlags, (SOCKADDR*)&fromAddr, &fromAddrLen);
                if (ret != SOCKET_ERROR)
                {
                    m_remoteAddr = fromAddr;
                }
                else
                {
                    int const err = ::GetLastError();
                    if (err == WSAEWOULDBLOCK)
                        break;
                    
                    if (err != WSAEMSGSIZE)
                    {
                        std::cerr << "<NetPlugin> Socket error: " << err << std::endl;
                        CloseSocket();
                        return;
                    }
                }
            }            
        }

        void ProcessMessages()
        {
            if (m_socket == INVALID_SOCKET || !m_pHidDevice)
                return;

            MsgBuffer msgBuf;
            int timeout_millis = 100; // Initial timeout is high to detect drop out
            while (!m_shutdown)
            {
                int nBytesRead = hid_read_timeout(m_pHidDevice, msgBuf.data, sizeof(msgBuf.data), timeout_millis);
                if (nBytesRead < 0)
                {
                    std::cerr << "<NetPlugin> Error reading HID device" << std::endl;
                    m_lastStatus = EnStatus_FAILED;
                    hid_close(m_pHidDevice);
                    m_pHidDevice = nullptr;
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    return;
                }

                if (nBytesRead == 0)
                {
                    if (timeout_millis > 0)
                    {
                        if (m_timeoutCount == 0)
                        {
                            std::cerr << "<NetPlugin> Timeout reading HID device" << std::endl;
                        }
                        ++m_timeoutCount;
                    }
                    return;
                }

                // Once we've received a message this update, we just want to check
                // quickly if there are any other messages pending
                timeout_millis = 0;
                m_timeoutCount = 0;

                if (nBytesRead < sizeof(msgBuf.data))
                {
                    std::cerr << "<NetPlugin> Runt message @ len = " << nBytesRead << std::endl;
                    continue;
                }

                unsigned int ver = (msgBuf.data[0] & 0x0F);
                unsigned int seq = msgBuf.data[1];
                auto const * pShorts = (signed __int16 const *)&msgBuf.data[2];

                bool const bChanged = (pShorts[0] != m_sLastValues[0]) ||
                                      (pShorts[1] != m_sLastValues[1]) ||
                                      (pShorts[2] != m_sLastValues[2]) ||
                                      (pShorts[3] != m_sLastValues[3]);
                if (!bChanged)
                    continue;

                osvrTimeValueGetNow(&msgBuf.timestamp);

                // Immediately send along to game
                // Hopefully this is fast enough that we still get quality timestamps in the game
                if (m_socket != INVALID_SOCKET && m_remoteAddr.sin_port != 0)
                {
                    ::sendto(m_socket, (char*)&msgBuf.data[0], nBytesRead, 0, (SOCKADDR*)&m_remoteAddr, sizeof(m_remoteAddr));
                }

                for (size_t i = 0; i < 4; ++i)
                    m_sLastValues[i] = pShorts[i];

                // Enqueue
                {
                    std::lock_guard<std::mutex> lockGuard(m_dataMutex);
                    m_recvQueue.emplace_back(msgBuf);
                }
             }
        }
    };

} // namespace

OSVR_PLUGIN(com_motionreality_OSVR_NetPlugin) {
    osvr::pluginkit::PluginContext context(ctx);

    std::cerr << "Loading plugin: com_motionreality_OSVR_NetPlugin" << std::endl;

    /// Create our device object
    static bool bOnce = false;
    if (!bOnce)
    {
        osvr::pluginkit::registerObjectForDeletion(ctx, new OSVR_NetPlugin(ctx));
        bOnce = true;
    }
    
    return OSVR_RETURN_SUCCESS;
}
