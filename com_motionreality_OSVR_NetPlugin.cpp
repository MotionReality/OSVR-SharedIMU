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

#pragma optimize("",off)
#pragma comment(lib,"Ws2_32.lib")

// Anonymous namespace to avoid symbol collision
namespace {

    class OSVR_NetPlugin
    {
    public:
        OSVR_NetPlugin(OSVR_PluginRegContext ctx)
            : m_socket(INVALID_SOCKET)
            , m_pHidDevice(nullptr)
            , m_lastOpenAttempt()
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
            m_devToken.initAsync(ctx, "OSVR_NetPlugin", opts);

            /// Send JSON descriptor
            m_devToken.sendJsonDescriptor(com_motionreality_OSVR_NetPlugin_json);

            /// Register update callback
            m_devToken.registerUpdateCallback(this);
        }

        ~OSVR_NetPlugin()
        {
            CloseDevice();
            CloseSocket();
        }

        OSVR_ReturnCode update()
        {
            if (m_socket == INVALID_SOCKET)
            {
                osvrDeviceMicrosleep(50000);
                return OSVR_RETURN_FAILURE;
            }

            if (!m_pHidDevice)
            {
                OSVR_TimeValue now;
                osvrTimeValueGetNow(&now);
                auto const ageSeconds = osvrTimeValueDurationSeconds(&now, &m_lastOpenAttempt);
                if (ageSeconds > 1)
                {
                    m_lastOpenAttempt = now;
                    if (!OpenDevice())
                    {
                        std::cerr << "<NetPlugin> Failed to open device" << std::endl;
                        return OSVR_RETURN_SUCCESS; // We want more callbacks (in case this return is ever used)
                    }
                }
                else
                {
                    osvrDeviceMicrosleep(50000);
                    return  OSVR_RETURN_SUCCESS; // We want more callbacks (in case this return is ever used)
                }
            }

            ++m_updateCounter;

            ProcessMessages();
            AcceptIncomingConnections();

            return OSVR_RETURN_SUCCESS;
        }

    private:
        osvr::pluginkit::DeviceToken m_devToken;
        OSVR_TrackerDeviceInterface m_tracker;
        OSVR_AnalogDeviceInterface m_analog;

        SOCKET m_socket;
        hid_device * m_pHidDevice;
        OSVR_TimeValue m_lastOpenAttempt;
        size_t m_updateCounter;
        SOCKADDR_IN m_remoteAddr;

        unsigned short m_sLastValues[4];

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
            if (m_updateCounter % 100) // Only check socket once every ~200ms
                return;

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


            unsigned char buf[64];
            int timeout_millis = 100; // Initial timeout is high to detect drop out
            while (true)
            {
                int nBytesRead = hid_read_timeout(m_pHidDevice, buf, sizeof(buf), timeout_millis);
                if (nBytesRead < 0)
                {
                    std::cerr << "<NetPlugin> Error reading HID device" << std::endl;
                    return;
                }

                if (nBytesRead == 0)
                {
                    std::cerr << "<NetPlugin> Timeout reading HID device" << std::endl;
                    return;
                }

                // Once we've received a message this update, we just want to check
                // quickly if there are any other messages pending
                timeout_millis = 0;

                if (nBytesRead < sizeof(short) * 8)
                {
                    std::cerr << "<NetPlugin> Runt message @ len = " << nBytesRead << std::endl;
                    continue;
                }

                unsigned int ver = (buf[0] & 0x0F);
                unsigned int seq = buf[1];
                auto const * pShorts = (signed __int16 const *)&buf[2];

                bool const bChanged = (pShorts[0] != m_sLastValues[0]) ||
                                      (pShorts[1] != m_sLastValues[1]) ||
                                      (pShorts[2] != m_sLastValues[2]) ||
                                      (pShorts[3] != m_sLastValues[3]);
                if (!bChanged)
                    continue;

                // Immediately send along to game
                // Hopefully this is fast enough that we still get quality timestamps in the game
                if (m_socket != INVALID_SOCKET && m_remoteAddr.sin_port != 0)
                {
                    ::sendto(m_socket, (char*)&buf[0], nBytesRead, 0, (SOCKADDR*)&m_remoteAddr, sizeof(m_remoteAddr));
                }

                for (size_t i = 0; i < 4; ++i)
                    m_sLastValues[i] = pShorts[i];
                
                double fValues[7];
                double const SCALE = 1.f / (1 << 14);
                for (size_t i = 0; i < 7; ++i)
                {
                    fValues[i] = pShorts[i] * SCALE;
                }

                OSVR_TimeValue tvNow;
                osvrTimeValueGetNow(&tvNow);

                OSVR_OrientationState q;
                osvrQuatSetX(&q, fValues[0]);
                osvrQuatSetY(&q, fValues[1]);
                osvrQuatSetZ(&q, fValues[2]);
                osvrQuatSetW(&q, fValues[3]);
                osvrDeviceTrackerSendOrientationTimestamped(m_devToken, m_tracker, &q, 0, &tvNow);

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
                    osvrDeviceTrackerSendAngularVelocityTimestamped(m_devToken, m_tracker, &dq, 0, &tvNow);
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
                    bool gotVideo = (buf[0] & 0x10) != 0;    // got video?
                    bool gotPortrait = (buf[0] & 0x20) != 0; // portrait mode?
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
                osvrDeviceAnalogSetValueTimestamped(m_devToken, m_analog, ver, 0, &tvNow);
                osvrDeviceAnalogSetValueTimestamped(m_devToken, m_analog, videoStatus, 1, &tvNow);
            }
        }
    };

} // namespace

OSVR_PLUGIN(com_motionreality_OSVR_NetPlugin) {
    osvr::pluginkit::PluginContext context(ctx);

    std::cerr << "Loading plugin: com_motionreality_OSVR_NetPlugin" << std::endl;

    /// Create our device object
    osvr::pluginkit::registerObjectForDeletion(ctx, new OSVR_NetPlugin(ctx));
    
    return OSVR_RETURN_SUCCESS;
}
