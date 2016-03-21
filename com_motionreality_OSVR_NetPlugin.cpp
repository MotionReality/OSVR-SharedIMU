#include <osvr/PluginKit/PluginKit.h>
#include <osvr/PluginKit/AnalogInterfaceC.h>
#include <osvr/PluginKit/TrackerInterfaceC.h>

// Generated JSON header file
#include "com_motionreality_OSVR_NetPlugin_json.h"

#include <iostream>

#include <WinSock2.h>
#include <ws2tcpip.h>
#include <Windows.h>

#pragma optimize("",off)
#pragma comment(lib,"Ws2_32.lib")

// Anonymous namespace to avoid symbol collision
namespace {

    class OSVR_NetPlugin
    {
    public:
        OSVR_NetPlugin(OSVR_PluginRegContext ctx)
            : m_socket(INVALID_SOCKET)
        {
            OpenSocket();            

            /// Create the initialization options
            OSVR_DeviceInitOptions opts = osvrDeviceCreateInitOptions(ctx);
            osvrDeviceTrackerConfigure(opts, &m_tracker);
            osvrDeviceAnalogConfigure(opts, &m_analog, 2);

            /// Create the device token with the options
            m_dev.initSync(ctx, "OSVR_NetPlugin", opts);

            /// Send JSON descriptor
            m_dev.sendJsonDescriptor(com_motionreality_OSVR_NetPlugin_json);

            /// Register update callback
            m_dev.registerUpdateCallback(this);
        }

        ~OSVR_NetPlugin()
        {
            CloseSocket();
        }

        OSVR_ReturnCode update()
        {
            if (m_socket == INVALID_SOCKET)
            {
                return OSVR_RETURN_FAILURE;
            }

            Msg msg;
            while (true)
            {
                int const recvFlags = 0;
                int const count = ::recv(m_socket, (char*)&msg, sizeof(msg), recvFlags);
                if (count == SOCKET_ERROR)
                {
                    int const err = ::WSAGetLastError();
                    if (err == WSAEMSGSIZE)
                    {
                        std::cerr << "WSAEMSGSIZE" << std::endl;
                        continue; // If it's too big, it must not be one of our messages anyhow
                    }
                    // This catches WSAEWOULDBLOCK as well
                    break; // We're done here
                }
                else if (count == 0)
                {
                    // Suppose to signal graceful shutdown of the 'connection'
                    // Could just be an empty packet
                    break;
                }
                else if (count == sizeof(Msg))
                {
                    ReportMessage(msg);
                    continue;
                }
                else
                {
                    continue; // Not one of our messages anyhow
                }
            }

            return OSVR_RETURN_SUCCESS;
        }

    private:
        SOCKET m_socket;

        osvr::pluginkit::DeviceToken m_dev;
        OSVR_TrackerDeviceInterface m_tracker;
        OSVR_AnalogDeviceInterface m_analog;

        struct Msg
        {
            unsigned __int32 version;
            unsigned __int32 videoStatus;

            float q_x;
            float q_y;
            float q_z;
            float q_w;

            // This should already be converted in "canonical" space, as per VRPN code
            float dq_x;
            float dq_y;
            float dq_z;
            float dq_w;
        };

        void ReportMessage(Msg const & msg)
        {
            OSVR_TimeValue tvNow;
            osvrTimeValueGetNow(&tvNow);

            OSVR_OrientationState q;
            osvrQuatSetX(&q, msg.q_x);
            osvrQuatSetY(&q, msg.q_y);
            osvrQuatSetZ(&q, msg.q_z);
            osvrQuatSetW(&q, msg.q_w);
            osvrDeviceTrackerSendOrientationTimestamped(m_dev, m_tracker, &q, 0, &tvNow);

            OSVR_AngularVelocityState dq;
            dq.dt = 1;
            osvrQuatSetX(&dq.incrementalRotation, msg.dq_x);
            osvrQuatSetY(&dq.incrementalRotation, msg.dq_y);
            osvrQuatSetZ(&dq.incrementalRotation, msg.dq_z);
            osvrQuatSetW(&dq.incrementalRotation, msg.dq_w);
            osvrDeviceTrackerSendAngularVelocityTimestamped(m_dev, m_tracker, &dq, 0, &tvNow);

            /// Report the value of channel 0
            osvrDeviceAnalogSetValueTimestamped(m_dev, m_analog, msg.version, 0, &tvNow);
            osvrDeviceAnalogSetValueTimestamped(m_dev, m_analog, msg.videoStatus, 1, &tvNow);

            static int counter = 0;
            if ((counter % 400) == 0)
            {
                fprintf(stderr, "MSG: %2.6f, %2.6f, %2.6f, %2.6f  (counter = %u)\n", msg.q_x, msg.q_y, msg.q_z, msg.q_w, counter);
            }
            ++counter;
        }

        void OpenSocket()
        {
            std::cerr << "Opening socket" << std::endl;

            WSADATA wsaData;
            if (::WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
            {
                std::cerr << "Failed to initialize WinSock." << std::endl;
                std::cerr << "GetLastError() = " << ::GetLastError() << std::endl;
                return;
            }

            m_socket = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            if (m_socket == INVALID_SOCKET)
            {
                std::cerr << "Failed to create socket" << std::endl;
                std::cerr << "GetLastError() = " << ::GetLastError() << std::endl;
                return;
            }

            {
                int val = 1;
                if (setsockopt(m_socket, SOL_SOCKET, SO_REUSEADDR, (char*)&val, sizeof(val)) != 0)
                {
                    std::cerr << "setsockopt(SO_REUSEADDR) failed" << std::endl;
                    std::cerr << "GetLastError() = " << ::GetLastError() << std::endl;
                    CloseSocket();
                    return;
                }
            }

            short const port = 6002;
            {
                SOCKADDR_IN addr = { 0 };
                addr.sin_family = AF_INET;
                addr.sin_port = htons(port);
                addr.sin_addr.S_un.S_addr = htonl(INADDR_LOOPBACK);

                std::cerr << "Binding socket" << std::endl;
                if (::bind(m_socket, (SOCKADDR*)&addr, sizeof(addr)) != 0)
                {
                    std::cerr << "Failed to bind socket to port " << port << std::endl;
                    std::cerr << "GetLastError() = " << ::GetLastError() << std::endl;
                    CloseSocket();
                    return;
                }
            }

            {
                // If iMode!=0, non-blocking mode is enabled.
                u_long iMode = 1;
                ioctlsocket(m_socket, FIONBIO, &iMode);
            }

            std::cerr << "Finished opening socket" << std::endl;
        }

        void CloseSocket()
        {
            if (m_socket != INVALID_SOCKET)
            {
                ::closesocket(m_socket);
                m_socket = INVALID_SOCKET;
            }
            ::WSACleanup();
        }
    };

    class HardwareDetection {
    public:
        HardwareDetection() : m_once(false) {}
        OSVR_ReturnCode operator()(OSVR_PluginRegContext ctx) {
            std::cout << "PLUGIN: Got a hardware detection request" << std::endl;
            if (!m_once) {
                m_once = true;

                /// Create our device object
                osvr::pluginkit::registerObjectForDeletion(ctx, new OSVR_NetPlugin(ctx));
            }
            return OSVR_RETURN_SUCCESS;
        }

    private:
        bool m_once; ///< @brief Only load the device once
    };
} // namespace

OSVR_PLUGIN(com_motionreality_OSVR_NetPlugin) {
    osvr::pluginkit::PluginContext context(ctx);

    std::cerr << "Loading plugin: com_motionreality_OSVR_NetPlugin" << std::endl;

    /// Create our device object
    //osvr::pluginkit::registerObjectForDeletion(ctx, new OSVR_NetPlugin(ctx));

    /// Register a detection callback function object.
    context.registerHardwareDetectCallback(new HardwareDetection());

    return OSVR_RETURN_SUCCESS;
}
